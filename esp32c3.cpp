/*
 * Copyright (C) 2026 ASAsoft
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <math.h> 

#define LED_PIN 8
#define LED_ON LOW
#define LED_OFF HIGH

static uint32_t seq = 0;
const float PI_CONST = 3.1415926535f;
static unsigned long last_led_ms = 0;
static unsigned long last_logic_ms = 0;

static const unsigned long LOGIC_INTERVAL_MS = 200; 
static const unsigned long LED_PULSE_MS = 50;
static uint32_t last_tlm_seq = 0;

static unsigned long last_valid_tlm_time = 0;
static const unsigned long TLM_TIMEOUT_MS = 3000;
static bool error_state = false;

enum ProgramState {
    PRG_IDLE,
    PRG_WAIT_START,
    PRG_ROTATE_CLIMB,
    PRG_ACCELERATE,
    PRG_PREPARE_HOLD,
    PRG_ALTITUDE_HOLD,
    PRG_COMPLETE
};
static ProgramState prg_state = PRG_IDLE;
static unsigned long program_start_time = 0;
static unsigned long last_cmd_sent = 0;
static float last_altitude = 0.0f;
static float last_velocity = 0.0f;
static float last_pitch = 0.0f;
static float last_bank = 0.0f;
static float last_heading = 0.0f;
static float last_apoapsis = 0.0f; 
static float prev_apoapsis = 0.0f; 
static float last_periapsis = 0.0f; 

static float last_trim_elevator = 0.0f;
static float last_trim_aileron = 0.0f;
static float last_trim_rudder = 0.0f;

static float target_pitch = 0.0f;
#define TARGET_HEADING_DEGREE 138.0f
static float target_heading_rad = TARGET_HEADING_DEGREE * (PI_CONST / 180.0f); 
static float target_bank_rad = 0.0f;
static float prev_pitch_cmd = 999.0f;
static float prev_yaw_cmd = 999.0f;
static float prev_roll_cmd = 999.0f;
static float prev_main_cmd = 999.0f;

static float pitch_integral = 0.0f;
static float prev_pitch_error = 0.0f;
static unsigned long last_pitch_update = 0;
static float prev_elevator_cmd = 0.0f;

static float prev_heading_error = 0.0f;
static unsigned long last_heading_update = 0;
static float prev_roll_command = 0.0f;

static bool gear_retracted = false;
static bool navmode_prograde_active = false;

String rxBuffer;
void sendCommand(const char* cmd, const char* val)
{
    Serial.print("SEQ=");
    Serial.print(seq++);
    Serial.print(";CMD=");
    Serial.print(cmd);
    Serial.print(";VAL=");
    Serial.println(val);
    last_cmd_sent = millis();
}

void resetAllState()
{
    prg_state = PRG_IDLE;
    program_start_time = 0;
    gear_retracted = false;
    navmode_prograde_active = false;
    
    last_tlm_seq = 0;
    
    prev_apoapsis = 0.0f;
    
    pitch_integral = 0.0f;
    prev_pitch_error = 0.0f;
    last_pitch_update = 0;
    prev_elevator_cmd = 0.0f;
    prev_heading_error = 0.0f;
    last_heading_update = 0;
    prev_roll_command = 0.0f;
    
    target_pitch = 0.0f;
    target_bank_rad = 0.0f;
    prev_pitch_cmd = 999.0f;
    prev_yaw_cmd = 999.0f;
    prev_roll_cmd = 999.0f;
    prev_main_cmd = 999.0f;
    
    error_state = false;
    rxBuffer.clear();
    
    last_apoapsis = 0.0f;
    last_periapsis = 0.0f;
    last_trim_elevator = 0.0f;
    last_trim_aileron = 0.0f;
    last_trim_rudder = 0.0f;
}

// Telemetry parsing functions (omitted for brevity, assume correct implementation)
bool parseTelemetryLine(const String& line)
{
    if (!line.startsWith("TLM=")) return false;
    if (line.indexOf(";VEL=") < 0) return false;
    if (line.indexOf(";ANG=") < 0) return false;
    if (line.indexOf(";ATT=") < 0) return false;
    if (line.indexOf(";APO=") < 0) return false;
    if (line.indexOf(";PER=") < 0) return false;
    if (line.indexOf(";TRIM=") < 0) return false;
    return true;
}

bool extractTlmSeq(const String& line, uint32_t& tlm)
{
    int p = line.indexOf(';');
    if (p < 0) return false;
    tlm = line.substring(4, p).toInt();
    return true;
}

bool extractVelocityMagnitude(const String& line, float& vmag)
{
    int p_vmag = line.indexOf(";VMAG=");
    if (p_vmag < 0) return false;
    
    int p_end = line.indexOf(';', p_vmag + 6);
    if (p_end < 0) p_end = line.length();
    String vmag_str = line.substring(p_vmag + 6, p_end);
    vmag = vmag_str.toFloat();
    return true;
}

bool extractAltitude(const String& line, float& alt)
{
    int p_alt = line.indexOf(";ALT=");
    if (p_alt < 0) return false;
    int p_end = line.indexOf(';', p_alt + 5);
    if (p_end < 0) p_end = line.length();
    String alt_str = line.substring(p_alt + 5, p_end);
    alt = alt_str.toFloat();
    return true;
}

bool extractPitch(const String& line, float& pitch)
{
    int p_att = line.indexOf(";ATT=");
    if (p_att < 0) return false;
    int p_end = line.indexOf(';', p_att + 5);
    if (p_end < 0) p_end = line.length();
    String att_str = line.substring(p_att + 5, p_end);
    
    int comma1 = att_str.indexOf(',');
    if (comma1 < 0) return false;
    String pitch_str = att_str.substring(0, comma1);
    pitch = pitch_str.toFloat();
    return true;
}

bool extractBank(const String& line, float& bank)
{
    int p_att = line.indexOf(";ATT=");
    if (p_att < 0) return false;
    int p_end = line.indexOf(';', p_att + 5);
    if (p_end < 0) p_end = line.length();
    String att_str = line.substring(p_att + 5, p_end);
    
    int comma1 = att_str.indexOf(',');
    int comma2 = att_str.indexOf(',', comma1 + 1);
    if (comma1 < 0 || comma2 < 0) return false;
    
    String bank_str = att_str.substring(comma1 + 1, comma2);
    bank = bank_str.toFloat();
    return true;
}

bool extractHeading(const String& line, float& heading)
{
    int p_att = line.indexOf(";ATT=");
    if (p_att < 0) return false;
    
    int p_end = line.indexOf(';', p_att + 5);
    if (p_end < 0) p_end = line.length();
    String att_str = line.substring(p_att + 5, p_end);
    
    int comma1 = att_str.indexOf(',');
    int comma2 = att_str.indexOf(',', comma1 + 1);
    if (comma1 < 0 || comma2 < 0) return false;
    
    String heading_str = att_str.substring(comma2 + 1);
    heading = heading_str.toFloat();
    return true;
}

bool extractApoapsis(const String& line, float& apo)
{
    int p_apo = line.indexOf(";APO=");
    if (p_apo < 0) return false;
    int p_end = line.indexOf(';', p_apo + 5); 
    if (p_end < 0) p_end = line.length();
    String apo_str = line.substring(p_apo + 5, p_end);
    apo = apo_str.toFloat(); 
    return true;
}

bool extractPeriapsis(const String& line, float& peri)
{
    int p_peri = line.indexOf(";PER="); 
    if (p_peri < 0) return false;
    int p_end = line.indexOf(';', p_peri + 5); 
    if (p_end < 0) p_end = line.length();
    String peri_str = line.substring(p_peri + 5, p_end);
    peri = peri_str.toFloat(); 
    return true;
}

bool extractTrimValues(const String& line, float& elev, float& ail, float& rud)
{
    int p_trim = line.indexOf(";TRIM=");
    if (p_trim < 0) return false;
    
    int p_start = p_trim + 6; 
    int p_end = line.indexOf(';', p_start);
    if (p_end < 0) p_end = line.length();
    String trim_data = line.substring(p_start, p_end);
    
    int comma1 = trim_data.indexOf(',');
    int comma2 = trim_data.indexOf(',', comma1 + 1);
    
    if (comma1 < 0 || comma2 < 0) {
        return false;
    }

    elev = trim_data.substring(0, comma1).toFloat();
    ail = trim_data.substring(comma1 + 1, comma2).toFloat();
    rud = trim_data.substring(comma2 + 1).toFloat();
    return true;
}

void processTelemetry(const String& line)
{
    if (!parseTelemetryLine(line)) {
        goto fail;
    }
    
    uint32_t tlm;
    if (!extractTlmSeq(line, tlm)) {
        goto fail;
    }
    
    if (tlm < last_tlm_seq) {
        return;
    }

    if (tlm > last_tlm_seq) {
        prev_apoapsis = last_apoapsis;
        last_tlm_seq = tlm;
        last_valid_tlm_time = millis();
        error_state = false;

        digitalWrite(LED_PIN, LED_ON);
        last_led_ms = millis();

        if (!extractAltitude(line, last_altitude)) goto fail;
        if (!extractVelocityMagnitude(line, last_velocity)) goto fail;
        if (!extractPitch(line, last_pitch)) goto fail;
        if (!extractBank(line, last_bank)) goto fail;
        if (!extractHeading(line, last_heading)) goto fail;
        if (!extractApoapsis(line, last_apoapsis)) goto fail;
        if (!extractPeriapsis(line, last_periapsis)) goto fail;
        
        if (!extractTrimValues(line, last_trim_elevator, last_trim_aileron, last_trim_rudder)) goto fail;

        if (prg_state == PRG_IDLE) {
            prg_state = PRG_WAIT_START;
            program_start_time = millis();
        }
    }
    return;
    fail:
        digitalWrite(LED_PIN, LED_ON);
        last_led_ms = millis();
        error_state = true;
        resetAllState();
        return;
}

float controlPitchAngle(float target_pitch_rad, float current_pitch_rad, float current_altitude, unsigned long now)
{
    
    if (last_pitch_update == 0) {
        last_pitch_update = now;
        return 0.0f;
    }
    
    float dt = (now - last_pitch_update) / 1000.0f;
    if (dt <= 0.0f || dt > 1.0f) dt = 0.2f;

    float Kp, Kd, Ki, max_cmd, alpha;
    
    const float ALT_SURF_MAX = 100000.0f; 
    const float ALT_RCS_MIN = 80000.0f;
    
    const float PITCH_FLOOR_ALT_START = 40000.0f;
    const float PITCH_FLOOR_RAD = 0.05236f;
    
    const float SURF_KP = 1.5f; 
    const float SURF_KD = 0.5f; 
    const float SURF_KI = 0.05f; 
    const float SURF_MAX_CMD = 0.3f;
    const float SURF_ALPHA = 0.4f;

    const float RCS_KP = 0.02f;
    const float RCS_KD = 0.01f;
    const float RCS_KI = 0.005f;
    const float RCS_MAX_CMD = 0.1f;
    const float RCS_ALPHA = 0.1f;
    
    if (current_altitude < ALT_SURF_MAX) {
        Kp = SURF_KP; Kd = SURF_KD;
        Ki = SURF_KI;
        max_cmd = SURF_MAX_CMD; alpha = SURF_ALPHA;
    } else if (current_altitude > ALT_RCS_MIN) {
        Kp = RCS_KP;
        Kd = RCS_KD; Ki = RCS_KI;
        max_cmd = RCS_MAX_CMD; alpha = RCS_ALPHA;
    } else {
        float blend_factor = (current_altitude - ALT_SURF_MAX) / (ALT_RCS_MIN - ALT_SURF_MAX);
        Kp = SURF_KP + (RCS_KP - SURF_KP) * blend_factor;
        Kd = SURF_KD + (RCS_KD - SURF_KD) * blend_factor;
        Ki = SURF_KI + (RCS_KI - SURF_KI) * blend_factor;
        max_cmd = SURF_MAX_CMD + (RCS_MAX_CMD - SURF_MAX_CMD) * blend_factor;
        alpha = SURF_ALPHA + (RCS_ALPHA - SURF_ALPHA) * blend_factor;
    }
    
    float pitch_error = target_pitch_rad - current_pitch_rad;
    float elevator_cmd;
    float output_cmd;
    
    if (pitch_error * prev_pitch_error < 0 && fabs(pitch_error) < 0.05) {
        pitch_integral = 0.0f;
    }

    float pitch_rate = (pitch_error - prev_pitch_error) / dt;
    float P = Kp * pitch_error;
    float D = -Kd * pitch_rate;
    pitch_integral += pitch_error * dt;
    
    float max_integral = 0.2f; 
    if (pitch_integral > max_integral) pitch_integral = max_integral;
    if (pitch_integral < -max_integral) pitch_integral = -max_integral;
    float I = Ki * pitch_integral;
    elevator_cmd = P + I + D;
    
    prev_pitch_error = pitch_error;
    
    if (elevator_cmd > max_cmd) elevator_cmd = max_cmd;
    
    const float MIN_FLOOR_CORRECTION_CMD = 0.1f; 
    if (current_altitude > PITCH_FLOOR_ALT_START) {
        if (current_pitch_rad < PITCH_FLOOR_RAD) {
            if (elevator_cmd < 0.0f) { 
                elevator_cmd = MIN_FLOOR_CORRECTION_CMD;
            }
            if (P + I + D < 0.0f) { 
                pitch_integral = 0.0f;
            }
        }
    }

    if (elevator_cmd < -max_cmd) elevator_cmd = -max_cmd;
    output_cmd = alpha * elevator_cmd + (1.0f - alpha) * prev_elevator_cmd;
    prev_elevator_cmd = output_cmd;
    
    last_pitch_update = now;
    
    return output_cmd;
}

float controlHeadingAngle(float target_heading_rad, float current_heading_rad, float current_bank_rad, float current_altitude, unsigned long now)
{
    static float turn_direction_sign = 0.0f; 

    if (last_heading_update == 0) {
        
        float initial_heading_error = target_heading_rad - current_heading_rad; 
        
        while (initial_heading_error > PI_CONST) initial_heading_error -= 2 * PI_CONST; 
        while (initial_heading_error < -PI_CONST) initial_heading_error += 2 * PI_CONST; 
        
        if (fabs(initial_heading_error) < 0.001f) {
            turn_direction_sign = 0.0f; 
        } else {
            turn_direction_sign = (initial_heading_error > 0.0f) ? -1.0f : 1.0f; 
        }

        last_heading_update = now; 
        return 0.0f; 
    }
    
    float dt = (now - last_heading_update) / 1000.0f; 
    if (dt <= 0.0f || dt > 1.0f) dt = 0.2f; 

    const float HEADING_TERMINATE_THRESHOLD_DEG = 1.0f; 
    const float HEADING_CAPTURE_THRESHOLD_DEG = 10.0f; 
    const float BANK_MAX_DEG = 35.0f; 
    const float BANK_CAPTURE_DEG = 5.0f; 
    
    const float HEADING_TERMINATE_THRESHOLD_RAD = HEADING_TERMINATE_THRESHOLD_DEG * (PI_CONST / 180.0f); 
    const float HEADING_CAPTURE_THRESHOLD_RAD = HEADING_CAPTURE_THRESHOLD_DEG * (PI_CONST / 180.0f); 
    const float BANK_MAX_RAD = BANK_MAX_DEG * (PI_CONST / 180.0f); 
    const float BANK_CAPTURE_RAD = BANK_CAPTURE_DEG * (PI_CONST / 180.0f); 

    float heading_error = target_heading_rad - current_heading_rad; 
    while (heading_error > PI_CONST) heading_error -= 2 * PI_CONST; 
    while (heading_error < -PI_CONST) heading_error += 2 * PI_CONST; 
    
    float abs_heading_error = fabs(heading_error); 
    float required_bank_rad; 
    float bank_error; 

    if (abs_heading_error < HEADING_TERMINATE_THRESHOLD_RAD) {
        prev_heading_error = 0.0f; 
        last_heading_update = now;
        turn_direction_sign = 0.0f; 
        target_bank_rad = 0.0f;
        return 0.0f;
        
    } else if (abs_heading_error <= HEADING_CAPTURE_THRESHOLD_RAD) {
        required_bank_rad = turn_direction_sign * BANK_CAPTURE_RAD; 
        
        bank_error = required_bank_rad - current_bank_rad; 
        
    } else {
        required_bank_rad = turn_direction_sign * BANK_MAX_RAD; 
        
        bank_error = required_bank_rad - current_bank_rad; 
    }
    
    target_bank_rad = required_bank_rad; 
    
    const float MAX_AILERON_CMD_SURF = 0.3f; 
    const float Kp_bank_SURF = -0.5f; 

    float Kp_bank = Kp_bank_SURF; 
    float MAX_AILERON_CMD = MAX_AILERON_CMD_SURF; 

    float aileron_cmd = Kp_bank * bank_error; 
    
    if (aileron_cmd > MAX_AILERON_CMD) aileron_cmd = MAX_AILERON_CMD; 
    if (aileron_cmd < -MAX_AILERON_CMD) aileron_cmd = -MAX_AILERON_CMD; 
    
    const float alpha_roll = 0.4f; 
    float filtered_cmd = alpha_roll * aileron_cmd + (1.0f - alpha_roll) * prev_roll_command; 
    
    prev_roll_command = filtered_cmd; 
    last_heading_update = now; 
    
    return filtered_cmd;
}

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF);

    Serial.begin(115200);
    while (!Serial) { delay(10); }

    rxBuffer.reserve(256);
    
    resetAllState();
}

void prg1_test_takeoff_program(unsigned long now, float altitude, float velocity)
{
    static unsigned long state_start_time = 0;
    static bool state_initialized = false;
    static float target_periapsis_alt = 0.0f; 
    
    if (!state_initialized) {
        state_start_time = now;
        state_initialized = true;
        
        prev_pitch_cmd = 999.0f; prev_yaw_cmd = 999.0f; prev_roll_cmd = 999.0f;
        
        pitch_integral = 0.0f; prev_pitch_error = 0.0f; last_pitch_update = 0; prev_elevator_cmd = 0.0f;
        prev_heading_error = 0.0f; last_heading_update = 0; prev_roll_command = 0.0f;
        
        if (prg_state == PRG_PREPARE_HOLD || prg_state == PRG_ALTITUDE_HOLD) {
            sendCommand("SURF", "PITCH=0.000,YAW=0.000,ROLL=0.000"); 
            sendCommand("RCS", "PITCH=0.000,YAW=0.000,ROLL=0.000");
        }
    }
    
    float pitch_cmd = 0.0f;
    float yaw_cmd = 0.0f;
    float roll_cmd = 0.0f;
    float main_cmd = -1.0f; 
    
    
    switch (prg_state) {
        case PRG_IDLE:
            break;
        case PRG_WAIT_START:
            if (now - program_start_time >= 10000) {
                prg_state = PRG_ROTATE_CLIMB;
                state_initialized = false;
            }
            break;
        case PRG_ROTATE_CLIMB:
            target_pitch = 0.2618f; 
            main_cmd = 1.0f;
            
            if (altitude >= 100.0f && !gear_retracted) {
                sendCommand("GEAR", "UP");
                gear_retracted = true;
            }
            
            if (altitude > 500.0f) {
                prg_state = PRG_ACCELERATE;
                state_initialized = false;
            }
            break;
        case PRG_ACCELERATE:
            { 
                main_cmd = 1.0f;
                
                const float PITCH_15_RAD = 0.2618f;
                const float PITCH_10_RAD = 0.1745f;
                const float PITCH_TRANSITION_ALT = 40000.0f;
                
                if (altitude < PITCH_TRANSITION_ALT) {
                    target_pitch = PITCH_15_RAD; 
                } else {
                    target_pitch = PITCH_10_RAD; 
                }
                
                // DYNAMIC AILERON TRIM MANAGEMENT
                const float TRIM_SCALE_FACTOR = 0.5f; 
                const float MAX_TRIM_MAGNITUDE = 0.1f; 
                const float TRIM_CHANGE_THRESHOLD = 0.005f;

                float new_trim_value = pitch_integral * TRIM_SCALE_FACTOR; 
                if (new_trim_value > MAX_TRIM_MAGNITUDE) new_trim_value = MAX_TRIM_MAGNITUDE;
                if (new_trim_value < -MAX_TRIM_MAGNITUDE) new_trim_value = -MAX_TRIM_MAGNITUDE;

                if (fabs(new_trim_value - last_trim_aileron) > TRIM_CHANGE_THRESHOLD) {
                    char trim_cmd[64];
                    sprintf(trim_cmd, "ELEVATOR=%.3f,AILERON=%.3f,RUDDER=%.3f", 
                            last_trim_elevator, 
                            new_trim_value,     
                            last_trim_rudder);  
                    sendCommand("TRIM", trim_cmd);
                    
                    last_trim_aileron = new_trim_value; 
                }
                
                // MECO CONDITION (Ascent Burn): Target 360 km APO
                const float MECO_APOAPSIS_TARGET = 330000.0f; 
                const float APOAPSIS_SAFETY_MARGIN = 500.0f;

                if (last_apoapsis >= (MECO_APOAPSIS_TARGET - APOAPSIS_SAFETY_MARGIN)) {
                    sendCommand("MAIN", "0.000"); 

                    prg_state = PRG_PREPARE_HOLD;
                    state_initialized = false;
                    navmode_prograde_active = false; 
                    prev_main_cmd = 0.0f;
                    return;
                }
                break;
            } 
        case PRG_PREPARE_HOLD: // Coast and Reorient Phase (Wait for APO)
            { 
                main_cmd = 0.0f; 

                // Use NAVMODE:PROGRADE for vacuum orientation above 80 km
                if (last_altitude > 80000.0f && !navmode_prograde_active) {
                    sendCommand("NAVMODE", "PROGRADE");
                    navmode_prograde_active = true;
                }

                // Apoapsis Prediction Convergence Lock
                const float APOAPSIS_CONVERGENCE_THRESHOLD = 50.0f; 
                
                // UPDATED: Reduced altitude margin to 500.0m for closer APO burn initiation.
                const float ALTITUDE_BURN_MARGIN = 500.0f;         
                
                const float MIN_APOAPSIS_FOR_BURN = 100000.0f;

                // Condition 1: APO prediction must be stable.
                bool apo_converged = fabs(last_apoapsis - prev_apoapsis) <= APOAPSIS_CONVERGENCE_THRESHOLD;
                
                // Condition 2: Current altitude must be near the predicted (and now stable) Apoapsis value.
                bool position_near_apo = last_altitude >= (last_apoapsis - ALTITUDE_BURN_MARGIN);

                // Transition to burn phase only if APO is stable and we are near the node.
                if (apo_converged && position_near_apo && last_apoapsis >= MIN_APOAPSIS_FOR_BURN) {
                    
                    // NEW: Calculate dynamic Periapsis target (APO - X)
                    target_periapsis_alt = last_apoapsis - 60000.0f; 
                    
                    prg_state = PRG_ALTITUDE_HOLD;
                    state_initialized = false;
                    return;
                }
                
                return;
            } 
        case PRG_ALTITUDE_HOLD: // Circularization Burn Phase (Raise PER)
            { 
                // UPDATED: Use the dynamically calculated target_periapsis_alt
                float PERIAPSIS_TARGET = target_periapsis_alt;
                const float PERIAPSIS_SAFETY_MARGIN = 1000.0f;
                
                if (last_altitude > 80000.0f && !navmode_prograde_active) {
                    sendCommand("NAVMODE", "PROGRADE");
                    navmode_prograde_active = true;
                }
                
                // Termination condition: Closed-Loop PER Check
                if (last_periapsis >= (PERIAPSIS_TARGET - PERIAPSIS_SAFETY_MARGIN)) {
                    sendCommand("MAIN", "0.000");
                    prg_state = PRG_COMPLETE;
                    state_initialized = false;
                    navmode_prograde_active = false;
                    prev_main_cmd = 0.0f;
                    return;
                }

                // Execute burn
                main_cmd = 1.0f;
                
                break;
            }
        case PRG_COMPLETE:
            
            if (!state_initialized) {
                sendCommand("MAIN", "0.000");
                sendCommand("SURF", "PITCH=0.000,YAW=0.000,ROLL=0.000"); 
                sendCommand("RCS", "PITCH=0.000,YAW=0.000,ROLL=0.000"); 
                
                sendCommand("RCS", "KILLROT");
                
                state_initialized = true;
                state_start_time = now;
            }
            main_cmd = 0.0f;
            return;
    }
    
    // --- Attitude Control Execution Block ---
    if (prg_state != PRG_COMPLETE) {
        
        // Manual attitude control is only active in the atmosphere (below 100 km)
        if (last_altitude < 100000.0f) {
            
            pitch_cmd = controlPitchAngle(target_pitch, last_pitch, last_altitude, now);
            roll_cmd = controlHeadingAngle(target_heading_rad, last_heading, last_bank, last_altitude, now);
        
            // Main Throttle Command Execution
            if (main_cmd >= 0.0f && fabs(main_cmd - prev_main_cmd) > 0.001f) {
                char main_str[16];
                sprintf(main_str, "%.3f", main_cmd);
                sendCommand("MAIN", main_str);
                prev_main_cmd = main_cmd;
            }
            
            // SURF/RCS Command Execution (atmospheric logic)
            if (fabs(pitch_cmd - prev_pitch_cmd) > 0.001f ||
                fabs(yaw_cmd - prev_yaw_cmd) > 0.001f ||
                fabs(roll_cmd - prev_roll_cmd) > 0.001f)
            {
                char rot_cmd[96];
                sprintf(rot_cmd, "PITCH=%.3f,YAW=%.3f,ROLL=%.3f", pitch_cmd, yaw_cmd, roll_cmd);
                
                if (last_altitude < 100000.0f) {
                    sendCommand("SURF", rot_cmd);
                } else {
                    sendCommand("RCS", rot_cmd);
                }
                
                prev_pitch_cmd = pitch_cmd;
                prev_yaw_cmd = yaw_cmd;
                prev_roll_cmd = roll_cmd;
            }
        } 
        else if (main_cmd >= 0.0f) {
            // Main Throttle Command is the only command executed in vacuum if not PRG_COMPLETE
            if (fabs(main_cmd - prev_main_cmd) > 0.001f) {
                char main_str[16];
                sprintf(main_str, "%.3f", main_cmd);
                sendCommand("MAIN", main_str);
                prev_main_cmd = main_cmd;
            }
        }
    } 
}

void loop()
{
    unsigned long now = millis();
    if (last_valid_tlm_time > 0 && (now - last_valid_tlm_time) > TLM_TIMEOUT_MS) {
        resetAllState();
        last_valid_tlm_time = 0;
        digitalWrite(LED_PIN, LED_OFF);
        return;
    }

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            bool telemetry_ok = parseTelemetryLine(rxBuffer);
            if (telemetry_ok) {
                processTelemetry(rxBuffer);
            }

            rxBuffer.clear();
        }
        else if (c != '\r') {
            if (rxBuffer.length() < 1024) { 
                rxBuffer += c;
            }
        }
    }

    if (now - last_logic_ms >= LOGIC_INTERVAL_MS) {
        last_logic_ms = now;
        if (prg_state != PRG_IDLE && !error_state) {
            prg1_test_takeoff_program(now, last_altitude, last_velocity);
        }
    }

    if (!error_state && digitalRead(LED_PIN) == LED_ON &&
        millis() - last_led_ms >= LED_PULSE_MS) {
        digitalWrite(LED_PIN, LED_OFF);
    }
}