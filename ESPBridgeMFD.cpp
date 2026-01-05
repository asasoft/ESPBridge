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

#define _CRT_SECURE_NO_WARNINGS
#define STRICT
#define ORBITER_MODULE


#include <windows.h>
#include <cstdio>
#include <cstring>
#include <string>
#include <chrono>
#include "orbitersdk.h"
#include <OrbiterAPI.h>
#include <MFDAPI.h>


#define AXIS_PITCH 0
#define AXIS_YAW   1
#define AXIS_ROLL  2

// These constants are defined to resolve the symbol issue and will be cast
// to AIRCTRL_TYPE in the function call.
#define AIRCTRL_ELEVATORTRIM 3
#define AIRCTRL_AILERONTRIM  4
#define AIRCTRL_RUDDERTRIM   5


static int g_mfdmode = -1;
/* =========================
   GLOBAL CORE LOGIC - DEDICATED BRIDGE INTERFACE
   ========================= */

struct ESPBridgeCore {
	// DEDICATED: Always track the target vessel
	VESSEL* gl02_vessel = nullptr;
	char target_vessel_name[32] = "GL-02"; // Target vessel identifier

	enum COM_STATE { DISCONNECTED, CONNECTED };
	COM_STATE com_state = DISCONNECTED;
	HANDLE serial_handle = INVALID_HANDLE_VALUE;
	char com_port[16]{};

	bool running = false;
	bool killrot = false;
	// Gear command tracking
	bool gear_cmd_received = false;
	char gear_cmd_val[32] = "";
	uint32_t gear_cmd_seq = 0;
	// Killrot display timer
	std::chrono::steady_clock::time_point killrot_activation_time{};
	const std::chrono::milliseconds killrot_min_display_time = std::chrono::seconds(3);

	int last_seq = -1;
	// Buffer size 64 to prevent overflow on long TRIM commands
	char last_cmd[64]{};
	char last_val[64]{};

	uint32_t telemetry_seq = 0;
	int last_port_checked = 2;

	std::chrono::steady_clock::time_point last_read_time{};
	std::chrono::steady_clock::time_point last_alive_check{};
	std::chrono::steady_clock::time_point last_tlm_send{};

	bool needs_cleanup = false;
	bool aero_engaged = false;
	// New members for Trim and Orbital Elements
	double trim_elevator = 0.0;
	double trim_aileron = 0.0;
	double trim_rudder = 0.0;
	double apoapsis = 0.0;
	double periapsis = 0.0;
	// find_gl02_vessel() function removed. Vessel identity is only checked in MsgProc.

		// Get target vessel (only checks validity, does not search/check identity)
	VESSEL* get_gl02_vessel() {
		// Check if the vessel, once found, is still valid (stability guard)
		if (gl02_vessel && !is_vessel_valid(gl02_vessel)) {
			gl02_vessel = nullptr;
		}
		return gl02_vessel;
	}

	// Check if vessel is valid
	bool is_vessel_valid(VESSEL* v) {
		if (!v) return false;
		try {
			const char* name = v->GetName();
			return (name != nullptr);
		}
		catch (...) {
			return false;
		}
	}

	// Clean up controls on a vessel
	void cleanup_vessel_controls(VESSEL* v) {
		if (!v || !is_vessel_valid(v)) return;
		try {
			v->SetControlSurfaceLevel(AIRCTRL_ELEVATOR, 0.0);
			v->SetControlSurfaceLevel(AIRCTRL_RUDDER, 0.0);
			v->SetControlSurfaceLevel(AIRCTRL_AILERON, 0.0);
			v->SetAttitudeRotLevel(AXIS_PITCH, 0.0);
			v->SetAttitudeRotLevel(AXIS_YAW, 0.0);
			v->SetAttitudeRotLevel(AXIS_ROLL, 0.0);

			// Stop engines
			THRUSTER_HANDLE th1 = v->GetThrusterHandleByIndex(0);
			THRUSTER_HANDLE th2 = v->GetThrusterHandleByIndex(1);
			if (th1) v->SetThrusterLevel(th1, 0.0);
			if (th2) v->SetThrusterLevel(th2, 0.0);
		}
		catch (...) {
			// Ignore errors during cleanup
		}
	}
};
static ESPBridgeCore g_core;

/* =========================
   HELPER FUNCTIONS
   ========================= */

static bool SerialAlive() {
	if (g_core.serial_handle == INVALID_HANDLE_VALUE) return false;
	DWORD errors = 0;
	COMSTAT stat{};
	return ClearCommError(g_core.serial_handle, &errors, &stat);
}

static void FullCleanup() {
	// Clean up target vessel controls
	if (g_core.gl02_vessel) {
		g_core.cleanup_vessel_controls(g_core.gl02_vessel);
	}

	// Clear serial
	if (g_core.serial_handle != INVALID_HANDLE_VALUE) {
		PurgeComm(g_core.serial_handle, PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR);
		CloseHandle(g_core.serial_handle);
		g_core.serial_handle = INVALID_HANDLE_VALUE;
	}

	// Reset state
	g_core.running = false;
	g_core.killrot = false;
	g_core.com_state = ESPBridgeCore::DISCONNECTED;
	g_core.last_seq = -1;
	g_core.telemetry_seq = 0;
	g_core.last_cmd[0] = 0;
	g_core.last_val[0] = 0;
	g_core.com_port[0] = 0;
	g_core.needs_cleanup = false;
	g_core.aero_engaged = false;
	g_core.trim_elevator = 0.0;
	g_core.trim_aileron = 0.0;
	g_core.trim_rudder = 0.0;
	g_core.apoapsis = 0.0;
	g_core.periapsis = 0.0;

	// Don't clear gl02_vessel - we'll find it again if needed
}

static void StopAll() {
	FullCleanup();
}

/* =========================
   COMMAND PARSER - APPLIES TO TARGET VESSEL
   ========================= */

static void ProcessLine(const char* line) {
	if (strncmp(line, "SEQ=", 4) != 0) return;
	int seq;
	// Buffer size 64 for command parsing
	char cmd[64], val[64];
	// sscanf limit %63
	if (sscanf(line, "SEQ=%d;CMD=%63[^;];VAL=%63s", &seq, cmd, val) != 3) return;
	// Always get target vessel
	VESSEL* v = g_core.get_gl02_vessel();
	// Store command regardless
	g_core.last_seq = seq;
	strcpy_s(g_core.last_cmd, cmd);
	strcpy_s(g_core.last_val, val);
	// If no target vessel, just store the command
	if (!v || !g_core.is_vessel_valid(v)) {
		return;
	}

	// GEAR command

	if (!_stricmp(cmd, "GEAR")) {
		// Store for display
		g_core.gear_cmd_received = true;
		strcpy_s(g_core.gear_cmd_val, val);
		g_core.gear_cmd_seq = seq;

		// Use the verified, current API function for key emulation
		if (!_stricmp(val, "UP") || !_stricmp(val, "DOWN")) {
			// Emulate 'G' key press to toggle gear
			// It injects the key event into the buffer, triggering the vessel's gear logic.
			oapiSimulateBufferedKey(OAPI_KEY_G);
		}
		return;
	}

	// TRIM command: New implementation for executing trim changes
	if (!_stricmp(cmd, "TRIM") && strlen(val) > 0) {
		float elevator = 0, aileron = 0, rudder = 0;
		// Expected format: ELEVATOR=Y,AILERON=Z,RUDDER=W
		if (sscanf(val, "ELEVATOR=%f,AILERON=%f,RUDDER=%f", &elevator, &aileron, &rudder) == 3) {
			try {
				// Apply new absolute trim settings using the predefined constants.
// Correct API call for persistent trim
				v->SetControlSurfaceLevel((AIRCTRL_TYPE)AIRCTRL_ELEVATORTRIM, elevator);
				v->SetControlSurfaceLevel((AIRCTRL_TYPE)AIRCTRL_AILERONTRIM, aileron);
				v->SetControlSurfaceLevel((AIRCTRL_TYPE)AIRCTRL_RUDDERTRIM, rudder);
			}
			catch (...) {
				// Exception handling for invalid vessel state
				g_core.gl02_vessel = nullptr;
			}
		}
		return;
	}


	// MAIN engine command
	if (!_stricmp(cmd, "MAIN")) {
		float level = (float)atof(val);
		if (level < 0.0f) level = 0.0f;
		if (level > 1.0f) level = 1.0f;
		try {
			THRUSTER_HANDLE th1 = v->GetThrusterHandleByIndex(0);
			THRUSTER_HANDLE th2 = v->GetThrusterHandleByIndex(1);
			if (th1) v->SetThrusterLevel(th1, level);
			if (th2) v->SetThrusterLevel(th2, level);
		}
		catch (...) {
			// Vessel might have become invalid
			g_core.gl02_vessel = nullptr;
		}
		return;
	}

	// SURF command
	if (!_stricmp(cmd, "SURF") && strlen(val) > 0) {
		float pitch = 0, yaw = 0, roll = 0;
		if (sscanf(val, "PITCH=%f,YAW=%f,ROLL=%f", &pitch, &yaw, &roll) == 3) {
			try {
				// Apply control surfaces (SURF) and disable RCS
				v->SetControlSurfaceLevel(AIRCTRL_ELEVATOR, pitch);
				v->SetControlSurfaceLevel(AIRCTRL_RUDDER, yaw);
				v->SetControlSurfaceLevel(AIRCTRL_AILERON, roll);
				v->SetAttitudeRotLevel(AXIS_PITCH, 0.0);
				v->SetAttitudeRotLevel(AXIS_YAW, 0.0);
				v->SetAttitudeRotLevel(AXIS_ROLL, 0.0);
				g_core.aero_engaged = true;
			}
			catch (...) {
				g_core.gl02_vessel = nullptr;
			}
		}
		return;
	}

	// RCS command
	if (!_stricmp(cmd, "RCS") && strlen(val) > 0) {
		float pitch = 0, yaw = 0, roll = 0;
		if (sscanf(val, "PITCH=%f,YAW=%f,ROLL=%f", &pitch, &yaw, &roll) == 3) {
			try {
				// Apply RCS rotational thrusters and disable control surfaces.
// This logic strictly respects the RCS command from the microcontroller.
				v->SetControlSurfaceLevel(AIRCTRL_ELEVATOR, 0.0);
				v->SetControlSurfaceLevel(AIRCTRL_RUDDER, 0.0);
				v->SetControlSurfaceLevel(AIRCTRL_AILERON, 0.0);

				v->SetAttitudeRotLevel(AXIS_PITCH, pitch);
				v->SetAttitudeRotLevel(AXIS_YAW, yaw);
				v->SetAttitudeRotLevel(AXIS_ROLL, roll);
				g_core.aero_engaged = false;

			}
			catch (...) {
				g_core.gl02_vessel = nullptr;
			}
		}
		return;
	}

	// NAVMODE commands (Includes consolidated KILLROT logic)
	if (!_stricmp(cmd, "NAVMODE")) {
		try {
			double altitude = v->GetAltitude();
			if (altitude > 80000.0) {
				if (!_stricmp(val, "PROGRADE")) {
					v->DeactivateNavmode(NAVMODE_KILLROT);
					v->ActivateNavmode(NAVMODE_PROGRADE);
				}
				else if (!_stricmp(val, "RETROGRADE")) {
					v->DeactivateNavmode(NAVMODE_KILLROT);
					v->ActivateNavmode(NAVMODE_RETROGRADE);
				}
				else if (!_stricmp(val, "NORMAL+")) {
					v->DeactivateNavmode(NAVMODE_KILLROT);
					v->ActivateNavmode(NAVMODE_NORMAL);
				}
				else if (!_stricmp(val, "NORMAL-")) {
					v->DeactivateNavmode(NAVMODE_KILLROT);
					v->ActivateNavmode(NAVMODE_ANTINORMAL);
				}
				// ADDED: KILLROT logic consolidated here as a NAVMODE value
				else if (!_stricmp(val, "KILLROT")) {
					v->ActivateNavmode(NAVMODE_KILLROT);
					g_core.killrot = true;
					g_core.killrot_activation_time = std::chrono::steady_clock::now();
				}
			}
		}
		catch (...) {
			g_core.gl02_vessel = nullptr;
		}
		return;
	}



}

/* =========================
   TELEMETRY - ALWAYS FROM TARGET VESSEL
   ========================= */

static void SendTelemetry() {
	// Always get target vessel
	VESSEL* v = g_core.get_gl02_vessel();
	// Increased buffer size for new TRIM and APO/PER fields
	char out[384];
	if (!v || !g_core.is_vessel_valid(v)) {
		// Send "no vessel" telemetry
		sprintf_s(out,
			"TLM=%u;T=%.3f;"
			"VEL=0.0,0.0,0.0;"
			"VMAG=0.0;"
			"ALT=0.0;"
			"ANG=0.0,0.0,0.0;"
			"ATT=0.0,0.0,0.0;"
			// NEW: Trim, APO, PER fields sent as 0.0
			"TRIM=0.0,0.0,0.0;"
			"APO=0.0;PER=0.0;"

			"FUEL=0.0;"
			"MAIN=0.0\n",
			g_core.telemetry_seq++,
			oapiGetSimTime()
		);
	}
	else {
		// Send target vessel telemetry
		try {
			VECTOR3 vel, ang;
			v->GetGlobalVel(vel);
			v->GetAngularVel(ang);

			double pitch = v->GetPitch();
			double bank = v->GetBank();
			double heading = v->GetYaw();
			double velocity_mag = v->GetAirspeed();
			double altitude = v->GetAltitude();
			// 1. Trim state - Cast integer to AIRCTRL_TYPE
			double elevator_trim = v->GetControlSurfaceLevel((AIRCTRL_TYPE)AIRCTRL_ELEVATORTRIM);
			double aileron_trim = v->GetControlSurfaceLevel((AIRCTRL_TYPE)AIRCTRL_AILERONTRIM);
			double rudder_trim = v->GetControlSurfaceLevel((AIRCTRL_TYPE)AIRCTRL_RUDDERTRIM);
			// 2. Orbital elements (Apoapsis and Periapsis)
			double apoapsis_center_dist = 0.0;
			double periapsis_center_dist = 0.0;
			// Get the reference body handle
			OBJHANDLE hRef = v->GetGravityRef();
			// Get the body radius for altitude calculation.
// Radius is 0.0 if hRef is invalid.
			double body_radius = (hRef) ? oapiGetSize(hRef) : 0.0;
			// CRITICAL FIX: Capture the return handle (hAp/hPe) to check for a valid orbit.
			OBJHANDLE hAp = v->GetApDist(apoapsis_center_dist);
			OBJHANDLE hPe = v->GetPeDist(periapsis_center_dist);

			double apoapsis_altitude = 0.0;
			double periapsis_altitude = 0.0;
			// Handle Apoapsis
			if (hAp) {
				// Closed orbit, hAp is valid
				apoapsis_altitude = (apoapsis_center_dist > body_radius) ? (apoapsis_center_dist - body_radius) : 0.0;
			}
			else {
				// Check if hyperbolic/escape (e >= 1.0)
				ELEMENTS el;
				double mjd_ref;
				if (v->GetElements(el, mjd_ref)) {
					if (el.e >= 1.0) {
						// Hyperbolic trajectory: effectively infinite apoapsis.
// Report a massive value to force MECO on the flight computer.
						apoapsis_altitude = 1.0e9;
						// 1,000,000 km
					}
				}
				// If GetElements fails or e < 1 (but hAp null, which is rare/error), it stays 0.0
			}

			// Handle Periapsis
			if (hPe) {
				periapsis_altitude = (periapsis_center_dist > body_radius) ?
					(periapsis_center_dist - body_radius) : 0.0;
			}

			// Store values in global core for MFD display
			g_core.trim_elevator = elevator_trim;
			g_core.trim_aileron = aileron_trim;
			g_core.trim_rudder = rudder_trim;

			// Set global members to the new, surface-relative values
			g_core.apoapsis = apoapsis_altitude;
			g_core.periapsis = periapsis_altitude;
			// Get fuel
			double fuel_mass = 0.0;
			int prop_count = v->GetPropellantCount();
			if (prop_count > 0) {
				PROPELLANT_HANDLE ph = v->GetPropellantHandleByIndex(0);
				if (ph) fuel_mass = v->GetPropellantMass(ph);
			}

			// Get main engine level
			double main_level = 0.0;
			THRUSTER_HANDLE th1 = v->GetThrusterHandleByIndex(0);
			THRUSTER_HANDLE th2 = v->GetThrusterHandleByIndex(1);

			if (th1 && th2) {
				main_level = (v->GetThrusterLevel(th1) + v->GetThrusterLevel(th2)) / 2.0;
			}
			else if (th1) {
				main_level = v->GetThrusterLevel(th1);
			}

			// CONSTRUCT THE TELEMETRY STRING WITH TRIM, APO, and PER
			sprintf_s(out,
				"TLM=%u;T=%.3f;"
				"VEL=%.3f,%.3f,%.3f;"
				"VMAG=%.3f;"
				"ALT=%.3f;"
				"ANG=%.3f,%.3f,%.3f;"

				"ATT=%.4f,%.4f,%.4f;"
				// Factual Trim state
				"TRIM=%.4f,%.4f,%.4f;"
				// Factual Apoapsis and Periapsis (Surface Altitude)
				"APO=%.1f;PER=%.1f;"

				"FUEL=%.3f;"
				"MAIN=%.3f\n",
				g_core.telemetry_seq++,
				oapiGetSimTime(),
				vel.x, vel.y, vel.z,

				velocity_mag,
				altitude,
				ang.x, ang.y, ang.z,
				pitch, bank, heading,
				elevator_trim, aileron_trim, rudder_trim,
				g_core.apoapsis, g_core.periapsis, // Surface-relative values are used here
				fuel_mass,
				main_level

			);
		}
		catch (...) {
			// Target vessel became invalid
			g_core.gl02_vessel = nullptr;
			// Send error telemetry
			sprintf_s(out,
				"TLM=%u;T=%.3f;"
				"VEL=0.0,0.0,0.0;"
				"VMAG=0.0;"
				"ALT=0.0;"
				"ANG=0.0,0.0,0.0;"

				"ATT=0.0,0.0,0.0;"
				// NEW: Trim, APO, PER fields sent as 0.0
				"TRIM=0.0,0.0,0.0;"
				"APO=0.0;PER=0.0;"
				"FUEL=0.0;"
				"MAIN=0.0\n",
				g_core.telemetry_seq++,
				oapiGetSimTime()
			);
		}
	}

	// Always send telemetry
	if (g_core.serial_handle != INVALID_HANDLE_VALUE) {
		DWORD written;
		WriteFile(g_core.serial_handle, out, (DWORD)strlen(out), &written, nullptr);
	}
}

/* =========================
   COM PORT SCAN
   ========================= */

static void ScanNextCOMPort() {
	if (g_core.com_state == ESPBridgeCore::CONNECTED) return;
	std::string port = "\\\\.\\COM" + std::to_string(g_core.last_port_checked);
	g_core.last_port_checked = (g_core.last_port_checked < 10) ? g_core.last_port_checked + 1 : 2;
	HANDLE h = CreateFileA(port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
	if (h == INVALID_HANDLE_VALUE) return;

	DCB dcb{};
	dcb.DCBlength = sizeof(dcb);
	if (!GetCommState(h, &dcb)) { CloseHandle(h); return; }

	dcb.BaudRate = 115200;
	dcb.ByteSize = 8;
	dcb.StopBits = ONESTOPBIT;
	dcb.Parity = NOPARITY;
	if (!SetCommState(h, &dcb)) { CloseHandle(h); return; }

	COMMTIMEOUTS t{};
	t.ReadIntervalTimeout = MAXDWORD;
	SetCommTimeouts(h, &t);
	PurgeComm(h, PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR);

	g_core.serial_handle = h;
	strcpy_s(g_core.com_port, port.c_str());
	g_core.com_state = ESPBridgeCore::CONNECTED;
}

/* =========================
   PRESTEP DRIVER
   ========================= */

static void ESPBridgeTick() {
	auto now = std::chrono::steady_clock::now();
	// Handle killrot timer: This logic is required to automatically deactivate Killrot
	// after a minimum display time to ensure control returns to the user/flight computer.
	if (g_core.killrot) {
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
			now - g_core.killrot_activation_time);
		if (elapsed >= g_core.killrot_min_display_time) {
			g_core.killrot = false;
			VESSEL* v = g_core.get_gl02_vessel();
			if (v && g_core.is_vessel_valid(v)) {
				try {
					v->DeactivateNavmode(NAVMODE_KILLROT);
				}
				catch (...) {
					// Ignore errors
				}
			}
		}
	}

	// Handle serial connection
	if (g_core.com_state == ESPBridgeCore::DISCONNECTED &&

		std::chrono::duration_cast<std::chrono::milliseconds>(now - g_core.last_read_time).count() >= 500) {
		g_core.last_read_time = now;
		ScanNextCOMPort();
	}

	// Check serial alive
	if (g_core.com_state == ESPBridgeCore::CONNECTED &&
		std::chrono::duration_cast<std::chrono::milliseconds>(now - g_core.last_alive_check).count() >= 2500) {
		g_core.last_alive_check = now;
		if (!SerialAlive()) {
			StopAll();
			g_core.needs_cleanup = true;
			return;
		}
	}

	// Read serial commands (if running) - STARTS IMMEDIATELY ON g_core.running = true
	if (g_core.running && g_core.com_state == ESPBridgeCore::CONNECTED &&
		std::chrono::duration_cast<std::chrono::milliseconds>(now - g_core.last_read_time).count() >= 200) {
		g_core.last_read_time = now;
		// Read available data
		char buf[256];
		DWORD read;
		if (ReadFile(g_core.serial_handle, buf, sizeof(buf), &read, nullptr) && read > 0) {
			static std::string buffer;
			buffer.append(buf, read);
			size_t pos;
			while ((pos = buffer.find('\n')) != std::string::npos) {
				std::string line = buffer.substr(0, pos);
				buffer.erase(0, pos + 1);
				ProcessLine(line.c_str());
			}
		}
	}

	// Send telemetry (ALWAYS from GL-02) - STARTS IMMEDIATELY ON g_core.running = true
	if (g_core.running && g_core.com_state == ESPBridgeCore::CONNECTED &&
		std::chrono::duration_cast<std::chrono::milliseconds>(now - g_core.last_tlm_send).count() >= 200) {
		g_core.last_tlm_send = now;
		SendTelemetry();
	}
}

/* =========================
   MODULE CALLBACKS
   ========================= */

DLLCLBK void opcPreStep(double, double, double) {
	ESPBridgeTick();
}

/* =========================
   MFD - ORIGINAL DISPLAY
   ========================= */

class ESPBridgeMFD : public MFD2 {
public:
	// Revert constructor to base class signature, removing local dimension storage
	ESPBridgeMFD(DWORD w, DWORD h, VESSEL* v) : MFD2(w, h, v) {
		// Just store for display
		g_core.gl02_vessel = v;
		// This MFD was opened from the target vessel
	}

	bool Update(oapi::Sketchpad* skp) override {
		if (!skp) return true;
		// Always get target vessel for display
		VESSEL* display_vessel = g_core.get_gl02_vessel();
		int y = 20;
		skp->Text(10, y, "ESPBridge MFD", 13);
		y += 20;

		char buf[128];
		sprintf_s(buf, "Running: %s", g_core.running ? "YES" : "NO");
		skp->Text(10, y, buf, strlen(buf)); y += 20;
		sprintf_s(buf, "COM Port: %s", g_core.com_port[0] ? g_core.com_port : "None");
		skp->Text(10, y, buf, strlen(buf)); y += 20;
		sprintf_s(buf, "ESP32 Connected: %s",
			g_core.com_state == ESPBridgeCore::CONNECTED ? "YES" : "NO");
		skp->Text(10, y, buf, strlen(buf)); y += 20;
		sprintf_s(buf, "SEQ: %d", g_core.last_seq);
		skp->Text(10, y, buf, strlen(buf)); y += 20;
		sprintf_s(buf, "CMD: %s", g_core.last_cmd);
		skp->Text(10, y, buf, strlen(buf));
		y += 20;
		sprintf_s(buf, "VAL: %s", g_core.last_val);
		skp->Text(10, y, buf, strlen(buf)); y += 20;
		if (display_vessel) {
			try {
				double altitude = display_vessel->GetAltitude();
				double dyn_press = display_vessel->GetDynPressure();
				sprintf_s(buf, "Control Mode: %s",
					(g_core.aero_engaged) ? "AERO (SURF)" : "RCS");
				skp->Text(10, y, buf, strlen(buf)); y += 20;
				sprintf_s(buf, "Dyn Press: %.1f Pa", dyn_press);
				skp->Text(10, y, buf, strlen(buf)); y += 20;
				// Control surface levels
				double elevator = display_vessel->GetControlSurfaceLevel(AIRCTRL_ELEVATOR);
				double aileron = display_vessel->GetControlSurfaceLevel(AIRCTRL_AILERON);
				double rudder = display_vessel->GetControlSurfaceLevel(AIRCTRL_RUDDER);
				sprintf_s(buf, "ELEV: %.2f  AIL: %.2f  RUD: %.2f", elevator, aileron, rudder);
				skp->Text(10, y, buf, strlen(buf)); y += 20;
				sprintf_s(buf, "TRIM (E/A/R): %.2f / %.2f / %.2f",
					g_core.trim_elevator, g_core.trim_aileron, g_core.trim_rudder);
				skp->Text(10, y, buf, strlen(buf)); y += 20;
				// Engine levels
				THRUSTER_HANDLE th1 = display_vessel->GetThrusterHandleByIndex(0);
				THRUSTER_HANDLE th2 = display_vessel->GetThrusterHandleByIndex(1);
				double eng1 = th1 ? display_vessel->GetThrusterLevel(th1) : 0.0;
				double eng2 = th2 ? display_vessel->GetThrusterLevel(th2) : 0.0;
				sprintf_s(buf, "ENG1: %.3f  ENG2: %.3f", eng1, eng2);
				skp->Text(10, y, buf, strlen(buf));
				y += 20;
				// Speed and altitude
				double airspeed = display_vessel->GetAirspeed();
				// double groundspeed = display_vessel->GetGroundspeed();
// Removed

				// MODIFIED: Relabel Airspeed to Velocity
				sprintf_s(buf, "Velocity: %.1f m/s", airspeed);
				skp->Text(10, y, buf, strlen(buf));
				y += 20;
				// MODIFIED: Ground Speed commented out
								/*
								sprintf_s(buf, "Ground Speed: %.1f m/s", groundspeed);
								skp->Text(10, y, buf, strlen(buf)); y += 20;
				*/

				sprintf_s(buf, "Altitude: %.1f m", altitude);
				skp->Text(10, y, buf, strlen(buf)); y += 20;

				// Pitch/Bank/Heading
				double pitch = display_vessel->GetPitch();
				double bank = display_vessel->GetBank();
				double heading = display_vessel->GetYaw();
				sprintf_s(buf, "Pitch: %.1f°  Bank: %.1f°  HDG: %.1f°",
					pitch * DEG, bank * DEG, heading * DEG);
				skp->Text(10, y, buf, strlen(buf)); y += 20;

				// Factual Apoapsis and Periapsis display (Surface Altitude)
				// Check for positive altitude to determine if orbit is defined.
				if (g_core.apoapsis > 0.0 || g_core.periapsis > 0.0) {
					sprintf_s(buf, "APO: %.1f km  PER: %.1f km",
						g_core.apoapsis / 1000.0, g_core.periapsis / 1000.0);
				}
				else {
					sprintf_s(buf, "APO: --  PER: -- (Sub-Orbital)");
				}
				skp->Text(10, y, buf, strlen(buf)); y += 20;
			}
			catch (...) {
				sprintf_s(buf, "GL-02 data unavailable");
				skp->Text(10, y, buf, strlen(buf)); y += 20;
			}
		}
		else {
			sprintf_s(buf, "GL-02 not found");
			skp->Text(10, y, buf, strlen(buf)); y += 20;
		}

		sprintf_s(buf, "KILLROT: %s", g_core.killrot ? "ACTIVE" : "OFF");
		skp->Text(10, y, buf, strlen(buf));
		// Add gear status display
		y += 20;
		char gear_buf[128];
		sprintf_s(gear_buf, "GEAR CMD: %s (SEQ: %u)",
			g_core.gear_cmd_received ? g_core.gear_cmd_val : "NONE",
			g_core.gear_cmd_seq);
		skp->Text(10, y, gear_buf, strlen(gear_buf));

		// --- Start Flashing Text (Operational Status Indicator) ---
		// Advance 'y' to the next line (the last line).
		y += 20;

		// Condition: Only display if g_core.running is true (STA button pressed)
		if (g_core.running) {
			// Blinking state: 0.5 second ON, 0.5 second OFF
			int blink_state = ((int)(oapiGetSimTime() * 2.0)) % 2;
			if (blink_state == 0) {
				// Use white color as requested (255, 255, 255).
				DWORD active_text_color = oapiGetColour(255, 255, 255);
				// White

								// Set text drawing mode
				skp->SetTextColor(active_text_color);
				// Ensure text alignment is LEFT (default MFD setting)
				skp->SetTextAlign(oapi::Sketchpad::LEFT);
				const char* status_text = "EXTERNAL COMPUTER IS CONTROLLING";

				// Draw text at fixed left position (X=10), avoiding centering logic entirely.
				skp->Text(10, y, status_text, (int)strlen(status_text));

				// Restore default MFD drawing state
				DWORD default_pen_colour = oapiGetColour(128, 255, 255);
				skp->SetTextColor(default_pen_colour);
			}
		}
		// --- End Flashing Text ---


		return true;
	}

	bool ConsumeButton(int bt, int event) override {
		if (!(event & PANEL_MOUSE_LBDOWN)) return false;
		if (bt == 0) {
			// IMMEDIATE ACTIVATION
			g_core.running = true;
			g_core.needs_cleanup = false;
		}
		if (bt == 1) StopAll(); // FULL SYSTEM RESET
		return true;
	}

	char* ButtonLabel(int bt) override {
		static char buf[2][8];
		if (bt == 0) {
			strcpy_s(buf[0], "STA"); return buf[0];
		}
		if (bt == 1) {
			strcpy_s(buf[1], "STO");
			return buf[1];
		}
		return nullptr;
	}

	// CONSOLIDATED CHECK IMPLEMENTATION
	static OAPI_MSGTYPE MsgProc(UINT msg, UINT, WPARAM wparam, LPARAM lparam) {
		if (msg == OAPI_MSG_MFD_OPENEDEX) {
			VESSEL* opening_vessel = (VESSEL*)lparam;
			// CHECK 1: Only show MFD if it is opened on the dedicated target vessel
			if (opening_vessel) {
				const char* name = opening_vessel->GetName();
				if (name && strstr(name, "GL-02")) {
					auto* ospec = (MFDMODEOPENSPEC*)wparam;
					return (OAPI_MSGTYPE)new ESPBridgeMFD(ospec->w, ospec->h, opening_vessel);
				}
			}
			// If not GL-02, do not open the MFD mode.
			return 0;
		}
		return 0;
	}
};
/* =========================
   ENTRY
   ========================= */

DLLCLBK void InitModule(HINSTANCE) {
	static char name[] = "ESPBridge";
	MFDMODESPECEX spec{};
	spec.name = name;
	spec.key = OAPI_KEY_V;
	spec.context = nullptr;
	spec.msgproc = ESPBridgeMFD::MsgProc;

	g_mfdmode = oapiRegisterMFDMode(spec);
}

DLLCLBK void ExitModule(HINSTANCE) {
	FullCleanup();
	if (g_mfdmode >= 0) {
		oapiUnregisterMFDMode(g_mfdmode);
	}
}