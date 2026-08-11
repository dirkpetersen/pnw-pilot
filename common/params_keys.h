#pragma once

#include <string>
#include <unordered_map>

#include "cereal/gen/cpp/log.capnp.h"

inline static std::unordered_map<std::string, ParamKeyAttributes> keys = {
    {"AccessToken", {CLEAR_ON_MANAGER_START | DONT_LOG, STRING}},
    {"AdbEnabled", {PERSISTENT, BOOL}},
    {"AlwaysOnDM", {PERSISTENT, BOOL}},
    {"ApiCache_Device", {PERSISTENT, STRING}},
    {"ApiCache_FirehoseStats", {PERSISTENT, JSON}},
    {"AssistNowToken", {PERSISTENT, STRING}},
    {"AthenadPid", {PERSISTENT, INT}},
    {"AthenadUploadQueue", {PERSISTENT, JSON}},
    {"AthenadRecentlyViewedRoutes", {PERSISTENT, STRING}},
    {"BootCount", {PERSISTENT, INT}},
    {"CalibrationParams", {PERSISTENT, BYTES}},
    {"CalibrationCar", {PERSISTENT, STRING}},  // calswap2pnw: carFingerprint the current calibration belongs to; a mismatch on a real fingerprint forces a full recalibration (device moved between cars).
    {"CameraDebugExpGain", {CLEAR_ON_MANAGER_START, STRING}},
    {"CameraDebugExpTime", {CLEAR_ON_MANAGER_START, STRING}},
    {"CarBatteryCapacity", {PERSISTENT, INT}},
    {"CarParams", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BYTES}},
    {"CarParamsCache", {CLEAR_ON_MANAGER_START, BYTES}},
    {"CarParamsPersistent", {PERSISTENT, BYTES}},
    {"CarParamsPrevRoute", {PERSISTENT, BYTES}},
    {"CompletedTrainingVersion", {PERSISTENT, STRING, "0"}},
    {"ControlsReady", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"CurrentBootlog", {PERSISTENT, STRING}},
    {"CurrentRoute", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, STRING}},
    {"DisableLogging", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"DisablePowerDown", {PERSISTENT, BOOL}},
    {"DisableUpdates", {PERSISTENT, BOOL}},
    {"DisengageOnAccelerator", {PERSISTENT, BOOL, "0"}},
    {"DongleId", {PERSISTENT, STRING}},
    {"DoReboot", {CLEAR_ON_MANAGER_START, BOOL}},
    {"DoShutdown", {CLEAR_ON_MANAGER_START, BOOL}},
    {"DoUninstall", {CLEAR_ON_MANAGER_START, BOOL}},
    {"DriverTooDistracted", {CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON, BOOL}},
    {"AlphaLongitudinalEnabled", {PERSISTENT | DEVELOPMENT_ONLY, BOOL}},
    {"ExperimentalMode", {PERSISTENT, BOOL}},
    {"ExperimentalModeConfirmed", {PERSISTENT, BOOL}},
    {"FirmwareQueryDone", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"ForcePowerDown", {PERSISTENT, BOOL}},
    {"GitBranch", {PERSISTENT, STRING}},
    {"GitCommit", {PERSISTENT, STRING}},
    {"GitCommitDate", {PERSISTENT, STRING}},
    {"GitDiff", {PERSISTENT, STRING}},
    {"GithubSshKeys", {PERSISTENT, STRING}},
    {"GithubUsername", {PERSISTENT, STRING}},
    {"GitRemote", {PERSISTENT, STRING}},
    {"GsmApn", {PERSISTENT, STRING}},
    {"GsmMetered", {PERSISTENT, BOOL, "1"}},
    {"GsmRoaming", {PERSISTENT, BOOL}},
    // network2xnor: perpetual tethering + priority-wifi arbitration (default OFF / blank)
    {"TetheringEnabled", {PERSISTENT, BOOL, "0"}},
    {"TetheringPriorityWifi", {PERSISTENT, STRING, ""}},
    {"TetheringHomeLocation", {PERSISTENT, STRING}},  // network2xnor: GPS [lat,lon] of the priority WiFi (auto-learned) -> geo-gate scanning (LEGACY single-home; migrated into TetheringPriorityNetworks)
    {"TetheringPriorityNetworks", {PERSISTENT, STRING}},  // network2xnor: JSON list of {label,ssid,lat,lon,portal} priority WiFi networks (multi-location + captive-portal)
    {"HardwareSerial", {PERSISTENT, STRING}},
    {"HasAcceptedTerms", {PERSISTENT, STRING, "0"}},
    {"InstallDate", {PERSISTENT, TIME}},
    {"IsDriverViewEnabled", {CLEAR_ON_MANAGER_START, BOOL}},
    {"IsEngaged", {PERSISTENT, BOOL}},
    {"IsLdwEnabled", {PERSISTENT, BOOL}},
    {"IsMetric", {PERSISTENT, BOOL}},
    {"IsOffroad", {CLEAR_ON_MANAGER_START, BOOL}},
    {"IsOnroad", {PERSISTENT, BOOL}},
    {"IsRhdDetected", {PERSISTENT, BOOL}},
    {"IsReleaseBranch", {CLEAR_ON_MANAGER_START, BOOL}},
    {"IsTakingSnapshot", {CLEAR_ON_MANAGER_START, BOOL}},
    {"IsTestedBranch", {CLEAR_ON_MANAGER_START, BOOL}},
    {"JoystickDebugMode", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    {"LanguageSetting", {PERSISTENT, STRING, "en"}},
    {"LastAthenaPingTime", {CLEAR_ON_MANAGER_START, INT}},
    {"LastGPSPosition", {PERSISTENT, STRING}},
    // mapd2pnw: settings store for the official pfeiferj mapd v2.0.6 binary (JSON; the binary
    // reads/writes this directly and reloads it on a mapdIn reloadSettings message).
    {"MapdSettings", {PERSISTENT, JSON}},
    // mapd2pnw: one-shot guard so the PNW map auto-download (WA/OR/ID) is requested only once.
    {"MapdPnwMapsRequested", {PERSISTENT, BOOL}},
    // mapd2xnor: keys used by the pfeiferj mapd binary + mapd_manager (OSM speed limits + map curve)
    {"MapSpeedLimit", {PERSISTENT, STRING}},
    {"NextMapSpeedLimit", {PERSISTENT, JSON}},
    {"RoadName", {PERSISTENT, STRING}},
    {"WayRef", {PERSISTENT, STRING}},        // location2pnw: mapd road ref (e.g. "I 5") bridged to mem params
    {"RoadContext", {PERSISTENT, STRING}},   // location2pnw: mapd road class 'freeway'|'city'|'unknown' (freeway-gate)
    {"MapOneWay", {PERSISTENT, STRING}},      // dmroad2pnw: mapd oneWay bridged to mem params ("1"/"0"); with MapLanes -> divided-multilane detect
    {"MapLanes", {PERSISTENT, STRING}},       // dmroad2pnw: mapd lane count bridged to mem params (int as string; 0 = unknown)
    // mapd220-2pnw PHASE 1: mapd v2.2.0 mapdOut fields (@24-@26) bridged to mem params; telemetry/
    // observation only, no consumer reads these yet (see docs/MAPD-V220-UPGRADE.md).
    {"MapHighwayClass", {PERSISTENT, STRING}},          // HighwayClass enum name, e.g. "motorway"
    {"MapWayId", {PERSISTENT, STRING}},                 // OSM way id (int64 as string)
    {"MapConditionalSpeedLimit", {PERSISTENT, STRING}}, // raw OSM maxspeed:conditional text; "" = none
    {"MapDownloadStatus", {CLEAR_ON_MANAGER_START, STRING}},  // mapd2pnw: live OSM DB download state ("OK"/"downloading X/Y"/"incomplete X/Y"/"none") for the debug overlay (updates ~1Hz -> not PERSISTENT)
    {"OsmDbUpdatesCheck", {PERSISTENT, BOOL}},
    {"OsmDownloadedDate", {PERSISTENT, STRING}},
    {"OsmLocationName", {PERSISTENT, STRING}},
    {"OsmStateName", {PERSISTENT, STRING, "WA,OR,ID"}},  // mapd2xnor: 2-letter codes (binary STATE_BOXES key), NOT full names
    {"OsmLocal", {PERSISTENT, BOOL}},
    {"OsmAutoRequested", {PERSISTENT, BOOL}},
    {"OSMDownloadLocations", {PERSISTENT, JSON}},
    {"OSMDownloadBounds", {PERSISTENT, STRING}},
    {"MapTargetVelocities", {PERSISTENT, JSON}},
    {"ShowRoadName", {PERSISTENT, BOOL, "1"}},
    {"ShowSpeedLimit", {PERSISTENT, BOOL, "0"}},  // mapd: speed-limit display consumer (default OFF; lives here so the foundation registers it)
    // mapd2pnw: "Get map for this location" on-demand download
    {"GetMapForLocation", {PERSISTENT, BOOL, "0"}},      // the toggle — ON downloads the region under current GPS
    {"MapForLocationRegion", {CLEAR_ON_MANAGER_START, STRING}},  // mapd_manager writes the region code under current GPS (for the UI to display/gate); "" = covered/unknown
    {"MapForLocationCovered", {CLEAR_ON_MANAGER_START, BOOL}},   // mapd_manager writes True when current GPS is already covered by a downloaded map (UI greys the toggle)
    {"Offroad_OSMUpdateRequired", {CLEAR_ON_MANAGER_START, JSON}},  // mapd2xnor: OSM map download needed alert
    {"NudgelessLaneChange", {PERSISTENT, BOOL, "0"}},  // auto2pnw: nudgeless lane change (Tesla + F-150 Lightning), default OFF
    {"NoDisengageOnBrake", {PERSISTENT, BOOL, "0"}},   // auto2pnw: stay engaged through brake (unsupported here; toggle greyed)
    // lanecenter2pnw: small bounded curvature trim toward lane-line center. Deliberately an OPT-OUT
    // (default "0" = NOT disabled = feature ON), the one exception to this fork's "new toggles
    // default OFF" rule — see selfdrive/controls/lib/lane_centering.py + toggles.py for why (the
    // correction is hard-clamped to a tiny curvature nudge, confidence-gated, releases smoothly,
    // and this toggle switches it off instantly). Tuning is a hot-reloaded JSON file, not a param.
    {"DisableLaneCentering", {PERSISTENT, BOOL, "0"}},
    {"FordAngleLateral", {PERSISTENT, BOOL, "0"}},     // angleenable: Ford angle-primary lateral (BluePilot bp-7.0 LateralAngleExt) driver opt-in, F-150 Lightning only (opendbc pnw_vehicle.angle_lat gates on four_signal_lat). Experimental — default OFF.
    {"FirehoseActive", {CLEAR_ON_MANAGER_START, BOOL, "0"}},  // connect2pnw: set by uploader while a pass-2 (video/rlog) transfer is in flight
    {"Pass1UploadActive", {CLEAR_ON_MANAGER_START, BOOL, "0"}},  // connect2pnw: set while pass-1 (qlog/qcam) uploads are making progress; sidebar shows GREEN (pass 1) vs BLUE (pass 2, FirehoseActive) per driver req 2026-07-09
    {"FirehoseSpeed", {CLEAR_ON_MANAGER_START, INT, "0"}},  // connect2pnw: Mbps of the in-flight pass-2 transfer; uploader publishes per completed HD file (~1/min); sidebar shows it next to CONNECT
    {"DeferHDVideoUpload", {PERSISTENT, BOOL, "0"}},  // connect2pnw: hold fcamera/ecamera/dcamera uploads (qlog/rlog/qcam still flow); default OFF = unchanged behavior
    // connectsel2pnw: connect backend selector (ported from BluePilot 7.0 BPConnectBackend; comma option
    // removed, PNW self-hosted is the default, Custom URL added). Per-backend dongle-ID caches make
    // switching reversible: each backend issues its own dongle ID at registration, so the last-seen ID
    // is stashed per backend and swapped back in when returning. See common/connect_backend.py.
    {"ConnectBackend", {PERSISTENT, INT, "0"}},  // connectsel2pnw: 0=PNW self-hosted 1=Konik 2=Custom 3=Offline; 0/unset = unchanged behavior
    {"ConnectCustomUrl", {PERSISTENT, STRING}},  // connectsel2pnw: driver-entered https:// base URL for the Custom backend
    {"ConnectActiveBackend", {PERSISTENT, STRING}},  // connectsel2pnw: backend token DongleId currently belongs to ("custom:<hash>" for Custom)
    {"DongleIdCachePnw", {PERSISTENT, STRING}},  // connectsel2pnw: last dongle ID seen on the PNW backend
    {"DongleIdCacheKonik", {PERSISTENT, STRING}},  // connectsel2pnw: last dongle ID issued by Konik
    {"DongleIdCacheCustom", {PERSISTENT, STRING}},  // connectsel2pnw: JSON {sha256(url)[:16]: dongle_id} for Custom URLs
    {"OnPriorityNetwork", {CLEAR_ON_MANAGER_START, BOOL, "0"}},  // uploadgate2pnw: network_arbiterd sets True while joined to a priority (home) SSID; pass-2 (rlog/HD) uploads run ONLY there (driver spec 2026-07-13)
    {"GearPark", {CLEAR_ON_MANAGER_START, BOOL, "0"}},  // uploadgate2pnw: card's change-only "gear is in Park" flag; lets background procs gate on parked WITHOUT a 100Hz carState msgq sub (2026-07-13 commIssue lesson)
    {"LastUploadError", {CLEAR_ON_MANAGER_START, STRING}},  // uploadretry2pnw: last hard upload failure (HTTP status/exc) for the CES overlay, change-only; removed on next success + on startup
    {"DmMode", {PERSISTENT, INT, "0"}},  // dmroad2pnw: 3-way driver-monitoring timeout selector. 0=Off (stock strict everywhere), 1=Highway (900s pose/1800s phone on freeway or divided-2-lane, stock elsewhere), 2=Relaxed (10800s/3600s everywhere). Default OFF. Does NOT touch the glare knobs.
    // lebowski2pnw: used by the master-snapshot modeld (usbgpu state); registration verbatim from commaai/master.
    // Without these, modeld crash-loops with UnknownKeyName at startup (caught in the port review).
    {"UsbGpuPresent", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    {"UsbGpuCompiled", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    // ces2xnor + light-ces-gentle: CES master + VTSC. CESMode is the 3-way source of truth; the bool is
    // kept for back-compat. (MapTargetVelocities already registered by mapd2pnw above.)
    {"ConditionalExperimentalSwitching", {PERSISTENT, BOOL, "0"}},  // ces2xnor: legacy master bool (back-compat; superseded by CESMode)
    {"CESMode", {PERSISTENT, INT, "0"}},  // light-ces-gentle: 3-way master 0=Off 1=Light(gentle) 2=Standard. Source of truth.
    {"AutoSpeedReduce", {PERSISTENT, INT, "0"}},  // speedadjust2pnw: 3-way auto cruise-speed reduction 0=Off 1=Police(ease to limit+5 ~30s ahead) 2=Police+Limits(also cap proportionally on a posted-limit drop). Reduce-only, op-long only, default OFF.
    {"HideCESDebug", {PERSISTENT, BOOL, "0"}},  // ces2pnw (driver req 2026-07-10): hide the onroad CES debug overlay; default OFF = overlay shows
    {"RainMode", {PERSISTENT, INT, "0"}},  // rain2pnw (driver req 2026-07-12): 3-way wet-weather curve-slowdown selector. 0=None, 1=Light (3 mph slower in curves by default), 2=Heavy (5 mph). Applies to BOTH cars, same reduction. Magnitudes tunable in /data/pnw/rain.json. Default None.
    {"CESCurves", {PERSISTENT, BOOL, "1"}},   // ces2xnor: per-condition enable
    {"CESStops", {PERSISTENT, BOOL, "1"}},    // ces2xnor
    {"CESLowSpeed", {PERSISTENT, BOOL, "1"}}, // ces2xnor
    {"CESLead", {PERSISTENT, BOOL, "1"}},     // ces2xnor
    {"CESTurns", {PERSISTENT, BOOL, "0"}},    // ces2core2pnw: CES2 TURN condition (blinker + no lane-change intent < 55 mph, CEM F2). Default OFF for the first drives (study 5.2)
    {"Ces2Core", {PERSISTENT, BOOL, "0"}},    // ces2core2pnw: CES2 decision core LIVE (1) vs shadow-only (0, DEFAULT: v1 decides byte-identically, CES2 logs ces2* fields to ces_events)
    {"CESButtonState", {CLEAR_ON_MANAGER_START, INT, "0"}},  // ces2xnor: 0=CES 1=Chill 2=Exp (per-drive)
    {"CESStatus", {CLEAR_ON_MANAGER_START, JSON}},  // ces2xnor: live telemetry (selfdrived -> UI overlay)
    {"IcbmTarget", {CLEAR_ON_MANAGER_START, JSON}}, // icbm2pnw: stock-ACC set-speed target (ces brain -> ford carcontroller executor), mem-param
    {"SpeedAdjustTarget", {CLEAR_ON_MANAGER_START, JSON}}, // speedadjust-exec2pnw: stock-ACC set-speed target for police/limit reduce-only caps (speedadjust brain -> ford carcontroller executor, arbitrated there against IcbmTarget), mem-param. Same {target,ceiling,ts,dir?} shape as IcbmTarget.
    {"FordLatStatus", {CLEAR_ON_MANAGER_START, JSON}}, // fordlatui2pnw: which lateral path is live (opendbc ford carcontroller -> UI mismatch warning), mem-param. Was previously UNREGISTERED (caught by the release-prep params check 2026-07-20) -- the UI read silently swallowed UnknownKeyName, so the mismatch warning was permanently dead.
    {"VTSCStatus", {CLEAR_ON_MANAGER_START, JSON}}, // vtsc: live status (plannerd -> UI overlay). Gated on the CES toggle.
    {"LaneCenterStatus", {CLEAR_ON_MANAGER_START, JSON}}, // lanecenter2pnw telemetry: per-tick status (controlsd -> CES event logger), ~5 Hz. mem-param, /dev/shm/params, same pattern as VTSCStatus.
    {"SteerLimitStatus", {CLEAR_ON_MANAGER_START, JSON}}, // steerlimit-log2pnw telemetry: PURE OBSERVATION per-tick steering-limit status (curvLim/safeLim/angDes/angAct/angErr/sat/latDem/latMax/curvMax) (controlsd -> CES event logger), ~5 Hz. mem-param, /dev/shm/params, same pattern as LaneCenterStatus. See docs/STEERING-LIMITS.md.
    {"VtscMapCurves", {PERSISTENT, BOOL, "1"}},  // ces-i90-2pnw: fold pfeiferj map curve speeds into VTSC for earlier/sharper-curve braking (MTSC). Default ON (the new pfeiferj mapd is reliable; lean into the longer map horizon so braking + the 1-mph cue start BEFORE the curve); decel-limited + V_MIN-floored so even a wrong map speed can never slam.
    // location2pnw: "Happening Ahead" display-only overlay (police/rest/EV). Never touches panda/safety/control.
    {"LocationServicesEnabled", {PERSISTENT, BOOL, "1"}},  // master toggle (UI), default ON; daemon idles + overlay hidden when off
    {"EvIncludeLevel2", {PERSISTENT, BOOL, "0"}},  // location2pnw: also surface slow Level 2 chargers (reads ev_other_chargers.geojson); default OFF
    {"LocationServices", {CLEAR_ON_MANAGER_START, JSON}},  // mem: JSON the daemon publishes for the lower-left overlay
    {"LastManagerExitReason", {CLEAR_ON_MANAGER_START, STRING}},
    {"LastOffroadStatusPacket", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, JSON}},
    {"LastAgnosPowerMonitorShutdown", {CLEAR_ON_MANAGER_START, STRING}},
    {"LastPowerDropDetected", {CLEAR_ON_MANAGER_START, STRING}},
    {"LastUpdateException", {CLEAR_ON_MANAGER_START, STRING}},
    {"LastUpdateRouteCount", {PERSISTENT, INT, "0"}},
    {"LastUpdateTime", {PERSISTENT, TIME}},
    {"LastUpdateUptimeOnroad", {PERSISTENT, FLOAT, "0.0"}},
    {"LiveDelay", {PERSISTENT, BYTES}},
    {"LiveParameters", {PERSISTENT, JSON}},
    {"LiveParametersV2", {PERSISTENT, BYTES}},
    {"LiveTorqueParameters", {PERSISTENT | DONT_LOG, BYTES}},
    {"LocationFilterInitialState", {PERSISTENT, BYTES}},
    {"LongitudinalManeuverMode", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    {"LongitudinalPersonality", {PERSISTENT, INT, std::to_string(static_cast<int>(cereal::LongitudinalPersonality::STANDARD))}},
    {"NetworkMetered", {PERSISTENT, BOOL}},
    {"ObdMultiplexingChanged", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"ObdMultiplexingEnabled", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"Offroad_CarUnrecognized", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, JSON}},
    {"Offroad_ConnectivityNeeded", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_ConnectivityNeededPrompt", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_ExcessiveActuation", {PERSISTENT, JSON}},
    {"Offroad_IsTakingSnapshot", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_NeosUpdate", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_NoFirmware", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, JSON}},
    {"Offroad_Recalibration", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, JSON}},
    {"Offroad_TemperatureTooHigh", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_UnregisteredHardware", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_UpdateFailed", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_DriverMonitoringUncertain", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, JSON}},
    {"OnroadCycleRequested", {CLEAR_ON_MANAGER_START, BOOL}},
    {"OpenpilotEnabledToggle", {PERSISTENT, BOOL, "1"}},
    {"PandaHeartbeatLost", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    {"PandaSomResetTriggered", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    {"PandaSignatures", {CLEAR_ON_MANAGER_START, BYTES}},
    {"PrimeType", {PERSISTENT, INT}},
    {"RecordAudio", {PERSISTENT, BOOL}},
    {"RecordAudioFeedback", {PERSISTENT, BOOL, "0"}},
    {"RecordFront", {PERSISTENT, BOOL}},
    {"RecordFrontLock", {PERSISTENT, BOOL}},  // for the internal fleet
    {"SecOCKey", {PERSISTENT | DONT_LOG, STRING}},
    {"ShowDebugInfo", {PERSISTENT, BOOL}},
    {"RouteCount", {PERSISTENT, INT, "0"}},
    {"SnoozeUpdate", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    {"SshEnabled", {PERSISTENT, BOOL}},
    {"UbloxAvailable", {PERSISTENT, BOOL}},
    {"UpdateAvailable", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"UpdateFailedCount", {CLEAR_ON_MANAGER_START, INT}},
    {"UpdaterAvailableBranches", {PERSISTENT, STRING}},
    {"UpdaterCurrentDescription", {CLEAR_ON_MANAGER_START, STRING}},
    {"UpdaterCurrentReleaseNotes", {CLEAR_ON_MANAGER_START, BYTES}},
    {"UpdaterFetchAvailable", {CLEAR_ON_MANAGER_START, BOOL}},
    {"UpdaterNewDescription", {CLEAR_ON_MANAGER_START, STRING}},
    {"UpdaterNewReleaseNotes", {CLEAR_ON_MANAGER_START, BYTES}},
    {"UpdaterState", {CLEAR_ON_MANAGER_START, STRING}},
    {"UpdaterTargetBranch", {CLEAR_ON_MANAGER_START, STRING}},
    {"UpdaterLastFetchTime", {PERSISTENT, TIME}},
    {"UptimeOffroad", {PERSISTENT, FLOAT, "0.0"}},
    {"UptimeOnroad", {PERSISTENT, FLOAT, "0.0"}},
    {"Version", {PERSISTENT, STRING}},
};
