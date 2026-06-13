using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# DO rename the structs
# DON'T change the identifier (e.g. @0x81c2f05a394cf4af)

struct CustomReserved0 @0x81c2f05a394cf4af {
}

struct CustomReserved1 @0xaedffd8f31e7b55d {
}

struct CustomReserved2 @0xf35cc4560bbf6ec2 {
}

struct CustomReserved3 @0xda96579883444c35 {
}

struct CustomReserved4 @0x80ae746ee2596b11 {
}

struct CustomReserved5 @0xa5cd762cd951a455 {
}

struct CustomReserved6 @0xf98d843bfd7004a3 {
}

struct CustomReserved7 @0xb86e6369214c01c8 {
}

# mapd2xnor: OSM map data published by mapd_manager (from pfeiferj mapd binary).
# Renamed from CustomReserved8; identifier preserved per cereal fork convention.
struct LiveMapDataSP @0xf416ec09499d9d19 {
  speedLimitValid @0 :Bool;
  speedLimit @1 :Float32;          # m/s
  speedLimitAheadValid @2 :Bool;
  speedLimitAhead @3 :Float32;     # m/s
  speedLimitAheadDistance @4 :Float32;  # meters
  roadName @5 :Text;
}

# vtsc (ces2xnor): Vision Turn Speed Control decision telemetry, logged each plannerd cycle so
# past drives are analyzable (the old /dev/shm VTSCStatus JSON was RAM-only, never logged).
# Renamed from CustomReserved9; identifier preserved per cereal fork convention.
struct VtscState @0xa1680744031fdb2d {
  enabled @0 :Bool;          # CES master toggle on + openpilotLongitudinalControl
  active @1 :Bool;           # currently capping below cruise (slowing for a curve)
  state @2 :Text;            # state machine: "idle" | "brake" | "hold" | "release"
  vCruise @3 :Float32;       # m/s, the set-cruise target VTSC may cap
  vTarget @4 :Float32;       # m/s, the applied cap (== vCruise when not slowing)
  vEgo @5 :Float32;          # m/s, vehicle speed
  apexDist @6 :Float32;      # m to the sharpest upcoming curve point (-1 if none)
  apexCurvature @7 :Float32; # 1/m at that point (0 if straight)
  vCurveSafe @8 :Float32;    # m/s, sqrt(A_LAT_TARGET / curvature) target through the curve
  timeToApex @9 :Float32;    # s, apexDist / vEgo (-1 if none)
}

struct CustomReserved10 @0xcb9fd56c7057593a {
}

struct CustomReserved11 @0xc2243c65e0340384 {
}

struct CustomReserved12 @0x9ccdc8676701b412 {
}

struct CustomReserved13 @0xcd96dafb67a082d0 {
}

struct CustomReserved14 @0xb057204d7deadf3f {
}

struct CustomReserved15 @0xbd443b539493bc68 {
}

struct CustomReserved16 @0xfc6241ed8877b611 {
}

struct CustomReserved17 @0xa30662f84033036c {
}

struct CustomReserved18 @0xc86a3d38d13eb3ef {
}

struct CustomReserved19 @0xa4f1eb3323f5f582 {
}
