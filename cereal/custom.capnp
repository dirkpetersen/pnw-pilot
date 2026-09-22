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

struct CustomReserved8 @0xf416ec09499d9d19 {
}

# vtsc (ces2xnor): Vision Turn Speed Control decision, logged for drive analysis.
# Reuses the CustomReserved9 wire ID (@0xa1680744031fdb2d) — same slot, renamed.
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

# mads2pnw / madsop2pnw: the openpilot-side view of the panda's parallel lateral authority
# (controls_allowed_lateral). Reuses the CustomReserved10 wire ID (@0xcb9fd56c7057593a) — same
# slot, renamed, exactly as VtscState/MapdOut did.
#
# This is the ONLY channel by which controlsd learns that lateral is still authorised while
# openpilot itself is disengaged. It is published by selfdrived every frame, next to
# selfdriveState, and is authoritative ONLY when `available` is true.
struct MadsState @0xcb9fd56c7057593a {
  # The ENABLE_MADS bit actually reached the panda this boot (car capability + PandaMadsSafety,
  # decoded from CarParams.alternativeExperience). False => this message carries no authority and
  # every consumer must fall back to selfdriveState. False on the Raven, and on the Lightning
  # until the panda is flashed and PandaMadsSafety is set by hand.
  available @0 :Bool;
  # Lateral authority is latched. Mirrors the panda's controls_allowed_lateral.
  enabled @1 :Bool;
  # openpilot should command lateral this frame. Equals selfdriveState.active whenever openpilot
  # itself is engaged; the two only differ in the lateralOnly state below.
  active @2 :Bool;
  # THE new state: steering is live while openpilot's own engagement is gone (the driver braked,
  # the PCM dropped cruise). Drives the "steering only" alert and the UI's engagement colour so
  # the car is never steering behind a UI that says "disengaged".
  lateralOnly @3 :Bool;
  # The MADS_DISENGAGE_LATERAL_ON_BRAKE policy bit as sent to the panda, i.e. the "Disengage on
  # brake" toggle as it was latched at car init. Reported so a log can be read without guessing.
  disengageOnBrake @4 :Bool;
}

# capnpfork2pnw: the fork's OWN onroad events, published by selfdrived as `onroadEventsPnw` in the
# same frame as (and just before) `onroadEvents`. Reuses the CustomReserved11 wire ID -- same slot,
# renamed, exactly as VtscState/MadsState did.
#
# WHY THEY ARE NOT IN log.capnp ANY MORE. They used to be enumerants @99-@104 of upstream's
# OnroadEvent.EventName, and upstream has since allocated every one of those ordinals itself
# (@99 lateralManeuver in 0.11.1; @100-@103 bigModel*/carNotReady in 0.11.2; @104
# userBookmarkNotPaired on master). A capnp ordinal is WIRE FORMAT, so that was a silent collision:
# on a newer upstream base our madsControlsMismatchLateral @102 decodes as bigModelFailed. Here they
# are in an enum only this fork allocates. docs/pnw/CAPNP-FORK-ORDINALS.md has the whole story and
# the migration for logs recorded with the old ordinals
# (selfdrive/test/process_replay/migration.py: migrate_pnwOnroadEvents).
struct OnroadEventsPnw @0xc2243c65e0340384 {
  events @0 :List(OnroadEventPnw);
}

struct OnroadEventPnw @0x84ddbb3c1051f1d9 {
  name @0 :EventName;

  # event types: the same fields, ordinals and meaning as log.capnp OnroadEvent
  enable @1 :Bool;
  noEntry @2 :Bool;
  warning @3 :Bool;
  userDisable @4 :Bool;
  softDisable @5 :Bool;
  immediateDisable @6 :Bool;
  preEnable @7 :Bool;
  permanent @8 :Bool;
  overrideLateral @10 :Bool;
  overrideLongitudinal @9 :Bool;

  # APPEND ONLY. An ordinal here is wire format exactly as it is upstream: never renumber, never
  # reuse -- retire a name by renaming it ...DEPRECATED. The order below is deliberately the order
  # the events had in log.capnp (@99..@104), so the relative sort order of the fork's events -- which
  # decides AlertManager ties -- is unchanged.
  enum EventName {
    greenLight @0;                   # was log.capnp EventName @99  (greenlight2pnw)
    leadDeparting @1;                # was @100 (greenlead2pnw)
    madsLateralOnly @2;              # was @101 (madsop2pnw)
    madsControlsMismatchLateral @3;  # was @102 (madsheartbeat2pnw) -- the MADS safety event
    cruiseOffRequested @4;           # was @103 (onebutton2pnw)
    madsResumeSetTooHigh @5;         # was @104 (engagegoal2pnw; nothing raises it since nosetcancel2pnw)
  }
}

# capnpfork2pnw: per-panda fields that are the fork's own and have no upstream-reserved slot in
# log.capnp's PandaState. Published by pandad as `pandaStatesPnw` right after `pandaStates`, one
# entry per panda in the SAME ORDER. Reuses the CustomReserved12 wire ID.
#
# Only fields that fit NO upstream-reserved PandaState slot live here. PandaState keeps
# controlsAllowedLateral (@38) and healthPacketMismatch (@39), both Bools in the two Bool slots
# upstream reserved for forks, because selfdrived's lateral mismatch detector must read them from
# the SAME message as controlsAllowed.
struct PandaStatesPnw @0x9ccdc8676701b412 {
  pandas @0 :List(PandaStatePnw);
}

struct PandaStatePnw @0x897372de8fe0decd {
  # madsheartbeat2pnw: the DisengageReason that last took lateral authority down
  # (opendbc/safety/pnw/mads_declarations.h). Diagnostic only; 0 = none. Was PandaState @39 :UInt8
  # until capnpfork2pnw -- a TYPE collision with upstream's reserved `controlsAllowedRESERVED2 :Bool`.
  # Logs recorded before capnpfork2pnw still hold it (data byte 74 of PandaState); read those with
  # the writer's own schema, it is NOT migrated (see docs/pnw/CAPNP-FORK-ORDINALS.md).
  madsDisengageReason @0 :UInt8;
}

struct CustomReserved13 @0xcd96dafb67a082d0 {
}

struct CustomReserved14 @0xb057204d7deadf3f {
}

struct CustomReserved15 @0xbd443b539493bc68 {
}

struct CustomReserved16 @0xfc6241ed8877b611 {
}

# mapd2pnw: official pfeiferj mapd v2.0.6 cereal types, copied VERBATIM (same @IDs) from
# github.com/pfeiferj/mapd cereal/custom/custom.capnp so the prebuilt binary and openpilot
# agree on wire layout. MapdExtendedOut/MapdIn/MapdOut reuse the CustomReserved17/18/19 wire
# IDs (@0xa30662…/@0xc86a3d…/@0xa4f1eb…); the helper structs and enums are new named types.
struct MapdDownloadLocationDetails @0xff889853e7b0987f {
  location @0 :Text;
  totalFiles @1 :UInt32;
  downloadedFiles @2 :UInt32;
}

struct MapdDownloadProgress @0xfaa35dcac85073a2 {
  active @0 :Bool;
  cancelled @1 :Bool;
  totalFiles @2 :UInt32;
  downloadedFiles @3 :UInt32;
  locations @4 :List(Text);
  locationDetails @5 :List(MapdDownloadLocationDetails);
}

struct MapdPathPoint @0xd6f78acca1bc3939 {
  latitude @0 :Float64;
  longitude @1 :Float64;
  curvature @2 :Float32;
  targetVelocity @3 :Float32;
}

struct MapdExtendedOut @0xa30662f84033036c {
  downloadProgress @0 :MapdDownloadProgress;
  settings @1 :Text;
  path @2 :List(MapdPathPoint);
}

enum MapdInputType {
  download @0;
  setTargetLateralAccel @1;
  setSpeedLimitOffset @2;
  setSpeedLimitControl @3;
  setMapCurveSpeedControl @4;
  setVisionCurveSpeedControl @5;
  setLogLevel @6;
  setVisionCurveTargetLatA @7;
  setVisionCurveMinTargetV @8;
  reloadSettings @9;
  saveSettings @10;
  setEnableSpeed @11;
  setVisionCurveUseEnableSpeed @12;
  setMapCurveUseEnableSpeed @13;
  setSpeedLimitUseEnableSpeed @14;
  setHoldLastSeenSpeedLimit @15;
  setTargetSpeedJerk @16;
  setTargetSpeedAccel @17;
  setTargetSpeedTimeOffset @18;
  setDefaultLaneWidth @19;
  setMapCurveTargetLatA @20;
  loadDefaultSettings @21;
  loadRecommendedSettings @22;
  setSlowDownForNextSpeedLimit @23;
  setSpeedUpForNextSpeedLimit @24;
  setHoldSpeedLimitWhileChangingSetSpeed @25;
  loadPersistentSettings @26;
  cancelDownload @27;
  setLogJson @28;
  setLogSource @29;
  setExternalSpeedLimitControl @30;
  setExternalSpeedLimit @31;
  setSpeedLimitPriority @32;
  setSpeedLimitChangeRequiresAccept @33;
  acceptSpeedLimit @34;
  setPressGasToAcceptSpeedLimit @35;
  setAdjustSetSpeedToAcceptSpeedLimit @36;
  setAcceptSpeedLimitTimeout @37;
  setPressGasToOverrideSpeedLimit @38;
}

enum WaySelectionType {
  current @0;
  predicted @1;
  possible @2;
  extended @3;
  fail @4;
}

enum SpeedLimitOffsetType {
  static @0;
  percent @1;
}

struct MapdIn @0xc86a3d38d13eb3ef {
  type @0 :MapdInputType;
  float @1 :Float32;
  str @2 :Text;
  bool @3 :Bool;
}

enum RoadContext {
  freeway @0;
  city @1;
  unknown @2;
}

# mapd2pnw v2.2.0: HighwayClass copied VERBATIM (same ordinals) from mapd's
# cereal/custom/custom.capnp (PR #89). mapd's own comment: must be kept in
# perfect sync (names and values) with the HighwayClass enum in
# cereal/offline/offline.capnp — state.go casts directly between the two
# generated enum types. unknown means the way's highway tag was not one of
# the listed values, or the loaded map tiles predate this field.
enum HighwayClass {
  unknown @0;
  motorway @1;
  motorwayLink @2;
  trunk @3;
  trunkLink @4;
  primary @5;
  primaryLink @6;
  secondary @7;
  secondaryLink @8;
  tertiary @9;
  tertiaryLink @10;
  unclassified @11;
  residential @12;
  livingStreet @13;
}

struct MapdOut @0xa4f1eb3323f5f582 {
  wayName @0 :Text;
  wayRef @1 :Text;
  roadName @2 :Text;
  speedLimit @3 :Float32;
  nextSpeedLimit @4 :Float32;
  nextSpeedLimitDistance @5 :Float32;
  hazard @6 :Text;
  nextHazard @7 :Text;
  nextHazardDistance @8 :Float32;
  advisorySpeed @9 :Float32;
  nextAdvisorySpeed @10 :Float32;
  nextAdvisorySpeedDistance @11 :Float32;
  oneWay @12 :Bool;
  lanes @13 :UInt8;
  tileLoaded @14 :Bool;
  speedLimitSuggestedSpeed @15 :Float32;
  suggestedSpeed @16 :Float32;
  estimatedRoadWidth @17 :Float32;
  roadContext @18 :RoadContext;
  distanceFromWayCenter @19 :Float32;
  visionCurveSpeed @20 :Float32;
  mapCurveSpeed @21 :Float32;
  waySelectionType @22 :WaySelectionType;
  speedLimitAccepted @23 :Bool;
  # mapd2pnw v2.2.0: append-only, ordinals/types copied VERBATIM from mapd's
  # v2.2.0 custom.capnp (PRs #89/#90/#93/#94). Do not renumber @0-@23 above.
  highwayClass @24 :HighwayClass;
  wayId @25 :Int64;
  conditionalSpeedLimit @26 :Text;
}
