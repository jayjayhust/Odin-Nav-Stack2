# Semantic terrain verdict from the terrain_recognizer slot.
#
# Strict superset of the legacy /stair_gait/status JSON: every consumer of the
# old payload can keep reading `legacy_state` / `steps` / `rise_m` / `span_m` /
# `confidence`, while new consumers (mission_manager, floor_manager, planning)
# key on the richer `type` and the recognized `geometry` polygon.
Header header

# flat | stair_up | stair_down | ramp_up | ramp_down | landing | unknown
string type

# 0..1, backend-defined (share of confident votes, mean class probability, ...).
float64 confidence

# Geometry of the verdict: steps (stairs only), rise_m (one tread height for a
# stair, the whole height change for a ramp), span_m (x extent it was measured
# over). Zero when the backend does not measure geometry.
int32 steps
float64 rise_m
float64 span_m

# False while the backend has no trustworthy verdict (keep the previous one).
bool valid

# Recognized region in header.frame_id (0x0 or 0-length polygon = no region).
geometry_msgs/PolygonStamped geometry

# Backend that produced this observation ("stair_gait", "ramp_slope", ...).
string backend

# Instance that produced it ("front", "left", ...; empty in single-instance
# mode). The instance also tells consumers *where* the verdict applies when
# several terrains coexist.
string instance

# flat | up | down | unknown - the /stair_gait/status vocabulary, kept so the
# NeuPAN escape terrain gate does not need to know about the richer classes.
string legacy_state
