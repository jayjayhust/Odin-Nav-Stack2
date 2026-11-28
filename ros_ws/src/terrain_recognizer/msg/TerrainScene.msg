# One frame of the terrain scene: every instance that produced a verdict.
#
# A single recognizer can only describe one region at a time. Sites where several
# terrains coexist (stairs ahead, a ramp to the right, a landing between them)
# run several instances and are consumed from this topic, so "which terrain" and
# "where" stay separable. Consumers that only care about the ground directly
# ahead should keep using /stair_gait/status (see the legacy reducer) or the
# forward instance's own topic.
Header header

# One entry per instance with a trustworthy verdict this frame. Empty when no
# instance could judge its region.
TerrainObservation[] observations
