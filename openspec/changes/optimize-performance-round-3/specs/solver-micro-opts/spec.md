## ADDED Requirements

### Requirement: Motion lock flags packed into bitmask
Per-body motion lock state (fixed rotation, locked linear axes, locked angular axis) SHALL be stored as a single `uint8` bitmask on `bodyState`, replacing individual bool fields for lock checks in the solver loop.

#### Scenario: Single-instruction motion lock check
- **WHEN** the solver checks whether a body has any motion locks during constraint solving
- **THEN** the check uses a single bitwise AND operation on the bitmask

#### Scenario: Lock flag setting preserves semantics
- **WHEN** `SetFixedRotation`, `SetLinearLocked`, or `SetAngularLocked` are called on a body
- **THEN** the corresponding bitmask bit is set/cleared and behavior is identical to current bool-field implementation

### Requirement: Pre-computed scaled gravity per substep
The gravity delta SHALL be factored so that `MulSV(h, world.gravity)` is computed once per substep, with only the per-body `gravityScale` multiply remaining in the per-body loop.

#### Scenario: Gravity pre-computation
- **WHEN** a substep begins in `Step()`
- **THEN** `scaledGravity = MulSV(h, w.gravity)` is computed once before the body integration loop

#### Scenario: Per-body gravity application
- **WHEN** gravity is applied to each body during integration
- **THEN** the computation is `MulSV(body.gravityScale, scaledGravity)` (one multiply instead of two)
