## Context

The go-box2d physics engine has undergone two optimization rounds, achieving 60-81% speedups and near-zero allocations in the solver hot path. The remaining performance bottlenecks are:

1. **Sensor state tracking** uses `map[sensorTouchKey]` with full clear-and-rebuild each step, causing allocations and poor cache behavior
2. **Contact manifold dispatch** uses a 30+ branch if-chain in `contactManifoldFromPair()`
3. **Joint solver dispatch** uses repeated if-type-checks per iteration per joint
4. **Per-body motion lock checks** require 3 separate field reads per body per iteration
5. **Gravity computation** is redundantly calculated per-body when it could be pre-computed per-substep

The engine runs on Apple M1 Max (ARM64) with Go 1.24. All changes are internal -- no public API modifications.

## Goals / Non-Goals

**Goals:**
- Eliminate remaining map-based allocations in sensor state tracking
- Reduce branch mispredictions in manifold and joint dispatch
- Reduce per-body overhead in the solver inner loop
- Maintain zero allocations per Step() call
- Achieve measurable improvement (target: 10-20% on joint/sensor-heavy benchmarks)

**Non-Goals:**
- SIMD or assembly optimizations (Go compiler handles vectorization)
- Parallel/concurrent solving (would require major architectural changes)
- Structure-of-arrays (SoA) body layout (too invasive for this round; revisit if profiling shows cache misses as dominant cost)
- Changing the public API surface

## Decisions

### 1. Sorted-merge for sensor state (over keeping map)

Replace `scratchCurrentSensors map[sensorTouchKey]...` and `scratchPreviousSensors` with sorted `[]sensorTouchKey` slices and merge-based begin/end diffing.

**Rationale**: This pattern already proved effective for active contacts (Round 2), eliminating map allocations entirely. Sensor pairs follow the same nearly-sorted-between-frames property.

**Alternative**: sync.Pool for map -- still allocates internally; sorted merge is allocation-free.

### 2. Function pointer dispatch table for manifold computation

Build a `[ShapeTypeCount][ShapeTypeCount]func(...)` array at init time, replacing the if-chain in `contactManifoldFromPair()`.

**Rationale**: O(1) indexed lookup vs O(n) branch chain. Shape type is already an integer enum. The function pointer table is small (6x6 = 36 entries) and cache-friendly.

**Alternative**: Switch statement -- better than if-chain but still branch-predicted; function table is branchless.

### 3. Switch statement for joint solver dispatch

Replace if-chains in `solveJointsLocked()` and joint initialization with switch statements on `joint.typ`.

**Rationale**: Go compiler generates jump tables for dense switch cases, which is better than sequential if-checks. Function pointer table is overkill here since joint types are fewer and the per-joint work is heavier (amortizes dispatch cost).

### 4. Bitmask motion lock flags

Pack `fixedRotation`, `linearLocked`, `angularLocked` into a single `uint8` bitmask on bodyState. Check all three with a single bitwise AND.

**Rationale**: Reduces 3 bool reads to 1 byte read + mask operation per body per solver iteration. Improves data density in the bodyState struct.

### 5. Pre-computed gravity delta per substep

Compute `gravityDelta = MulSV(h * body.gravityScale, w.gravity)` factoring: pre-compute `scaledGravity = MulSV(h, w.gravity)` once per substep, then per-body: `MulSV(body.gravityScale, scaledGravity)`.

**Rationale**: Saves one multiply per body per substep. Minor but free.

## Risks / Trade-offs

- **[Function pointer table indirection]** -> Indirect function calls can be slower than direct calls if the Go compiler can't inline them. Mitigation: benchmark both approaches; fall back to switch if table is slower.
- **[Sensor sorted-merge complexity]** -> More code than map approach. Mitigation: follow the exact pattern from active contact merge (proven correct).
- **[Bitmask readability]** -> Bit operations are less readable than bool fields. Mitigation: use named constants and helper methods.
- **[Diminishing returns]** -> Some optimizations may not show measurable improvement. Mitigation: benchmark each change independently; only keep changes that show improvement.
