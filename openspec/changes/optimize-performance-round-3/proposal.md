## Why

After two rounds of optimization achieving 60-81% speedups and near-zero allocations in the solver hot path, diminishing returns remain in several areas: sensor state management still uses map-based diffing, contact manifold dispatch relies on long if-chains, joint solving uses repeated type checks, and body/shape data layouts are not cache-optimal. A third round targeting these areas can squeeze additional throughput from the engine, particularly for scenes with many bodies, sensors, or joints.

## What Changes

- Replace map-based sensor state tracking with sorted parallel slices and merge-based diffing (matching the pattern already used for active contacts)
- Convert contact manifold dispatch from if-chain to function pointer table for O(1) dispatch
- Convert joint solver dispatch from if-chains to switch statements for better branch prediction
- Pack per-body motion lock flags into a single bitmask for cheaper per-iteration checks
- Pre-compute scaled gravity per substep instead of per-body
- Explore structure-of-arrays (SoA) layout for hot body fields (position, velocity, rotation) to improve cache line utilization in the solver loop

## Capabilities

### New Capabilities
- `sensor-sorted-merge`: Replace map-based sensor state tracking with sorted-slice merge diffing for zero-alloc sensor updates
- `dispatch-tables`: Use function pointer dispatch tables for contact manifold and joint solver type resolution
- `solver-micro-opts`: Micro-optimizations in the solver loop: bitmask motion locks, pre-computed gravity, improved data layout

### Modified Capabilities

## Impact

- `world.go`: Primary target -- sensor refresh, contact manifold dispatch, joint solver, Step() loop
- `collision.go`: Manifold dispatch table construction
- `math.go`: Possible additions for bitmask utilities
- No API changes -- all optimizations are internal
- Benchmarks expected to show measurable improvement on JointGrid, LargePyramid, and sensor-heavy scenarios
