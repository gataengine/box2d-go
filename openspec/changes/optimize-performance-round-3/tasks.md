## 1. Sensor Sorted-Merge

- [x] 1.1 Add scratch slices for sensor state to World struct (`scratchPrevSensorKeys []sensorTouchKey`, `scratchCurrSensorKeys []sensorTouchKey`)
- [x] 1.2 Implement insertion sort for `[]sensorTouchKey` (reuse pattern from broadphase pair sorting)
- [x] 1.3 Replace map-based sensor diffing in `refreshContactAndSensorStateLocked()` with sorted-merge begin/end event generation
- [x] 1.4 Remove `scratchCurrentSensors` and `scratchPreviousSensors` map fields from World struct
- [x] 1.5 Verify sensor tests pass and benchmark sensor-heavy scenarios

## 2. Contact Manifold Dispatch Table

- [x] 2.1 Define `manifoldFunc` type and build `var manifoldDispatch [shapeTypeCount][shapeTypeCount]manifoldFunc` table at package init
- [x] 2.2 Populate dispatch table entries for all shape-pair combinations (circle, capsule, polygon, segment, chain-segment)
- [x] 2.3 Replace if-chain in `contactManifoldFromPair()` with dispatch table lookup
- [x] 2.4 Run collision tests and verify all manifold results are identical

## 3. Joint Solver Switch Dispatch

- [x] 3.1 Replace if-chain in `solveJointsLocked()` velocity solving with switch on `joint.typ`
- [x] 3.2 Replace if-chain in joint warm-start / initialization with switch on `joint.typ`
- [x] 3.3 Run joint tests and verify behavioral equivalence

## 4. Solver Micro-Optimizations

- [x] 4.1 Define motion lock bitmask constants (`lockFlagFixedRotation`, `lockFlagLinearX`, etc.) and add `lockFlags uint8` field to bodyState
- [x] 4.2 Update `SetFixedRotation`, `SetLinearLocked`, `SetAngularLocked` to set/clear bitmask bits
- [x] 4.3 Replace individual bool checks in solver loop with single bitmask AND
- [x] 4.4 Pre-compute `scaledGravity = MulSV(h, w.gravity)` once per substep; change per-body gravity to `MulSV(gravityScale, scaledGravity)`

## 5. Validation & Benchmarking

- [x] 5.1 Run full test suite (`go test ./...`) and fix any regressions
- [x] 5.2 Run benchmarks before and after, capture results to `bench/3-round3.txt`
- [x] 5.3 Update PERFORMANCE.md with Round 3 results
