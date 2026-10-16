## ADDED Requirements

### Requirement: Sensor state uses sorted-slice merge diffing
The sensor state tracking system SHALL use sorted parallel slices (`[]sensorTouchKey`) instead of maps for tracking current and previous sensor overlaps. Begin/end touch events SHALL be computed via a single-pass sorted merge of the previous and current slices.

#### Scenario: Zero allocations during sensor refresh
- **WHEN** `refreshContactAndSensorStateLocked()` executes with active sensor pairs
- **THEN** zero heap allocations occur for sensor state tracking (scratch slices are reused)

#### Scenario: Correct begin/end sensor events via merge
- **WHEN** sensor pairs change between steps (new overlaps begin, existing overlaps end)
- **THEN** `SensorBeginTouchEvent` and `SensorEndTouchEvent` are emitted identically to the current map-based implementation

### Requirement: Sensor pairs are sorted by composite key
Sensor touch keys SHALL be maintained in sorted order (by shape ID pair) to enable O(n) merge diffing.

#### Scenario: Sensor pairs sorted after broadphase
- **WHEN** broadphase produces new sensor candidate pairs
- **THEN** the resulting sensor touch key slice is sorted using insertion sort (exploiting near-sorted property between frames)

### Requirement: Scratch slices for sensor state
The World struct SHALL hold reusable scratch slices for sensor state, following the same `[:0]` reuse pattern as active contact scratch buffers.

#### Scenario: Scratch slices grow but never shrink
- **WHEN** the number of sensor pairs increases beyond current scratch capacity
- **THEN** scratch slices grow to accommodate, and retain capacity for subsequent steps
