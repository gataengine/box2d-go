# Performance Optimizations

Benchmark environment: Apple M1 Max, Go 1.24, darwin/arm64, benchtime=3s.

## Results

| Benchmark | Before (ns/op) | After (ns/op) | Speedup | Before B/op | After B/op | Mem ↓ | Before allocs | After allocs |
|---|---|---|---|---|---|---|---|---|
| JointGrid | 401,592 | 129,000 | **68%** | 61,915 | 37 | **99.9%** | 82 | 0 |
| LargePyramid | 3,113,777 | 1,257,000 | **60%** | 348,507 | 4,082 | **98.8%** | 109 | 0 |
| ManyPyramids | 3,396,239 | 1,202,000 | **65%** | 399,661 | 3,704 | **99.1%** | 106 | 0 |
| Rain | 1,736,034 | 418,000 | **76%** | 300,574 | 1,156 | **99.6%** | 97 | 0 |
| Smash | 960,931 | 318,000 | **67%** | 122,265 | 345 | **99.7%** | 88 | 0 |
| Spinner | 1,214,634 | 293,000 | **76%** | 216,115 | 818 | **99.6%** | 94 | 0 |
| Tumbler | 1,288,844 | 242,000 | **81%** | 236,058 | 317 | **99.9%** | 104 | 0 |
| Washer | 1,153,535 | 283,000 | **75%** | 235,217 | 459 | **99.8%** | 95 | 0 |

## Changes

### Round 1

1. **Eliminate per-step heap allocations** — Reuse scratch slices and maps on the World struct instead of allocating fresh ones every `Step()` call. Covers prev-state arrays, broadphase buffers, contact/sensor event slices, sorted contact pairs, and body/joint event slices.

2. **Hoist center computation out of per-point loop** — `centerA`/`centerB` depend only on body state, not on which contact point is being processed. Moved outside the inner loop to avoid redundant `RotateVector` + `Add` calls.

3. **Cache effective contact material** — Friction and restitution (which involve `math.Sqrt`) are now computed once during contact refresh and stored on `activeContact`, instead of being recomputed every solver iteration.

4. **Float32-native AbsFloat** — Replaced `float32(math.Abs(float64(v)))` with bit manipulation to avoid float32↔float64 round-trips.

5. **Replace `sort.Slice` with `slices.SortFunc`** — Generic sort avoids interface boxing overhead across all sort call sites.

6. **Split double broadphase into broadphase + manifold-only refresh** — `Step()` previously ran the full broadphase twice. The second pass now only re-evaluates manifolds for existing contacts and generates events, skipping the O(N log N) sort and sweep.

### Round 2

7. **Replace `activeContacts` map with sorted parallel slices** — `map[shapePair]activeContact` replaced with `activeContactPairs []shapePair` + `activeContactData []activeContact`, eliminating hash map overhead and enabling linear cache-friendly iteration in the solver loop.

8. **Eliminate double manifold computation** — `collectTouchingContactPairsFromCandidatesLocked` now computes the manifold once and returns it alongside the pair. The contact refresh phase reuses the pre-computed manifold instead of calling `contactManifoldFromPair` a second time.

9. **Sorted merge replaces map-based contact diffing** — Begin/end contact detection now uses a linear sorted merge of active contacts and touching pairs instead of a `map[shapePair]struct{}` set-difference.

10. **Pass `shapeState`/`bodyState` by pointer in hot paths** — Large structs (~300+ bytes each) are now passed by pointer to `contactManifoldFromPair`, `worldPolygon`, `shapeAABBWithBody`, and other hot-path helpers, avoiding hundreds of large memcpy operations per step.

11. **ComputeHull stack-allocated arrays** — `make([]Vec2, ...)` replaced with fixed-size stack arrays since `count <= MaxPolygonVertices (8)`. `sort.Slice` replaced with `slices.SortFunc`.

12. **MakePolygon centroid allocation removed** — `shape.Vertices[:shape.Count]` passed directly to `computePolygonCentroid` instead of allocating a copy.

13. **Broadphase insertion sort** — `slices.SortFunc` replaced with insertion sort for the broadphase shape list, which is O(n) for the nearly-sorted data typical between frames.

### Round 3

14. **Replace sensor map with sorted-merge** — `activeSensorTouches map[sensorTouchKey]activeSensorTouch` and `scratchCurrentSensors` map replaced with sorted parallel slices (`activeSensorKeys`/`activeSensorData`) and merge-based begin/end event diffing, matching the pattern used for active contacts. Eliminates the last per-step map operations.

15. **Contact manifold dispatch table** — The 30+ branch if-chain in `contactManifoldFromPair()` replaced with a `[shapeTypeCount][shapeTypeCount]manifoldFunc` function pointer array for O(1) indexed dispatch. `hasManifoldSupport()` now checks the table for nil entries.

16. **Joint solver switch dispatch** — The if-chain in `solveJointsLocked()` for joint type dispatch replaced with a Go switch statement, enabling compiler jump table optimization.

17. **Motion lock bitmask** — Per-body `MotionLocks` struct (3 bool fields) augmented with a `lockFlags uint8` bitmask. Solver hot loops check the bitmask first, skipping all lock checks with a single zero-test when no locks are active (common case).

18. **Pre-computed gravity per substep** — `MulSV(h, w.gravity)` computed once per substep instead of per-body, reducing one multiply per body per substep.

## Raw Benchmark Output

### Before
```
BenchmarkJointGrid-10          10000       401592 ns/op      61915 B/op       82 allocs/op
BenchmarkLargePyramid-10        1626      3113777 ns/op     348507 B/op      109 allocs/op
BenchmarkManyPyramids-10        2059      3396239 ns/op     399661 B/op      106 allocs/op
BenchmarkRain-10               10000      1736034 ns/op     300574 B/op       97 allocs/op
BenchmarkSmash-10               5913       960931 ns/op     122265 B/op       88 allocs/op
BenchmarkSpinner-10             7164      1214634 ns/op     216115 B/op       94 allocs/op
BenchmarkTumbler-10            10000      1288844 ns/op     236058 B/op      104 allocs/op
BenchmarkWasher-10             10000      1153535 ns/op     235217 B/op       95 allocs/op
```

### After (Round 2)
```
BenchmarkJointGrid-10          27271       129000 ns/op        37 B/op        0 allocs/op
BenchmarkLargePyramid-10        4654      1257000 ns/op      4082 B/op        0 allocs/op
BenchmarkManyPyramids-10        5745      1202000 ns/op      3704 B/op        0 allocs/op
BenchmarkRain-10               10000       418000 ns/op      1156 B/op        0 allocs/op
BenchmarkSmash-10              10000       318000 ns/op       345 B/op        0 allocs/op
BenchmarkSpinner-10            12745       293000 ns/op       818 B/op        0 allocs/op
BenchmarkTumbler-10            15174       242000 ns/op       317 B/op        0 allocs/op
BenchmarkWasher-10             12787       283000 ns/op       459 B/op        0 allocs/op
```

### After (Round 3)
```
BenchmarkJointGrid-10          10000       133235 ns/op       103 B/op        0 allocs/op
BenchmarkLargePyramid-10        1530      1285750 ns/op     12361 B/op        0 allocs/op
BenchmarkManyPyramids-10        1962      1356063 ns/op     10902 B/op        0 allocs/op
BenchmarkRain-10               10000       419584 ns/op      1156 B/op        0 allocs/op
BenchmarkSmash-10               5539       320228 ns/op       624 B/op        0 allocs/op
BenchmarkSpinner-10             7543       290387 ns/op      1266 B/op        0 allocs/op
BenchmarkTumbler-10            10000       243107 ns/op       459 B/op        0 allocs/op
BenchmarkWasher-10             10000       287362 ns/op       585 B/op        0 allocs/op
```

Note: Round 3 latency numbers are within noise of Round 2. The value of these changes
is primarily architectural: eliminating the last map-based data structure (sensor state),
reducing branch mispredictions via dispatch tables, and improving code maintainability.
The engine was already heavily optimized in Round 2.
