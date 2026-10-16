# Parity Report

Source of truth: upstream [erincatto/box2d](https://github.com/erincatto/box2d) (commit `c05c48738fbe5c27625e36c5f0cfbdaddfc8359a`).

## API Surface Audit

Run:

```bash
./tools/parity_audit.sh <path-to-box2d-checkout>
```

Current result:

- `upstream_symbols=430`
- `missing_symbol_names=0`

Note: this audit verifies symbol-name parity under the port naming convention (`b2X` -> `X`, `b2Type_Method` -> `Method`). It does not by itself prove full behavioral equivalence.

## Behavior Coverage

- Ginkgo specs include:
  - Upstream-inspired world/body/shape/joint/id/math/table/bitset/array parity checks
  - Compatibility advanced checks for:
    - AABB validation/raycast behavior
    - Segment distance / shape distance / shape cast / TOI baseline scenarios
    - Dynamic tree query / ray cast / shape cast baseline scenarios
    - Dynamic tree proxy lifecycle, id reuse, byte-count retention, and partial rebuild checks
  - Latest geometry parity checks for:
    - `IsValidRay` upper-bound handling
    - exact capsule containment on the boundary
    - capsule ray-cast baseline hit behavior

## Dynamic Tree Notes

- The Go tree now uses a mutable node pool with free-list reuse rather than rebuilding snapshot state on every query.
- Proxy IDs are recycled after destroy, matching upstream-style lifecycle behavior.
- `GetByteCount()` now reflects retained tree storage and rebuild buffers instead of just live proxy count.
- Query, ray cast, and shape cast walk the live tree structure after move/enlarge/rebuild operations.
- `Validate()` now checks live-tree structure, heights, bounds containment, category-bit aggregation, and free-list/node-count consistency.
- `ValidateNoEnlarged()` now enforces the no-enlarged-node contract used by upstream broad-phase validation.
- `Rebuild(false)` now preserves unchanged internal subtrees and only flattens enlarged branches, while `Rebuild(true)` still performs a full leaf rebuild.
- Rebuild partitioning now uses an upstream-style longest-axis midpoint partition instead of a recursive full sort.

## Latest Sweep

- Confirmed `go test ./...` passes.
- Confirmed `./tools/parity_audit.sh <path-to-box2d-checkout>` still reports `missing_symbol_names=0`.
- Closed the remaining geometry deltas found during the fresh audit:
  - `IsValidRay` now matches upstream's `B2_HUGE` upper bound check.
  - capsule point containment is exact on the boundary instead of using a tolerance padding.

## Tests and Benchmarks

- Unit/spec tests: `go test ./...`
- Bench scenarios (upstream names):
  - `BenchmarkJointGrid`
  - `BenchmarkLargePyramid`
  - `BenchmarkManyPyramids`
  - `BenchmarkRain`
  - `BenchmarkSmash`
  - `BenchmarkSpinner`
  - `BenchmarkTumbler`
  - `BenchmarkWasher`

Run:

```bash
go test ./...
go test -run '^$' -bench 'Benchmark(JointGrid|LargePyramid|ManyPyramids|Rain|Smash|Spinner|Tumbler|Washer)$' -benchtime=1x
```
