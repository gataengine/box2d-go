# Samples

This repo uses tests as executable samples.

Start with:

- `world_test.go` for world lifecycle, stepping, queries, events.
- `body_test.go` for transforms, forces, mass, velocity APIs.
- `shape_test.go` for shape creation and contact queries.
- `joint_test.go` for all joint families.

Run all sample-like tests:

```bash
go test ./...
```

Run benchmark scenarios:

```bash
go test -run '^$' -bench 'Benchmark(JointGrid|LargePyramid|ManyPyramids|Rain|Smash|Spinner|Tumbler|Washer)$' -benchtime=1x
```
