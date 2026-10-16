# FAQ

## Is this API C-compatible?

The public surface maps upstream symbols, but intended usage is Go receivers (`world.Step`, `body.ApplyForce`, etc.).

## Where are examples?

Primary examples live in test files (`*_test.go`).

## How do I draw debug geometry?

Use `DefaultDebugDraw()` and world draw APIs when integrating with your renderer.

## Is parity complete?

See [../PARITY_REPORT.md](../PARITY_REPORT.md) for current parity status and validation commands.
