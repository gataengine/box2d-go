# box2d-go

Go port of Box2D with an object-oriented API surface and Ginkgo-based parity tests.

## Install

```bash
go get github.com/gataengine/box2d-go
```

## Quick Start

```go
package main

import (
	"fmt"
	"log"

	box2d "github.com/gataengine/box2d-go"
)

func main() {
	worldDef := box2d.DefaultWorldDef()
	world, err := box2d.NewWorld(worldDef)
	if err != nil {
		log.Fatal(err)
	}
	defer world.Destroy()

	groundDef := box2d.DefaultBodyDef()
	groundDef.Type = box2d.StaticBody
	ground := mustBody(world, groundDef)
	_, _ = ground.CreatePolygonShape(box2d.DefaultShapeDef(), box2d.MakeBox(50, 10))

	bodyDef := box2d.DefaultBodyDef()
	bodyDef.Type = box2d.DynamicBody
	bodyDef.Position = box2d.Vec2{X: 0, Y: 4}
	body := mustBody(world, bodyDef)

	shapeDef := box2d.DefaultShapeDef()
	shapeDef.Density = 1.0
	_, _ = body.CreatePolygonShape(shapeDef, box2d.MakeBox(1, 1))

	for i := 0; i < 60; i++ {
		_ = world.Step(1.0/60.0, 4)
		p, _ := body.GetPosition()
		r, _ := body.GetRotation()
		fmt.Printf("%02d: y=%.3f angle=%.3f\n", i, p.Y, box2d.RotGetAngle(r))
	}
}

func mustBody(world *box2d.World, def box2d.BodyDef) *box2d.Body {
	b, err := world.CreateBody(def)
	if err != nil {
		panic(err)
	}
	return b
}
```

## Documentation

- [Overview](docs/overview.md)
- [Hello](docs/hello.md)
- [Simulation](docs/simulation.md)
- [Collision](docs/collision.md)
- [Character](docs/character.md)
- [Foundation](docs/foundation.md)
- [Migration](docs/migration.md)
- [Samples](docs/samples.md)
- [FAQ](docs/FAQ.md)

## Parity Notes

See [PARITY_REPORT.md](PARITY_REPORT.md) for API-surface and behavior parity status against upstream Box2D.
