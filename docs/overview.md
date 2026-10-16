# Overview (Go Port)

This package ports modern Box2D concepts into Go with receiver-oriented APIs.

## Core Objects

- `World` manages simulation state.
- `Body` holds rigid body state.
- `Shape` attaches geometry and material to bodies.
- `Joint` constrains relative body motion.

## Typical Flow

```go
world, _ := box2d.NewWorld(box2d.DefaultWorldDef())
defer world.Destroy()

bodyDef := box2d.DefaultBodyDef()
bodyDef.Type = box2d.DynamicBody
body, _ := world.CreateBody(bodyDef)

shapeDef := box2d.DefaultShapeDef()
shapeDef.Density = 1
_, _ = body.CreateCircleShape(shapeDef, box2d.Circle{Radius: 0.5})

_ = world.Step(1.0/60.0, 4)
```

## IDs and Handles

IDs (`WorldID`, `BodyID`, `ShapeID`, `JointID`, `ContactID`) are available for parity and serialization-style workflows.

## Query APIs

Use world methods for overlap/raycast/shape cast:

```go
hits, _ := world.CastRay(box2d.RayCastInput{
	Origin:      box2d.Vec2{X: -10, Y: 0},
	Translation: box2d.Vec2{X: 20, Y: 0},
	MaxFraction: 1,
})
_ = hits
```
