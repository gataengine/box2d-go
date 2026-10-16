# Simulation

## World stepping

```go
const dt = 1.0 / 60.0
const subSteps = 4
_ = world.Step(dt, subSteps)
```

## Body setup

```go
def := box2d.DefaultBodyDef()
def.Type = box2d.DynamicBody
def.Position = box2d.Vec2{X: 1, Y: 2}
body, _ := world.CreateBody(def)
```

## Shape setup

```go
shapeDef := box2d.DefaultShapeDef()
shapeDef.Density = 1.0
shapeDef.Material.Friction = 0.6
_, _ = body.CreateCapsuleShape(shapeDef, box2d.Capsule{
	Center1: box2d.Vec2{X: -0.5, Y: 0},
	Center2: box2d.Vec2{X: 0.5, Y: 0},
	Radius:  0.2,
})
```

## Forces and impulses

```go
_ = body.ApplyForceToCenter(box2d.Vec2{X: 5, Y: 0}, true)
_ = body.ApplyLinearImpulseToCenter(box2d.Vec2{X: 0, Y: 1}, true)
```

## Events

```go
contacts, _ := world.GetContactEvents()
sensors, _ := world.GetSensorEvents()
bodyEvents, _ := world.GetBodyEvents()
_, _, _ = contacts, sensors, bodyEvents
```
