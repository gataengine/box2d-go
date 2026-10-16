# Migration Notes (C/C++ -> Go Port)

## Handle model

- C API uses `b2*Id` handles and free functions.
- Go port keeps IDs, but primary usage is receiver methods.

## Example: world

```go
world, err := box2d.NewWorld(box2d.DefaultWorldDef())
if err != nil {
	panic(err)
}
defer world.Destroy()
```

## Example: body + shape

```go
bodyDef := box2d.DefaultBodyDef()
bodyDef.Type = box2d.DynamicBody
body, _ := world.CreateBody(bodyDef)

shapeDef := box2d.DefaultShapeDef()
shapeDef.Density = 1
_, _ = body.CreatePolygonShape(shapeDef, box2d.MakeBox(1, 1))
```

## Example: step and events

```go
_ = world.Step(1.0/60.0, 4)
contacts, _ := world.GetContactEvents()
_ = contacts
```
