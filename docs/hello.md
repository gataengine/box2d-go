# Hello Box2D in Go

## 1. Create a world

```go
worldDef := box2d.DefaultWorldDef()
world, err := box2d.NewWorld(worldDef)
if err != nil {
	panic(err)
}
defer world.Destroy()
```

## 2. Create ground

```go
groundDef := box2d.DefaultBodyDef()
groundDef.Type = box2d.StaticBody
ground, _ := world.CreateBody(groundDef)
_, _ = ground.CreatePolygonShape(box2d.DefaultShapeDef(), box2d.MakeBox(50, 10))
```

## 3. Create dynamic box

```go
bodyDef := box2d.DefaultBodyDef()
bodyDef.Type = box2d.DynamicBody
bodyDef.Position = box2d.Vec2{X: 0, Y: 4}
body, _ := world.CreateBody(bodyDef)

shapeDef := box2d.DefaultShapeDef()
shapeDef.Density = 1
_, _ = body.CreatePolygonShape(shapeDef, box2d.MakeBox(1, 1))
```

## 4. Simulate

```go
for i := 0; i < 60; i++ {
	_ = world.Step(1.0/60.0, 4)
	p, _ := body.GetPosition()
	r, _ := body.GetRotation()
	fmt.Println(i, p.X, p.Y, box2d.RotGetAngle(r))
}
```
