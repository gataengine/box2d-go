# Character Mover APIs

The character workflow is:

1. sweep a mover shape
2. gather collision planes
3. solve for legal translation
4. clip velocity

## Cast mover

```go
t, _ := world.CastMoverWithFilter(box2d.DefaultQueryFilter())
_ = t
```

## Collide mover and solve planes

```go
_ = world.CollideMoverWithFilter(box2d.DefaultQueryFilter())

planes := []box2d.CollisionPlane{
	{Plane: box2d.Plane{Normal: box2d.Vec2{X: 0, Y: 1}, Offset: 0}, PushLimit: 1e9, ClipVelocity: true},
}
solve := box2d.SolvePlanes(box2d.Vec2{X: 0.2, Y: -0.5}, planes, len(planes))
clipped := box2d.ClipVector(box2d.Vec2{X: 1, Y: -3}, planes, len(planes))
_, _ = solve, clipped
```
