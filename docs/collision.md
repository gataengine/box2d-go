# Collision and Geometry

## Primitive creation

```go
circle := box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5}
capsule := box2d.Capsule{Center1: box2d.Vec2{X: -0.5}, Center2: box2d.Vec2{X: 0.5}, Radius: 0.2}
box := box2d.MakeBox(0.5, 1.0)
rounded := box2d.MakeRoundedBox(0.5, 1.0, 0.1)
_, _, _, _ = circle, capsule, box, rounded
```

## Hull + polygon

```go
points := []box2d.Vec2{{X: -1, Y: 0}, {X: 1, Y: 0}, {X: 0, Y: 1}}
hull := box2d.ComputeHull(points)
poly := box2d.MakePolygon(&hull, 0.05)
_ = poly
```

## Point tests

```go
inside := box2d.PointInPolygon(&poly, box2d.Vec2{X: 0, Y: 0.2})
_ = inside
```

## Ray cast / shape cast

```go
rayOut := box2d.RayCastPolygon(&poly, &box2d.RayCastInput{
	Origin:      box2d.Vec2{X: -5, Y: 0},
	Translation: box2d.Vec2{X: 10, Y: 0},
	MaxFraction: 1,
})

proxy := box2d.MakeProxy([]box2d.Vec2{{X: 0, Y: 0}}, 1, 0.2)
shapeOut := box2d.ShapeCastCircle(&circle, &box2d.ShapeCastInput{
	Proxy:       proxy,
	Translation: box2d.Vec2{X: 5, Y: 0},
	MaxFraction: 1,
})

_, _ = rayOut, shapeOut
```
