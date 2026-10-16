package box2d

import (
	"math"
	"testing"
)

func mustWorld(b *testing.B) *World {
	b.Helper()
	w, err := NewWorld(DefaultWorldDef())
	if err != nil {
		b.Fatalf("new world: %v", err)
	}
	return w
}

func mustBody(b *testing.B, w *World, def BodyDef) *Body {
	b.Helper()
	body, err := w.CreateBody(def)
	if err != nil {
		b.Fatalf("create body: %v", err)
	}
	return body
}

func mustStep(b *testing.B, w *World, n int) {
	b.Helper()
	for i := 0; i < n; i++ {
		if err := w.Step(1.0/60.0, 4); err != nil {
			b.Fatalf("step: %v", err)
		}
	}
}

func setupGround(b *testing.B, w *World) {
	ground := DefaultBodyDef()
	ground.Type = StaticBody
	groundBody := mustBody(b, w, ground)
	_, err := groundBody.CreatePolygonShape(DefaultShapeDef(), MakeBox(100, 2))
	if err != nil {
		b.Fatalf("ground shape: %v", err)
	}
}

func setupPyramid(b *testing.B, w *World, rows int) {
	setupGround(b, w)
	shapeDef := DefaultShapeDef()
	shape := MakeBox(0.5, 0.5)
	for i := 0; i < rows; i++ {
		for j := 0; j < rows-i; j++ {
			def := DefaultBodyDef()
			def.Type = DynamicBody
			def.Position = Vec2{X: float32(j) - float32(rows-i)/2, Y: float32(i) + 1}
			body := mustBody(b, w, def)
			if _, err := body.CreatePolygonShape(shapeDef, shape); err != nil {
				b.Fatalf("pyramid shape: %v", err)
			}
		}
	}
}

func setupRain(b *testing.B, w *World, count int) {
	setupGround(b, w)
	shapeDef := DefaultShapeDef()
	for i := 0; i < count; i++ {
		def := DefaultBodyDef()
		def.Type = DynamicBody
		def.Position = Vec2{X: float32((i%20)-10) * 0.8, Y: float32(i/20) + 8}
		body := mustBody(b, w, def)
		if _, err := body.CreateCircleShape(shapeDef, Circle{Center: Vec2Zero, Radius: 0.35}); err != nil {
			b.Fatalf("rain shape: %v", err)
		}
	}
}

func setupSpinner(b *testing.B, w *World, count int) {
	setupGround(b, w)
	shapeDef := DefaultShapeDef()
	for i := 0; i < count; i++ {
		def := DefaultBodyDef()
		def.Type = DynamicBody
		def.Position = Vec2{X: float32((i%10)-5) * 1.2, Y: float32(i/10) + 4}
		body := mustBody(b, w, def)
		if _, err := body.CreateCapsuleShape(shapeDef, Capsule{
			Center1: Vec2{X: -0.8, Y: 0},
			Center2: Vec2{X: 0.8, Y: 0},
			Radius:  0.2,
		}); err != nil {
			b.Fatalf("spinner shape: %v", err)
		}
		body.SetAngularVelocity(12)
	}
}

func setupTumbler(b *testing.B, w *World, count int) {
	setupGround(b, w)
	shapeDef := DefaultShapeDef()
	shapeDef.Density = 1
	containerDef := DefaultBodyDef()
	containerDef.Type = DynamicBody
	containerDef.Position = Vec2{X: 0, Y: 8}
	container := mustBody(b, w, containerDef)
	_, _ = container.CreatePolygonShape(shapeDef, MakeBox(4, 0.2))
	_, _ = container.CreatePolygonShape(shapeDef, MakeOffsetBox(0.2, 4, Vec2{X: 4, Y: 0}, RotIdentity))
	_, _ = container.CreatePolygonShape(shapeDef, MakeOffsetBox(0.2, 4, Vec2{X: -4, Y: 0}, RotIdentity))
	container.SetAngularVelocity(2.5)

	for i := 0; i < count; i++ {
		def := DefaultBodyDef()
		def.Type = DynamicBody
		def.Position = Vec2{X: float32((i%8)-4) * 0.6, Y: float32(i/8) + 7}
		body := mustBody(b, w, def)
		if _, err := body.CreateCircleShape(shapeDef, Circle{Center: Vec2Zero, Radius: 0.25}); err != nil {
			b.Fatalf("tumbler shape: %v", err)
		}
	}
}

func BenchmarkJointGrid(b *testing.B) {
	w := mustWorld(b)
	defer w.Destroy()
	setupGround(b, w)
	shapeDef := DefaultShapeDef()
	for y := 0; y < 8; y++ {
		for x := 0; x < 8; x++ {
			def := DefaultBodyDef()
			def.Type = DynamicBody
			def.Position = Vec2{X: float32(x) - 4, Y: float32(y) + 4}
			body := mustBody(b, w, def)
			_, _ = body.CreatePolygonShape(shapeDef, MakeBox(0.3, 0.3))
		}
	}
	b.ResetTimer()
	mustStep(b, w, b.N)
}

func BenchmarkLargePyramid(b *testing.B) {
	w := mustWorld(b)
	defer w.Destroy()
	setupPyramid(b, w, 22)
	b.ResetTimer()
	mustStep(b, w, b.N)
}

func BenchmarkManyPyramids(b *testing.B) {
	w := mustWorld(b)
	defer w.Destroy()
	setupGround(b, w)
	shapeDef := DefaultShapeDef()
	shape := MakeBox(0.35, 0.35)
	for pi := 0; pi < 6; pi++ {
		baseX := float32((pi%3)-1) * 10
		baseY := float32((pi/3))*7 + 1
		rows := 10
		for i := 0; i < rows; i++ {
			for j := 0; j < rows-i; j++ {
				def := DefaultBodyDef()
				def.Type = DynamicBody
				def.Position = Vec2{
					X: baseX + float32(j) - float32(rows-i)/2,
					Y: baseY + float32(i),
				}
				body := mustBody(b, w, def)
				_, _ = body.CreatePolygonShape(shapeDef, shape)
			}
		}
	}
	b.ResetTimer()
	mustStep(b, w, b.N)
}

func BenchmarkRain(b *testing.B) {
	w := mustWorld(b)
	defer w.Destroy()
	setupRain(b, w, 320)
	b.ResetTimer()
	mustStep(b, w, b.N)
}

func BenchmarkSmash(b *testing.B) {
	w := mustWorld(b)
	defer w.Destroy()
	setupGround(b, w)
	shapeDef := DefaultShapeDef()
	for i := 0; i < 120; i++ {
		def := DefaultBodyDef()
		def.Type = DynamicBody
		def.Position = Vec2{X: float32((i%15)-7) * 0.7, Y: float32(i/15) + 2}
		body := mustBody(b, w, def)
		_, _ = body.CreatePolygonShape(shapeDef, MakeBox(0.3, 0.3))
	}
	projectileDef := DefaultBodyDef()
	projectileDef.Type = DynamicBody
	projectileDef.Position = Vec2{X: -30, Y: 10}
	projectileDef.LinearVelocity = Vec2{X: 70, Y: 0}
	projectile := mustBody(b, w, projectileDef)
	_, _ = projectile.CreateCircleShape(shapeDef, Circle{Center: Vec2Zero, Radius: 0.8})
	b.ResetTimer()
	mustStep(b, w, b.N)
}

func BenchmarkSpinner(b *testing.B) {
	w := mustWorld(b)
	defer w.Destroy()
	setupSpinner(b, w, 120)
	b.ResetTimer()
	mustStep(b, w, b.N)
}

func BenchmarkTumbler(b *testing.B) {
	w := mustWorld(b)
	defer w.Destroy()
	setupTumbler(b, w, 180)
	b.ResetTimer()
	mustStep(b, w, b.N)
}

func BenchmarkWasher(b *testing.B) {
	w := mustWorld(b)
	defer w.Destroy()
	setupGround(b, w)
	shapeDef := DefaultShapeDef()
	rings := 10
	for ring := 1; ring <= rings; ring++ {
		r := float32(ring) * 0.9
		segments := 24
		for i := 0; i < segments; i++ {
			a := 2 * Pi * float32(i) / float32(segments)
			def := DefaultBodyDef()
			def.Type = DynamicBody
			def.Position = Vec2{X: r * float32(math.Cos(float64(a))), Y: r*float32(math.Sin(float64(a))) + 12}
			body := mustBody(b, w, def)
			_, _ = body.CreateCircleShape(shapeDef, Circle{Center: Vec2Zero, Radius: 0.2})
		}
	}
	b.ResetTimer()
	mustStep(b, w, b.N)
}
