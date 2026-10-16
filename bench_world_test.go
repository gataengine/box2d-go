package box2d

import "testing"

func BenchmarkWorldOps(b *testing.B) {
	b.Run("CreateAndDestroyWorld", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			w, err := NewWorld(DefaultWorldDef())
			if err != nil {
				b.Fatal(err)
			}
			w.Destroy()
		}
	})
	b.Run("CreateBody", func(b *testing.B) {
		w := mustWorld(b)
		defer w.Destroy()
		def := DefaultBodyDef()
		def.Type = DynamicBody
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			_, err := w.CreateBody(def)
			if err != nil {
				b.Fatal(err)
			}
		}
	})
	b.Run("CreateCircleShape", func(b *testing.B) {
		w := mustWorld(b)
		defer w.Destroy()
		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		body := mustBody(b, w, bodyDef)
		shapeDef := DefaultShapeDef()
		circle := Circle{Center: Vec2Zero, Radius: 0.5}
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			_, err := body.CreateCircleShape(shapeDef, circle)
			if err != nil {
				b.Fatal(err)
			}
		}
	})
	b.Run("CreatePolygonShape", func(b *testing.B) {
		w := mustWorld(b)
		defer w.Destroy()
		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		body := mustBody(b, w, bodyDef)
		shapeDef := DefaultShapeDef()
		box := MakeBox(0.5, 0.5)
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			_, err := body.CreatePolygonShape(shapeDef, box)
			if err != nil {
				b.Fatal(err)
			}
		}
	})
	b.Run("Step/Empty", func(b *testing.B) {
		w := mustWorld(b)
		defer w.Destroy()
		b.ResetTimer()
		mustStep(b, w, b.N)
	})
	b.Run("Step/10Bodies", func(b *testing.B) {
		w := mustWorld(b)
		defer w.Destroy()
		setupGround(b, w)
		def := DefaultBodyDef()
		def.Type = DynamicBody
		shapeDef := DefaultShapeDef()
		box := MakeBox(0.5, 0.5)
		for i := 0; i < 10; i++ {
			def.Position = Vec2{X: float32(i) - 5, Y: 5}
			body := mustBody(b, w, def)
			body.CreatePolygonShape(shapeDef, box)
		}
		b.ResetTimer()
		mustStep(b, w, b.N)
	})
	b.Run("Step/100Bodies", func(b *testing.B) {
		w := mustWorld(b)
		defer w.Destroy()
		setupGround(b, w)
		def := DefaultBodyDef()
		def.Type = DynamicBody
		shapeDef := DefaultShapeDef()
		box := MakeBox(0.5, 0.5)
		for i := 0; i < 100; i++ {
			def.Position = Vec2{X: float32(i%10) - 5, Y: float32(i/10) + 2}
			body := mustBody(b, w, def)
			body.CreatePolygonShape(shapeDef, box)
		}
		b.ResetTimer()
		mustStep(b, w, b.N)
	})
}
