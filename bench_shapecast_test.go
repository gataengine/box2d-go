package box2d

import "testing"

func BenchmarkShapeCast(b *testing.B) {
	circle := Circle{Center: Vec2{X: -5, Y: 0}, Radius: 0.5}
	capsule := Capsule{Center1: Vec2{X: -5, Y: -0.5}, Center2: Vec2{X: -5, Y: 0.5}, Radius: 0.3}
	segment := Segment{Point1: Vec2{X: -5, Y: -0.5}, Point2: Vec2{X: -5, Y: 0.5}}

	targetBox := MakeBox(1.0, 1.0)
	targetPoints := make([]Vec2, targetBox.Count)
	copy(targetPoints, targetBox.Vertices[:targetBox.Count])
	targetProxy := MakeProxy(targetPoints, targetBox.Count, targetBox.Radius)

	input := ShapeCastInput{
		Proxy:       targetProxy,
		Translation: Vec2{X: -10, Y: 0},
		MaxFraction: 1.0,
	}

	proxyA := MakeProxy(targetPoints, targetBox.Count, 0)
	circleProxy := MakeProxy([]Vec2{circle.Center}, 1, circle.Radius)
	pairInput := ShapeCastPairInput{
		ProxyA:       proxyA,
		ProxyB:       circleProxy,
		TransformA:   TransformIdentity,
		TransformB:   TransformIdentity,
		TranslationB: Vec2{X: 10, Y: 0},
		MaxFraction:  1.0,
	}

	b.Run("ShapeCast/PairInput", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkCastOutput = ShapeCast(&pairInput)
		}
	})
	b.Run("ShapeCastCircle", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkCastOutput = ShapeCastCircle(&circle, &input)
		}
	})
	b.Run("ShapeCastCapsule", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkCastOutput = ShapeCastCapsule(&capsule, &input)
		}
	})
	b.Run("ShapeCastSegment", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkCastOutput = ShapeCastSegment(&segment, &input)
		}
	})
	b.Run("ShapeCastPolygon", func(b *testing.B) {
		// Offset polygon to -5,0 for the cast
		offsetPoly := MakeOffsetBox(0.5, 0.5, Vec2{X: -5, Y: 0}, RotIdentity)
		for i := 0; i < b.N; i++ {
			sinkCastOutput = ShapeCastPolygon(&offsetPoly, &input)
		}
	})
}
