package box2d

import "testing"

var (
	sinkSegDistResult SegmentDistanceResult
	sinkDistOutput    DistanceOutput
	sinkTOIOutput     TOIOutput
)

func BenchmarkDistance(b *testing.B) {
	b.Run("SegmentDistance/Parallel", func(b *testing.B) {
		p1 := Vec2{X: 0, Y: 0}
		q1 := Vec2{X: 2, Y: 0}
		p2 := Vec2{X: 0, Y: 1}
		q2 := Vec2{X: 2, Y: 1}
		for i := 0; i < b.N; i++ {
			sinkSegDistResult = SegmentDistance(p1, q1, p2, q2)
		}
	})
	b.Run("SegmentDistance/Crossing", func(b *testing.B) {
		p1 := Vec2{X: -1, Y: 0}
		q1 := Vec2{X: 1, Y: 0}
		p2 := Vec2{X: 0, Y: -1}
		q2 := Vec2{X: 0, Y: 1}
		for i := 0; i < b.N; i++ {
			sinkSegDistResult = SegmentDistance(p1, q1, p2, q2)
		}
	})

	boxA := MakeBox(0.5, 0.5)
	boxB := MakeBox(0.5, 0.5)
	pointsA := make([]Vec2, boxA.Count)
	pointsB := make([]Vec2, boxB.Count)
	copy(pointsA, boxA.Vertices[:boxA.Count])
	copy(pointsB, boxB.Vertices[:boxB.Count])

	b.Run("ShapeDistance/CircleCircle", func(b *testing.B) {
		input := DistanceInput{
			ProxyA:     MakeProxy([]Vec2{{X: 0, Y: 0}}, 1, 0.5),
			ProxyB:     MakeProxy([]Vec2{{X: 3, Y: 0}}, 1, 0.5),
			TransformA: TransformIdentity,
			TransformB: TransformIdentity,
			UseRadii:   true,
		}
		cache := SimplexCache{}
		for i := 0; i < b.N; i++ {
			cache = SimplexCache{}
			sinkDistOutput = ShapeDistance(&input, &cache, nil, 0)
		}
	})
	b.Run("ShapeDistance/BoxBox", func(b *testing.B) {
		input := DistanceInput{
			ProxyA:     MakeProxy(pointsA, boxA.Count, 0),
			ProxyB:     MakeProxy(pointsB, boxB.Count, 0),
			TransformA: TransformIdentity,
			TransformB: Transform{P: Vec2{X: 3, Y: 0}, Q: RotIdentity},
			UseRadii:   false,
		}
		cache := SimplexCache{}
		for i := 0; i < b.N; i++ {
			cache = SimplexCache{}
			sinkDistOutput = ShapeDistance(&input, &cache, nil, 0)
		}
	})

	b.Run("TimeOfImpact/BoxBox", func(b *testing.B) {
		input := TOIInput{
			ProxyA: MakeProxy(pointsA, boxA.Count, 0),
			ProxyB: MakeProxy(pointsB, boxB.Count, 0),
			SweepA: Sweep{
				LocalCenter: Vec2Zero,
				C1:          Vec2{X: -5, Y: 0},
				C2:          Vec2{X: 0, Y: 0},
				Q1:          RotIdentity,
				Q2:          RotIdentity,
			},
			SweepB: Sweep{
				LocalCenter: Vec2Zero,
				C1:          Vec2{X: 5, Y: 0},
				C2:          Vec2{X: 0, Y: 0},
				Q1:          RotIdentity,
				Q2:          RotIdentity,
			},
			MaxFraction: 1.0,
		}
		for i := 0; i < b.N; i++ {
			sinkTOIOutput = TimeOfImpact(&input)
		}
	})
}
