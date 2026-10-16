package box2d

import "testing"

var sinkManifold Manifold

func BenchmarkManifold(b *testing.B) {
	xfA := Transform{P: Vec2{X: 0, Y: 0}, Q: RotIdentity}
	xfB := Transform{P: Vec2{X: 1.5, Y: 0}, Q: RotIdentity}

	circleA := Circle{Center: Vec2Zero, Radius: 1.0}
	circleB := Circle{Center: Vec2Zero, Radius: 1.0}
	capsuleA := Capsule{Center1: Vec2{X: -0.5, Y: 0}, Center2: Vec2{X: 0.5, Y: 0}, Radius: 0.5}
	capsuleB := Capsule{Center1: Vec2{X: -0.5, Y: 0}, Center2: Vec2{X: 0.5, Y: 0}, Radius: 0.5}
	segmentA := Segment{Point1: Vec2{X: -1, Y: 0}, Point2: Vec2{X: 1, Y: 0}}
	boxA := MakeBox(1.0, 0.5)
	boxB := MakeBox(0.8, 0.6)
	chainSeg := ChainSegment{
		Ghost1:  Vec2{X: -3, Y: 0},
		Segment: segmentA,
		Ghost2:  Vec2{X: 3, Y: 0},
	}

	b.Run("CollideCircles", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkManifold = CollideCircles(&circleA, xfA, &circleB, xfB)
		}
	})
	b.Run("CollideCapsuleAndCircle", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkManifold = CollideCapsuleAndCircle(&capsuleA, xfA, &circleB, xfB)
		}
	})
	b.Run("CollideSegmentAndCircle", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkManifold = CollideSegmentAndCircle(&segmentA, xfA, &circleB, xfB)
		}
	})
	b.Run("CollidePolygonAndCircle", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkManifold = CollidePolygonAndCircle(&boxA, xfA, &circleB, xfB)
		}
	})
	b.Run("CollideCapsules", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkManifold = CollideCapsules(&capsuleA, xfA, &capsuleB, xfB)
		}
	})
	b.Run("CollideSegmentAndCapsule", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkManifold = CollideSegmentAndCapsule(&segmentA, xfA, &capsuleB, xfB)
		}
	})
	b.Run("CollidePolygonAndCapsule", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkManifold = CollidePolygonAndCapsule(&boxA, xfA, &capsuleB, xfB)
		}
	})
	b.Run("CollidePolygons", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkManifold = CollidePolygons(&boxA, xfA, &boxB, xfB)
		}
	})
	b.Run("CollideSegmentAndPolygon", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkManifold = CollideSegmentAndPolygon(&segmentA, xfA, &boxB, xfB)
		}
	})
	b.Run("CollideChainSegmentAndCircle", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkManifold = CollideChainSegmentAndCircle(&chainSeg, xfA, &circleB, xfB)
		}
	})
	b.Run("CollideChainSegmentAndCapsule", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkManifold = CollideChainSegmentAndCapsule(&chainSeg, xfA, &capsuleB, xfB)
		}
	})
	b.Run("CollideChainSegmentAndPolygon", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkManifold = CollideChainSegmentAndPolygon(&chainSeg, xfA, &boxB, xfB)
		}
	})
}
