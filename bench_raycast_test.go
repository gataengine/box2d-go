package box2d

import "testing"

var sinkCastOutput CastOutput

func BenchmarkRayCast(b *testing.B) {
	circle := Circle{Center: Vec2Zero, Radius: 1.0}
	capsule := Capsule{Center1: Vec2{X: -1, Y: 0}, Center2: Vec2{X: 1, Y: 0}, Radius: 0.5}
	segment := Segment{Point1: Vec2{X: -2, Y: 0}, Point2: Vec2{X: 2, Y: 0}}
	box := MakeBox(1.0, 0.5)

	hitRay := RayCastInput{Origin: Vec2{X: -5, Y: 0}, Translation: Vec2{X: 10, Y: 0}, MaxFraction: 1.0}
	missRay := RayCastInput{Origin: Vec2{X: -5, Y: 10}, Translation: Vec2{X: 10, Y: 0}, MaxFraction: 1.0}

	aabb := AABB{LowerBound: Vec2{X: -1, Y: -1}, UpperBound: Vec2{X: 1, Y: 1}}

	b.Run("Circle/Hit", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkCastOutput = RayCastCircle(&circle, &hitRay)
		}
	})
	b.Run("Circle/Miss", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkCastOutput = RayCastCircle(&circle, &missRay)
		}
	})
	b.Run("Capsule/Hit", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkCastOutput = RayCastCapsule(&capsule, &hitRay)
		}
	})
	b.Run("Capsule/Miss", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkCastOutput = RayCastCapsule(&capsule, &missRay)
		}
	})
	b.Run("Segment/Hit", func(b *testing.B) {
		segRay := RayCastInput{Origin: Vec2{X: 0, Y: -5}, Translation: Vec2{X: 0, Y: 10}, MaxFraction: 1.0}
		for i := 0; i < b.N; i++ {
			sinkCastOutput = RayCastSegment(&segment, &segRay, false)
		}
	})
	b.Run("Segment/Miss", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkCastOutput = RayCastSegment(&segment, &missRay, false)
		}
	})
	b.Run("Segment/OneSided", func(b *testing.B) {
		segRay := RayCastInput{Origin: Vec2{X: 0, Y: -5}, Translation: Vec2{X: 0, Y: 10}, MaxFraction: 1.0}
		for i := 0; i < b.N; i++ {
			sinkCastOutput = RayCastSegment(&segment, &segRay, true)
		}
	})
	b.Run("Polygon/Hit", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkCastOutput = RayCastPolygon(&box, &hitRay)
		}
	})
	b.Run("Polygon/Miss", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkCastOutput = RayCastPolygon(&box, &missRay)
		}
	})
	b.Run("AABB/Hit", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			_ = AABBRayCast(aabb, Vec2{X: -5, Y: 0}, Vec2{X: 5, Y: 0})
		}
	})
	b.Run("AABB/Miss", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			_ = AABBRayCast(aabb, Vec2{X: -5, Y: 10}, Vec2{X: 5, Y: 10})
		}
	})
}
