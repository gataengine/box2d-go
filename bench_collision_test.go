package box2d

import "testing"

var (
	sinkPolygon Polygon
	sinkHull    Hull
	sinkAABB    AABB
)

func BenchmarkCollision(b *testing.B) {
	hullPoints := []Vec2{
		{X: -1, Y: -1}, {X: 1, Y: -1}, {X: 1.5, Y: 0},
		{X: 1, Y: 1}, {X: -1, Y: 1}, {X: -1.5, Y: 0},
	}
	hull := ComputeHull(hullPoints)
	box := MakeBox(1.0, 0.5)
	xf := Transform{P: Vec2{X: 1, Y: 2}, Q: MakeRot(0.3)}

	aabbA := AABB{LowerBound: Vec2{X: -1, Y: -1}, UpperBound: Vec2{X: 1, Y: 1}}
	aabbB := AABB{LowerBound: Vec2{X: 0, Y: 0}, UpperBound: Vec2{X: 2, Y: 2}}
	aabbC := AABB{LowerBound: Vec2{X: 5, Y: 5}, UpperBound: Vec2{X: 6, Y: 6}}
	aabbSmall := AABB{LowerBound: Vec2{X: -0.5, Y: -0.5}, UpperBound: Vec2{X: 0.5, Y: 0.5}}

	b.Run("ComputeHull", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkHull = ComputeHull(hullPoints)
		}
	})
	b.Run("ValidateHull", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkBool = ValidateHull(&hull)
		}
	})
	b.Run("MakePolygon", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkPolygon = MakePolygon(&hull, 0)
		}
	})
	b.Run("MakeBox", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkPolygon = MakeBox(1.0, 0.5)
		}
	})
	b.Run("MakeSquare", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkPolygon = MakeSquare(0.5)
		}
	})
	b.Run("MakeRoundedBox", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkPolygon = MakeRoundedBox(1.0, 0.5, 0.1)
		}
	})
	b.Run("MakeOffsetBox", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkPolygon = MakeOffsetBox(1.0, 0.5, Vec2{X: 2, Y: 3}, RotIdentity)
		}
	})
	b.Run("TransformPolygon", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkPolygon = TransformPolygon(xf, &box)
		}
	})
	b.Run("AABBOverlaps/Hit", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkBool = AABBOverlaps(aabbA, aabbB)
		}
	})
	b.Run("AABBOverlaps/Miss", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkBool = AABBOverlaps(aabbA, aabbC)
		}
	})
	b.Run("AABBContains/True", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkBool = AABBContains(aabbA, aabbSmall)
		}
	})
	b.Run("AABBContains/False", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkBool = AABBContains(aabbSmall, aabbA)
		}
	})
	b.Run("PointInCircle/Inside", func(b *testing.B) {
		circle := Circle{Center: Vec2Zero, Radius: 1.0}
		pt := Vec2{X: 0.5, Y: 0.5}
		for i := 0; i < b.N; i++ {
			sinkBool = PointInCircle(&circle, pt)
		}
	})
	b.Run("PointInCircle/Outside", func(b *testing.B) {
		circle := Circle{Center: Vec2Zero, Radius: 1.0}
		pt := Vec2{X: 5, Y: 5}
		for i := 0; i < b.N; i++ {
			sinkBool = PointInCircle(&circle, pt)
		}
	})
	b.Run("PointInCapsule/Inside", func(b *testing.B) {
		cap := Capsule{Center1: Vec2{X: -1, Y: 0}, Center2: Vec2{X: 1, Y: 0}, Radius: 0.5}
		pt := Vec2{X: 0, Y: 0.3}
		for i := 0; i < b.N; i++ {
			sinkBool = PointInCapsule(&cap, pt)
		}
	})
	b.Run("PointInCapsule/Outside", func(b *testing.B) {
		cap := Capsule{Center1: Vec2{X: -1, Y: 0}, Center2: Vec2{X: 1, Y: 0}, Radius: 0.5}
		pt := Vec2{X: 5, Y: 5}
		for i := 0; i < b.N; i++ {
			sinkBool = PointInCapsule(&cap, pt)
		}
	})
	b.Run("PointInPolygon/Inside", func(b *testing.B) {
		pt := Vec2{X: 0.1, Y: 0.1}
		for i := 0; i < b.N; i++ {
			sinkBool = PointInPolygon(&box, pt)
		}
	})
	b.Run("PointInPolygon/Outside", func(b *testing.B) {
		pt := Vec2{X: 5, Y: 5}
		for i := 0; i < b.N; i++ {
			sinkBool = PointInPolygon(&box, pt)
		}
	})
}
