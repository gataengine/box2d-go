package box2d

import "testing"

var (
	sinkMassData MassData
)

func BenchmarkMass(b *testing.B) {
	circle := Circle{Center: Vec2{X: 0.5, Y: 0.5}, Radius: 1.0}
	capsule := Capsule{Center1: Vec2{X: -1, Y: 0}, Center2: Vec2{X: 1, Y: 0}, Radius: 0.5}
	box := MakeBox(1.0, 0.5)
	hullPts := []Vec2{
		{X: -1, Y: -1}, {X: 1, Y: -1}, {X: 1.5, Y: 0},
		{X: 1, Y: 1}, {X: -1, Y: 1}, {X: -1.5, Y: 0},
	}
	hull := ComputeHull(hullPts)
	hexagon := MakePolygon(&hull, 0)
	segment := Segment{Point1: Vec2{X: -2, Y: 0}, Point2: Vec2{X: 2, Y: 0}}
	xf := Transform{P: Vec2{X: 1, Y: 2}, Q: MakeRot(0.5)}

	b.Run("ComputeCircleMass", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkMassData = ComputeCircleMass(&circle, 1.0)
		}
	})
	b.Run("ComputeCapsuleMass", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkMassData = ComputeCapsuleMass(&capsule, 1.0)
		}
	})
	b.Run("ComputePolygonMass/Box", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkMassData = ComputePolygonMass(&box, 1.0)
		}
	})
	b.Run("ComputePolygonMass/Hexagon", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkMassData = ComputePolygonMass(&hexagon, 1.0)
		}
	})
	b.Run("ComputeCircleAABB", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkAABB = ComputeCircleAABB(&circle, xf)
		}
	})
	b.Run("ComputeCapsuleAABB", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkAABB = ComputeCapsuleAABB(&capsule, xf)
		}
	})
	b.Run("ComputePolygonAABB/Box", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkAABB = ComputePolygonAABB(&box, xf)
		}
	})
	b.Run("ComputePolygonAABB/Hexagon", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkAABB = ComputePolygonAABB(&hexagon, xf)
		}
	})
	b.Run("ComputeSegmentAABB", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkAABB = ComputeSegmentAABB(&segment, xf)
		}
	})
}
