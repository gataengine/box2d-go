package box2d

import "testing"

var (
	sinkF32       float32
	sinkVec2      Vec2
	sinkRot       Rot
	sinkTransform Transform
	sinkCosSin    CosSin
	sinkBool      bool
)

func BenchmarkMath(b *testing.B) {
	a := Vec2{X: 3.0, Y: 4.0}
	c := Vec2{X: 1.0, Y: 2.0}
	rot := MakeRot(0.7)
	rot2 := MakeRot(1.4)
	xf := Transform{P: Vec2{X: -2, Y: 3}, Q: rot}
	xf2 := Transform{P: Vec2{X: 1, Y: 0}, Q: rot2}
	uv1 := Normalize(Vec2{X: 0.6, Y: 0.8})
	uv2 := Normalize(Vec2{X: -0.3, Y: 0.95})

	b.Run("Dot", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkF32 = Dot(a, c)
		}
	})
	b.Run("Cross", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkF32 = Cross(a, c)
		}
	})
	b.Run("CrossVS", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkVec2 = CrossVS(a, 2.5)
		}
	})
	b.Run("CrossSV", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkVec2 = CrossSV(2.5, a)
		}
	})
	b.Run("Add", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkVec2 = Add(a, c)
		}
	})
	b.Run("Sub", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkVec2 = Sub(a, c)
		}
	})
	b.Run("MulSV", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkVec2 = MulSV(2.5, a)
		}
	})
	b.Run("Neg", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkVec2 = Neg(a)
		}
	})
	b.Run("Length", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkF32 = Length(a)
		}
	})
	b.Run("LengthSquared", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkF32 = LengthSquared(a)
		}
	})
	b.Run("Distance", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkF32 = Distance(a, c)
		}
	})
	b.Run("DistanceSquared", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkF32 = DistanceSquared(a, c)
		}
	})
	b.Run("Normalize", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkVec2 = Normalize(a)
		}
	})
	b.Run("GetLengthAndNormalize", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkF32, sinkVec2 = GetLengthAndNormalize(a)
		}
	})
	b.Run("Lerp", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkVec2 = Lerp(a, c, 0.5)
		}
	})
	b.Run("MulAdd", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkVec2 = MulAdd(a, 2.5, c)
		}
	})
	b.Run("MulSub", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkVec2 = MulSub(a, 2.5, c)
		}
	})
	b.Run("LeftPerp", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkVec2 = LeftPerp(a)
		}
	})
	b.Run("RightPerp", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkVec2 = RightPerp(a)
		}
	})
	b.Run("MakeRot", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkRot = MakeRot(1.5)
		}
	})
	b.Run("ComputeCosSin", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkCosSin = ComputeCosSin(1.5)
		}
	})
	b.Run("NormalizeRot", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkRot = NormalizeRot(rot)
		}
	})
	b.Run("RotateVector", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkVec2 = RotateVector(rot, a)
		}
	})
	b.Run("InvRotateVector", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkVec2 = InvRotateVector(rot, a)
		}
	})
	b.Run("TransformPoint", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkVec2 = TransformPoint(xf, a)
		}
	})
	b.Run("InvTransformPoint", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkVec2 = InvTransformPoint(xf, a)
		}
	})
	b.Run("MulRot", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkRot = MulRot(rot, rot2)
		}
	})
	b.Run("MulTransforms", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkTransform = MulTransforms(xf, xf2)
		}
	})
	b.Run("NLerp", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkRot = NLerp(rot, rot2, 0.5)
		}
	})
	b.Run("RotGetAngle", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkF32 = RotGetAngle(rot)
		}
	})
	b.Run("RelativeAngle", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkF32 = RelativeAngle(rot, rot2)
		}
	})
	b.Run("UnwindAngle", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkF32 = UnwindAngle(7.5)
		}
	})
	b.Run("Atan2", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkF32 = Atan2(0.7, 0.3)
		}
	})
	b.Run("ComputeRotationBetweenUnitVectors", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkRot = ComputeRotationBetweenUnitVectors(uv1, uv2)
		}
	})
	b.Run("AbsFloat", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			sinkF32 = AbsFloat(-3.14)
		}
	})
}
