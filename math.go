package box2d

import "math"

var lengthUnitsPerMeter float32 = 1.0

const Pi float32 = 3.14159265359
const floatEpsilon float32 = 1.1920928955078125e-07

// SetLengthUnitsPerMeter sets the global length-units scale.
func SetLengthUnitsPerMeter(lengthUnits float32) {
	lengthUnitsPerMeter = lengthUnits
}

// GetLengthUnitsPerMeter gets the global length-units scale.
func GetLengthUnitsPerMeter() float32 {
	return lengthUnitsPerMeter
}

// Vec2 is a 2D vector.
type Vec2 struct {
	X float32
	Y float32
}

// Rot is a 2D rotation represented as cosine/sine.
type Rot struct {
	C float32
	S float32
}

// Transform is a translation + rotation pair.
type Transform struct {
	P Vec2
	Q Rot
}

var RotIdentity = Rot{C: 1.0, S: 0.0}
var Vec2Zero = Vec2{X: 0.0, Y: 0.0}
var TransformIdentity = Transform{P: Vec2Zero, Q: RotIdentity}

type CosSin struct {
	Cosine float32
	Sine   float32
}

func IsValidFloat(a float32) bool {
	return !math.IsNaN(float64(a)) && !math.IsInf(float64(a), 0)
}

func IsValidVec2(v Vec2) bool {
	return IsValidFloat(v.X) && IsValidFloat(v.Y)
}

func IsValidRotation(q Rot) bool {
	return IsValidFloat(q.C) && IsValidFloat(q.S)
}

func IsValidTransform(t Transform) bool {
	return IsValidVec2(t.P) && IsValidRotation(t.Q)
}

func IsValidAABB(aabb AABB) bool {
	return IsValidVec2(aabb.LowerBound) &&
		IsValidVec2(aabb.UpperBound) &&
		aabb.LowerBound.X <= aabb.UpperBound.X &&
		aabb.LowerBound.Y <= aabb.UpperBound.Y
}

type Plane struct {
	Normal Vec2
	Offset float32
}

func IsValidPlane(a Plane) bool {
	return IsValidVec2(a.Normal) && IsValidFloat(a.Offset)
}

func Dot(a, b Vec2) float32 {
	return a.X*b.X + a.Y*b.Y
}

func CrossVS(v Vec2, s float32) Vec2 {
	return Vec2{X: s * v.Y, Y: -s * v.X}
}

func CrossSV(s float32, v Vec2) Vec2 {
	return Vec2{X: -s * v.Y, Y: s * v.X}
}

func Cross(a, b Vec2) float32 {
	return a.X*b.Y - a.Y*b.X
}

func MinFloat(a, b float32) float32 {
	if a < b {
		return a
	}
	return b
}

func MaxFloat(a, b float32) float32 {
	if a > b {
		return a
	}
	return b
}

func ClampFloat(a, lower, upper float32) float32 {
	if a < lower {
		return lower
	}
	if a > upper {
		return upper
	}
	return a
}

func Min(a, b Vec2) Vec2 {
	return Vec2{X: MinFloat(a.X, b.X), Y: MinFloat(a.Y, b.Y)}
}

func Max(a, b Vec2) Vec2 {
	return Vec2{X: MaxFloat(a.X, b.X), Y: MaxFloat(a.Y, b.Y)}
}

func Length(v Vec2) float32 {
	return float32(math.Sqrt(float64(v.X*v.X + v.Y*v.Y)))
}

func LengthSquared(v Vec2) float32 {
	return v.X*v.X + v.Y*v.Y
}

func Distance(a, b Vec2) float32 {
	return Length(Sub(b, a))
}

func DistanceSquared(a, b Vec2) float32 {
	c := Sub(b, a)
	return LengthSquared(c)
}

func Normalize(v Vec2) Vec2 {
	length := Length(v)
	if length < floatEpsilon {
		return Vec2Zero
	}
	invLength := 1.0 / length
	return Vec2{X: invLength * v.X, Y: invLength * v.Y}
}

func NormalizeRot(q Rot) Rot {
	mag := float32(math.Sqrt(float64(q.S*q.S + q.C*q.C)))
	var invMag float32
	if mag > 0.0 {
		invMag = 1.0 / mag
	}
	return Rot{C: q.C * invMag, S: q.S * invMag}
}

func Atan2(y, x float32) float32 {
	return float32(math.Atan2(float64(y), float64(x)))
}

func ComputeCosSin(radians float32) CosSin {
	return CosSin{
		Cosine: float32(math.Cos(float64(radians))),
		Sine:   float32(math.Sin(float64(radians))),
	}
}

func MakeRot(radians float32) Rot {
	cs := ComputeCosSin(radians)
	return Rot{
		C: cs.Cosine,
		S: cs.Sine,
	}
}

func MakeRotFromUnitVector(unitVector Vec2) Rot {
	return Rot{C: unitVector.X, S: unitVector.Y}
}

func ComputeRotationBetweenUnitVectors(v1, v2 Vec2) Rot {
	rot := Rot{
		C: Dot(v1, v2),
		S: Cross(v1, v2),
	}
	return NormalizeRot(rot)
}

func NLerp(q1, q2 Rot, t float32) Rot {
	omt := 1.0 - t
	q := Rot{
		C: omt*q1.C + t*q2.C,
		S: omt*q1.S + t*q2.S,
	}
	return NormalizeRot(q)
}

func RotGetAngle(q Rot) float32 {
	return Atan2(q.S, q.C)
}

func RelativeAngle(a, b Rot) float32 {
	s := a.C*b.S - a.S*b.C
	c := a.C*b.C + a.S*b.S
	return Atan2(s, c)
}

func UnwindAngle(radians float32) float32 {
	return float32(math.Remainder(float64(radians), float64(2.0*Pi)))
}

func Add(a, b Vec2) Vec2 {
	return Vec2{X: a.X + b.X, Y: a.Y + b.Y}
}

func Sub(a, b Vec2) Vec2 {
	return Vec2{X: a.X - b.X, Y: a.Y - b.Y}
}

func MulSV(s float32, v Vec2) Vec2 {
	return Vec2{X: s * v.X, Y: s * v.Y}
}

func Neg(a Vec2) Vec2 {
	return Vec2{X: -a.X, Y: -a.Y}
}

func Lerp(a, b Vec2, t float32) Vec2 {
	return Vec2{
		X: (1.0-t)*a.X + t*b.X,
		Y: (1.0-t)*a.Y + t*b.Y,
	}
}

func MulAdd(a Vec2, s float32, b Vec2) Vec2 {
	return Vec2{
		X: a.X + s*b.X,
		Y: a.Y + s*b.Y,
	}
}

func MulSub(a Vec2, s float32, b Vec2) Vec2 {
	return Vec2{
		X: a.X - s*b.X,
		Y: a.Y - s*b.Y,
	}
}

func LeftPerp(v Vec2) Vec2 {
	return Vec2{X: -v.Y, Y: v.X}
}

func RightPerp(v Vec2) Vec2 {
	return Vec2{X: v.Y, Y: -v.X}
}

func GetLengthAndNormalize(v Vec2) (float32, Vec2) {
	length := Length(v)
	if length < floatEpsilon {
		return 0, Vec2Zero
	}
	inv := 1.0 / length
	return length, Vec2{X: inv * v.X, Y: inv * v.Y}
}

func RotateVector(q Rot, v Vec2) Vec2 {
	return Vec2{
		X: q.C*v.X - q.S*v.Y,
		Y: q.S*v.X + q.C*v.Y,
	}
}

func InvRotateVector(q Rot, v Vec2) Vec2 {
	return Vec2{
		X: q.C*v.X + q.S*v.Y,
		Y: -q.S*v.X + q.C*v.Y,
	}
}

func TransformPoint(t Transform, p Vec2) Vec2 {
	return Vec2{
		X: (t.Q.C*p.X - t.Q.S*p.Y) + t.P.X,
		Y: (t.Q.S*p.X + t.Q.C*p.Y) + t.P.Y,
	}
}

func InvTransformPoint(t Transform, p Vec2) Vec2 {
	vx := p.X - t.P.X
	vy := p.Y - t.P.Y
	return Vec2{
		X: t.Q.C*vx + t.Q.S*vy,
		Y: -t.Q.S*vx + t.Q.C*vy,
	}
}

func MulRot(q, r Rot) Rot {
	return Rot{
		S: q.S*r.C + q.C*r.S,
		C: q.C*r.C - q.S*r.S,
	}
}

func MulTransforms(a, b Transform) Transform {
	return Transform{
		Q: MulRot(a.Q, b.Q),
		P: Add(RotateVector(a.Q, b.P), a.P),
	}
}

func AbsFloat(v float32) float32 {
	return math.Float32frombits(math.Float32bits(v) &^ (1 << 31))
}
