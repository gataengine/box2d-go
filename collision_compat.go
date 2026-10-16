package box2d

import "math"

type ChainSegment struct {
	Ghost1  Vec2
	Segment Segment
	Ghost2  Vec2
	ChainID int
}

type ShapeProxy struct {
	Points [MaxPolygonVertices]Vec2
	Count  int
	Radius float32
}

type ShapeCastInput struct {
	Proxy       ShapeProxy
	Translation Vec2
	MaxFraction float32
	CanEncroach bool
}

type SegmentDistanceResult struct {
	Closest1        Vec2
	Closest2        Vec2
	Fraction1       float32
	Fraction2       float32
	DistanceSquared float32
}

type SimplexCache struct {
	Count  uint16
	IndexA [3]uint8
	IndexB [3]uint8
}

type SimplexVertex struct {
	WA     Vec2
	WB     Vec2
	W      Vec2
	A      float32
	IndexA int
	IndexB int
}

type Simplex struct {
	V1    SimplexVertex
	V2    SimplexVertex
	V3    SimplexVertex
	Count int
}

type DistanceInput struct {
	ProxyA     ShapeProxy
	ProxyB     ShapeProxy
	TransformA Transform
	TransformB Transform
	UseRadii   bool
}

type DistanceOutput struct {
	PointA       Vec2
	PointB       Vec2
	Normal       Vec2
	Distance     float32
	Iterations   int
	SimplexCount int
}

type ShapeCastPairInput struct {
	ProxyA       ShapeProxy
	ProxyB       ShapeProxy
	TransformA   Transform
	TransformB   Transform
	TranslationB Vec2
	MaxFraction  float32
	CanEncroach  bool
}

type Sweep struct {
	LocalCenter Vec2
	C1          Vec2
	C2          Vec2
	Q1          Rot
	Q2          Rot
}

type TOIState int

const (
	TOIStateUnknown TOIState = iota
	TOIStateFailed
	TOIStateOverlapped
	TOIStateHit
	TOIStateSeparated
)

type TOIInput struct {
	ProxyA      ShapeProxy
	ProxyB      ShapeProxy
	SweepA      Sweep
	SweepB      Sweep
	MaxFraction float32
}

type TOIOutput struct {
	State    TOIState
	Point    Vec2
	Normal   Vec2
	Fraction float32
}

type CollisionPlane struct {
	Plane        Plane
	PushLimit    float32
	Push         float32
	ClipVelocity bool
}

type PlaneSolverResult struct {
	Translation    Vec2
	IterationCount int
}

type AABBRayCastOutput struct {
	Normal   Vec2
	Point    Vec2
	Fraction float32
	Hit      bool
}

func MakeRoundedBox(halfWidth, halfHeight, radius float32) Polygon {
	p := MakeBox(halfWidth, halfHeight)
	p.Radius = radius
	return p
}

func TransformPolygon(transform Transform, polygon *Polygon) Polygon {
	out := *polygon
	for i := 0; i < out.Count; i++ {
		out.Vertices[i] = TransformPoint(transform, out.Vertices[i])
		out.Normals[i] = RotateVector(transform.Q, out.Normals[i])
	}
	out.Centroid = TransformPoint(transform, out.Centroid)
	return out
}

func MakeOffsetBox(halfWidth, halfHeight float32, center Vec2, rotation Rot) Polygon {
	p := MakeBox(halfWidth, halfHeight)
	xf := Transform{P: center, Q: rotation}
	return TransformPolygon(xf, &p)
}

func MakeOffsetRoundedBox(halfWidth, halfHeight float32, center Vec2, rotation Rot, radius float32) Polygon {
	p := MakeOffsetBox(halfWidth, halfHeight, center, rotation)
	p.Radius = radius
	return p
}

func MakeOffsetPolygon(hull *Hull, position Vec2, rotation Rot) Polygon {
	p := MakePolygon(hull, 0)
	return TransformPolygon(Transform{P: position, Q: rotation}, &p)
}

func MakeOffsetRoundedPolygon(hull *Hull, position Vec2, rotation Rot, radius float32) Polygon {
	p := MakePolygon(hull, radius)
	return TransformPolygon(Transform{P: position, Q: rotation}, &p)
}

func PointInCapsule(shape *Capsule, point Vec2) bool {
	return pointInCapsuleFrame(point, shape.Center1, shape.Center2, shape.Radius)
}

func SegmentDistance(p1, q1, p2, q2 Vec2) SegmentDistanceResult {
	d1 := Sub(q1, p1)
	d2 := Sub(q2, p2)
	r := Sub(p1, p2)
	a := Dot(d1, d1)
	e := Dot(d2, d2)
	f := Dot(d2, r)

	var s, t float32
	if a <= floatEpsilon && e <= floatEpsilon {
		return SegmentDistanceResult{Closest1: p1, Closest2: p2, DistanceSquared: DistanceSquared(p1, p2)}
	}
	if a <= floatEpsilon {
		s = 0
		t = ClampFloat(f/e, 0, 1)
	} else {
		c := Dot(d1, r)
		if e <= floatEpsilon {
			t = 0
			s = ClampFloat(-c/a, 0, 1)
		} else {
			b := Dot(d1, d2)
			denom := a*e - b*b
			if denom != 0 {
				s = ClampFloat((b*f-c*e)/denom, 0, 1)
			}
			t = (b*s + f) / e
			if t < 0 {
				t = 0
				s = ClampFloat(-c/a, 0, 1)
			} else if t > 1 {
				t = 1
				s = ClampFloat((b-c)/a, 0, 1)
			}
		}
	}

	c1 := MulAdd(p1, s, d1)
	c2 := MulAdd(p2, t, d2)
	return SegmentDistanceResult{
		Closest1:        c1,
		Closest2:        c2,
		Fraction1:       s,
		Fraction2:       t,
		DistanceSquared: DistanceSquared(c1, c2),
	}
}

func MakeProxy(points []Vec2, count int, radius float32) ShapeProxy {
	var proxy ShapeProxy
	if count > len(points) {
		count = len(points)
	}
	if count > MaxPolygonVertices {
		count = MaxPolygonVertices
	}
	if count < 0 {
		count = 0
	}
	for i := 0; i < count; i++ {
		proxy.Points[i] = points[i]
	}
	proxy.Count = count
	proxy.Radius = radius
	return proxy
}

func MakeOffsetProxy(points []Vec2, count int, radius float32, position Vec2, rotation Rot) ShapeProxy {
	proxy := MakeProxy(points, count, radius)
	for i := 0; i < proxy.Count; i++ {
		proxy.Points[i] = Add(position, RotateVector(rotation, proxy.Points[i]))
	}
	return proxy
}

func GetSweepTransform(sweep *Sweep, time float32) Transform {
	c := Lerp(sweep.C1, sweep.C2, time)
	q := NLerp(sweep.Q1, sweep.Q2, time)
	return Transform{
		P: Sub(c, RotateVector(q, sweep.LocalCenter)),
		Q: q,
	}
}

func invMulTransforms(a, b Transform) Transform {
	return Transform{
		P: InvTransformPoint(a, b.P),
		Q: Rot{
			C: a.Q.C*b.Q.C + a.Q.S*b.Q.S,
			S: a.Q.C*b.Q.S - a.Q.S*b.Q.C,
		},
	}
}

func findSupport(proxy *ShapeProxy, direction Vec2) int {
	bestIndex := 0
	bestValue := Dot(proxy.Points[0], direction)
	for i := 1; i < proxy.Count; i++ {
		value := Dot(proxy.Points[i], direction)
		if value > bestValue {
			bestIndex = i
			bestValue = value
		}
	}
	return bestIndex
}

func makeSimplexFromCache(cache *SimplexCache, proxyA, proxyB *ShapeProxy) Simplex {
	var simplex Simplex
	if cache != nil {
		simplex.Count = int(cache.Count)
	}

	vertices := []*SimplexVertex{&simplex.V1, &simplex.V2, &simplex.V3}
	for i := 0; i < simplex.Count; i++ {
		v := vertices[i]
		v.IndexA = int(cache.IndexA[i])
		v.IndexB = int(cache.IndexB[i])
		v.WA = proxyA.Points[v.IndexA]
		v.WB = proxyB.Points[v.IndexB]
		v.W = Sub(v.WA, v.WB)
		v.A = -1
	}

	if simplex.Count == 0 {
		v := vertices[0]
		v.IndexA = 0
		v.IndexB = 0
		v.WA = proxyA.Points[0]
		v.WB = proxyB.Points[0]
		v.W = Sub(v.WA, v.WB)
		v.A = 1
		simplex.Count = 1
	}

	return simplex
}

func makeSimplexCache(cache *SimplexCache, simplex *Simplex) {
	if cache == nil {
		return
	}
	cache.Count = uint16(simplex.Count)
	vertices := []*SimplexVertex{&simplex.V1, &simplex.V2, &simplex.V3}
	for i := 0; i < simplex.Count; i++ {
		cache.IndexA[i] = uint8(vertices[i].IndexA)
		cache.IndexB[i] = uint8(vertices[i].IndexB)
	}
}

func computeWitnessPoints(simplex *Simplex) (Vec2, Vec2) {
	switch simplex.Count {
	case 1:
		return simplex.V1.WA, simplex.V1.WB
	case 2:
		return Vec2{
				X: simplex.V1.A*simplex.V1.WA.X + simplex.V2.A*simplex.V2.WA.X,
				Y: simplex.V1.A*simplex.V1.WA.Y + simplex.V2.A*simplex.V2.WA.Y,
			},
			Vec2{
				X: simplex.V1.A*simplex.V1.WB.X + simplex.V2.A*simplex.V2.WB.X,
				Y: simplex.V1.A*simplex.V1.WB.Y + simplex.V2.A*simplex.V2.WB.Y,
			}
	case 3:
		a := Vec2{
			X: simplex.V1.A*simplex.V1.WA.X + simplex.V2.A*simplex.V2.WA.X + simplex.V3.A*simplex.V3.WA.X,
			Y: simplex.V1.A*simplex.V1.WA.Y + simplex.V2.A*simplex.V2.WA.Y + simplex.V3.A*simplex.V3.WA.Y,
		}
		return a, a
	default:
		return Vec2Zero, Vec2Zero
	}
}

func computeSimplexClosestPoint(simplex *Simplex) Vec2 {
	switch simplex.Count {
	case 1:
		return simplex.V1.W
	case 2:
		return Vec2{
			X: simplex.V1.A*simplex.V1.W.X + simplex.V2.A*simplex.V2.W.X,
			Y: simplex.V1.A*simplex.V1.W.Y + simplex.V2.A*simplex.V2.W.Y,
		}
	default:
		return Vec2Zero
	}
}

func solveSimplex2(simplex *Simplex) Vec2 {
	w1 := simplex.V1.W
	w2 := simplex.V2.W
	e12 := Sub(w2, w1)

	d12_2 := -Dot(w1, e12)
	if d12_2 <= 0 {
		simplex.V1.A = 1
		simplex.Count = 1
		return Neg(w1)
	}

	d12_1 := Dot(w2, e12)
	if d12_1 <= 0 {
		simplex.V2.A = 1
		simplex.Count = 1
		simplex.V1 = simplex.V2
		return Neg(w2)
	}

	invD12 := 1.0 / (d12_1 + d12_2)
	simplex.V1.A = d12_1 * invD12
	simplex.V2.A = d12_2 * invD12
	simplex.Count = 2
	return CrossSV(Cross(Add(w1, w2), e12), e12)
}

func solveSimplex3(simplex *Simplex) Vec2 {
	w1 := simplex.V1.W
	w2 := simplex.V2.W
	w3 := simplex.V3.W

	e12 := Sub(w2, w1)
	w1e12 := Dot(w1, e12)
	w2e12 := Dot(w2, e12)
	d12_1 := w2e12
	d12_2 := -w1e12

	e13 := Sub(w3, w1)
	w1e13 := Dot(w1, e13)
	w3e13 := Dot(w3, e13)
	d13_1 := w3e13
	d13_2 := -w1e13

	e23 := Sub(w3, w2)
	w2e23 := Dot(w2, e23)
	w3e23 := Dot(w3, e23)
	d23_1 := w3e23
	d23_2 := -w2e23

	n123 := Cross(e12, e13)
	d123_1 := n123 * Cross(w2, w3)
	d123_2 := n123 * Cross(w3, w1)
	d123_3 := n123 * Cross(w1, w2)

	if d12_2 <= 0 && d13_2 <= 0 {
		simplex.V1.A = 1
		simplex.Count = 1
		return Neg(w1)
	}

	if d12_1 > 0 && d12_2 > 0 && d123_3 <= 0 {
		invD12 := 1.0 / (d12_1 + d12_2)
		simplex.V1.A = d12_1 * invD12
		simplex.V2.A = d12_2 * invD12
		simplex.Count = 2
		return CrossSV(Cross(Add(w1, w2), e12), e12)
	}

	if d13_1 > 0 && d13_2 > 0 && d123_2 <= 0 {
		invD13 := 1.0 / (d13_1 + d13_2)
		simplex.V1.A = d13_1 * invD13
		simplex.V3.A = d13_2 * invD13
		simplex.Count = 2
		simplex.V2 = simplex.V3
		return CrossSV(Cross(Add(w1, w3), e13), e13)
	}

	if d12_1 <= 0 && d23_2 <= 0 {
		simplex.V2.A = 1
		simplex.Count = 1
		simplex.V1 = simplex.V2
		return Neg(w2)
	}

	if d13_1 <= 0 && d23_1 <= 0 {
		simplex.V3.A = 1
		simplex.Count = 1
		simplex.V1 = simplex.V3
		return Neg(w3)
	}

	if d23_1 > 0 && d23_2 > 0 && d123_1 <= 0 {
		invD23 := 1.0 / (d23_1 + d23_2)
		simplex.V2.A = d23_1 * invD23
		simplex.V3.A = d23_2 * invD23
		simplex.Count = 2
		simplex.V1 = simplex.V3
		return CrossSV(Cross(Add(w2, w3), e23), e23)
	}

	invD123 := 1.0 / (d123_1 + d123_2 + d123_3)
	simplex.V1.A = d123_1 * invD123
	simplex.V2.A = d123_2 * invD123
	simplex.V3.A = d123_3 * invD123
	simplex.Count = 3
	return Vec2Zero
}

func ShapeDistance(input *DistanceInput, cache *SimplexCache, simplexes []Simplex, simplexCapacity int) DistanceOutput {
	if input == nil || input.ProxyA.Count == 0 || input.ProxyB.Count == 0 {
		return DistanceOutput{}
	}

	localProxyB := ShapeProxy{
		Count:  input.ProxyB.Count,
		Radius: input.ProxyB.Radius,
	}
	localTransform := invMulTransforms(input.TransformA, input.TransformB)
	for i := 0; i < localProxyB.Count; i++ {
		localProxyB.Points[i] = TransformPoint(localTransform, input.ProxyB.Points[i])
	}

	simplex := makeSimplexFromCache(cache, &input.ProxyA, &localProxyB)
	vertices := []*SimplexVertex{&simplex.V1, &simplex.V2, &simplex.V3}

	simplexIndex := 0
	if simplexes != nil && simplexIndex < simplexCapacity {
		simplexes[simplexIndex] = simplex
		simplexIndex++
	}

	var nonUnitNormal Vec2
	var saveA [3]int
	var saveB [3]int
	iteration := 0
	const maxIterations = 20

	for iteration < maxIterations {
		saveCount := simplex.Count
		for i := 0; i < saveCount; i++ {
			saveA[i] = vertices[i].IndexA
			saveB[i] = vertices[i].IndexB
		}

		var d Vec2
		switch simplex.Count {
		case 1:
			d = Neg(simplex.V1.W)
		case 2:
			d = solveSimplex2(&simplex)
		case 3:
			d = solveSimplex3(&simplex)
		default:
			d = Vec2Zero
		}

		if simplex.Count == 3 {
			localPointA, localPointB := computeWitnessPoints(&simplex)
			return DistanceOutput{
				PointA: TransformPoint(input.TransformA, localPointA),
				PointB: TransformPoint(input.TransformA, localPointB),
			}
		}

		if simplexes != nil && simplexIndex < simplexCapacity {
			simplexes[simplexIndex] = simplex
			simplexIndex++
		}

		if LengthSquared(d) < floatEpsilon*floatEpsilon {
			localPointA, localPointB := computeWitnessPoints(&simplex)
			return DistanceOutput{
				PointA: TransformPoint(input.TransformA, localPointA),
				PointB: TransformPoint(input.TransformA, localPointB),
			}
		}

		nonUnitNormal = d

		vertex := vertices[simplex.Count]
		vertex.IndexA = findSupport(&input.ProxyA, d)
		vertex.WA = input.ProxyA.Points[vertex.IndexA]
		vertex.IndexB = findSupport(&localProxyB, Neg(d))
		vertex.WB = localProxyB.Points[vertex.IndexB]
		vertex.W = Sub(vertex.WA, vertex.WB)

		iteration++

		duplicate := false
		for i := 0; i < saveCount; i++ {
			if vertex.IndexA == saveA[i] && vertex.IndexB == saveB[i] {
				duplicate = true
				break
			}
		}
		if duplicate {
			break
		}

		simplex.Count++
	}

	normal := Normalize(nonUnitNormal)
	normal = RotateVector(input.TransformA.Q, normal)

	localPointA, localPointB := computeWitnessPoints(&simplex)
	output := DistanceOutput{
		PointA:       TransformPoint(input.TransformA, localPointA),
		PointB:       TransformPoint(input.TransformA, localPointB),
		Normal:       normal,
		Distance:     Distance(localPointA, localPointB),
		Iterations:   iteration,
		SimplexCount: simplexIndex,
	}

	makeSimplexCache(cache, &simplex)

	if input.UseRadii {
		radiusA := input.ProxyA.Radius
		radiusB := input.ProxyB.Radius
		output.Distance = MaxFloat(0, output.Distance-radiusA-radiusB)
		output.PointA = MulAdd(output.PointA, radiusA, normal)
		output.PointB = MulSub(output.PointB, radiusB, normal)
	}

	return output
}

func ShapeCast(input *ShapeCastPairInput) CastOutput {
	if input == nil || input.ProxyA.Count == 0 || input.ProxyB.Count == 0 {
		return CastOutput{}
	}

	linearSlop := linearSlop()
	totalRadius := input.ProxyA.Radius + input.ProxyB.Radius
	target := MaxFloat(linearSlop, totalRadius-linearSlop)
	tolerance := 0.25 * linearSlop
	cache := SimplexCache{}

	var output CastOutput
	fraction := float32(0)
	iteration := 0
	distanceInput := DistanceInput{
		ProxyA:     input.ProxyA,
		ProxyB:     input.ProxyB,
		TransformA: input.TransformA,
		TransformB: input.TransformB,
		UseRadii:   false,
	}
	delta2 := input.TranslationB

	for iteration < 20 {
		output.Iterations++
		distanceOutput := ShapeDistance(&distanceInput, &cache, nil, 0)

		if distanceOutput.Distance < target+tolerance {
			if iteration == 0 {
				if input.CanEncroach && distanceOutput.Distance > 2*linearSlop {
					target = distanceOutput.Distance - linearSlop
				} else {
					output.Hit = true
					c1 := MulAdd(distanceOutput.PointA, input.ProxyA.Radius, distanceOutput.Normal)
					c2 := MulAdd(distanceOutput.PointB, -input.ProxyB.Radius, distanceOutput.Normal)
					output.Point = Lerp(c1, c2, 0.5)
					return output
				}
			} else {
				output.Fraction = fraction
				output.Point = MulAdd(distanceOutput.PointA, input.ProxyA.Radius, distanceOutput.Normal)
				output.Normal = distanceOutput.Normal
				output.Hit = true
				return output
			}
		}

		denominator := Dot(delta2, distanceOutput.Normal)
		if denominator >= 0 {
			return output
		}

		fraction += (target - distanceOutput.Distance) / denominator
		if fraction >= input.MaxFraction {
			return output
		}

		distanceInput.TransformB.P = MulAdd(input.TransformB.P, fraction, delta2)
		iteration++
	}

	return output
}

func ShapeCastCircle(shape *Circle, input *ShapeCastInput) CastOutput {
	pairInput := ShapeCastPairInput{
		ProxyA:       MakeProxy([]Vec2{shape.Center}, 1, shape.Radius),
		ProxyB:       input.Proxy,
		TransformA:   TransformIdentity,
		TransformB:   TransformIdentity,
		TranslationB: input.Translation,
		MaxFraction:  input.MaxFraction,
		CanEncroach:  input.CanEncroach,
	}
	return ShapeCast(&pairInput)
}

func ShapeCastCapsule(shape *Capsule, input *ShapeCastInput) CastOutput {
	pairInput := ShapeCastPairInput{
		ProxyA:       MakeProxy([]Vec2{shape.Center1, shape.Center2}, 2, shape.Radius),
		ProxyB:       input.Proxy,
		TransformA:   TransformIdentity,
		TransformB:   TransformIdentity,
		TranslationB: input.Translation,
		MaxFraction:  input.MaxFraction,
		CanEncroach:  input.CanEncroach,
	}
	return ShapeCast(&pairInput)
}

func ShapeCastSegment(shape *Segment, input *ShapeCastInput) CastOutput {
	pairInput := ShapeCastPairInput{
		ProxyA:       MakeProxy([]Vec2{shape.Point1, shape.Point2}, 2, 0),
		ProxyB:       input.Proxy,
		TransformA:   TransformIdentity,
		TransformB:   TransformIdentity,
		TranslationB: input.Translation,
		MaxFraction:  input.MaxFraction,
		CanEncroach:  input.CanEncroach,
	}
	return ShapeCast(&pairInput)
}

func ShapeCastPolygon(shape *Polygon, input *ShapeCastInput) CastOutput {
	points := make([]Vec2, shape.Count)
	copy(points, shape.Vertices[:shape.Count])
	pairInput := ShapeCastPairInput{
		ProxyA:       MakeProxy(points, shape.Count, shape.Radius),
		ProxyB:       input.Proxy,
		TransformA:   TransformIdentity,
		TransformB:   TransformIdentity,
		TranslationB: input.Translation,
		MaxFraction:  input.MaxFraction,
		CanEncroach:  input.CanEncroach,
	}
	return ShapeCast(&pairInput)
}

type toiSeparationType int

const (
	toiPointsType toiSeparationType = iota
	toiFaceAType
	toiFaceBType
)

type toiSeparationFunction struct {
	ProxyA     *ShapeProxy
	ProxyB     *ShapeProxy
	SweepA     Sweep
	SweepB     Sweep
	LocalPoint Vec2
	Axis       Vec2
	Type       toiSeparationType
}

func makeSeparationFunction(cache *SimplexCache, proxyA *ShapeProxy, sweepA *Sweep, proxyB *ShapeProxy, sweepB *Sweep, t1 float32) toiSeparationFunction {
	f := toiSeparationFunction{
		ProxyA: proxyA,
		ProxyB: proxyB,
		SweepA: *sweepA,
		SweepB: *sweepB,
	}

	count := 0
	if cache != nil {
		count = int(cache.Count)
	}
	if count == 0 {
		return f
	}

	xfA := GetSweepTransform(sweepA, t1)
	xfB := GetSweepTransform(sweepB, t1)

	switch count {
	case 1:
		f.Type = toiPointsType
		localPointA := proxyA.Points[cache.IndexA[0]]
		localPointB := proxyB.Points[cache.IndexB[0]]
		pointA := TransformPoint(xfA, localPointA)
		pointB := TransformPoint(xfB, localPointB)
		f.Axis = Normalize(Sub(pointB, pointA))
	case 2:
		if cache.IndexA[0] == cache.IndexA[1] {
			f.Type = toiFaceBType
			localPointB1 := proxyB.Points[cache.IndexB[0]]
			localPointB2 := proxyB.Points[cache.IndexB[1]]
			f.Axis = Normalize(CrossVS(Sub(localPointB2, localPointB1), 1.0))
			normal := RotateVector(xfB.Q, f.Axis)
			f.LocalPoint = Vec2{X: 0.5 * (localPointB1.X + localPointB2.X), Y: 0.5 * (localPointB1.Y + localPointB2.Y)}
			pointB := TransformPoint(xfB, f.LocalPoint)
			localPointA := proxyA.Points[cache.IndexA[0]]
			pointA := TransformPoint(xfA, localPointA)
			if Dot(Sub(pointA, pointB), normal) < 0 {
				f.Axis = Neg(f.Axis)
			}
		} else {
			f.Type = toiFaceAType
			localPointA1 := proxyA.Points[cache.IndexA[0]]
			localPointA2 := proxyA.Points[cache.IndexA[1]]
			f.Axis = Normalize(CrossVS(Sub(localPointA2, localPointA1), 1.0))
			normal := RotateVector(xfA.Q, f.Axis)
			f.LocalPoint = Vec2{X: 0.5 * (localPointA1.X + localPointA2.X), Y: 0.5 * (localPointA1.Y + localPointA2.Y)}
			pointA := TransformPoint(xfA, f.LocalPoint)
			localPointB := proxyB.Points[cache.IndexB[0]]
			pointB := TransformPoint(xfB, localPointB)
			if Dot(Sub(pointB, pointA), normal) < 0 {
				f.Axis = Neg(f.Axis)
			}
		}
	}

	return f
}

func findMinSeparation(f *toiSeparationFunction, indexA, indexB *int, t float32) float32 {
	xfA := GetSweepTransform(&f.SweepA, t)
	xfB := GetSweepTransform(&f.SweepB, t)

	switch f.Type {
	case toiPointsType:
		axisA := InvRotateVector(xfA.Q, f.Axis)
		axisB := InvRotateVector(xfB.Q, Neg(f.Axis))
		*indexA = findSupport(f.ProxyA, axisA)
		*indexB = findSupport(f.ProxyB, axisB)
		pointA := TransformPoint(xfA, f.ProxyA.Points[*indexA])
		pointB := TransformPoint(xfB, f.ProxyB.Points[*indexB])
		return Dot(Sub(pointB, pointA), f.Axis)
	case toiFaceAType:
		normal := RotateVector(xfA.Q, f.Axis)
		pointA := TransformPoint(xfA, f.LocalPoint)
		axisB := InvRotateVector(xfB.Q, Neg(normal))
		*indexA = -1
		*indexB = findSupport(f.ProxyB, axisB)
		pointB := TransformPoint(xfB, f.ProxyB.Points[*indexB])
		return Dot(Sub(pointB, pointA), normal)
	case toiFaceBType:
		normal := RotateVector(xfB.Q, f.Axis)
		pointB := TransformPoint(xfB, f.LocalPoint)
		axisA := InvRotateVector(xfA.Q, Neg(normal))
		*indexB = -1
		*indexA = findSupport(f.ProxyA, axisA)
		pointA := TransformPoint(xfA, f.ProxyA.Points[*indexA])
		return Dot(Sub(pointA, pointB), normal)
	default:
		*indexA = -1
		*indexB = -1
		return 0
	}
}

func evaluateSeparation(f *toiSeparationFunction, indexA, indexB int, t float32) float32 {
	xfA := GetSweepTransform(&f.SweepA, t)
	xfB := GetSweepTransform(&f.SweepB, t)

	switch f.Type {
	case toiPointsType:
		pointA := TransformPoint(xfA, f.ProxyA.Points[indexA])
		pointB := TransformPoint(xfB, f.ProxyB.Points[indexB])
		return Dot(Sub(pointB, pointA), f.Axis)
	case toiFaceAType:
		normal := RotateVector(xfA.Q, f.Axis)
		pointA := TransformPoint(xfA, f.LocalPoint)
		pointB := TransformPoint(xfB, f.ProxyB.Points[indexB])
		return Dot(Sub(pointB, pointA), normal)
	case toiFaceBType:
		normal := RotateVector(xfB.Q, f.Axis)
		pointB := TransformPoint(xfB, f.LocalPoint)
		pointA := TransformPoint(xfA, f.ProxyA.Points[indexA])
		return Dot(Sub(pointA, pointB), normal)
	default:
		return 0
	}
}

func TimeOfImpact(input *TOIInput) TOIOutput {
	if input == nil || input.ProxyA.Count == 0 || input.ProxyB.Count == 0 {
		return TOIOutput{}
	}

	output := TOIOutput{
		State:    TOIStateUnknown,
		Fraction: input.MaxFraction,
	}

	sweepA := input.SweepA
	sweepB := input.SweepB

	totalRadius := input.ProxyA.Radius + input.ProxyB.Radius
	target := MaxFloat(linearSlop(), totalRadius-linearSlop())
	tolerance := 0.25 * linearSlop()

	t1 := float32(0)
	const maxIterations = 20
	distanceIterations := 0
	cache := SimplexCache{}
	distanceInput := DistanceInput{
		ProxyA:   input.ProxyA,
		ProxyB:   input.ProxyB,
		UseRadii: false,
	}

	for {
		xfA := GetSweepTransform(&sweepA, t1)
		xfB := GetSweepTransform(&sweepB, t1)
		distanceInput.TransformA = xfA
		distanceInput.TransformB = xfB

		distanceOutput := ShapeDistance(&distanceInput, &cache, nil, 0)
		distanceIterations++

		if distanceOutput.Distance <= 0 {
			output.State = TOIStateOverlapped
			output.Fraction = 0
			break
		}

		if distanceOutput.Distance <= target+tolerance {
			output.State = TOIStateHit
			pA := MulAdd(distanceOutput.PointA, input.ProxyA.Radius, distanceOutput.Normal)
			pB := MulAdd(distanceOutput.PointB, -input.ProxyB.Radius, distanceOutput.Normal)
			output.Point = Lerp(pA, pB, 0.5)
			output.Normal = distanceOutput.Normal
			output.Fraction = t1
			break
		}

		fcn := makeSeparationFunction(&cache, &input.ProxyA, &sweepA, &input.ProxyB, &sweepB, t1)

		done := false
		t2 := input.MaxFraction
		pushBackIterations := 0
		for !done {
			var indexA, indexB int
			s2 := findMinSeparation(&fcn, &indexA, &indexB, t2)
			if s2 > target+tolerance {
				output.State = TOIStateSeparated
				output.Fraction = input.MaxFraction
				done = true
				break
			}

			if s2 > target-tolerance {
				t1 = t2
				break
			}

			s1 := evaluateSeparation(&fcn, indexA, indexB, t1)
			if s1 < target-tolerance {
				output.State = TOIStateFailed
				output.Fraction = t1
				done = true
				break
			}

			if s1 <= target+tolerance {
				output.State = TOIStateHit
				pA := MulAdd(distanceOutput.PointA, input.ProxyA.Radius, distanceOutput.Normal)
				pB := MulAdd(distanceOutput.PointB, -input.ProxyB.Radius, distanceOutput.Normal)
				output.Point = Lerp(pA, pB, 0.5)
				output.Normal = distanceOutput.Normal
				output.Fraction = t1
				done = true
				break
			}

			rootIterationCount := 0
			a1 := t1
			a2 := t2
			for {
				var t float32
				if rootIterationCount&1 == 1 {
					t = a1 + (target-s1)*(a2-a1)/(s2-s1)
				} else {
					t = 0.5 * (a1 + a2)
				}
				rootIterationCount++

				s := evaluateSeparation(&fcn, indexA, indexB, t)
				if AbsFloat(s-target) < tolerance {
					t2 = t
					break
				}

				if s > target {
					a1 = t
					s1 = s
				} else {
					a2 = t
					s2 = s
				}

				if rootIterationCount == 50 {
					break
				}
			}

			pushBackIterations++
			if pushBackIterations == MaxPolygonVertices {
				break
			}
		}

		if done {
			break
		}

		if distanceIterations == maxIterations {
			output.State = TOIStateFailed
			pA := MulAdd(distanceOutput.PointA, input.ProxyA.Radius, distanceOutput.Normal)
			pB := MulAdd(distanceOutput.PointB, -input.ProxyB.Radius, distanceOutput.Normal)
			output.Point = Lerp(pA, pB, 0.5)
			output.Normal = distanceOutput.Normal
			output.Fraction = t1
			break
		}
	}

	return output
}

func overlapManifold(aabbA, aabbB AABB) Manifold {
	if !AABBOverlaps(aabbA, aabbB) {
		return Manifold{}
	}
	centerA := MulSV(0.5, Add(aabbA.LowerBound, aabbA.UpperBound))
	centerB := MulSV(0.5, Add(aabbB.LowerBound, aabbB.UpperBound))
	normal := Normalize(Sub(centerB, centerA))
	if LengthSquared(normal) == 0 {
		normal = Vec2{X: 1, Y: 0}
	}
	p := MulSV(0.5, Add(centerA, centerB))
	return Manifold{
		Normal: normal,
		Point1: ManifoldPoint{
			Point:      p,
			Separation: -0.001,
			ID:         1,
			Persisted:  false,
		},
		PointCount: 1,
	}
}

func CollideCircles(circleA *Circle, xfA Transform, circleB *Circle, xfB Transform) Manifold {
	a := Circle{Center: TransformPoint(xfA, circleA.Center), Radius: circleA.Radius}
	b := Circle{Center: TransformPoint(xfB, circleB.Center), Radius: circleB.Radius}
	delta := Sub(b.Center, a.Center)
	d2 := LengthSquared(delta)
	r := a.Radius + b.Radius
	if d2 > r*r {
		return Manifold{}
	}
	n := Normalize(delta)
	if LengthSquared(n) == 0 {
		n = Vec2{X: 1, Y: 0}
	}
	p := Add(a.Center, MulSV(a.Radius, n))
	return Manifold{
		Normal: n,
		Point1: ManifoldPoint{
			Point:      p,
			Separation: float32(math.Sqrt(float64(d2))) - r,
			ID:         1,
		},
		PointCount: 1,
	}
}

func CollideCapsuleAndCircle(capsuleA *Capsule, xfA Transform, circleB *Circle, xfB Transform) Manifold {
	aabbA := ComputeCapsuleAABB(capsuleA, xfA)
	aabbB := ComputeCircleAABB(circleB, xfB)
	return overlapManifold(aabbA, aabbB)
}

func CollideSegmentAndCircle(segmentA *Segment, xfA Transform, circleB *Circle, xfB Transform) Manifold {
	aabbA := ComputeSegmentAABB(segmentA, xfA)
	aabbB := ComputeCircleAABB(circleB, xfB)
	return overlapManifold(aabbA, aabbB)
}

func CollidePolygonAndCircle(polygonA *Polygon, xfA Transform, circleB *Circle, xfB Transform) Manifold {
	aabbA := ComputePolygonAABB(polygonA, xfA)
	aabbB := ComputeCircleAABB(circleB, xfB)
	return overlapManifold(aabbA, aabbB)
}

func CollideCapsules(capsuleA *Capsule, xfA Transform, capsuleB *Capsule, xfB Transform) Manifold {
	return overlapManifold(ComputeCapsuleAABB(capsuleA, xfA), ComputeCapsuleAABB(capsuleB, xfB))
}

func CollideSegmentAndCapsule(segmentA *Segment, xfA Transform, capsuleB *Capsule, xfB Transform) Manifold {
	return overlapManifold(ComputeSegmentAABB(segmentA, xfA), ComputeCapsuleAABB(capsuleB, xfB))
}

func CollidePolygonAndCapsule(polygonA *Polygon, xfA Transform, capsuleB *Capsule, xfB Transform) Manifold {
	return overlapManifold(ComputePolygonAABB(polygonA, xfA), ComputeCapsuleAABB(capsuleB, xfB))
}

func CollidePolygons(polygonA *Polygon, xfA Transform, polygonB *Polygon, xfB Transform) Manifold {
	return overlapManifold(ComputePolygonAABB(polygonA, xfA), ComputePolygonAABB(polygonB, xfB))
}

func CollideSegmentAndPolygon(segmentA *Segment, xfA Transform, polygonB *Polygon, xfB Transform) Manifold {
	return overlapManifold(ComputeSegmentAABB(segmentA, xfA), ComputePolygonAABB(polygonB, xfB))
}

func CollideChainSegmentAndCircle(segmentA *ChainSegment, xfA Transform, circleB *Circle, xfB Transform) Manifold {
	return CollideSegmentAndCircle(&segmentA.Segment, xfA, circleB, xfB)
}

func CollideChainSegmentAndCapsule(segmentA *ChainSegment, xfA Transform, capsuleB *Capsule, xfB Transform) Manifold {
	return CollideSegmentAndCapsule(&segmentA.Segment, xfA, capsuleB, xfB)
}

func CollideChainSegmentAndPolygon(segmentA *ChainSegment, xfA Transform, polygonB *Polygon, xfB Transform) Manifold {
	return CollideSegmentAndPolygon(&segmentA.Segment, xfA, polygonB, xfB)
}

func SolvePlanes(targetDelta Vec2, planes []CollisionPlane, count int) PlaneSolverResult {
	if count > len(planes) {
		count = len(planes)
	}
	translation := targetDelta
	for i := 0; i < count; i++ {
		p := planes[i]
		push := Dot(p.Plane.Normal, translation) - p.Plane.Offset
		if push < 0 {
			correction := -push
			if p.PushLimit > 0 && correction > p.PushLimit {
				correction = p.PushLimit
			}
			translation = Add(translation, MulSV(correction, p.Plane.Normal))
		}
	}
	return PlaneSolverResult{Translation: translation, IterationCount: count}
}

func ClipVector(vector Vec2, planes []CollisionPlane, count int) Vec2 {
	if count > len(planes) {
		count = len(planes)
	}
	v := vector
	for i := 0; i < count; i++ {
		p := planes[i]
		if !p.ClipVelocity || p.Push == 0 {
			continue
		}
		n := p.Plane.Normal
		d := Dot(v, n)
		if d < 0 {
			v = Sub(v, MulSV(d, n))
		}
	}
	return v
}

func AABBContains(a, b AABB) bool {
	return a.LowerBound.X <= b.LowerBound.X &&
		a.LowerBound.Y <= b.LowerBound.Y &&
		a.UpperBound.X >= b.UpperBound.X &&
		a.UpperBound.Y >= b.UpperBound.Y
}

func AABBRayCast(aabb AABB, p1, p2 Vec2) AABBRayCastOutput {
	input := RayCastInput{
		Origin:      p1,
		Translation: Sub(p2, p1),
		MaxFraction: 1,
	}
	return rayCastAABB(input, aabb)
}

func sweepAABB(target AABB, mover AABB, translation Vec2, maxFraction float32) AABBRayCastOutput {
	half := MulSV(0.5, Sub(mover.UpperBound, mover.LowerBound))
	expanded := AABB{
		LowerBound: Sub(target.LowerBound, half),
		UpperBound: Add(target.UpperBound, half),
	}
	center := MulSV(0.5, Add(mover.LowerBound, mover.UpperBound))
	input := RayCastInput{Origin: center, Translation: translation, MaxFraction: maxFraction}
	return rayCastAABB(input, expanded)
}

func rayCastAABB(input RayCastInput, aabb AABB) AABBRayCastOutput {
	output := AABBRayCastOutput{}
	if input.Origin.X >= aabb.LowerBound.X && input.Origin.X <= aabb.UpperBound.X &&
		input.Origin.Y >= aabb.LowerBound.Y && input.Origin.Y <= aabb.UpperBound.Y {
		return output
	}
	tmin := float32(0.0)
	tmax := input.MaxFraction
	p := input.Origin
	d := input.Translation
	normal := Vec2Zero

	for axis := 0; axis < 2; axis++ {
		var pAxis, dAxis, lo, hi float32
		if axis == 0 {
			pAxis, dAxis, lo, hi = p.X, d.X, aabb.LowerBound.X, aabb.UpperBound.X
		} else {
			pAxis, dAxis, lo, hi = p.Y, d.Y, aabb.LowerBound.Y, aabb.UpperBound.Y
		}

		if AbsFloat(dAxis) < floatEpsilon {
			if pAxis < lo || pAxis > hi {
				return output
			}
			continue
		}

		invD := 1.0 / dAxis
		t1 := (lo - pAxis) * invD
		t2 := (hi - pAxis) * invD
		s := float32(-1.0)
		if t1 > t2 {
			t1, t2 = t2, t1
			s = 1.0
		}
		if t1 > tmin {
			tmin = t1
			if axis == 0 {
				normal = Vec2{X: s, Y: 0}
			} else {
				normal = Vec2{X: 0, Y: s}
			}
		}
		tmax = MinFloat(tmax, t2)
		if tmin > tmax {
			return output
		}
	}

	if tmin < 0 || tmin > input.MaxFraction {
		return output
	}

	output.Hit = true
	output.Fraction = tmin
	output.Normal = normal
	output.Point = MulAdd(input.Origin, tmin, input.Translation)
	return output
}
