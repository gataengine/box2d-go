package box2d

import (
	"cmp"
	"math"
	"slices"
)

const MaxPolygonVertices = 8

type RayCastInput struct {
	Origin      Vec2
	Translation Vec2
	MaxFraction float32
}

type CastOutput struct {
	Normal     Vec2
	Point      Vec2
	Fraction   float32
	Iterations int
	Hit        bool
}

type RayHit struct {
	ShapeID ShapeID
	Output  CastOutput
}

type MassData struct {
	Mass              float32
	Center            Vec2
	RotationalInertia float32
}

type ManifoldPoint struct {
	Point      Vec2
	Separation float32
	ID         uint16
	Persisted  bool
}

type Manifold struct {
	Normal     Vec2
	Point1     ManifoldPoint
	Point2     ManifoldPoint
	PointCount int
}

// ContactData carries touching-contact ids and manifold payload for query APIs.
type ContactData struct {
	ContactID ContactID
	ShapeIDA  ShapeID
	ShapeIDB  ShapeID
	Manifold  Manifold
}

type Circle struct {
	Center Vec2
	Radius float32
}

type Capsule struct {
	Center1 Vec2
	Center2 Vec2
	Radius  float32
}

type Polygon struct {
	Vertices [MaxPolygonVertices]Vec2
	Normals  [MaxPolygonVertices]Vec2
	Centroid Vec2
	Radius   float32
	Count    int
}

type Segment struct {
	Point1 Vec2
	Point2 Vec2
}

type Hull struct {
	Points [MaxPolygonVertices]Vec2
	Count  int
}

type AABB struct {
	LowerBound Vec2
	UpperBound Vec2
}

func AABBOverlaps(a, b AABB) bool {
	return a.LowerBound.X <= b.UpperBound.X &&
		a.UpperBound.X >= b.LowerBound.X &&
		a.LowerBound.Y <= b.UpperBound.Y &&
		a.UpperBound.Y >= b.LowerBound.Y
}

func IsValidRay(input RayCastInput) bool {
	return IsValidVec2(input.Origin) &&
		IsValidVec2(input.Translation) &&
		IsValidFloat(input.MaxFraction) &&
		0.0 <= input.MaxFraction &&
		input.MaxFraction < 100000.0*GetLengthUnitsPerMeter()
}

func computePolygonCentroid(vertices []Vec2) Vec2 {
	center := Vec2Zero
	area := float32(0.0)
	origin := vertices[0]
	inv3 := float32(1.0 / 3.0)

	for i := 1; i < len(vertices)-1; i++ {
		e1 := Sub(vertices[i], origin)
		e2 := Sub(vertices[i+1], origin)
		a := 0.5 * Cross(e1, e2)
		center = MulAdd(center, a*inv3, Add(e1, e2))
		area += a
	}

	if AbsFloat(area) < floatEpsilon {
		return origin
	}
	invArea := 1.0 / area
	center.X *= invArea
	center.Y *= invArea
	return Add(origin, center)
}

func MakePolygon(hull *Hull, radius float32) Polygon {
	if !ValidateHull(hull) || hull.Count < 3 {
		return MakeSquare(0.5)
	}

	shape := Polygon{
		Count:  hull.Count,
		Radius: radius,
	}

	for i := 0; i < shape.Count; i++ {
		shape.Vertices[i] = hull.Points[i]
	}

	for i := 0; i < shape.Count; i++ {
		i1 := i
		i2 := 0
		if i+1 < shape.Count {
			i2 = i + 1
		}
		edge := Sub(shape.Vertices[i2], shape.Vertices[i1])
		shape.Normals[i] = Normalize(CrossVS(edge, 1.0))
	}

	shape.Centroid = computePolygonCentroid(shape.Vertices[:shape.Count])
	return shape
}

func MakeSquare(halfWidth float32) Polygon {
	return MakeBox(halfWidth, halfWidth)
}

func MakeBox(halfWidth, halfHeight float32) Polygon {
	shape := Polygon{
		Count:  4,
		Radius: 0.0,
	}
	shape.Vertices[0] = Vec2{X: -halfWidth, Y: -halfHeight}
	shape.Vertices[1] = Vec2{X: halfWidth, Y: -halfHeight}
	shape.Vertices[2] = Vec2{X: halfWidth, Y: halfHeight}
	shape.Vertices[3] = Vec2{X: -halfWidth, Y: halfHeight}
	shape.Normals[0] = Vec2{X: 0.0, Y: -1.0}
	shape.Normals[1] = Vec2{X: 1.0, Y: 0.0}
	shape.Normals[2] = Vec2{X: 0.0, Y: 1.0}
	shape.Normals[3] = Vec2{X: -1.0, Y: 0.0}
	shape.Centroid = Vec2Zero
	return shape
}

func ComputeCircleMass(shape *Circle, density float32) MassData {
	rr := shape.Radius * shape.Radius
	mass := density * Pi * rr
	return MassData{
		Mass:              mass,
		Center:            shape.Center,
		RotationalInertia: mass * 0.5 * rr,
	}
}

func ComputeCapsuleMass(shape *Capsule, density float32) MassData {
	radius := shape.Radius
	rr := radius * radius
	p1 := shape.Center1
	p2 := shape.Center2
	length := Distance(p1, p2)
	ll := length * length

	circleMass := density * (Pi * rr)
	boxMass := density * (2.0 * radius * length)
	mass := circleMass + boxMass

	lc := 4.0 * radius / (3.0 * Pi)
	h := 0.5 * length
	circleInertia := circleMass * (0.5*rr + h*h + 2.0*h*lc)
	boxInertia := boxMass * (4.0*rr + ll) / 12.0

	return MassData{
		Mass:              mass,
		Center:            Vec2{X: 0.5 * (p1.X + p2.X), Y: 0.5 * (p1.Y + p2.Y)},
		RotationalInertia: circleInertia + boxInertia,
	}
}

func ComputePolygonMass(shape *Polygon, density float32) MassData {
	if shape.Count <= 0 {
		return MassData{}
	}

	if shape.Count == 1 {
		circle := Circle{Center: shape.Vertices[0], Radius: shape.Radius}
		return ComputeCircleMass(&circle, density)
	}

	if shape.Count == 2 {
		capsule := Capsule{Center1: shape.Vertices[0], Center2: shape.Vertices[1], Radius: shape.Radius}
		return ComputeCapsuleMass(&capsule, density)
	}

	vertices := make([]Vec2, shape.Count)
	radius := shape.Radius
	if radius > 0.0 {
		sqrt2 := float32(1.412)
		for i := 0; i < shape.Count; i++ {
			j := i - 1
			if i == 0 {
				j = shape.Count - 1
			}
			n1 := shape.Normals[j]
			n2 := shape.Normals[i]
			mid := Normalize(Add(n1, n2))
			vertices[i] = MulAdd(shape.Vertices[i], sqrt2*radius, mid)
		}
	} else {
		copy(vertices, shape.Vertices[:shape.Count])
	}

	center := Vec2Zero
	area := float32(0.0)
	rotationalInertia := float32(0.0)
	r := vertices[0]
	inv3 := float32(1.0 / 3.0)

	for i := 1; i < shape.Count-1; i++ {
		e1 := Sub(vertices[i], r)
		e2 := Sub(vertices[i+1], r)
		d := Cross(e1, e2)
		triangleArea := 0.5 * d
		area += triangleArea
		center = MulAdd(center, triangleArea*inv3, Add(e1, e2))

		ex1, ey1 := e1.X, e1.Y
		ex2, ey2 := e2.X, e2.Y
		intx2 := ex1*ex1 + ex2*ex1 + ex2*ex2
		inty2 := ey1*ey1 + ey2*ey1 + ey2*ey2
		rotationalInertia += (0.25 * inv3 * d) * (intx2 + inty2)
	}

	if AbsFloat(area) < floatEpsilon {
		return MassData{}
	}

	invArea := 1.0 / area
	center.X *= invArea
	center.Y *= invArea
	mass := density * area
	rotInertia := density*rotationalInertia - mass*Dot(center, center)
	if rotInertia < 0 {
		rotInertia = 0
	}

	return MassData{
		Mass:              mass,
		Center:            Add(r, center),
		RotationalInertia: rotInertia,
	}
}

func ComputeCircleAABB(shape *Circle, xf Transform) AABB {
	p := TransformPoint(xf, shape.Center)
	r := shape.Radius
	return AABB{
		LowerBound: Vec2{X: p.X - r, Y: p.Y - r},
		UpperBound: Vec2{X: p.X + r, Y: p.Y + r},
	}
}

func ComputeCapsuleAABB(shape *Capsule, xf Transform) AABB {
	v1 := TransformPoint(xf, shape.Center1)
	v2 := TransformPoint(xf, shape.Center2)
	r := Vec2{X: shape.Radius, Y: shape.Radius}
	return AABB{
		LowerBound: Sub(Min(v1, v2), r),
		UpperBound: Add(Max(v1, v2), r),
	}
}

func ComputePolygonAABB(shape *Polygon, xf Transform) AABB {
	lower := TransformPoint(xf, shape.Vertices[0])
	upper := lower
	for i := 1; i < shape.Count; i++ {
		v := TransformPoint(xf, shape.Vertices[i])
		lower = Min(lower, v)
		upper = Max(upper, v)
	}
	r := Vec2{X: shape.Radius, Y: shape.Radius}
	return AABB{
		LowerBound: Sub(lower, r),
		UpperBound: Add(upper, r),
	}
}

func ComputeSegmentAABB(shape *Segment, xf Transform) AABB {
	v1 := TransformPoint(xf, shape.Point1)
	v2 := TransformPoint(xf, shape.Point2)
	return AABB{
		LowerBound: Min(v1, v2),
		UpperBound: Max(v1, v2),
	}
}

func PointInCircle(shape *Circle, point Vec2) bool {
	return DistanceSquared(point, shape.Center) <= shape.Radius*shape.Radius
}

func PointInPolygon(shape *Polygon, point Vec2) bool {
	for i := 0; i < shape.Count; i++ {
		s := Dot(shape.Normals[i], Sub(point, shape.Vertices[i]))
		if s > shape.Radius {
			return false
		}
	}
	return true
}

func RayCastCircle(shape *Circle, input *RayCastInput) CastOutput {
	output := CastOutput{}
	p := shape.Center
	s := Sub(input.Origin, p)
	rr := shape.Radius * shape.Radius

	length, d := GetLengthAndNormalize(input.Translation)
	if length == 0.0 {
		if LengthSquared(s) < rr {
			output.Point = input.Origin
			output.Hit = true
		}
		return output
	}

	t := -Dot(s, d)
	c := MulAdd(s, t, d)
	cc := Dot(c, c)
	if cc > rr {
		return output
	}

	h := float32(math.Sqrt(float64(rr - cc)))
	fraction := t - h
	if fraction < 0.0 || input.MaxFraction*length < fraction {
		if LengthSquared(s) < rr {
			output.Point = input.Origin
			output.Hit = true
		}
		return output
	}

	hitPoint := MulAdd(s, fraction, d)
	output.Fraction = fraction / length
	output.Normal = Normalize(hitPoint)
	output.Point = MulAdd(p, shape.Radius, output.Normal)
	output.Hit = true
	return output
}

func pointInCapsuleFrame(point, center1, center2 Vec2, radius float32) bool {
	segment := Sub(center2, center1)
	length2 := Dot(segment, segment)
	var t float32
	if length2 > 0 {
		t = ClampFloat(Dot(Sub(point, center1), segment)/length2, 0, 1)
	}
	closest := MulAdd(center1, t, segment)
	return DistanceSquared(point, closest) <= radius*radius
}

func RayCastCapsule(shape *Capsule, input *RayCastInput) CastOutput {
	segment := Sub(shape.Center2, shape.Center1)
	length, axis := GetLengthAndNormalize(segment)
	if length == 0 {
		circle := Circle{
			Center: shape.Center1,
			Radius: shape.Radius,
		}
		return RayCastCircle(&circle, input)
	}

	center := MulSV(0.5, Add(shape.Center1, shape.Center2))
	frame := Transform{
		P: center,
		Q: Rot{C: axis.X, S: axis.Y},
	}

	localInput := RayCastInput{
		Origin:      InvTransformPoint(frame, input.Origin),
		Translation: InvRotateVector(frame.Q, input.Translation),
		MaxFraction: input.MaxFraction,
	}

	halfLength := 0.5 * length
	box := MakeBox(halfLength, shape.Radius)
	circle1 := Circle{Center: Vec2{X: -halfLength, Y: 0}, Radius: shape.Radius}
	circle2 := Circle{Center: Vec2{X: halfLength, Y: 0}, Radius: shape.Radius}

	candidates := [3]CastOutput{
		RayCastPolygon(&box, &localInput),
		RayCastCircle(&circle1, &localInput),
		RayCastCircle(&circle2, &localInput),
	}

	segmentStart := Vec2{X: -halfLength, Y: 0}
	segmentEnd := Vec2{X: halfLength, Y: 0}
	best := CastOutput{}
	found := false
	bestFraction := input.MaxFraction

	for _, candidate := range candidates {
		if !candidate.Hit {
			continue
		}
		if candidate.Fraction < 0 || candidate.Fraction > input.MaxFraction {
			continue
		}
		if !pointInCapsuleFrame(candidate.Point, segmentStart, segmentEnd, shape.Radius) {
			continue
		}
		if !found || candidate.Fraction < bestFraction {
			best = candidate
			bestFraction = candidate.Fraction
			found = true
		}
	}

	if !found {
		return CastOutput{}
	}

	best.Point = TransformPoint(frame, best.Point)
	best.Normal = RotateVector(frame.Q, best.Normal)
	return best
}

func RayCastSegment(shape *Segment, input *RayCastInput, oneSided bool) CastOutput {
	if oneSided {
		offset := Cross(Sub(input.Origin, shape.Point1), Sub(shape.Point2, shape.Point1))
		if offset < 0.0 {
			return CastOutput{}
		}
	}

	p1 := input.Origin
	d := input.Translation
	v1 := shape.Point1
	v2 := shape.Point2
	e := Sub(v2, v1)
	output := CastOutput{}

	length, eUnit := GetLengthAndNormalize(e)
	if length == 0.0 {
		return output
	}

	normal := RightPerp(eUnit)
	numerator := Dot(normal, Sub(v1, p1))
	denominator := Dot(normal, d)
	if denominator == 0.0 {
		return output
	}

	t := numerator / denominator
	if t < 0.0 || input.MaxFraction < t {
		return output
	}

	p := MulAdd(p1, t, d)
	s := Dot(Sub(p, v1), eUnit)
	if s < 0.0 || length < s {
		return output
	}

	if numerator > 0.0 {
		normal = Neg(normal)
	}

	output.Fraction = t
	output.Point = p
	output.Normal = normal
	output.Hit = true
	return output
}

func RayCastPolygon(shape *Polygon, input *RayCastInput) CastOutput {
	base := shape.Vertices[0]
	p1 := Sub(input.Origin, base)
	d := input.Translation

	lower, upper := float32(0.0), input.MaxFraction
	index := -1
	output := CastOutput{}

	for edgeIndex := 0; edgeIndex < shape.Count; edgeIndex++ {
		vertex := Sub(shape.Vertices[edgeIndex], base)
		numerator := Dot(shape.Normals[edgeIndex], Sub(vertex, p1))
		denominator := Dot(shape.Normals[edgeIndex], d)

		if denominator == 0.0 {
			if numerator < 0.0 {
				return output
			}
		} else {
			if denominator < 0.0 && numerator < lower*denominator {
				lower = numerator / denominator
				index = edgeIndex
			} else if denominator > 0.0 && numerator < upper*denominator {
				upper = numerator / denominator
			}
		}

		if upper < lower {
			return output
		}
	}

	if index >= 0 {
		output.Fraction = lower
		output.Normal = shape.Normals[index]
		output.Point = MulAdd(input.Origin, lower, d)
		output.Hit = true
	} else {
		output.Point = input.Origin
		output.Hit = true
	}

	return output
}

func linearSlop() float32 {
	return 0.005 * GetLengthUnitsPerMeter()
}

func ComputeHull(points []Vec2) Hull {
	hull := Hull{}
	count := len(points)
	if count < 3 || count > MaxPolygonVertices {
		return hull
	}

	tol := 16.0 * linearSlop() * linearSlop()
	var uniqueBuf [MaxPolygonVertices]Vec2
	unique := uniqueBuf[:0]
	for i := 0; i < count; i++ {
		vi := points[i]
		isUnique := true
		for j := 0; j < len(unique); j++ {
			if DistanceSquared(vi, unique[j]) < tol {
				isUnique = false
				break
			}
		}
		if isUnique {
			unique = append(unique, vi)
		}
	}
	if len(unique) < 3 {
		return hull
	}

	slices.SortFunc(unique, func(a, b Vec2) int {
		if c := cmp.Compare(a.X, b.X); c != 0 {
			return c
		}
		return cmp.Compare(a.Y, b.Y)
	})

	cross3 := func(o, a, b Vec2) float32 {
		return Cross(Sub(a, o), Sub(b, o))
	}

	var chainBuf [MaxPolygonVertices * 2]Vec2
	chain := chainBuf[:0]
	for _, p := range unique {
		for len(chain) >= 2 {
			n := len(chain)
			if cross3(chain[n-2], chain[n-1], p) > linearSlop() {
				break
			}
			chain = chain[:n-1]
		}
		chain = append(chain, p)
	}

	lowerSize := len(chain)
	for i := len(unique) - 2; i >= 0; i-- {
		p := unique[i]
		for len(chain) > lowerSize {
			n := len(chain)
			if cross3(chain[n-2], chain[n-1], p) > linearSlop() {
				break
			}
			chain = chain[:n-1]
		}
		chain = append(chain, p)
	}

	if len(chain) > 1 {
		chain = chain[:len(chain)-1]
	}
	if len(chain) < 3 {
		return Hull{}
	}

	// Remove near-collinear points.
	searching := true
	for searching && len(chain) > 2 {
		searching = false
		for i := 0; i < len(chain); i++ {
			i1 := i
			i2 := (i + 1) % len(chain)
			i3 := (i + 2) % len(chain)
			s1, s2, s3 := chain[i1], chain[i2], chain[i3]
			r := Normalize(Sub(s3, s1))
			distance := Cross(Sub(s2, s1), r)
			if distance <= 2.0*linearSlop() {
				chain = append(chain[:i2], chain[i2+1:]...)
				searching = true
				break
			}
		}
	}

	if len(chain) < 3 || len(chain) > MaxPolygonVertices {
		return Hull{}
	}

	hull.Count = len(chain)
	for i := 0; i < hull.Count; i++ {
		hull.Points[i] = chain[i]
	}
	return hull
}

func ValidateHull(hull *Hull) bool {
	if hull.Count < 3 || hull.Count > MaxPolygonVertices {
		return false
	}

	for i := 0; i < hull.Count; i++ {
		i1 := i
		i2 := 0
		if i < hull.Count-1 {
			i2 = i1 + 1
		}
		p := hull.Points[i1]
		e := Normalize(Sub(hull.Points[i2], p))

		for j := 0; j < hull.Count; j++ {
			if j == i1 || j == i2 {
				continue
			}
			distance := Cross(Sub(hull.Points[j], p), e)
			if distance >= 0.0 {
				return false
			}
		}
	}

	ls := linearSlop()
	for i := 0; i < hull.Count; i++ {
		i1 := i
		i2 := (i + 1) % hull.Count
		i3 := (i + 2) % hull.Count
		p1, p2, p3 := hull.Points[i1], hull.Points[i2], hull.Points[i3]
		e := Normalize(Sub(p3, p1))
		distance := Cross(Sub(p2, p1), e)
		if distance <= ls {
			return false
		}
	}
	return true
}
