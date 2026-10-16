package box2d_test

import (
	"math"

	"github.com/gataengine/box2d-go"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

var _ = Describe("Collision Shape Primitives", func() {
	It("matches upstream test_shape.c behavior", func() {
		capsule := box2d.Capsule{
			Center1: box2d.Vec2{X: -1.0, Y: 0.0},
			Center2: box2d.Vec2{X: 1.0, Y: 0.0},
			Radius:  1.0,
		}
		circle := box2d.Circle{
			Center: box2d.Vec2{X: 1.0, Y: 0.0},
			Radius: 1.0,
		}
		box := box2d.MakeBox(1.0, 1.0)
		segment := box2d.Segment{
			Point1: box2d.Vec2{X: 0.0, Y: 1.0},
			Point2: box2d.Vec2{X: 0.0, Y: -1.0},
		}

		eps := float32(1.1920928955078125e-07)

		{
			md := box2d.ComputeCircleMass(&circle, 1.0)
			Expect(box2d.AbsFloat(md.Mass - box2d.Pi)).To(BeNumerically("<=", eps))
			Expect(md.Center.X).To(BeNumerically("==", float32(1.0)))
			Expect(md.Center.Y).To(BeNumerically("==", float32(0.0)))
			Expect(box2d.AbsFloat(md.RotationalInertia - 0.5*box2d.Pi)).To(BeNumerically("<=", eps))
		}

		{
			radius := capsule.Radius
			length := box2d.Distance(capsule.Center1, capsule.Center2)
			md := box2d.ComputeCapsuleMass(&capsule, 1.0)

			r := box2d.MakeBox(radius, radius+0.5*length)
			mdr := box2d.ComputePolygonMass(&r, 1.0)

			points := make([]box2d.Vec2, 8)
			d := box2d.Pi / float32(4-1)
			angle := float32(-0.5) * box2d.Pi
			for i := 0; i < 4; i++ {
				points[i].X = 1.0 + radius*float32(math.Cos(float64(angle)))
				points[i].Y = radius * float32(math.Sin(float64(angle)))
				angle += d
			}

			angle = float32(0.5) * box2d.Pi
			for i := 4; i < 8; i++ {
				points[i].X = -1.0 + radius*float32(math.Cos(float64(angle)))
				points[i].Y = radius * float32(math.Sin(float64(angle)))
				angle += d
			}

			hull := box2d.ComputeHull(points)
			ac := box2d.MakePolygon(&hull, 0.0)
			ma := box2d.ComputePolygonMass(&ac, 1.0)

			Expect(ma.Mass).To(BeNumerically("<", md.Mass))
			Expect(md.Mass).To(BeNumerically("<", mdr.Mass))
			Expect(ma.RotationalInertia).To(BeNumerically("<", md.RotationalInertia))
			Expect(md.RotationalInertia).To(BeNumerically("<", mdr.RotationalInertia))
		}

		{
			md := box2d.ComputePolygonMass(&box, 1.0)
			Expect(box2d.AbsFloat(md.Mass - 4.0)).To(BeNumerically("<=", eps))
			Expect(box2d.AbsFloat(md.Center.X)).To(BeNumerically("<=", eps))
			Expect(box2d.AbsFloat(md.Center.Y)).To(BeNumerically("<=", eps))
			Expect(box2d.AbsFloat(md.RotationalInertia - 8.0/3.0)).To(BeNumerically("<=", 2.0*eps))
		}

		{
			b := box2d.ComputeCircleAABB(&circle, box2d.TransformIdentity)
			Expect(box2d.AbsFloat(b.LowerBound.X)).To(BeNumerically("<=", eps))
			Expect(box2d.AbsFloat(b.LowerBound.Y + 1.0)).To(BeNumerically("<=", eps))
			Expect(box2d.AbsFloat(b.UpperBound.X - 2.0)).To(BeNumerically("<=", eps))
			Expect(box2d.AbsFloat(b.UpperBound.Y - 1.0)).To(BeNumerically("<=", eps))
		}

		{
			b := box2d.ComputePolygonAABB(&box, box2d.TransformIdentity)
			Expect(box2d.AbsFloat(b.LowerBound.X + 1.0)).To(BeNumerically("<=", eps))
			Expect(box2d.AbsFloat(b.LowerBound.Y + 1.0)).To(BeNumerically("<=", eps))
			Expect(box2d.AbsFloat(b.UpperBound.X - 1.0)).To(BeNumerically("<=", eps))
			Expect(box2d.AbsFloat(b.UpperBound.Y - 1.0)).To(BeNumerically("<=", eps))
		}

		{
			b := box2d.ComputeSegmentAABB(&segment, box2d.TransformIdentity)
			Expect(box2d.AbsFloat(b.LowerBound.X)).To(BeNumerically("<=", eps))
			Expect(box2d.AbsFloat(b.LowerBound.Y + 1.0)).To(BeNumerically("<=", eps))
			Expect(box2d.AbsFloat(b.UpperBound.X)).To(BeNumerically("<=", eps))
			Expect(box2d.AbsFloat(b.UpperBound.Y - 1.0)).To(BeNumerically("<=", eps))
		}

		p1 := box2d.Vec2{X: 0.5, Y: 0.5}
		p2 := box2d.Vec2{X: 4.0, Y: -4.0}
		Expect(box2d.PointInCircle(&circle, p1)).To(BeTrue())
		Expect(box2d.PointInCircle(&circle, p2)).To(BeFalse())
		Expect(box2d.PointInPolygon(&box, p1)).To(BeTrue())
		Expect(box2d.PointInPolygon(&box, p2)).To(BeFalse())

		input := box2d.RayCastInput{
			Origin:      box2d.Vec2{X: -4.0, Y: 0.0},
			Translation: box2d.Vec2{X: 8.0, Y: 0.0},
			MaxFraction: 1.0,
		}

		{
			output := box2d.RayCastCircle(&circle, &input)
			Expect(output.Hit).To(BeTrue())
			Expect(box2d.AbsFloat(output.Normal.X + 1.0)).To(BeNumerically("<=", eps))
			Expect(box2d.AbsFloat(output.Normal.Y)).To(BeNumerically("<=", eps))
			Expect(box2d.AbsFloat(output.Fraction - 0.5)).To(BeNumerically("<=", eps))
		}

		{
			output := box2d.RayCastPolygon(&box, &input)
			Expect(output.Hit).To(BeTrue())
			Expect(box2d.AbsFloat(output.Normal.X + 1.0)).To(BeNumerically("<=", eps))
			Expect(box2d.AbsFloat(output.Normal.Y)).To(BeNumerically("<=", eps))
			Expect(box2d.AbsFloat(output.Fraction - 3.0/8.0)).To(BeNumerically("<=", eps))
		}

		{
			output := box2d.RayCastSegment(&segment, &input, true)
			Expect(output.Hit).To(BeTrue())
			Expect(box2d.AbsFloat(output.Normal.X + 1.0)).To(BeNumerically("<=", eps))
			Expect(box2d.AbsFloat(output.Normal.Y)).To(BeNumerically("<=", eps))
			Expect(box2d.AbsFloat(output.Fraction - 0.5)).To(BeNumerically("<=", eps))
		}

		{
			Expect(box2d.IsValidRay(box2d.RayCastInput{
				Origin:      box2d.Vec2{X: -4.0, Y: 0.0},
				Translation: box2d.Vec2{X: 8.0, Y: 0.0},
				MaxFraction: 99999.0,
			})).To(BeTrue())
			Expect(box2d.IsValidRay(box2d.RayCastInput{
				Origin:      box2d.Vec2{X: -4.0, Y: 0.0},
				Translation: box2d.Vec2{X: 8.0, Y: 0.0},
				MaxFraction: 100000.0,
			})).To(BeFalse())
		}

		{
			Expect(box2d.PointInCapsule(&capsule, box2d.Vec2{X: 2.0, Y: 0.0})).To(BeTrue())
			Expect(box2d.PointInCapsule(&capsule, box2d.Vec2{X: 2.0001, Y: 0.0})).To(BeFalse())
		}

		{
			output := box2d.RayCastCapsule(&capsule, &input)
			Expect(output.Hit).To(BeTrue())
			Expect(box2d.AbsFloat(output.Normal.X + 1.0)).To(BeNumerically("<=", eps))
			Expect(box2d.AbsFloat(output.Normal.Y)).To(BeNumerically("<=", eps))
			Expect(box2d.AbsFloat(output.Fraction - 0.25)).To(BeNumerically("<=", eps))
			Expect(box2d.AbsFloat(output.Point.X + 2.0)).To(BeNumerically("<=", eps))
			Expect(box2d.AbsFloat(output.Point.Y)).To(BeNumerically("<=", eps))
		}
	})
})
