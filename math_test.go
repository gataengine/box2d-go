package box2d_test

import (
	"math"

	"github.com/gataengine/box2d-go"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

const atanTol float32 = 0.00004

var _ = Describe("Math", func() {
	It("matches upstream test_math.c behavior", func() {
		for t := float32(-10.0); t < 10.0; t += 0.01 {
			angle := box2d.Pi * t
			r := box2d.MakeRot(angle)
			c := float32(math.Cos(float64(angle)))
			s := float32(math.Sin(float64(angle)))

			Expect(box2d.AbsFloat(r.C - c)).To(BeNumerically("<=", float32(0.002)))
			Expect(box2d.AbsFloat(r.S - s)).To(BeNumerically("<=", float32(0.002)))

			xn := box2d.UnwindAngle(angle)
			Expect(xn).To(BeNumerically(">=", -box2d.Pi))
			Expect(xn).To(BeNumerically("<=", box2d.Pi))

			a := box2d.Atan2(s, c)
			Expect(box2d.IsValidFloat(a)).To(BeTrue())

			diff := box2d.AbsFloat(a - xn)
			if diff > box2d.Pi {
				diff -= 2.0 * box2d.Pi
			}
			Expect(box2d.AbsFloat(diff)).To(BeNumerically("<=", atanTol))
		}

		for y := float32(-1.0); y <= 1.0; y += 0.01 {
			for x := float32(-1.0); x <= 1.0; x += 0.01 {
				a1 := box2d.Atan2(y, x)
				a2 := float32(math.Atan2(float64(y), float64(x)))
				diff := box2d.AbsFloat(a1 - a2)
				Expect(box2d.IsValidFloat(a1)).To(BeTrue())
				Expect(diff).To(BeNumerically("<=", atanTol))
			}
		}

		for _, c := range []struct {
			y float32
			x float32
		}{
			{y: 1.0, x: 0.0},
			{y: -1.0, x: 0.0},
			{y: 0.0, x: 1.0},
			{y: 0.0, x: -1.0},
			{y: 0.0, x: 0.0},
		} {
			a1 := box2d.Atan2(c.y, c.x)
			a2 := float32(math.Atan2(float64(c.y), float64(c.x)))
			diff := box2d.AbsFloat(a1 - a2)
			Expect(box2d.IsValidFloat(a1)).To(BeTrue())
			Expect(diff).To(BeNumerically("<=", atanTol))
		}

		zero := box2d.Vec2Zero
		one := box2d.Vec2{X: 1.0, Y: 1.0}
		two := box2d.Vec2{X: 2.0, Y: 2.0}

		v := box2d.Add(one, two)
		Expect(v).To(Equal(box2d.Vec2{X: 3.0, Y: 3.0}))

		v = box2d.Sub(zero, two)
		Expect(v).To(Equal(box2d.Vec2{X: -2.0, Y: -2.0}))

		v = box2d.Add(two, two)
		Expect(v).NotTo(Equal(box2d.Vec2{X: 5.0, Y: 5.0}))

		transform1 := box2d.Transform{
			P: box2d.Vec2{X: -2.0, Y: 3.0},
			Q: box2d.MakeRot(1.0),
		}
		transform2 := box2d.Transform{
			P: box2d.Vec2{X: 1.0, Y: 0.0},
			Q: box2d.MakeRot(-2.0),
		}

		transform := box2d.MulTransforms(transform2, transform1)
		v = box2d.TransformPoint(transform2, box2d.TransformPoint(transform1, two))
		u := box2d.TransformPoint(transform, two)

		epsilon := float32(1.1920928955078125e-07)
		Expect(box2d.AbsFloat(u.X - v.X)).To(BeNumerically("<=", 10.0*epsilon))
		Expect(box2d.AbsFloat(u.Y - v.Y)).To(BeNumerically("<=", 10.0*epsilon))

		v = box2d.TransformPoint(transform1, two)
		v = box2d.InvTransformPoint(transform1, v)
		Expect(box2d.AbsFloat(v.X - two.X)).To(BeNumerically("<=", 8.0*epsilon))
		Expect(box2d.AbsFloat(v.Y - two.Y)).To(BeNumerically("<=", 8.0*epsilon))

		v = box2d.Normalize(box2d.Vec2{X: 0.2, Y: -0.5})
		for y := float32(-1.0); y <= 1.0; y += 0.01 {
			for x := float32(-1.0); x <= 1.0; x += 0.01 {
				if x == 0.0 && y == 0.0 {
					continue
				}

				u = box2d.Normalize(box2d.Vec2{X: x, Y: y})
				r := box2d.ComputeRotationBetweenUnitVectors(v, u)
				w := box2d.RotateVector(r, v)
				Expect(box2d.AbsFloat(w.X - u.X)).To(BeNumerically("<=", 4.0*epsilon))
				Expect(box2d.AbsFloat(w.Y - u.Y)).To(BeNumerically("<=", 4.0*epsilon))
			}
		}

		q1 := box2d.RotIdentity
		q2 := box2d.MakeRot(0.5 * box2d.Pi)
		n := 100
		for i := range n + 1 {
			alpha := float32(i) / float32(n)
			q := box2d.NLerp(q1, q2, alpha)
			angle := box2d.RotGetAngle(q)
			Expect(box2d.AbsFloat(alpha*0.5*box2d.Pi - angle)).To(BeNumerically("<=", 5.0*box2d.Pi/180.0))
		}

		baseAngle := float32(0.75) * box2d.Pi
		q1 = box2d.MakeRot(baseAngle)
		for t := float32(-10.0); t < 10.0; t += 0.01 {
			angle := box2d.Pi * t
			q2 = box2d.MakeRot(angle)

			relativeAngle := box2d.RelativeAngle(q1, q2)
			unwoundAngle := box2d.UnwindAngle(angle - baseAngle)
			tolerance := float32(0.1) * box2d.Pi / 180.0
			Expect(box2d.AbsFloat(relativeAngle - unwoundAngle)).To(BeNumerically("<=", tolerance))
		}
	})
})
