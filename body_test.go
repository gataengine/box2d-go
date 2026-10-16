package box2d_test

import (
	"github.com/gataengine/box2d-go"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

var _ = Describe("Body Lifecycle", func() {
	It("ports test_world.c TestIsValid with stale id checks", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())
		Expect(world.IsValid()).To(BeTrue())
		worldID := world.ID()

		bodyDef := box2d.DefaultBodyDef()
		body1, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())
		Expect(body1.IsValid()).To(BeTrue())
		bodyID1 := body1.ID()
		Expect(box2d.IsBodyIDValid(bodyID1)).To(BeTrue())

		body2, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())
		Expect(body2.IsValid()).To(BeTrue())
		bodyID2 := body2.ID()
		Expect(box2d.IsBodyIDValid(bodyID2)).To(BeTrue())

		Expect(world.DestroyBody(bodyID1)).To(Succeed())
		Expect(body1.IsValid()).To(BeFalse())
		Expect(box2d.IsBodyIDValid(bodyID1)).To(BeFalse())
		Expect(box2d.IsBodyIDValid(bodyID2)).To(BeTrue())

		Expect(world.Destroy()).To(Succeed())
		Expect(world.IsValid()).To(BeFalse())
		Expect(box2d.IsWorldIDValid(worldID)).To(BeFalse())
		Expect(box2d.IsBodyIDValid(bodyID2)).To(BeFalse())
		Expect(box2d.IsBodyIDValid(bodyID1)).To(BeFalse())
	})

	It("tracks body counters and supports world-managed destroy", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody

		bodies := make([]*box2d.Body, 0, 10)
		for range 10 {
			b, err := world.CreateBody(def)
			Expect(err).NotTo(HaveOccurred())
			bodies = append(bodies, b)
		}

		c, err := world.Counters()
		Expect(err).NotTo(HaveOccurred())
		Expect(c.BodyCount).To(Equal(10))

		for i := len(bodies) - 1; i >= 0; i-- {
			Expect(world.DestroyBody(bodies[i].ID())).To(Succeed())
		}

		c, err = world.Counters()
		Expect(err).NotTo(HaveOccurred())
		Expect(c.BodyCount).To(Equal(0))

		Expect(world.Destroy()).To(Succeed())
	})

	It("rejects uninitialized body definitions", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		_, err = world.CreateBody(box2d.BodyDef{})
		Expect(err).To(MatchError(box2d.ErrInvalidBodyDef))
	})

	It("supports velocity/damping/gravity-scale body dynamics properties", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		Expect(body.SetLinearVelocity(box2d.Vec2{X: 3.0, Y: -2.0})).To(Succeed())
		Expect(body.SetAngularVelocity(1.25)).To(Succeed())
		Expect(body.SetLinearDamping(0.4)).To(Succeed())
		Expect(body.SetAngularDamping(0.6)).To(Succeed())
		Expect(body.SetGravityScale(2.0)).To(Succeed())

		lv, err := body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(lv).To(Equal(box2d.Vec2{X: 3.0, Y: -2.0}))

		av, err := body.GetAngularVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(av).To(Equal(float32(1.25)))

		ld, err := body.GetLinearDamping()
		Expect(err).NotTo(HaveOccurred())
		Expect(ld).To(Equal(float32(0.4)))

		ad, err := body.GetAngularDamping()
		Expect(err).NotTo(HaveOccurred())
		Expect(ad).To(Equal(float32(0.6)))

		gs, err := body.GetGravityScale()
		Expect(err).NotTo(HaveOccurred())
		Expect(gs).To(Equal(float32(2.0)))
	})

	It("applies force/torque and clears accumulators after each world step", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())
		Expect(body.SetMassData(box2d.MassData{Mass: 1, Center: box2d.Vec2Zero, RotationalInertia: 1})).To(Succeed())

		Expect(body.ApplyForceToCenter(box2d.Vec2{X: 6, Y: 0}, false)).To(Succeed())
		Expect(body.ApplyTorque(4, false)).To(Succeed())
		Expect(world.Step(0.5, 1)).To(Succeed())

		v1, err := body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(v1.X).To(BeNumerically("~", 3.0, 1e-6))
		Expect(v1.Y).To(BeNumerically("~", 0.0, 1e-6))

		w1, err := body.GetAngularVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(w1).To(BeNumerically("~", 2.0, 1e-6))

		Expect(world.Step(0.5, 1)).To(Succeed())

		v2, err := body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(v2.X).To(BeNumerically("~", 3.0, 1e-6))
		Expect(v2.Y).To(BeNumerically("~", 0.0, 1e-6))

		w2, err := body.GetAngularVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(w2).To(BeNumerically("~", 2.0, 1e-6))
	})

	It("supports point-force application and explicit force clearing", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())
		Expect(body.SetMassData(box2d.MassData{Mass: 1, Center: box2d.Vec2Zero, RotationalInertia: 1})).To(Succeed())

		Expect(body.ApplyForce(box2d.Vec2{X: 2, Y: 0}, box2d.Vec2{X: 0, Y: 1}, false)).To(Succeed())
		Expect(body.ClearForces()).To(Succeed())
		Expect(world.Step(1.0, 1)).To(Succeed())

		v, err := body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(v.X).To(BeNumerically("~", 0.0, 1e-6))
		Expect(v.Y).To(BeNumerically("~", 0.0, 1e-6))

		w, err := body.GetAngularVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(w).To(BeNumerically("~", 0.0, 1e-6))
	})

	It("scales force response by shape-derived inverse mass", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 2.0
		_, err = body.CreatePolygonShape(shapeDef, box2d.MakeSquare(0.5))
		Expect(err).NotTo(HaveOccurred())

		Expect(body.ApplyForceToCenter(box2d.Vec2{X: 4, Y: 0}, false)).To(Succeed())
		Expect(world.Step(1.0, 1)).To(Succeed())

		v, err := body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(v.X).To(BeNumerically("~", 2.0, 1e-5))
		Expect(v.Y).To(BeNumerically("~", 0.0, 1e-5))
	})

	It("combines gravity scale and applied force during integration", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2{X: 0, Y: -10}
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		def.GravityScale = 0.5
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 2.0
		_, err = body.CreatePolygonShape(shapeDef, box2d.MakeSquare(0.5))
		Expect(err).NotTo(HaveOccurred())

		Expect(body.ApplyForceToCenter(box2d.Vec2{X: 0, Y: 4}, false)).To(Succeed())
		Expect(world.Step(1.0, 1)).To(Succeed())

		v, err := body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(v.X).To(BeNumerically("~", 0.0, 1e-5))
		Expect(v.Y).To(BeNumerically("~", -3.0, 1e-5))
	})

	It("matches wake behavior for applied forces on sleeping bodies", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		def.IsAwake = false
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())
		Expect(body.SetMassData(box2d.MassData{Mass: 1, Center: box2d.Vec2Zero, RotationalInertia: 1})).To(Succeed())

		Expect(body.ApplyForceToCenter(box2d.Vec2{X: 5, Y: 0}, false)).To(Succeed())
		Expect(world.Step(1.0, 1)).To(Succeed())

		v, err := body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(v.X).To(BeNumerically("~", 0.0, 1e-6))
		Expect(v.Y).To(BeNumerically("~", 0.0, 1e-6))

		Expect(body.ApplyForceToCenter(box2d.Vec2{X: 5, Y: 0}, true)).To(Succeed())
		Expect(world.Step(1.0, 1)).To(Succeed())

		v, err = body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(v.X).To(BeNumerically("~", 5.0, 1e-6))
		Expect(v.Y).To(BeNumerically("~", 0.0, 1e-6))
	})

	It("supports awake/sleep toggles and sleep threshold accessors", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())
		Expect(body.SetMassData(box2d.MassData{Mass: 1, Center: box2d.Vec2Zero, RotationalInertia: 1})).To(Succeed())

		awake, err := body.IsAwake()
		Expect(err).NotTo(HaveOccurred())
		Expect(awake).To(BeTrue())

		sleepEnabled, err := body.IsSleepEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(sleepEnabled).To(BeTrue())

		sleepThreshold, err := body.GetSleepThreshold()
		Expect(err).NotTo(HaveOccurred())
		Expect(sleepThreshold).To(BeNumerically("~", def.SleepThreshold, 1e-6))

		Expect(body.SetAwake(false)).To(Succeed())
		awake, err = body.IsAwake()
		Expect(err).NotTo(HaveOccurred())
		Expect(awake).To(BeFalse())

		Expect(body.ApplyForceToCenter(box2d.Vec2{X: 5, Y: 0}, false)).To(Succeed())
		Expect(world.Step(1.0, 1)).To(Succeed())

		v, err := body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(v.X).To(BeNumerically("~", 0.0, 1e-6))
		Expect(v.Y).To(BeNumerically("~", 0.0, 1e-6))

		Expect(body.EnableSleep(false)).To(Succeed())
		sleepEnabled, err = body.IsSleepEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(sleepEnabled).To(BeFalse())

		awake, err = body.IsAwake()
		Expect(err).NotTo(HaveOccurred())
		Expect(awake).To(BeTrue())

		Expect(body.ApplyForceToCenter(box2d.Vec2{X: 5, Y: 0}, false)).To(Succeed())
		Expect(world.Step(1.0, 1)).To(Succeed())

		v, err = body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(v.X).To(BeNumerically("~", 5.0, 1e-6))
		Expect(v.Y).To(BeNumerically("~", 0.0, 1e-6))

		Expect(body.SetSleepThreshold(0.25)).To(Succeed())
		sleepThreshold, err = body.GetSleepThreshold()
		Expect(err).NotTo(HaveOccurred())
		Expect(sleepThreshold).To(BeNumerically("~", 0.25, 1e-6))
	})

	It("wakes overlapping bodies via WakeTouching", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		defA := box2d.DefaultBodyDef()
		defA.Type = box2d.DynamicBody
		bodyA, err := world.CreateBody(defA)
		Expect(err).NotTo(HaveOccurred())

		defB := box2d.DefaultBodyDef()
		defB.Type = box2d.DynamicBody
		defB.Position = box2d.Vec2{X: 0.5, Y: 0}
		defB.IsAwake = false
		bodyB, err := world.CreateBody(defB)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		_, err = bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		awakeB, err := bodyB.IsAwake()
		Expect(err).NotTo(HaveOccurred())
		Expect(awakeB).To(BeFalse())

		Expect(bodyA.WakeTouching()).To(Succeed())

		awakeB, err = bodyB.IsAwake()
		Expect(err).NotTo(HaveOccurred())
		Expect(awakeB).To(BeTrue())

		Expect(bodyA.Destroy()).To(Succeed())
		Expect(bodyA.WakeTouching()).To(MatchError(box2d.ErrBodyInvalid))
	})

	It("ignores force and impulse while disabled until re-enabled", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())
		Expect(body.SetMassData(box2d.MassData{Mass: 1, Center: box2d.Vec2Zero, RotationalInertia: 1})).To(Succeed())

		enabled, err := body.IsEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(enabled).To(BeTrue())

		Expect(body.Disable()).To(Succeed())
		enabled, err = body.IsEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(enabled).To(BeFalse())

		Expect(body.ApplyForceToCenter(box2d.Vec2{X: 2, Y: 0}, true)).To(Succeed())
		Expect(body.ApplyLinearImpulseToCenter(box2d.Vec2{X: 3, Y: 0}, true)).To(Succeed())
		Expect(world.Step(1.0, 1)).To(Succeed())

		v, err := body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(v.X).To(BeNumerically("~", 0.0, 1e-6))
		Expect(v.Y).To(BeNumerically("~", 0.0, 1e-6))

		Expect(body.Enable()).To(Succeed())
		enabled, err = body.IsEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(enabled).To(BeTrue())

		Expect(body.ApplyLinearImpulseToCenter(box2d.Vec2{X: 3, Y: 0}, false)).To(Succeed())
		v, err = body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(v.X).To(BeNumerically("~", 3.0, 1e-6))
		Expect(v.Y).To(BeNumerically("~", 0.0, 1e-6))
	})

	It("supports runtime motion lock updates", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		def.LinearVelocity = box2d.Vec2{X: 2, Y: 3}
		def.AngularVelocity = 1
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())
		Expect(body.SetMassData(box2d.MassData{Mass: 1, Center: box2d.Vec2Zero, RotationalInertia: 1})).To(Succeed())

		locks := box2d.MotionLocks{LinearX: true, LinearY: true, AngularZ: true}
		Expect(body.SetMotionLocks(locks)).To(Succeed())

		gotLocks, err := body.GetMotionLocks()
		Expect(err).NotTo(HaveOccurred())
		Expect(gotLocks).To(Equal(locks))

		Expect(world.Step(1.0, 1)).To(Succeed())
		v, err := body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(v.X).To(BeNumerically("~", 0.0, 1e-6))
		Expect(v.Y).To(BeNumerically("~", 0.0, 1e-6))

		w, err := body.GetAngularVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(w).To(BeNumerically("~", 0.0, 1e-6))

		Expect(body.SetMotionLocks(box2d.MotionLocks{})).To(Succeed())
		Expect(body.SetLinearVelocity(box2d.Vec2{X: 2, Y: 3})).To(Succeed())
		Expect(body.SetAngularVelocity(1)).To(Succeed())
		Expect(world.Step(1.0, 1)).To(Succeed())

		v, err = body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(v.X).To(BeNumerically("~", 2.0, 1e-6))
		Expect(v.Y).To(BeNumerically("~", 3.0, 1e-6))

		w, err = body.GetAngularVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(w).To(BeNumerically("~", 1.0, 1e-6))
	})

	It("solves target transform into linear and angular velocity", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())
		Expect(body.SetMassData(box2d.MassData{
			Mass:              1,
			Center:            box2d.Vec2{X: 1, Y: 0},
			RotationalInertia: 1,
		})).To(Succeed())

		target := box2d.Transform{
			P: box2d.Vec2{X: 2, Y: 0},
			Q: box2d.MakeRot(box2d.Pi / 2),
		}
		Expect(body.SetTargetTransform(target, 0.5, false)).To(Succeed())

		v, err := body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(v.X).To(BeNumerically("~", 2.0, 1e-6))
		Expect(v.Y).To(BeNumerically("~", 2.0, 1e-6))

		w, err := body.GetAngularVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(w).To(BeNumerically("~", box2d.Pi, 1e-5))
	})

	It("applies sleepy early-out for target transform on sleeping bodies", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		def.IsAwake = false
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())
		Expect(body.SetMassData(box2d.MassData{
			Mass:              1,
			Center:            box2d.Vec2Zero,
			RotationalInertia: 1,
		})).To(Succeed())

		Expect(body.SetSleepThreshold(1.0)).To(Succeed())
		smallTarget := box2d.Transform{
			P: box2d.Vec2{X: 0.2, Y: 0},
			Q: box2d.MakeRot(0.1),
		}
		Expect(body.SetTargetTransform(smallTarget, 1.0, true)).To(Succeed())

		awake, err := body.IsAwake()
		Expect(err).NotTo(HaveOccurred())
		Expect(awake).To(BeFalse())

		v, err := body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(v.X).To(BeNumerically("~", 0.0, 1e-6))
		Expect(v.Y).To(BeNumerically("~", 0.0, 1e-6))

		w, err := body.GetAngularVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(w).To(BeNumerically("~", 0.0, 1e-6))

		largeTarget := box2d.Transform{
			P: box2d.Vec2{X: 2, Y: 0},
			Q: box2d.MakeRot(0.5),
		}
		Expect(body.SetTargetTransform(largeTarget, 1.0, true)).To(Succeed())

		awake, err = body.IsAwake()
		Expect(err).NotTo(HaveOccurred())
		Expect(awake).To(BeTrue())

		v, err = body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(v.X).To(BeNumerically("~", 2.0, 1e-6))
		Expect(v.Y).To(BeNumerically("~", 0.0, 1e-6))

		w, err = body.GetAngularVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(w).To(BeNumerically("~", 0.5, 1e-6))
	})

	It("uses body extent in target-transform sleep gating", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		def.IsAwake = false
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.UpdateBodyMass = true
		_, err = body.CreateCircleShape(shapeDef, box2d.Circle{
			Center: box2d.Vec2Zero,
			Radius: 10.0,
		})
		Expect(err).NotTo(HaveOccurred())

		Expect(body.SetSleepThreshold(1.0)).To(Succeed())
		target := box2d.Transform{
			P: box2d.Vec2Zero,
			Q: box2d.MakeRot(0.2),
		}
		Expect(body.SetTargetTransform(target, 1.0, true)).To(Succeed())

		awake, err := body.IsAwake()
		Expect(err).NotTo(HaveOccurred())
		Expect(awake).To(BeTrue())

		w, err := body.GetAngularVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(w).To(BeNumerically("~", 0.2, 1e-6))
	})

	It("supports bullet flag accessors", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		isBullet, err := body.IsBullet()
		Expect(err).NotTo(HaveOccurred())
		Expect(isBullet).To(BeFalse())

		Expect(body.SetBullet(true)).To(Succeed())
		isBullet, err = body.IsBullet()
		Expect(err).NotTo(HaveOccurred())
		Expect(isBullet).To(BeTrue())

		Expect(body.SetBullet(false)).To(Succeed())
		isBullet, err = body.IsBullet()
		Expect(err).NotTo(HaveOccurred())
		Expect(isBullet).To(BeFalse())
	})

	It("enables body-level contact and hit events across attached shapes", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.EnableContactEvents = false
		shapeDef.EnableHitEvents = false

		circle, err := body.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())
		poly, err := body.CreatePolygonShape(shapeDef, box2d.MakeSquare(0.5))
		Expect(err).NotTo(HaveOccurred())

		worldID, err := body.GetWorld()
		Expect(err).NotTo(HaveOccurred())
		Expect(worldID).To(Equal(world.ID()))

		contactEnabled, err := circle.ContactEventsEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEnabled).To(BeFalse())
		hitEnabled, err := circle.HitEventsEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(hitEnabled).To(BeFalse())

		Expect(body.EnableContactEvents(true)).To(Succeed())
		Expect(body.EnableHitEvents(true)).To(Succeed())

		contactEnabled, err = circle.ContactEventsEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEnabled).To(BeTrue())
		hitEnabled, err = circle.HitEventsEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(hitEnabled).To(BeTrue())

		contactEnabled, err = poly.ContactEventsEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEnabled).To(BeTrue())
		hitEnabled, err = poly.HitEventsEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(hitEnabled).To(BeTrue())

		Expect(body.EnableContactEvents(false)).To(Succeed())
		Expect(body.EnableHitEvents(false)).To(Succeed())

		contactEnabled, err = poly.ContactEventsEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEnabled).To(BeFalse())
		hitEnabled, err = poly.HitEventsEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(hitEnabled).To(BeFalse())
	})

	It("provides shape enumeration and body AABB helpers", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		def.Position = box2d.Vec2{X: 2, Y: 3}
		def.Rotation = box2d.MakeRot(box2d.Pi / 2)
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		Expect(body.SetName("hero")).To(Succeed())
		Expect(body.SetUserData(1234)).To(Succeed())

		name, err := body.GetName()
		Expect(err).NotTo(HaveOccurred())
		Expect(name).To(Equal("hero"))

		userData, err := body.GetUserData()
		Expect(err).NotTo(HaveOccurred())
		Expect(userData).To(Equal(1234))

		emptyAABB, err := body.ComputeAABB()
		Expect(err).NotTo(HaveOccurred())
		Expect(emptyAABB.LowerBound.X).To(BeNumerically("~", def.Position.X, 1e-6))
		Expect(emptyAABB.LowerBound.Y).To(BeNumerically("~", def.Position.Y, 1e-6))
		Expect(emptyAABB.UpperBound.X).To(BeNumerically("~", def.Position.X, 1e-6))
		Expect(emptyAABB.UpperBound.Y).To(BeNumerically("~", def.Position.Y, 1e-6))

		shapeDef := box2d.DefaultShapeDef()
		circle := box2d.Circle{Center: box2d.Vec2{X: 1, Y: 0}, Radius: 0.5}
		circleShape, err := body.CreateCircleShape(shapeDef, circle)
		Expect(err).NotTo(HaveOccurred())

		poly := box2d.MakeBox(0.25, 0.75)
		polyShape, err := body.CreatePolygonShape(shapeDef, poly)
		Expect(err).NotTo(HaveOccurred())

		count, err := body.GetShapeCount()
		Expect(err).NotTo(HaveOccurred())
		Expect(count).To(Equal(2))

		shapes, err := body.GetShapes()
		Expect(err).NotTo(HaveOccurred())
		Expect(shapes).To(HaveLen(2))

		shapeSet := map[uint64]bool{}
		for _, id := range shapes {
			shapeSet[box2d.StoreShapeID(id)] = true
		}
		Expect(shapeSet).To(HaveKey(box2d.StoreShapeID(circleShape.ID())))
		Expect(shapeSet).To(HaveKey(box2d.StoreShapeID(polyShape.ID())))

		gotAABB, err := body.ComputeAABB()
		Expect(err).NotTo(HaveOccurred())

		transform := box2d.Transform{P: def.Position, Q: def.Rotation}
		circleAABB := box2d.ComputeCircleAABB(&circle, transform)
		polyAABB := box2d.ComputePolygonAABB(&poly, transform)
		expectedLower := box2d.Min(circleAABB.LowerBound, polyAABB.LowerBound)
		expectedUpper := box2d.Max(circleAABB.UpperBound, polyAABB.UpperBound)

		Expect(gotAABB.LowerBound.X).To(BeNumerically("~", expectedLower.X, 1e-6))
		Expect(gotAABB.LowerBound.Y).To(BeNumerically("~", expectedLower.Y, 1e-6))
		Expect(gotAABB.UpperBound.X).To(BeNumerically("~", expectedUpper.X, 1e-6))
		Expect(gotAABB.UpperBound.Y).To(BeNumerically("~", expectedUpper.Y, 1e-6))
	})

	It("provides joint enumeration placeholders with validation semantics", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		body, err := world.CreateBody(box2d.DefaultBodyDef())
		Expect(err).NotTo(HaveOccurred())

		count, err := body.GetJointCount()
		Expect(err).NotTo(HaveOccurred())
		Expect(count).To(Equal(0))

		joints, err := body.GetJoints()
		Expect(err).NotTo(HaveOccurred())
		Expect(joints).To(BeEmpty())

		Expect(body.Destroy()).To(Succeed())

		_, err = body.GetJointCount()
		Expect(err).To(MatchError(box2d.ErrBodyInvalid))
		_, err = body.GetJoints()
		Expect(err).To(MatchError(box2d.ErrBodyInvalid))
	})

	It("enumerates joints attached to a body", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		bodyA, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())
		bodyB, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		joint, err := world.CreateDistanceJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())

		countA, err := bodyA.GetJointCount()
		Expect(err).NotTo(HaveOccurred())
		Expect(countA).To(Equal(1))
		jointsA, err := bodyA.GetJoints()
		Expect(err).NotTo(HaveOccurred())
		Expect(jointsA).To(HaveLen(1))
		Expect(box2d.JointIDEqual(jointsA[0], joint.ID())).To(BeTrue())

		countB, err := bodyB.GetJointCount()
		Expect(err).NotTo(HaveOccurred())
		Expect(countB).To(Equal(1))
		jointsB, err := bodyB.GetJoints()
		Expect(err).NotTo(HaveOccurred())
		Expect(jointsB).To(HaveLen(1))
		Expect(box2d.JointIDEqual(jointsB[0], joint.ID())).To(BeTrue())
	})

	It("provides contact query placeholders with validation semantics", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		body, err := world.CreateBody(box2d.DefaultBodyDef())
		Expect(err).NotTo(HaveOccurred())

		capacity, err := body.GetContactCapacity()
		Expect(err).NotTo(HaveOccurred())
		Expect(capacity).To(Equal(0))

		data, err := body.GetContactData(16)
		Expect(err).NotTo(HaveOccurred())
		Expect(data).To(BeEmpty())

		data, err = body.GetContactData(0)
		Expect(err).NotTo(HaveOccurred())
		Expect(data).To(BeEmpty())

		Expect(body.Destroy()).To(Succeed())

		_, err = body.GetContactCapacity()
		Expect(err).To(MatchError(box2d.ErrBodyInvalid))
		_, err = body.GetContactData(16)
		Expect(err).To(MatchError(box2d.ErrBodyInvalid))
	})

	It("reports touching contact capacity and data for overlapping bodies", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		defA := box2d.DefaultBodyDef()
		defA.Type = box2d.DynamicBody
		bodyA, err := world.CreateBody(defA)
		Expect(err).NotTo(HaveOccurred())

		defB := box2d.DefaultBodyDef()
		defB.Type = box2d.DynamicBody
		defB.Position = box2d.Vec2{X: 0.5, Y: 0}
		bodyB, err := world.CreateBody(defB)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeA, err := bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())
		shapeB, err := bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		capacity, err := bodyA.GetContactCapacity()
		Expect(err).NotTo(HaveOccurred())
		Expect(capacity).To(Equal(1))

		data, err := bodyA.GetContactData(8)
		Expect(err).NotTo(HaveOccurred())
		Expect(data).To(HaveLen(1))
		Expect(data[0].Manifold.PointCount).To(Equal(1))
		Expect(data[0].Manifold.Point1.Separation).To(BeNumerically("<", 0.0))

		a := data[0].ShapeIDA
		b := data[0].ShapeIDB
		matchForward := box2d.ShapeIDEqual(a, shapeA.ID()) && box2d.ShapeIDEqual(b, shapeB.ID())
		matchReverse := box2d.ShapeIDEqual(a, shapeB.ID()) && box2d.ShapeIDEqual(b, shapeA.ID())
		Expect(matchForward || matchReverse).To(BeTrue())
	})

	It("matches upstream massless-dynamic behavior without shapes", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2{X: 0, Y: -10}
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		mass, err := body.GetMass()
		Expect(err).NotTo(HaveOccurred())
		Expect(mass).To(BeNumerically("~", 0.0, 1e-6))

		inertia, err := body.GetRotationalInertia()
		Expect(err).NotTo(HaveOccurred())
		Expect(inertia).To(BeNumerically("~", 0.0, 1e-6))

		Expect(body.ApplyForceToCenter(box2d.Vec2{X: 10, Y: 0}, true)).To(Succeed())
		Expect(world.Step(1.0, 1)).To(Succeed())

		v, err := body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(v.X).To(BeNumerically("~", 0.0, 1e-6))
		Expect(v.Y).To(BeNumerically("~", 0.0, 1e-6))
	})

	It("computes COM-shifted inertia from multiple shapes", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1.0

		_, err = body.CreateCircleShape(shapeDef, box2d.Circle{
			Center: box2d.Vec2{X: -1, Y: 0},
			Radius: 0.5,
		})
		Expect(err).NotTo(HaveOccurred())

		_, err = body.CreateCircleShape(shapeDef, box2d.Circle{
			Center: box2d.Vec2{X: 1, Y: 0},
			Radius: 0.5,
		})
		Expect(err).NotTo(HaveOccurred())

		massData, err := body.GetMassData()
		Expect(err).NotTo(HaveOccurred())

		perMass := box2d.Pi * 0.25
		perInertia := perMass * 0.5 * 0.25
		expectedMass := 2 * perMass
		expectedInertia := 2 * (perInertia + perMass*1.0)

		Expect(massData.Center.X).To(BeNumerically("~", 0.0, 1e-5))
		Expect(massData.Center.Y).To(BeNumerically("~", 0.0, 1e-5))
		Expect(massData.Mass).To(BeNumerically("~", expectedMass, 1e-5))
		Expect(massData.RotationalInertia).To(BeNumerically("~", expectedInertia, 1e-5))
	})

	It("supports mass-data accessor/mutator round trip and validation", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		custom := box2d.MassData{
			Mass:              4,
			Center:            box2d.Vec2{X: 0.5, Y: 0},
			RotationalInertia: 2,
		}
		Expect(body.SetMassData(custom)).To(Succeed())

		got, err := body.GetMassData()
		Expect(err).NotTo(HaveOccurred())
		Expect(got.Mass).To(Equal(custom.Mass))
		Expect(got.Center).To(Equal(custom.Center))
		Expect(got.RotationalInertia).To(Equal(custom.RotationalInertia))

		mass, err := body.GetMass()
		Expect(err).NotTo(HaveOccurred())
		Expect(mass).To(Equal(custom.Mass))

		inertia, err := body.GetRotationalInertia()
		Expect(err).NotTo(HaveOccurred())
		Expect(inertia).To(Equal(custom.RotationalInertia))

		Expect(body.ApplyForce(box2d.Vec2{X: 8, Y: 0}, box2d.Vec2{X: 0.5, Y: 0}, false)).To(Succeed())
		Expect(world.Step(1.0, 1)).To(Succeed())

		v, err := body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(v.X).To(BeNumerically("~", 2.0, 1e-6))
		Expect(v.Y).To(BeNumerically("~", 0.0, 1e-6))

		Expect(body.SetMassData(box2d.MassData{Mass: -1})).To(MatchError(box2d.ErrInvalidMassData))
	})

	It("reports local/world center of mass from mass data", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		def.Position = box2d.Vec2{X: 2, Y: 3}
		def.Rotation = box2d.MakeRot(box2d.Pi / 2)
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		massData := box2d.MassData{
			Mass:              1,
			Center:            box2d.Vec2{X: 0.5, Y: -0.25},
			RotationalInertia: 1,
		}
		Expect(body.SetMassData(massData)).To(Succeed())

		localCenter, err := body.GetLocalCenterOfMass()
		Expect(err).NotTo(HaveOccurred())
		Expect(localCenter.X).To(BeNumerically("~", massData.Center.X, 1e-6))
		Expect(localCenter.Y).To(BeNumerically("~", massData.Center.Y, 1e-6))

		worldCenter, err := body.GetWorldCenterOfMass()
		Expect(err).NotTo(HaveOccurred())
		expectedWorldCenter := box2d.Add(def.Position, box2d.RotateVector(def.Rotation, massData.Center))
		Expect(worldCenter.X).To(BeNumerically("~", expectedWorldCenter.X, 1e-6))
		Expect(worldCenter.Y).To(BeNumerically("~", expectedWorldCenter.Y, 1e-6))
	})

	It("provides transform and local/world conversion helpers", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		def.Position = box2d.Vec2{X: 2, Y: -1}
		def.Rotation = box2d.MakeRot(box2d.Pi / 4)
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		massData := box2d.MassData{
			Mass:              1,
			Center:            box2d.Vec2{X: 0.5, Y: -0.25},
			RotationalInertia: 1,
		}
		Expect(body.SetMassData(massData)).To(Succeed())

		transform, err := body.GetTransform()
		Expect(err).NotTo(HaveOccurred())
		Expect(transform.P.X).To(BeNumerically("~", def.Position.X, 1e-6))
		Expect(transform.P.Y).To(BeNumerically("~", def.Position.Y, 1e-6))
		Expect(transform.Q.C).To(BeNumerically("~", def.Rotation.C, 1e-6))
		Expect(transform.Q.S).To(BeNumerically("~", def.Rotation.S, 1e-6))

		originWorld, err := body.GetWorldPoint(box2d.Vec2Zero)
		Expect(err).NotTo(HaveOccurred())
		Expect(originWorld.X).To(BeNumerically("~", transform.P.X, 1e-6))
		Expect(originWorld.Y).To(BeNumerically("~", transform.P.Y, 1e-6))

		localPoint := box2d.Vec2{X: 1, Y: -2}
		worldPoint, err := body.GetWorldPoint(localPoint)
		Expect(err).NotTo(HaveOccurred())
		expectedWorldPoint := box2d.TransformPoint(transform, localPoint)
		Expect(worldPoint.X).To(BeNumerically("~", expectedWorldPoint.X, 1e-6))
		Expect(worldPoint.Y).To(BeNumerically("~", expectedWorldPoint.Y, 1e-6))

		localRoundTrip, err := body.GetLocalPoint(worldPoint)
		Expect(err).NotTo(HaveOccurred())
		Expect(localRoundTrip.X).To(BeNumerically("~", localPoint.X, 1e-6))
		Expect(localRoundTrip.Y).To(BeNumerically("~", localPoint.Y, 1e-6))

		localVector := box2d.Vec2{X: 3, Y: -4}
		worldVector, err := body.GetWorldVector(localVector)
		Expect(err).NotTo(HaveOccurred())
		expectedWorldVector := box2d.RotateVector(transform.Q, localVector)
		Expect(worldVector.X).To(BeNumerically("~", expectedWorldVector.X, 1e-6))
		Expect(worldVector.Y).To(BeNumerically("~", expectedWorldVector.Y, 1e-6))

		localVectorRoundTrip, err := body.GetLocalVector(worldVector)
		Expect(err).NotTo(HaveOccurred())
		Expect(localVectorRoundTrip.X).To(BeNumerically("~", localVector.X, 1e-6))
		Expect(localVectorRoundTrip.Y).To(BeNumerically("~", localVector.Y, 1e-6))

		comWorld, err := body.GetWorldCenterOfMass()
		Expect(err).NotTo(HaveOccurred())
		Expect(comWorld.X).To(BeNumerically("~", box2d.TransformPoint(transform, massData.Center).X, 1e-6))
		Expect(comWorld.Y).To(BeNumerically("~", box2d.TransformPoint(transform, massData.Center).Y, 1e-6))
	})

	It("keeps COM-consistent origin motion when local center is offset", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		def.AngularVelocity = 1.0
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		Expect(body.SetMassData(box2d.MassData{
			Mass:              1,
			Center:            box2d.Vec2{X: 1, Y: 0},
			RotationalInertia: 1,
		})).To(Succeed())

		Expect(world.Step(1.0, 1)).To(Succeed())

		position, err := body.GetPosition()
		Expect(err).NotTo(HaveOccurred())
		rotation, err := body.GetRotation()
		Expect(err).NotTo(HaveOccurred())
		center, err := body.GetWorldCenterOfMass()
		Expect(err).NotTo(HaveOccurred())

		Expect(center.X).To(BeNumerically("~", 1.0, 1e-6))
		Expect(center.Y).To(BeNumerically("~", 0.0, 1e-6))
		expectedPosition := box2d.Sub(center, box2d.RotateVector(rotation, box2d.Vec2{X: 1, Y: 0}))
		Expect(position.X).To(BeNumerically("~", expectedPosition.X, 1e-6))
		Expect(position.Y).To(BeNumerically("~", expectedPosition.Y, 1e-6))
	})

	It("returns COM-aware local/world point velocities", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		def.Position = box2d.Vec2{X: 2, Y: 3}
		def.Rotation = box2d.MakeRot(box2d.Pi / 2)
		def.LinearVelocity = box2d.Vec2{X: 5, Y: -1}
		def.AngularVelocity = 2
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		Expect(body.SetMassData(box2d.MassData{
			Mass:              1,
			Center:            box2d.Vec2{X: 1, Y: 0},
			RotationalInertia: 1,
		})).To(Succeed())

		localPoint := box2d.Vec2{X: 2, Y: 0}
		vLocal, err := body.GetLocalPointVelocity(localPoint)
		Expect(err).NotTo(HaveOccurred())
		Expect(vLocal.X).To(BeNumerically("~", 3.0, 1e-6))
		Expect(vLocal.Y).To(BeNumerically("~", -1.0, 1e-6))

		worldPoint := box2d.Add(def.Position, box2d.RotateVector(def.Rotation, localPoint))
		vWorld, err := body.GetWorldPointVelocity(worldPoint)
		Expect(err).NotTo(HaveOccurred())
		Expect(vWorld.X).To(BeNumerically("~", 3.0, 1e-6))
		Expect(vWorld.Y).To(BeNumerically("~", -1.0, 1e-6))

		centerVel, err := body.GetLocalPointVelocity(box2d.Vec2{X: 1, Y: 0})
		Expect(err).NotTo(HaveOccurred())
		Expect(centerVel.X).To(BeNumerically("~", 5.0, 1e-6))
		Expect(centerVel.Y).To(BeNumerically("~", -1.0, 1e-6))
	})

	It("uses center-of-mass torque arm for off-center forces", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		makeBody := func() *box2d.Body {
			def := box2d.DefaultBodyDef()
			def.Type = box2d.DynamicBody
			body, err := world.CreateBody(def)
			Expect(err).NotTo(HaveOccurred())
			Expect(body.SetMassData(box2d.MassData{
				Mass:              2,
				Center:            box2d.Vec2{X: 0.5, Y: 0},
				RotationalInertia: 4,
			})).To(Succeed())
			return body
		}

		bodyCenter := makeBody()
		centerPoint, err := bodyCenter.GetWorldCenterOfMass()
		Expect(err).NotTo(HaveOccurred())
		Expect(bodyCenter.ApplyForce(box2d.Vec2{X: 4, Y: 0}, centerPoint, false)).To(Succeed())
		Expect(world.Step(0.5, 1)).To(Succeed())

		vCenter, err := bodyCenter.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(vCenter.X).To(BeNumerically("~", 1.0, 1e-6))
		Expect(vCenter.Y).To(BeNumerically("~", 0.0, 1e-6))

		wCenter, err := bodyCenter.GetAngularVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(wCenter).To(BeNumerically("~", 0.0, 1e-6))

		bodyOffset := makeBody()
		offsetCom, err := bodyOffset.GetWorldCenterOfMass()
		Expect(err).NotTo(HaveOccurred())
		offsetPoint := box2d.Add(offsetCom, box2d.Vec2{X: 0, Y: 1})
		Expect(bodyOffset.ApplyForce(box2d.Vec2{X: 4, Y: 0}, offsetPoint, false)).To(Succeed())
		Expect(world.Step(0.5, 1)).To(Succeed())

		vOffset, err := bodyOffset.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(vOffset.X).To(BeNumerically("~", 1.0, 1e-6))
		Expect(vOffset.Y).To(BeNumerically("~", 0.0, 1e-6))

		wOffset, err := bodyOffset.GetAngularVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(wOffset).To(BeNumerically("~", -0.5, 1e-6))

		newCom, err := bodyOffset.GetWorldCenterOfMass()
		Expect(err).NotTo(HaveOccurred())
		comVel, err := bodyOffset.GetWorldPointVelocity(newCom)
		Expect(err).NotTo(HaveOccurred())
		Expect(comVel.X).To(BeNumerically("~", 1.0, 1e-6))
		Expect(comVel.Y).To(BeNumerically("~", 0.0, 1e-6))
	})

	It("matches impulse response with equivalent force over a time step", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		makeBody := func() *box2d.Body {
			def := box2d.DefaultBodyDef()
			def.Type = box2d.DynamicBody
			body, err := world.CreateBody(def)
			Expect(err).NotTo(HaveOccurred())
			Expect(body.SetMassData(box2d.MassData{
				Mass:              2,
				Center:            box2d.Vec2Zero,
				RotationalInertia: 4,
			})).To(Succeed())
			return body
		}

		forceBody := makeBody()
		impulseBody := makeBody()

		force := box2d.Vec2{X: 6, Y: 0}
		dt := float32(0.5)
		impulse := box2d.MulSV(dt, force)

		center, err := forceBody.GetWorldCenterOfMass()
		Expect(err).NotTo(HaveOccurred())
		Expect(forceBody.ApplyForce(force, center, false)).To(Succeed())
		Expect(impulseBody.ApplyLinearImpulseToCenter(impulse, false)).To(Succeed())

		Expect(world.Step(dt, 1)).To(Succeed())

		vForce, err := forceBody.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		vImpulse, err := impulseBody.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())

		Expect(vForce.X).To(BeNumerically("~", vImpulse.X, 1e-6))
		Expect(vForce.Y).To(BeNumerically("~", vImpulse.Y, 1e-6))
		Expect(vForce.X).To(BeNumerically("~", 1.5, 1e-6))
		Expect(vForce.Y).To(BeNumerically("~", 0.0, 1e-6))
	})

	It("matches wake and torque behavior for impulses", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		def.IsAwake = false
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())
		Expect(body.SetMassData(box2d.MassData{
			Mass:              2,
			Center:            box2d.Vec2{X: 0.5, Y: 0},
			RotationalInertia: 4,
		})).To(Succeed())

		Expect(body.ApplyLinearImpulseToCenter(box2d.Vec2{X: 4, Y: 0}, false)).To(Succeed())
		Expect(body.ApplyAngularImpulse(2, false)).To(Succeed())

		v, err := body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(v.X).To(BeNumerically("~", 0.0, 1e-6))
		Expect(v.Y).To(BeNumerically("~", 0.0, 1e-6))
		w, err := body.GetAngularVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(w).To(BeNumerically("~", 0.0, 1e-6))

		com, err := body.GetWorldCenterOfMass()
		Expect(err).NotTo(HaveOccurred())
		point := box2d.Add(com, box2d.Vec2{X: 0, Y: 1})
		Expect(body.ApplyLinearImpulse(box2d.Vec2{X: 4, Y: 0}, point, true)).To(Succeed())

		v, err = body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(v.X).To(BeNumerically("~", 2.0, 1e-6))
		Expect(v.Y).To(BeNumerically("~", 0.0, 1e-6))
		w, err = body.GetAngularVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(w).To(BeNumerically("~", -1.0, 1e-6))

		Expect(body.ApplyAngularImpulse(2, false)).To(Succeed())
		w, err = body.GetAngularVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(w).To(BeNumerically("~", -0.5, 1e-6))
	})
})
