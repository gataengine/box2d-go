package box2d_test

import (
	"github.com/gataengine/box2d-go"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

var _ = Describe("World Lifecycle", func() {
	It("ports test_world.c EmptyWorld", func() {
		def := box2d.DefaultWorldDef()
		world, err := box2d.NewWorld(def)
		Expect(err).NotTo(HaveOccurred())
		Expect(world.IsValid()).To(BeTrue())
		Expect(box2d.IsWorldIDValid(world.ID())).To(BeTrue())

		timeStep := float32(1.0 / 60.0)
		subStepCount := 1
		for range 60 {
			Expect(world.Step(timeStep, subStepCount)).To(Succeed())
		}

		Expect(world.Destroy()).To(Succeed())
		Expect(world.IsValid()).To(BeFalse())
		Expect(box2d.IsWorldIDValid(world.ID())).To(BeFalse())
	})

	It("ports a HelloWorld-style falling-box collision subset", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2{X: 0.0, Y: -10.0}

		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyDef.Position = box2d.Vec2{X: 0.0, Y: 4.0}

		body, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		groundDef := box2d.DefaultBodyDef()
		groundDef.Position = box2d.Vec2{X: 0.0, Y: -10.0}
		ground, err := world.CreateBody(groundDef)
		Expect(err).NotTo(HaveOccurred())
		_, err = ground.CreatePolygonShape(box2d.DefaultShapeDef(), box2d.MakeBox(50.0, 10.0))
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1.0
		shapeDef.Material.Friction = 0.3
		_, err = body.CreatePolygonShape(shapeDef, box2d.MakeBox(1.0, 1.0))
		Expect(err).NotTo(HaveOccurred())

		const steps = 90
		timeStep := float32(1.0 / 60.0)
		subStepCount := 4
		for range steps {
			Expect(world.Step(timeStep, subStepCount)).To(Succeed())
		}

		position, err := body.GetPosition()
		Expect(err).NotTo(HaveOccurred())
		rotation, err := body.GetRotation()
		Expect(err).NotTo(HaveOccurred())

		Expect(position.X).To(BeNumerically("~", 0.0, 1e-5))
		Expect(position.Y).To(BeNumerically("~", 0.95, 2e-2))
		Expect(box2d.RotGetAngle(rotation)).To(BeNumerically("~", 0.0, 1e-5))

		Expect(world.Destroy()).To(Succeed())
	})

	It("applies restitution callback during contact response", func() {
		makeWorld := func(withCallback bool) (*box2d.World, *box2d.Body) {
			worldDef := box2d.DefaultWorldDef()
			worldDef.Gravity = box2d.Vec2Zero
			world, err := box2d.NewWorld(worldDef)
			Expect(err).NotTo(HaveOccurred())

			Expect(world.SetRestitutionThreshold(0)).To(Succeed())
			if withCallback {
				Expect(world.SetRestitutionCallback(func(shapeA, shapeB box2d.ShapeID, restitution float32) float32 {
					return 1.0
				})).To(Succeed())
			}

			groundDef := box2d.DefaultBodyDef()
			groundDef.Position = box2d.Vec2{X: 0, Y: -1}
			ground, err := world.CreateBody(groundDef)
			Expect(err).NotTo(HaveOccurred())
			_, err = ground.CreatePolygonShape(box2d.DefaultShapeDef(), box2d.MakeBox(10, 1))
			Expect(err).NotTo(HaveOccurred())

			bodyDef := box2d.DefaultBodyDef()
			bodyDef.Type = box2d.DynamicBody
			bodyDef.Position = box2d.Vec2{X: 0, Y: 2}
			bodyDef.LinearVelocity = box2d.Vec2{X: 0, Y: -5}
			body, err := world.CreateBody(bodyDef)
			Expect(err).NotTo(HaveOccurred())

			shapeDef := box2d.DefaultShapeDef()
			shapeDef.Density = 1
			_, err = body.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
			Expect(err).NotTo(HaveOccurred())
			return world, body
		}

		worldNoBounce, bodyNoBounce := makeWorld(false)
		worldBounce, bodyBounce := makeWorld(true)

		for range 40 {
			Expect(worldNoBounce.Step(1.0/60.0, 4)).To(Succeed())
			Expect(worldBounce.Step(1.0/60.0, 4)).To(Succeed())
		}

		vNoBounce, err := bodyNoBounce.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		vBounce, err := bodyBounce.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())

		Expect(vNoBounce.Y).To(BeNumerically("~", 0.0, 1e-4))
		Expect(vBounce.Y).To(BeNumerically(">", 0.1))
	})

	It("applies friction callback during contact response", func() {
		makeWorld := func(frictionValue float32) (*box2d.World, *box2d.Body) {
			worldDef := box2d.DefaultWorldDef()
			worldDef.Gravity = box2d.Vec2{X: 0, Y: -10}
			world, err := box2d.NewWorld(worldDef)
			Expect(err).NotTo(HaveOccurred())
			Expect(world.SetFrictionCallback(func(shapeA, shapeB box2d.ShapeID, friction float32) float32 {
				return frictionValue
			})).To(Succeed())

			groundDef := box2d.DefaultBodyDef()
			groundDef.Position = box2d.Vec2{X: 0, Y: -1}
			ground, err := world.CreateBody(groundDef)
			Expect(err).NotTo(HaveOccurred())
			_, err = ground.CreatePolygonShape(box2d.DefaultShapeDef(), box2d.MakeBox(20, 1))
			Expect(err).NotTo(HaveOccurred())

			bodyDef := box2d.DefaultBodyDef()
			bodyDef.Type = box2d.DynamicBody
			bodyDef.Position = box2d.Vec2{X: 0, Y: 1.0}
			bodyDef.LinearVelocity = box2d.Vec2{X: 8, Y: 0}
			body, err := world.CreateBody(bodyDef)
			Expect(err).NotTo(HaveOccurred())

			shapeDef := box2d.DefaultShapeDef()
			shapeDef.Density = 1
			_, err = body.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
			Expect(err).NotTo(HaveOccurred())
			return world, body
		}

		worldNoFriction, bodyNoFriction := makeWorld(0)
		worldHighFriction, bodyHighFriction := makeWorld(1)

		for range 120 {
			Expect(worldNoFriction.Step(1.0/60.0, 4)).To(Succeed())
			Expect(worldHighFriction.Step(1.0/60.0, 4)).To(Succeed())
		}

		vNoFriction, err := bodyNoFriction.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		vHighFriction, err := bodyHighFriction.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())

		Expect(vNoFriction.X).To(BeNumerically(">", 7.5))
		Expect(vHighFriction.X).To(BeNumerically("<", vNoFriction.X-1.0))
	})

	It("ports test_world.c DestroyAllBodiesWorld", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())
		Expect(world.IsValid()).To(BeTrue())

		const bodyCount = 10
		count := 0
		creating := true

		ids := make([]*box2d.Body, bodyCount)
		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		square := box2d.MakeSquare(0.5)

		for i := 0; i < 2*bodyCount+10; i++ {
			if creating {
				if count < bodyCount {
					ids[count], err = world.CreateBody(bodyDef)
					Expect(err).NotTo(HaveOccurred())
					_, err = ids[count].CreatePolygonShape(box2d.DefaultShapeDef(), square)
					Expect(err).NotTo(HaveOccurred())
					count++
				} else {
					creating = false
				}
			} else if count > 0 {
				Expect(ids[count-1].Destroy()).To(Succeed())
				ids[count-1] = nil
				count--
			}

			Expect(world.Step(1.0/60.0, 3)).To(Succeed())
		}

		counters, err := world.Counters()
		Expect(err).NotTo(HaveOccurred())
		Expect(counters.BodyCount).To(Equal(0))
		Expect(counters.ShapeCount).To(Equal(0))

		Expect(world.Destroy()).To(Succeed())
		Expect(world.IsValid()).To(BeFalse())
	})

	It("applies damping and motion locks during stepping", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2{X: 0, Y: -10}
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		dampedDef := box2d.DefaultBodyDef()
		dampedDef.Type = box2d.DynamicBody
		dampedDef.GravityScale = 0
		dampedDef.LinearVelocity = box2d.Vec2{X: 10, Y: 0}
		dampedDef.AngularVelocity = 4
		dampedDef.LinearDamping = 2
		dampedDef.AngularDamping = 1
		damped, err := world.CreateBody(dampedDef)
		Expect(err).NotTo(HaveOccurred())

		lockedDef := box2d.DefaultBodyDef()
		lockedDef.Type = box2d.DynamicBody
		lockedDef.Position = box2d.Vec2{X: 1, Y: 2}
		lockedDef.Rotation = box2d.MakeRot(0.4)
		lockedDef.LinearVelocity = box2d.Vec2{X: 5, Y: 5}
		lockedDef.AngularVelocity = 3
		lockedDef.MotionLocks = box2d.MotionLocks{LinearX: true, LinearY: true, AngularZ: true}
		locked, err := world.CreateBody(lockedDef)
		Expect(err).NotTo(HaveOccurred())

		const h = float32(0.5)
		Expect(world.Step(h, 1)).To(Succeed())

		dampedVelocity, err := damped.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(dampedVelocity.X).To(BeNumerically("~", 5.0, 1e-6))
		Expect(dampedVelocity.Y).To(BeNumerically("~", 0.0, 1e-6))

		dampedAngularVelocity, err := damped.GetAngularVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(dampedAngularVelocity).To(BeNumerically("~", 8.0/3.0, 1e-6))

		dampedPos, err := damped.GetPosition()
		Expect(err).NotTo(HaveOccurred())
		Expect(dampedPos.X).To(BeNumerically("~", 2.5, 1e-6))
		Expect(dampedPos.Y).To(BeNumerically("~", 0.0, 1e-6))

		dampedRot, err := damped.GetRotation()
		Expect(err).NotTo(HaveOccurred())
		expectedDampedRot := box2d.NormalizeRot(box2d.Rot{C: 1, S: h * (8.0 / 3.0)})
		Expect(dampedRot.C).To(BeNumerically("~", expectedDampedRot.C, 1e-6))
		Expect(dampedRot.S).To(BeNumerically("~", expectedDampedRot.S, 1e-6))

		lockedVelocity, err := locked.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(lockedVelocity.X).To(BeNumerically("~", 0.0, 1e-6))
		Expect(lockedVelocity.Y).To(BeNumerically("~", 0.0, 1e-6))

		lockedAngularVelocity, err := locked.GetAngularVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(lockedAngularVelocity).To(BeNumerically("~", 0.0, 1e-6))

		lockedPos, err := locked.GetPosition()
		Expect(err).NotTo(HaveOccurred())
		Expect(lockedPos.X).To(BeNumerically("~", 1.0, 1e-6))
		Expect(lockedPos.Y).To(BeNumerically("~", 2.0, 1e-6))

		lockedRot, err := locked.GetRotation()
		Expect(err).NotTo(HaveOccurred())
		Expect(lockedRot.C).To(BeNumerically("~", lockedDef.Rotation.C, 1e-6))
		Expect(lockedRot.S).To(BeNumerically("~", lockedDef.Rotation.S, 1e-6))

		Expect(world.Destroy()).To(Succeed())
	})

	It("transitions idle dynamic bodies to sleep and wakes on impulse", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		body, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())
		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1
		_, err = body.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())

		for range 40 {
			Expect(world.Step(1.0/60.0, 1)).To(Succeed())
		}
		awake, err := body.IsAwake()
		Expect(err).NotTo(HaveOccurred())
		Expect(awake).To(BeFalse())

		Expect(body.ApplyLinearImpulseToCenter(box2d.Vec2{X: 2, Y: 0}, true)).To(Succeed())
		awake, err = body.IsAwake()
		Expect(err).NotTo(HaveOccurred())
		Expect(awake).To(BeTrue())
	})

	It("keeps dynamic bodies awake when sleeping is disabled", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())
		Expect(world.EnableSleeping(false)).To(Succeed())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		body, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())
		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1
		_, err = body.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())

		for range 120 {
			Expect(world.Step(1.0/60.0, 1)).To(Succeed())
		}
		awake, err := body.IsAwake()
		Expect(err).NotTo(HaveOccurred())
		Expect(awake).To(BeTrue())
	})

	It("reports step/profile counters after simulation", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		body, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())
		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1
		_, err = body.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())

		Expect(world.Step(1.0/120.0, 2)).To(Succeed())

		profile, err := world.GetProfile()
		Expect(err).NotTo(HaveOccurred())
		Expect(profile.StepCount).To(Equal(1))
		Expect(profile.SubStepCount).To(Equal(2))
		Expect(profile.LastTimeStep).To(BeNumerically("~", 1.0/120.0, 1e-6))
		Expect(profile.BodyCount).To(Equal(1))
		Expect(profile.ShapeCount).To(Equal(1))
		Expect(profile.AwakeBodyCount).To(Equal(1))
	})

	It("is deterministic across identical world runs", func() {
		buildAndRun := func() (box2d.Vec2, box2d.Vec2, float32) {
			worldDef := box2d.DefaultWorldDef()
			worldDef.Gravity = box2d.Vec2{X: 0, Y: -10}
			world, err := box2d.NewWorld(worldDef)
			Expect(err).NotTo(HaveOccurred())
			defer func() {
				Expect(world.Destroy()).To(Succeed())
			}()

			groundDef := box2d.DefaultBodyDef()
			groundDef.Position = box2d.Vec2{X: 0, Y: -10}
			ground, err := world.CreateBody(groundDef)
			Expect(err).NotTo(HaveOccurred())
			_, err = ground.CreatePolygonShape(box2d.DefaultShapeDef(), box2d.MakeBox(50, 10))
			Expect(err).NotTo(HaveOccurred())

			bodyDefA := box2d.DefaultBodyDef()
			bodyDefA.Type = box2d.DynamicBody
			bodyDefA.Position = box2d.Vec2{X: -1, Y: 4}
			bodyA, err := world.CreateBody(bodyDefA)
			Expect(err).NotTo(HaveOccurred())

			bodyDefB := box2d.DefaultBodyDef()
			bodyDefB.Type = box2d.DynamicBody
			bodyDefB.Position = box2d.Vec2{X: 1, Y: 5}
			bodyB, err := world.CreateBody(bodyDefB)
			Expect(err).NotTo(HaveOccurred())

			shapeDef := box2d.DefaultShapeDef()
			shapeDef.Density = 1
			_, err = bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.75})
			Expect(err).NotTo(HaveOccurred())
			_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.75})
			Expect(err).NotTo(HaveOccurred())

			joint, err := world.CreateDistanceJoint(bodyA.ID(), bodyB.ID())
			Expect(err).NotTo(HaveOccurred())
			Expect(joint.SetLength(1.8)).To(Succeed())
			Expect(joint.EnableMotor(true)).To(Succeed())
			Expect(joint.SetMotorSpeed(1.2)).To(Succeed())
			Expect(joint.SetMaxMotorForce(40)).To(Succeed())

			for range 240 {
				Expect(world.Step(1.0/60.0, 4)).To(Succeed())
			}

			posA, err := bodyA.GetPosition()
			Expect(err).NotTo(HaveOccurred())
			posB, err := bodyB.GetPosition()
			Expect(err).NotTo(HaveOccurred())
			length, err := joint.GetCurrentLength()
			Expect(err).NotTo(HaveOccurred())
			return posA, posB, length
		}

		posA1, posB1, length1 := buildAndRun()
		posA2, posB2, length2 := buildAndRun()
		Expect(posA1.X).To(BeNumerically("~", posA2.X, 1e-6))
		Expect(posA1.Y).To(BeNumerically("~", posA2.Y, 1e-6))
		Expect(posB1.X).To(BeNumerically("~", posB2.X, 1e-6))
		Expect(posB1.Y).To(BeNumerically("~", posB2.Y, 1e-6))
		Expect(length1).To(BeNumerically("~", length2, 1e-6))
	})

	It("recycles world slots without leaking validity", func() {
		const worldCount = box2d.MaxWorlds / 2
		const loops = 100

		for range loops {
			worlds := make([]*box2d.World, 0, worldCount)
			for range worldCount {
				w, err := box2d.NewWorld(box2d.DefaultWorldDef())
				Expect(err).NotTo(HaveOccurred())
				Expect(w.IsValid()).To(BeTrue())
				worlds = append(worlds, w)
			}

			for _, w := range worlds {
				for range 10 {
					Expect(w.Step(1.0/60.0, 1)).To(Succeed())
				}
			}

			for i := len(worlds) - 1; i >= 0; i-- {
				id := worlds[i].ID()
				Expect(worlds[i].Destroy()).To(Succeed())
				Expect(box2d.IsWorldIDValid(id)).To(BeFalse())
			}
		}
	})

	It("rejects uninitialized world definitions", func() {
		_, err := box2d.NewWorld(box2d.WorldDef{})
		Expect(err).To(MatchError(box2d.ErrInvalidWorldDef))
	})

	It("supports world configuration and state accessors", func() {
		def := box2d.DefaultWorldDef()
		world, err := box2d.NewWorld(def)
		Expect(err).NotTo(HaveOccurred())

		g, err := world.GetGravity()
		Expect(err).NotTo(HaveOccurred())
		Expect(g).To(Equal(def.Gravity))

		Expect(world.SetUserData("meta")).To(Succeed())
		ud, err := world.GetUserData()
		Expect(err).NotTo(HaveOccurred())
		Expect(ud).To(Equal("meta"))

		Expect(world.SetRestitutionThreshold(2.5)).To(Succeed())
		rt, err := world.GetRestitutionThreshold()
		Expect(err).NotTo(HaveOccurred())
		Expect(rt).To(BeNumerically("~", 2.5, 1e-6))

		Expect(world.SetHitEventThreshold(3.0)).To(Succeed())
		ht, err := world.GetHitEventThreshold()
		Expect(err).NotTo(HaveOccurred())
		Expect(ht).To(BeNumerically("~", 3.0, 1e-6))

		Expect(world.SetMaximumLinearSpeed(123.0)).To(Succeed())
		mls, err := world.GetMaximumLinearSpeed()
		Expect(err).NotTo(HaveOccurred())
		Expect(mls).To(BeNumerically("~", 123.0, 1e-6))

		Expect(world.SetContactTuning(10, 0.7, 2.0)).To(Succeed())

		sleepEnabled, err := world.IsSleepingEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(sleepEnabled).To(Equal(def.EnableSleep))
		Expect(world.EnableSleeping(false)).To(Succeed())
		sleepEnabled, err = world.IsSleepingEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(sleepEnabled).To(BeFalse())

		continuousEnabled, err := world.IsContinuousEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(continuousEnabled).To(Equal(def.EnableContinuous))
		Expect(world.EnableContinuous(false)).To(Succeed())
		continuousEnabled, err = world.IsContinuousEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(continuousEnabled).To(BeFalse())

		warmStartingEnabled, err := world.IsWarmStartingEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(warmStartingEnabled).To(BeTrue())
		Expect(world.EnableWarmStarting(false)).To(Succeed())
		warmStartingEnabled, err = world.IsWarmStartingEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(warmStartingEnabled).To(BeFalse())

		Expect(world.EnableSpeculative(false)).To(Succeed())
		speculativeEnabled, err := world.IsSpeculativeEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(speculativeEnabled).To(BeFalse())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyDef.IsAwake = false
		_, err = world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		awakeCount, err := world.GetAwakeBodyCount()
		Expect(err).NotTo(HaveOccurred())
		Expect(awakeCount).To(Equal(0))

		Expect(world.EnableSleeping(false)).To(Succeed())
		awakeCount, err = world.GetAwakeBodyCount()
		Expect(err).NotTo(HaveOccurred())
		Expect(awakeCount).To(Equal(1))

		counters, err := world.GetCounters()
		Expect(err).NotTo(HaveOccurred())
		Expect(counters.BodyCount).To(Equal(1))
		bodyCount, err := world.GetBodyCount()
		Expect(err).NotTo(HaveOccurred())
		Expect(bodyCount).To(Equal(1))
		shapeCount, err := world.GetShapeCount()
		Expect(err).NotTo(HaveOccurred())
		Expect(shapeCount).To(Equal(0))
		jointCount, err := world.GetJointCount()
		Expect(err).NotTo(HaveOccurred())
		Expect(jointCount).To(Equal(0))
		chainCount, err := world.GetChainCount()
		Expect(err).NotTo(HaveOccurred())
		Expect(chainCount).To(Equal(0))
		proxyCount, err := world.GetProxyCount()
		Expect(err).NotTo(HaveOccurred())
		Expect(proxyCount).To(Equal(0))
	})

	It("supports overlap and ray-cast world queries", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		makeCircleBody := func(x float32) (*box2d.Shape, error) {
			bodyDef := box2d.DefaultBodyDef()
			bodyDef.Position = box2d.Vec2{X: x, Y: 0}
			body, err := world.CreateBody(bodyDef)
			if err != nil {
				return nil, err
			}
			return body.CreateCircleShape(box2d.DefaultShapeDef(), box2d.Circle{
				Center: box2d.Vec2Zero,
				Radius: 0.5,
			})
		}

		shapeA, err := makeCircleBody(0)
		Expect(err).NotTo(HaveOccurred())
		shapeB, err := makeCircleBody(5)
		Expect(err).NotTo(HaveOccurred())
		proxyCount, err := world.GetProxyCount()
		Expect(err).NotTo(HaveOccurred())
		Expect(proxyCount).To(Equal(2))

		overlaps, err := world.OverlapAABB(box2d.AABB{
			LowerBound: box2d.Vec2{X: -1, Y: -1},
			UpperBound: box2d.Vec2{X: 1, Y: 1},
		})
		Expect(err).NotTo(HaveOccurred())
		Expect(overlaps).To(HaveLen(1))
		Expect(box2d.ShapeIDEqual(overlaps[0], shapeA.ID())).To(BeTrue())

		input := box2d.RayCastInput{
			Origin:      box2d.Vec2{X: -2, Y: 0},
			Translation: box2d.Vec2{X: 10, Y: 0},
			MaxFraction: 1.0,
		}
		hits, err := world.CastRay(input)
		Expect(err).NotTo(HaveOccurred())
		Expect(hits).To(HaveLen(2))
		Expect(box2d.ShapeIDEqual(hits[0].ShapeID, shapeA.ID())).To(BeTrue())
		Expect(box2d.ShapeIDEqual(hits[1].ShapeID, shapeB.ID())).To(BeTrue())
		Expect(hits[0].Output.Fraction).To(BeNumerically("<", hits[1].Output.Fraction))

		closest, hit, err := world.CastRayClosest(input)
		Expect(err).NotTo(HaveOccurred())
		Expect(hit).To(BeTrue())
		Expect(box2d.ShapeIDEqual(closest.ShapeID, shapeA.ID())).To(BeTrue())
		Expect(closest.Output.Fraction).To(BeNumerically("~", hits[0].Output.Fraction, 1e-6))
	})

	It("applies query filters to overlap and ray-cast queries", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		makeCircle := func(x float32, category uint64) *box2d.Shape {
			bodyDef := box2d.DefaultBodyDef()
			bodyDef.Type = box2d.DynamicBody
			bodyDef.Position = box2d.Vec2{X: x, Y: 0}
			body, createErr := world.CreateBody(bodyDef)
			Expect(createErr).NotTo(HaveOccurred())

			shapeDef := box2d.DefaultShapeDef()
			shapeDef.Filter.CategoryBits = category
			shape, createErr := body.CreateCircleShape(shapeDef, box2d.Circle{
				Center: box2d.Vec2Zero,
				Radius: 0.5,
			})
			Expect(createErr).NotTo(HaveOccurred())
			return shape
		}

		shapeA := makeCircle(-1, 0x0002)
		_ = makeCircle(1, 0x0004)

		filter := box2d.DefaultQueryFilter()
		filter.MaskBits = 0x0002

		overlaps, err := world.OverlapAABBWithFilter(box2d.AABB{
			LowerBound: box2d.Vec2{X: -3, Y: -1},
			UpperBound: box2d.Vec2{X: 3, Y: 1},
		}, filter)
		Expect(err).NotTo(HaveOccurred())
		Expect(overlaps).To(HaveLen(1))
		Expect(box2d.ShapeIDEqual(overlaps[0], shapeA.ID())).To(BeTrue())

		input := box2d.RayCastInput{
			Origin:      box2d.Vec2{X: -3, Y: 0},
			Translation: box2d.Vec2{X: 6, Y: 0},
			MaxFraction: 1.0,
		}
		hits, err := world.CastRayWithFilter(input, filter)
		Expect(err).NotTo(HaveOccurred())
		Expect(hits).To(HaveLen(1))
		Expect(box2d.ShapeIDEqual(hits[0].ShapeID, shapeA.ID())).To(BeTrue())

		closest, hit, err := world.CastRayClosestWithFilter(input, filter)
		Expect(err).NotTo(HaveOccurred())
		Expect(hit).To(BeTrue())
		Expect(box2d.ShapeIDEqual(closest.ShapeID, shapeA.ID())).To(BeTrue())
	})

	It("applies query category bits against shape mask bits", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		body, err := world.CreateBody(box2d.DefaultBodyDef())
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Filter.CategoryBits = 0x0002
		shapeDef.Filter.MaskBits = 0x0008
		shape, err := body.CreateCircleShape(shapeDef, box2d.Circle{
			Center: box2d.Vec2Zero,
			Radius: 0.5,
		})
		Expect(err).NotTo(HaveOccurred())

		filter := box2d.DefaultQueryFilter()
		filter.MaskBits = 0x0002
		filter.CategoryBits = 0x0004

		overlaps, err := world.OverlapAABBWithFilter(box2d.AABB{
			LowerBound: box2d.Vec2{X: -1, Y: -1},
			UpperBound: box2d.Vec2{X: 1, Y: 1},
		}, filter)
		Expect(err).NotTo(HaveOccurred())
		Expect(overlaps).To(BeEmpty())

		filter.CategoryBits = 0x0008
		overlaps, err = world.OverlapAABBWithFilter(box2d.AABB{
			LowerBound: box2d.Vec2{X: -1, Y: -1},
			UpperBound: box2d.Vec2{X: 1, Y: 1},
		}, filter)
		Expect(err).NotTo(HaveOccurred())
		Expect(overlaps).To(HaveLen(1))
		Expect(box2d.ShapeIDEqual(overlaps[0], shape.ID())).To(BeTrue())
	})

	It("ray casts capsule shapes in world queries", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		body, err := world.CreateBody(box2d.DefaultBodyDef())
		Expect(err).NotTo(HaveOccurred())
		_, err = body.CreateCapsuleShape(box2d.DefaultShapeDef(), box2d.Capsule{
			Center1: box2d.Vec2{X: -1, Y: 0},
			Center2: box2d.Vec2{X: 1, Y: 0},
			Radius:  0.5,
		})
		Expect(err).NotTo(HaveOccurred())

		input := box2d.RayCastInput{
			Origin:      box2d.Vec2{X: -3, Y: 0},
			Translation: box2d.Vec2{X: 6, Y: 0},
			MaxFraction: 1.0,
		}
		hits, err := world.CastRay(input)
		Expect(err).NotTo(HaveOccurred())
		Expect(hits).To(HaveLen(1))
		Expect(hits[0].Output.Hit).To(BeTrue())
		Expect(hits[0].Output.Fraction).To(BeNumerically("~", 0.25, 1e-5))
	})

	It("breaks CastRayClosest ties by shape id order", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		makeCircle := func() (*box2d.Shape, error) {
			body, err := world.CreateBody(box2d.DefaultBodyDef())
			if err != nil {
				return nil, err
			}
			return body.CreateCircleShape(box2d.DefaultShapeDef(), box2d.Circle{
				Center: box2d.Vec2Zero,
				Radius: 0.5,
			})
		}

		shapeA, err := makeCircle()
		Expect(err).NotTo(HaveOccurred())
		shapeB, err := makeCircle()
		Expect(err).NotTo(HaveOccurred())

		input := box2d.RayCastInput{
			Origin:      box2d.Vec2{X: -2, Y: 0},
			Translation: box2d.Vec2{X: 4, Y: 0},
			MaxFraction: 1.0,
		}
		closest, hit, err := world.CastRayClosest(input)
		Expect(err).NotTo(HaveOccurred())
		Expect(hit).To(BeTrue())
		Expect(box2d.ShapeIDEqual(closest.ShapeID, shapeA.ID())).To(BeTrue())

		hits, err := world.CastRay(input)
		Expect(err).NotTo(HaveOccurred())
		Expect(hits).To(HaveLen(2))
		Expect(box2d.ShapeIDEqual(hits[0].ShapeID, shapeA.ID())).To(BeTrue())
		Expect(box2d.ShapeIDEqual(hits[1].ShapeID, shapeB.ID())).To(BeTrue())
	})

	It("ignores initial-overlap hits in CastRayClosest", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		makeCircleBody := func(x float32, radius float32) *box2d.Shape {
			bodyDef := box2d.DefaultBodyDef()
			bodyDef.Position = box2d.Vec2{X: x, Y: 0}
			body, createErr := world.CreateBody(bodyDef)
			Expect(createErr).NotTo(HaveOccurred())
			shape, createErr := body.CreateCircleShape(box2d.DefaultShapeDef(), box2d.Circle{
				Center: box2d.Vec2Zero,
				Radius: radius,
			})
			Expect(createErr).NotTo(HaveOccurred())
			return shape
		}

		overlapShape := makeCircleBody(0, 1.0)
		farShape := makeCircleBody(4, 1.0)

		input := box2d.RayCastInput{
			Origin:      box2d.Vec2{X: 0, Y: 0},
			Translation: box2d.Vec2{X: 8, Y: 0},
			MaxFraction: 1.0,
		}

		allHits, err := world.CastRay(input)
		Expect(err).NotTo(HaveOccurred())
		Expect(allHits).To(HaveLen(2))
		Expect(box2d.ShapeIDEqual(allHits[0].ShapeID, overlapShape.ID())).To(BeTrue())
		Expect(allHits[0].Output.Fraction).To(BeNumerically("~", 0.0, 1e-6))

		closest, hit, err := world.CastRayClosest(input)
		Expect(err).NotTo(HaveOccurred())
		Expect(hit).To(BeTrue())
		Expect(box2d.ShapeIDEqual(closest.ShapeID, farShape.ID())).To(BeTrue())
		Expect(closest.Output.Fraction).To(BeNumerically(">", 0.0))
	})

	It("returns no hit when CastRayClosest only sees initial overlap", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		body, err := world.CreateBody(box2d.DefaultBodyDef())
		Expect(err).NotTo(HaveOccurred())
		_, err = body.CreateCircleShape(box2d.DefaultShapeDef(), box2d.Circle{
			Center: box2d.Vec2Zero,
			Radius: 1.0,
		})
		Expect(err).NotTo(HaveOccurred())

		input := box2d.RayCastInput{
			Origin:      box2d.Vec2{X: 0, Y: 0},
			Translation: box2d.Vec2{X: 0.25, Y: 0},
			MaxFraction: 1.0,
		}
		_, hit, err := world.CastRayClosest(input)
		Expect(err).NotTo(HaveOccurred())
		Expect(hit).To(BeFalse())
	})

	It("returns OverlapAABB results in ascending shape id order", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		createShape := func(position box2d.Vec2) (*box2d.Shape, error) {
			bodyDef := box2d.DefaultBodyDef()
			bodyDef.Position = position
			body, err := world.CreateBody(bodyDef)
			if err != nil {
				return nil, err
			}
			return body.CreateCircleShape(box2d.DefaultShapeDef(), box2d.Circle{
				Center: box2d.Vec2Zero,
				Radius: 0.25,
			})
		}

		shapeA, err := createShape(box2d.Vec2{X: 2, Y: 0})
		Expect(err).NotTo(HaveOccurred())
		shapeB, err := createShape(box2d.Vec2{X: -2, Y: 0})
		Expect(err).NotTo(HaveOccurred())
		shapeC, err := createShape(box2d.Vec2{X: 0, Y: 0})
		Expect(err).NotTo(HaveOccurred())

		overlaps, err := world.OverlapAABB(box2d.AABB{
			LowerBound: box2d.Vec2{X: -3, Y: -1},
			UpperBound: box2d.Vec2{X: 3, Y: 1},
		})
		Expect(err).NotTo(HaveOccurred())
		Expect(overlaps).To(HaveLen(3))
		Expect(box2d.ShapeIDEqual(overlaps[0], shapeA.ID())).To(BeTrue())
		Expect(box2d.ShapeIDEqual(overlaps[1], shapeB.ID())).To(BeTrue())
		Expect(box2d.ShapeIDEqual(overlaps[2], shapeC.ID())).To(BeTrue())
	})

	It("preserves manifold point ids across steps", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyADef := box2d.DefaultBodyDef()
		bodyADef.Type = box2d.DynamicBody
		bodyADef.Position = box2d.Vec2{X: 0, Y: 0}
		bodyA, err := world.CreateBody(bodyADef)
		Expect(err).NotTo(HaveOccurred())

		bodyBDef := box2d.DefaultBodyDef()
		bodyBDef.Position = box2d.Vec2{X: 2, Y: 0}
		bodyB, err := world.CreateBody(bodyBDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		_, err = bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		data, err := bodyA.GetContactData(4)
		Expect(err).NotTo(HaveOccurred())
		Expect(data).To(HaveLen(1))
		Expect(data[0].Manifold.PointCount).To(Equal(1))
		firstPoint := data[0].Manifold.Point1
		Expect(firstPoint.Persisted).To(BeFalse())
		Expect(firstPoint.ID).NotTo(Equal(uint16(0)))

		Expect(world.Step(1.0/60.0, 1)).To(Succeed())

		data, err = bodyA.GetContactData(4)
		Expect(err).NotTo(HaveOccurred())
		Expect(data).To(HaveLen(1))
		Expect(data[0].Manifold.PointCount).To(Equal(1))
		secondPoint := data[0].Manifold.Point1
		Expect(secondPoint.ID).To(Equal(firstPoint.ID))
		Expect(secondPoint.Persisted).To(BeTrue())
	})

	It("produces deterministic results across repeated identical runs", func() {
		run := func() (box2d.Vec2, float32, box2d.Vec2, float32) {
			worldDef := box2d.DefaultWorldDef()
			worldDef.Gravity = box2d.Vec2{X: 0, Y: -10}
			world, err := box2d.NewWorld(worldDef)
			Expect(err).NotTo(HaveOccurred())
			defer func() { _ = world.Destroy() }()

			makeBody := func(position, velocity box2d.Vec2) *box2d.Body {
				def := box2d.DefaultBodyDef()
				def.Type = box2d.DynamicBody
				def.Position = position
				def.LinearVelocity = velocity
				body, createErr := world.CreateBody(def)
				Expect(createErr).NotTo(HaveOccurred())
				shapeDef := box2d.DefaultShapeDef()
				shapeDef.Density = 1
				_, createErr = body.CreateCircleShape(shapeDef, box2d.Circle{
					Center: box2d.Vec2Zero,
					Radius: 0.5,
				})
				Expect(createErr).NotTo(HaveOccurred())
				return body
			}

			bodyA := makeBody(box2d.Vec2{X: -1, Y: 3}, box2d.Vec2{X: 2.5, Y: 0})
			bodyB := makeBody(box2d.Vec2{X: 1, Y: 3}, box2d.Vec2{X: -2.5, Y: 0})

			for range 120 {
				Expect(world.Step(1.0/60.0, 4)).To(Succeed())
			}

			posA, err := bodyA.GetPosition()
			Expect(err).NotTo(HaveOccurred())
			angA, err := bodyA.GetRotation()
			Expect(err).NotTo(HaveOccurred())
			posB, err := bodyB.GetPosition()
			Expect(err).NotTo(HaveOccurred())
			angB, err := bodyB.GetRotation()
			Expect(err).NotTo(HaveOccurred())

			return posA, box2d.RotGetAngle(angA), posB, box2d.RotGetAngle(angB)
		}

		posA1, angA1, posB1, angB1 := run()
		posA2, angA2, posB2, angB2 := run()

		Expect(posA1.X).To(BeNumerically("~", posA2.X, 1e-6))
		Expect(posA1.Y).To(BeNumerically("~", posA2.Y, 1e-6))
		Expect(angA1).To(BeNumerically("~", angA2, 1e-6))
		Expect(posB1.X).To(BeNumerically("~", posB2.X, 1e-6))
		Expect(posB1.Y).To(BeNumerically("~", posB2.Y, 1e-6))
		Expect(angB1).To(BeNumerically("~", angB2, 1e-6))
	})

	It("excludes shapes on disabled bodies from overlap and ray queries", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		makeCircle := func(x float32) (*box2d.Body, *box2d.Shape) {
			bodyDef := box2d.DefaultBodyDef()
			bodyDef.Position = box2d.Vec2{X: x, Y: 0}
			body, createErr := world.CreateBody(bodyDef)
			Expect(createErr).NotTo(HaveOccurred())
			shape, createErr := body.CreateCircleShape(box2d.DefaultShapeDef(), box2d.Circle{
				Center: box2d.Vec2Zero,
				Radius: 0.5,
			})
			Expect(createErr).NotTo(HaveOccurred())
			return body, shape
		}

		enabledBody, enabledShape := makeCircle(-1)
		disabledBody, _ := makeCircle(1)
		Expect(disabledBody.Disable()).To(Succeed())

		overlaps, err := world.OverlapAABB(box2d.AABB{
			LowerBound: box2d.Vec2{X: -2, Y: -1},
			UpperBound: box2d.Vec2{X: 2, Y: 1},
		})
		Expect(err).NotTo(HaveOccurred())
		Expect(overlaps).To(HaveLen(1))
		Expect(box2d.ShapeIDEqual(overlaps[0], enabledShape.ID())).To(BeTrue())

		input := box2d.RayCastInput{
			Origin:      box2d.Vec2{X: -3, Y: 0},
			Translation: box2d.Vec2{X: 6, Y: 0},
			MaxFraction: 1.0,
		}
		hits, err := world.CastRay(input)
		Expect(err).NotTo(HaveOccurred())
		Expect(hits).To(HaveLen(1))
		Expect(box2d.ShapeIDEqual(hits[0].ShapeID, enabledShape.ID())).To(BeTrue())

		Expect(enabledBody.Disable()).To(Succeed())
		overlaps, err = world.OverlapAABB(box2d.AABB{
			LowerBound: box2d.Vec2{X: -2, Y: -1},
			UpperBound: box2d.Vec2{X: 2, Y: 1},
		})
		Expect(err).NotTo(HaveOccurred())
		Expect(overlaps).To(BeEmpty())
	})

	It("refreshes profile and memory contact counts on demand", func() {
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

		_, err = bodyA.CreateCircleShape(box2d.DefaultShapeDef(), box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(box2d.DefaultShapeDef(), box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		profile, err := world.GetProfile()
		Expect(err).NotTo(HaveOccurred())
		Expect(profile.ContactCount).To(Equal(1))

		stats, err := world.DumpMemoryStats()
		Expect(err).NotTo(HaveOccurred())
		Expect(stats).To(ContainSubstring(`"contacts":{"active":1}`))
	})

	It("provides placeholder advanced world APIs with validation semantics", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		Expect(world.SetCustomFilterCallback(func(shapeA, shapeB box2d.ShapeID) bool { return true })).To(Succeed())
		Expect(world.SetFrictionCallback(func(shapeA, shapeB box2d.ShapeID, friction float32) float32 { return friction })).To(Succeed())
		Expect(world.SetRestitutionCallback(func(shapeA, shapeB box2d.ShapeID, restitution float32) float32 { return restitution })).To(Succeed())
		Expect(world.SetPreSolveCallback(func(shapeA, shapeB box2d.ShapeID) bool { return true })).To(Succeed())

		shapeOverlap, err := world.OverlapShape()
		Expect(err).NotTo(HaveOccurred())
		Expect(shapeOverlap).To(BeEmpty())

		shapeCast, err := world.CastShape()
		Expect(err).NotTo(HaveOccurred())
		Expect(shapeCast).To(BeEmpty())

		_, err = world.CastMover()
		Expect(err).NotTo(HaveOccurred())
		Expect(world.CollideMover()).To(Succeed())
		Expect(world.Explode()).To(Succeed())
		Expect(world.Draw()).To(Succeed())
		profileAfterDraw, err := world.GetProfile()
		Expect(err).NotTo(HaveOccurred())
		Expect(profileAfterDraw.DrawCount).To(Equal(1))
		memoryStats, err := world.DumpMemoryStats()
		Expect(err).NotTo(HaveOccurred())
		Expect(memoryStats).To(ContainSubstring(`"bodies"`))
		Expect(memoryStats).To(ContainSubstring(`"shapes"`))
		Expect(memoryStats).To(ContainSubstring(`"joints"`))
		Expect(memoryStats).To(ContainSubstring(`"broadPhase"`))
		Expect(world.RebuildStaticTree()).To(Succeed())
		memoryStats, err = world.DumpMemoryStats()
		Expect(err).NotTo(HaveOccurred())
		Expect(memoryStats).To(ContainSubstring(`"rebuilds"`))

		bodyEvents, err := world.GetBodyEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(bodyEvents.Count).To(Equal(0))
		contactEvents, err := world.GetContactEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEvents.Count).To(Equal(0))
		sensorEvents, err := world.GetSensorEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(sensorEvents.Count).To(Equal(0))
		jointEvents, err := world.GetJointEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(jointEvents.Count).To(Equal(0))
		profile, err := world.GetProfile()
		Expect(err).NotTo(HaveOccurred())
		Expect(profile.StepCount).To(Equal(0))
		Expect(profile.BodyCount).To(Equal(0))

		Expect(world.Destroy()).To(Succeed())
		Expect(world.Draw()).To(MatchError(box2d.ErrWorldInvalid))
		_, err = world.CastShape()
		Expect(err).To(MatchError(box2d.ErrWorldInvalid))
		_, err = world.CastMover()
		Expect(err).To(MatchError(box2d.ErrWorldInvalid))
		err = world.CollideMover()
		Expect(err).To(MatchError(box2d.ErrWorldInvalid))
		err = world.Explode()
		Expect(err).To(MatchError(box2d.ErrWorldInvalid))
		_, err = world.OverlapShape()
		Expect(err).To(MatchError(box2d.ErrWorldInvalid))
		_, err = world.GetBodyCount()
		Expect(err).To(MatchError(box2d.ErrWorldInvalid))
		_, err = world.GetShapeCount()
		Expect(err).To(MatchError(box2d.ErrWorldInvalid))
		_, err = world.GetJointCount()
		Expect(err).To(MatchError(box2d.ErrWorldInvalid))
		_, err = world.GetChainCount()
		Expect(err).To(MatchError(box2d.ErrWorldInvalid))
		_, err = world.GetProxyCount()
		Expect(err).To(MatchError(box2d.ErrWorldInvalid))
		_, err = world.GetContactCount()
		Expect(err).To(MatchError(box2d.ErrWorldInvalid))
		_, err = world.GetSensorTouchCount()
		Expect(err).To(MatchError(box2d.ErrWorldInvalid))
	})

	It("applies explosion impulse to nearby dynamic bodies", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyDef.Position = box2d.Vec2{X: 2, Y: 0}
		body, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())
		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1
		_, err = body.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())

		v0, err := body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(v0).To(Equal(box2d.Vec2Zero))

		Expect(world.Explode()).To(Succeed())
		v1, err := body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(box2d.Length(v1)).To(BeNumerically(">", 0.0))
	})

	It("applies configurable explosion impulse with mask filtering", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		makeBody := func(position box2d.Vec2, category uint64) *box2d.Body {
			bodyDef := box2d.DefaultBodyDef()
			bodyDef.Type = box2d.DynamicBody
			bodyDef.Position = position
			body, createErr := world.CreateBody(bodyDef)
			Expect(createErr).NotTo(HaveOccurred())

			shapeDef := box2d.DefaultShapeDef()
			shapeDef.Density = 1
			shapeDef.Filter.CategoryBits = category
			_, createErr = body.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
			Expect(createErr).NotTo(HaveOccurred())
			return body
		}

		bodyInMask := makeBody(box2d.Vec2{X: 2, Y: 0}, 0x0002)
		bodyOutMask := makeBody(box2d.Vec2{X: 2, Y: 1}, 0x0004)

		explosion := box2d.DefaultExplosionDef()
		explosion.Position = box2d.Vec2Zero
		explosion.Radius = 3
		explosion.Falloff = 2
		explosion.ImpulsePerLength = 40
		explosion.MaskBits = 0x0002
		Expect(world.ExplodeWithDef(explosion)).To(Succeed())

		vin, err := bodyInMask.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(box2d.Length(vin)).To(BeNumerically(">", 0.0))

		vout, err := bodyOutMask.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(box2d.Length(vout)).To(BeNumerically("~", 0.0, 1e-6))
	})

	It("applies explosion impulse from the closest shape point and generates torque", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyDef.Position = box2d.Vec2{X: 4.5, Y: 1.0}
		body, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1
		_, err = body.CreatePolygonShape(shapeDef, box2d.MakeBox(2.0, 2.0))
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultExplosionDef()
		def.Position = box2d.Vec2Zero
		def.Radius = 4
		def.ImpulsePerLength = 25
		Expect(world.ExplodeWithDef(def)).To(Succeed())

		v, err := body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(box2d.Length(v)).To(BeNumerically(">", 0.0))

		angularVelocity, err := body.GetAngularVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(box2d.AbsFloat(angularVelocity)).To(BeNumerically(">", 0.0))
	})

	It("applies explosion impulse using matching shape positions, not only body COM", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		body, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1
		shapeDef.Filter.CategoryBits = 0x0002
		_, err = body.CreateCircleShape(shapeDef, box2d.Circle{
			Center: box2d.Vec2{X: -5, Y: 0},
			Radius: 0.5,
		})
		Expect(err).NotTo(HaveOccurred())
		_, err = body.CreateCircleShape(shapeDef, box2d.Circle{
			Center: box2d.Vec2{X: 5, Y: 0},
			Radius: 0.5,
		})
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultExplosionDef()
		def.Position = box2d.Vec2{X: 4, Y: 0}
		def.Radius = 2
		def.ImpulsePerLength = 30
		def.MaskBits = 0x0002
		Expect(world.ExplodeWithDef(def)).To(Succeed())

		v, err := body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(v.X).To(BeNumerically(">", 0.0))
	})

	It("validates explosion definitions", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		err = world.ExplodeWithDef(box2d.ExplosionDef{})
		Expect(err).To(MatchError(box2d.ErrInvalidExplosionDef))

		Expect(world.Destroy()).To(Succeed())
		def := box2d.DefaultExplosionDef()
		err = world.ExplodeWithDef(def)
		Expect(err).To(MatchError(box2d.ErrWorldInvalid))
	})

	It("supports implosion via negative explosion impulse", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyDef.Position = box2d.Vec2{X: 2, Y: 0}
		body, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())
		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1
		_, err = body.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultExplosionDef()
		def.Position = box2d.Vec2Zero
		def.Radius = 4
		def.ImpulsePerLength = -20
		Expect(world.ExplodeWithDef(def)).To(Succeed())

		v, err := body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(v.X).To(BeNumerically("<", 0.0))
	})

	It("applies weaker explosion impulse in the falloff zone", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		makeBody := func(x float32) *box2d.Body {
			bodyDef := box2d.DefaultBodyDef()
			bodyDef.Type = box2d.DynamicBody
			bodyDef.Position = box2d.Vec2{X: x, Y: 0}
			body, createErr := world.CreateBody(bodyDef)
			Expect(createErr).NotTo(HaveOccurred())
			shapeDef := box2d.DefaultShapeDef()
			shapeDef.Density = 1
			_, createErr = body.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
			Expect(createErr).NotTo(HaveOccurred())
			return body
		}

		nearBody := makeBody(2.0)
		farBody := makeBody(6.0)

		def := box2d.DefaultExplosionDef()
		def.Position = box2d.Vec2Zero
		def.Radius = 3
		def.Falloff = 4
		def.ImpulsePerLength = 20
		Expect(world.ExplodeWithDef(def)).To(Succeed())

		vNear, err := nearBody.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		vFar, err := farBody.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())

		Expect(box2d.Length(vNear)).To(BeNumerically(">", box2d.Length(vFar)))
		Expect(box2d.Length(vFar)).To(BeNumerically(">", 0.0))
	})

	It("returns touching non-sensor shapes from OverlapShape", func() {
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

		shapeA, err := bodyA.CreateCircleShape(box2d.DefaultShapeDef(), box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())
		shapeB, err := bodyB.CreateCircleShape(box2d.DefaultShapeDef(), box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		overlaps, err := world.OverlapShape()
		Expect(err).NotTo(HaveOccurred())
		Expect(overlaps).To(HaveLen(2))
		Expect(box2d.ShapeIDEqual(overlaps[0], shapeA.ID())).To(BeTrue())
		Expect(box2d.ShapeIDEqual(overlaps[1], shapeB.ID())).To(BeTrue())
	})

	It("includes sensor overlaps in OverlapShape", func() {
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

		sensorDef := box2d.DefaultShapeDef()
		sensorDef.IsSensor = true
		sensorShape, err := bodyA.CreateCircleShape(sensorDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())
		visitorShape, err := bodyB.CreateCircleShape(box2d.DefaultShapeDef(), box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		overlaps, err := world.OverlapShape()
		Expect(err).NotTo(HaveOccurred())
		Expect(overlaps).To(HaveLen(2))
		Expect(box2d.ShapeIDEqual(overlaps[0], sensorShape.ID())).To(BeTrue())
		Expect(box2d.ShapeIDEqual(overlaps[1], visitorShape.ID())).To(BeTrue())
	})

	It("returns non-empty cast-shape and cast-mover results for touching contacts", func() {
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

		shapeA, err := bodyA.CreateCircleShape(box2d.DefaultShapeDef(), box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(box2d.DefaultShapeDef(), box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		shapeCast, err := world.CastShape()
		Expect(err).NotTo(HaveOccurred())
		Expect(shapeCast).To(HaveLen(2))
		Expect(shapeCast[0].Output.Hit).To(BeTrue())
		Expect(shapeCast[1].Output.Hit).To(BeTrue())
		Expect(box2d.ShapeIDEqual(shapeCast[0].ShapeID, shapeA.ID())).To(BeTrue())

		moverCast, err := world.CastMover()
		Expect(err).NotTo(HaveOccurred())
		Expect(moverCast.Hit).To(BeTrue())

		Expect(world.CollideMover()).To(Succeed())
	})

	It("applies query filters to advanced overlap and cast APIs", func() {
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

		shapeDefA := box2d.DefaultShapeDef()
		shapeDefA.Filter.CategoryBits = 0x0002
		shapeA, err := bodyA.CreateCircleShape(shapeDefA, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		shapeDefB := box2d.DefaultShapeDef()
		shapeDefB.Filter.CategoryBits = 0x0004
		_, err = bodyB.CreateCircleShape(shapeDefB, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		filter := box2d.DefaultQueryFilter()
		filter.MaskBits = 0x0002

		overlaps, err := world.OverlapShapeWithFilter(filter)
		Expect(err).NotTo(HaveOccurred())
		Expect(overlaps).To(HaveLen(1))
		Expect(box2d.ShapeIDEqual(overlaps[0], shapeA.ID())).To(BeTrue())

		hits, err := world.CastShapeWithFilter(filter)
		Expect(err).NotTo(HaveOccurred())
		Expect(hits).To(HaveLen(1))
		Expect(box2d.ShapeIDEqual(hits[0].ShapeID, shapeA.ID())).To(BeTrue())

		moverHit, err := world.CastMoverWithFilter(filter)
		Expect(err).NotTo(HaveOccurred())
		Expect(moverHit.Hit).To(BeTrue())
		Expect(moverHit.Normal.X).To(BeNumerically(">", 0))

		Expect(world.CollideMoverWithFilter(filter)).To(Succeed())
		stats, err := world.DumpMemoryStats()
		Expect(err).NotTo(HaveOccurred())
		Expect(stats).To(ContainSubstring(`"mover":{"matches":1}`))

		filter.MaskBits = 0x0004
		moverHit, err = world.CastMoverWithFilter(filter)
		Expect(err).NotTo(HaveOccurred())
		Expect(moverHit.Hit).To(BeTrue())
		Expect(moverHit.Normal.X).To(BeNumerically("<", 0))

		Expect(world.CollideMoverWithFilter(filter)).To(Succeed())
		stats, err = world.DumpMemoryStats()
		Expect(err).NotTo(HaveOccurred())
		Expect(stats).To(ContainSubstring(`"mover":{"matches":1}`))

		filter.MaskBits = 0x0010
		Expect(world.CollideMoverWithFilter(filter)).To(Succeed())
		stats, err = world.DumpMemoryStats()
		Expect(err).NotTo(HaveOccurred())
		Expect(stats).To(ContainSubstring(`"mover":{"matches":0}`))
	})

	It("reports body move events from simulation steps", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyDef.LinearVelocity = box2d.Vec2{X: 2, Y: 0}
		bodyDef.UserData = "actor"
		body, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		bodyEvents, err := world.GetBodyEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(bodyEvents.Count).To(Equal(0))
		Expect(bodyEvents.MoveCount).To(Equal(0))
		Expect(bodyEvents.MoveEvents).To(BeEmpty())

		Expect(world.Step(1.0, 1)).To(Succeed())
		bodyEvents, err = world.GetBodyEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(bodyEvents.Count).To(Equal(1))
		Expect(bodyEvents.MoveCount).To(Equal(1))
		Expect(bodyEvents.MoveEvents).To(HaveLen(1))
		Expect(box2d.BodyIDEqual(bodyEvents.MoveEvents[0].BodyID, body.ID())).To(BeTrue())
		Expect(bodyEvents.MoveEvents[0].UserData).To(Equal("actor"))
		Expect(bodyEvents.MoveEvents[0].FellAsleep).To(BeFalse())

		transform, err := body.GetTransform()
		Expect(err).NotTo(HaveOccurred())
		Expect(bodyEvents.MoveEvents[0].Transform).To(Equal(transform))

		Expect(body.SetAwake(false)).To(Succeed())
		Expect(world.Step(1.0, 1)).To(Succeed())
		bodyEvents, err = world.GetBodyEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(bodyEvents.Count).To(Equal(0))
		Expect(bodyEvents.MoveCount).To(Equal(0))
		Expect(bodyEvents.MoveEvents).To(BeEmpty())
	})

	It("reports joint events when thresholds are exceeded for awake joints", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyA, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())
		bodyDef.Position = box2d.Vec2{X: 2, Y: 0}
		bodyB, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		joint, err := world.CreateDistanceJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		Expect(joint.SetUserData("joint-user")).To(Succeed())

		Expect(world.Step(1.0/60.0, 1)).To(Succeed())
		jointEvents, err := world.GetJointEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(jointEvents.Count).To(Equal(0))
		Expect(jointEvents.Events).To(BeEmpty())

		Expect(joint.SetForceThreshold(0)).To(Succeed())
		Expect(world.Step(1.0/60.0, 1)).To(Succeed())
		jointEvents, err = world.GetJointEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(jointEvents.Count).To(Equal(1))
		Expect(jointEvents.Events).To(HaveLen(1))
		Expect(box2d.JointIDEqual(jointEvents.Events[0].JointID, joint.ID())).To(BeTrue())
		Expect(jointEvents.Events[0].UserData).To(Equal("joint-user"))

		Expect(bodyA.SetAwake(false)).To(Succeed())
		Expect(bodyB.SetAwake(false)).To(Succeed())
		Expect(world.Step(1.0/60.0, 1)).To(Succeed())
		jointEvents, err = world.GetJointEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(jointEvents.Count).To(Equal(0))
		Expect(jointEvents.Events).To(BeEmpty())
	})

	It("enforces distance-joint length during stepping", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyDef.Position = box2d.Vec2{X: 0, Y: 0}
		bodyA, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())
		bodyDef.Position = box2d.Vec2{X: 5, Y: 0}
		bodyB, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1
		_, err = bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())

		joint, err := world.CreateDistanceJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		Expect(joint.SetLength(2.0)).To(Succeed())

		initialLength, err := joint.GetCurrentLength()
		Expect(err).NotTo(HaveOccurred())
		Expect(initialLength).To(BeNumerically("~", 5.0, 1e-4))

		for range 120 {
			Expect(world.Step(1.0/60.0, 4)).To(Succeed())
		}

		finalLength, err := joint.GetCurrentLength()
		Expect(err).NotTo(HaveOccurred())
		Expect(finalLength).To(BeNumerically("<", initialLength))
		Expect(finalLength).To(BeNumerically("~", 2.0, 0.5))
	})

	It("suppresses contacts for joint-connected bodies when collide-connected is disabled", func() {
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
		shapeDef.Density = 1
		_, err = bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		joint, err := world.CreateDistanceJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())

		contacts, err := world.GetContactEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(contacts.Count).To(Equal(0))

		Expect(joint.SetCollideConnected(true)).To(Succeed())
		contacts, err = world.GetContactEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(contacts.Count).To(BeNumerically(">", 0))
	})

	It("suppresses sensor overlaps for joint-connected bodies when collide-connected is disabled", func() {
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

		sensorDef := box2d.DefaultShapeDef()
		sensorDef.IsSensor = true
		_, err = bodyA.CreateCircleShape(sensorDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1
		_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		joint, err := world.CreateDistanceJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())

		sensors, err := world.GetSensorEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(sensors.Count).To(Equal(0))

		Expect(joint.SetCollideConnected(true)).To(Succeed())
		sensors, err = world.GetSensorEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(sensors.Count).To(BeNumerically(">", 0))
	})

	It("drives relative body velocities with a motor joint", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyA, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())
		bodyB, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1
		_, err = bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())

		joint, err := world.CreateMotorJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		Expect(joint.SetLinearVelocity(box2d.Vec2{X: 3, Y: -1})).To(Succeed())
		Expect(joint.SetAngularVelocity(2.0)).To(Succeed())
		Expect(joint.SetMaxVelocityForce(200)).To(Succeed())
		Expect(joint.SetMaxVelocityTorque(100)).To(Succeed())

		for range 60 {
			Expect(world.Step(1.0/60.0, 2)).To(Succeed())
		}

		vA, err := bodyA.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		vB, err := bodyB.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		relV := box2d.Sub(vB, vA)
		Expect(relV.X).To(BeNumerically("~", 3.0, 0.3))
		Expect(relV.Y).To(BeNumerically("~", -1.0, 0.3))

		wA, err := bodyA.GetAngularVelocity()
		Expect(err).NotTo(HaveOccurred())
		wB, err := bodyB.GetAngularVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(wB - wA).To(BeNumerically("~", 2.0, 0.3))
	})

	It("applies distance-joint motor force during stepping", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyDef.Position = box2d.Vec2{X: 0, Y: 0}
		bodyA, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())
		bodyDef.Position = box2d.Vec2{X: 3, Y: 0}
		bodyB, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1
		_, err = bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())

		joint, err := world.CreateDistanceJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		Expect(joint.SetLength(3.0)).To(Succeed())
		Expect(joint.EnableMotor(true)).To(Succeed())
		Expect(joint.SetMotorSpeed(4.0)).To(Succeed())
		Expect(joint.SetMaxMotorForce(100)).To(Succeed())

		Expect(world.Step(1.0/60.0, 4)).To(Succeed())
		motorForce, err := joint.GetMotorForce()
		Expect(err).NotTo(HaveOccurred())
		Expect(motorForce).To(BeNumerically(">", 0.0))
	})

	It("applies motor-joint spring force to reduce anchor error", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyDef.Position = box2d.Vec2{X: 0, Y: 0}
		bodyA, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())
		bodyDef.Position = box2d.Vec2{X: 4, Y: 0}
		bodyB, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1
		_, err = bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())

		joint, err := world.CreateMotorJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		Expect(joint.SetMaxVelocityForce(0)).To(Succeed())
		Expect(joint.SetLinearHertz(6)).To(Succeed())
		Expect(joint.SetMaxSpringForce(200)).To(Succeed())

		initialSep, err := joint.GetLinearSeparation()
		Expect(err).NotTo(HaveOccurred())
		Expect(initialSep).To(BeNumerically(">", 0.0))

		for range 120 {
			Expect(world.Step(1.0/60.0, 4)).To(Succeed())
		}

		finalSep, err := joint.GetLinearSeparation()
		Expect(err).NotTo(HaveOccurred())
		Expect(finalSep).To(BeNumerically("<", initialSep))
	})

	It("applies prismatic motor force during stepping", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyA, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())
		bodyB, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1
		_, err = bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())

		joint, err := world.CreatePrismaticJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		Expect(joint.EnableMotor(true)).To(Succeed())
		Expect(joint.SetMotorSpeed(2.5)).To(Succeed())
		Expect(joint.SetMaxMotorForce(80)).To(Succeed())

		Expect(world.Step(1.0/60.0, 2)).To(Succeed())
		force, err := joint.GetMotorForce()
		Expect(err).NotTo(HaveOccurred())
		Expect(force).To(BeNumerically(">", 0.0))
	})

	It("applies revolute motor torque during stepping", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyA, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())
		bodyB, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1
		_, err = bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())

		joint, err := world.CreateRevoluteJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		Expect(joint.EnableMotor(true)).To(Succeed())
		Expect(joint.SetMotorSpeed(3.0)).To(Succeed())
		Expect(joint.SetMaxMotorTorque(60)).To(Succeed())

		Expect(world.Step(1.0/60.0, 2)).To(Succeed())
		torque, err := joint.GetMotorTorque()
		Expect(err).NotTo(HaveOccurred())
		Expect(torque).To(BeNumerically(">", 0.0))
		constraintTorque, err := joint.GetConstraintTorque()
		Expect(err).NotTo(HaveOccurred())
		Expect(constraintTorque).To(BeNumerically(">", 0.0))
	})

	It("applies wheel motor torque during stepping", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyA, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())
		bodyB, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1
		_, err = bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())

		joint, err := world.CreateWheelJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		Expect(joint.EnableMotor(true)).To(Succeed())
		Expect(joint.SetMotorSpeed(3.0)).To(Succeed())
		Expect(joint.SetMaxMotorTorque(60)).To(Succeed())

		Expect(world.Step(1.0/60.0, 2)).To(Succeed())
		torque, err := joint.GetMotorTorque()
		Expect(err).NotTo(HaveOccurred())
		Expect(torque).To(BeNumerically(">", 0.0))
	})

	It("enforces prismatic limits during stepping", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyDef.Position = box2d.Vec2{X: 0, Y: 0}
		bodyA, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())
		bodyDef.Position = box2d.Vec2{X: 4, Y: 0}
		bodyB, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1
		_, err = bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())

		joint, err := world.CreatePrismaticJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		Expect(joint.EnableLimit(true)).To(Succeed())
		Expect(joint.SetLimits(-1, 1)).To(Succeed())

		for range 120 {
			Expect(world.Step(1.0/60.0, 2)).To(Succeed())
		}

		translation, err := joint.GetTranslation()
		Expect(err).NotTo(HaveOccurred())
		Expect(translation).To(BeNumerically("<=", 1.2))
	})

	It("drives revolute spring target angle during stepping", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyDef.Rotation = box2d.MakeRot(0)
		bodyA, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())
		bodyDef.Rotation = box2d.MakeRot(1.0)
		bodyB, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1
		_, err = bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())

		joint, err := world.CreateRevoluteJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		Expect(joint.EnableSpring(true)).To(Succeed())
		Expect(joint.SetTargetAngle(0.2)).To(Succeed())
		Expect(joint.SetSpringHertz(6)).To(Succeed())

		initialAngle, err := joint.GetAngle()
		Expect(err).NotTo(HaveOccurred())
		for range 120 {
			Expect(world.Step(1.0/60.0, 2)).To(Succeed())
		}
		finalAngle, err := joint.GetAngle()
		Expect(err).NotTo(HaveOccurred())
		Expect(box2d.AbsFloat(finalAngle - 0.2)).To(BeNumerically("<", box2d.AbsFloat(initialAngle-0.2)))
	})

	It("enforces wheel limits during stepping", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyDef.Position = box2d.Vec2{X: 0, Y: 0}
		bodyA, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())
		bodyDef.Position = box2d.Vec2{X: 4, Y: 0}
		bodyB, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1
		_, err = bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())

		joint, err := world.CreateWheelJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		Expect(joint.EnableLimit(true)).To(Succeed())
		Expect(joint.SetLimits(-1, 1)).To(Succeed())

		for range 120 {
			Expect(world.Step(1.0/60.0, 2)).To(Succeed())
		}

		posA, err := bodyA.GetPosition()
		Expect(err).NotTo(HaveOccurred())
		posB, err := bodyB.GetPosition()
		Expect(err).NotTo(HaveOccurred())
		Expect(box2d.AbsFloat(posB.X - posA.X)).To(BeNumerically("<=", 1.2))
	})

	It("reduces linear and angular separation with weld joint solving", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyDef.Position = box2d.Vec2{X: 0, Y: 0}
		bodyDef.Rotation = box2d.MakeRot(0)
		bodyA, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())
		bodyDef.Position = box2d.Vec2{X: 3, Y: 2}
		bodyDef.Rotation = box2d.MakeRot(1.0)
		bodyB, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1
		_, err = bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())

		joint, err := world.CreateWeldJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		Expect(joint.SetLinearHertz(6)).To(Succeed())
		Expect(joint.SetAngularHertz(6)).To(Succeed())

		initialLinear, err := joint.GetLinearSeparation()
		Expect(err).NotTo(HaveOccurred())
		initialAngular, err := joint.GetAngularSeparation()
		Expect(err).NotTo(HaveOccurred())
		for range 120 {
			Expect(world.Step(1.0/60.0, 2)).To(Succeed())
		}
		finalLinear, err := joint.GetLinearSeparation()
		Expect(err).NotTo(HaveOccurred())
		finalAngular, err := joint.GetAngularSeparation()
		Expect(err).NotTo(HaveOccurred())

		Expect(finalLinear).To(BeNumerically("<", initialLinear))
		Expect(box2d.AbsFloat(finalAngular)).To(BeNumerically("<", box2d.AbsFloat(initialAngular)))
	})

	It("enforces distance-joint limits during stepping", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyDef.Position = box2d.Vec2{X: 0, Y: 0}
		bodyA, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())
		bodyDef.Position = box2d.Vec2{X: 4, Y: 0}
		bodyB, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1
		_, err = bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())

		joint, err := world.CreateDistanceJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		Expect(joint.EnableLimit(true)).To(Succeed())
		Expect(joint.SetLengthRange(1.0, 2.0)).To(Succeed())

		for range 120 {
			Expect(world.Step(1.0/60.0, 2)).To(Succeed())
		}

		length, err := joint.GetCurrentLength()
		Expect(err).NotTo(HaveOccurred())
		Expect(length).To(BeNumerically("<=", 2.2))
	})

	It("enforces revolute limits during stepping", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyDef.Rotation = box2d.MakeRot(0)
		bodyA, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())
		bodyDef.Rotation = box2d.MakeRot(1.2)
		bodyB, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1
		_, err = bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())

		joint, err := world.CreateRevoluteJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		Expect(joint.EnableLimit(true)).To(Succeed())
		Expect(joint.SetLimits(-0.2, 0.2)).To(Succeed())

		for range 120 {
			Expect(world.Step(1.0/60.0, 2)).To(Succeed())
		}

		angle, err := joint.GetAngle()
		Expect(err).NotTo(HaveOccurred())
		Expect(box2d.AbsFloat(angle)).To(BeNumerically("<=", 0.3))
	})

	It("reports joint events for positive thresholds from estimated constraint magnitudes", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyDef.Position = box2d.Vec2{X: 0, Y: 0}
		bodyA, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef.Position = box2d.Vec2{X: 5, Y: 0}
		bodyB, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		joint, err := world.CreateDistanceJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())

		Expect(joint.SetForceThreshold(1000)).To(Succeed())
		Expect(world.Step(1.0/60.0, 1)).To(Succeed())
		jointEvents, err := world.GetJointEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(jointEvents.Count).To(Equal(1))

		Expect(joint.SetForceThreshold(20000)).To(Succeed())
		Expect(world.Step(1.0/60.0, 1)).To(Succeed())
		jointEvents, err = world.GetJointEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(jointEvents.Count).To(Equal(0))
	})

	It("reports joint events for torque thresholds from angular separation", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyDef.Rotation = box2d.MakeRot(0)
		bodyA, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef.Rotation = box2d.MakeRot(1.0)
		bodyB, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		joint, err := world.CreateRevoluteJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())

		Expect(joint.SetTorqueThreshold(500)).To(Succeed())
		Expect(world.Step(1.0/60.0, 1)).To(Succeed())
		jointEvents, err := world.GetJointEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(jointEvents.Count).To(Equal(1))

		Expect(joint.SetTorqueThreshold(10000)).To(Succeed())
		Expect(world.Step(1.0/60.0, 1)).To(Succeed())
		jointEvents, err = world.GetJointEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(jointEvents.Count).To(Equal(0))
	})

	It("reports motor-joint events from velocity target mismatch", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyA, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())
		bodyB, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		joint, err := world.CreateMotorJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		Expect(joint.SetLinearVelocity(box2d.Vec2{X: 10, Y: 0})).To(Succeed())
		Expect(joint.SetForceThreshold(1000)).To(Succeed())

		Expect(world.Step(1.0/60.0, 1)).To(Succeed())
		jointEvents, err := world.GetJointEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(jointEvents.Count).To(Equal(1))

		Expect(joint.SetForceThreshold(100000)).To(Succeed())
		Expect(world.Step(1.0/60.0, 1)).To(Succeed())
		jointEvents, err = world.GetJointEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(jointEvents.Count).To(Equal(0))
	})

	It("reports motor-joint events from angular velocity mismatch", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyA, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())
		bodyB, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		joint, err := world.CreateMotorJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		Expect(joint.SetAngularVelocity(5)).To(Succeed())
		Expect(joint.SetTorqueThreshold(1000)).To(Succeed())

		Expect(world.Step(1.0/60.0, 1)).To(Succeed())
		jointEvents, err := world.GetJointEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(jointEvents.Count).To(Equal(1))

		Expect(joint.SetTorqueThreshold(100000)).To(Succeed())
		Expect(world.Step(1.0/60.0, 1)).To(Succeed())
		jointEvents, err = world.GetJointEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(jointEvents.Count).To(Equal(0))
	})

	It("reports contact and sensor event counts from current overlaps", func() {
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
		_, err = bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		contactEvents, err := world.GetContactEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEvents.Count).To(Equal(1))

		sensorDef := box2d.DefaultShapeDef()
		sensorDef.IsSensor = true
		_, err = bodyA.CreateCircleShape(sensorDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		sensorEvents, err := world.GetSensorEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(sensorEvents.Count).To(Equal(1))
		contactCount, err := world.GetContactCount()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactCount).To(Equal(1))
		sensorTouchCount, err := world.GetSensorTouchCount()
		Expect(err).NotTo(HaveOccurred())
		Expect(sensorTouchCount).To(Equal(1))
	})

	It("avoids circle-circle contact false positives from AABB-only overlap", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		defA := box2d.DefaultBodyDef()
		defA.Type = box2d.DynamicBody
		bodyA, err := world.CreateBody(defA)
		Expect(err).NotTo(HaveOccurred())

		defB := box2d.DefaultBodyDef()
		defB.Type = box2d.DynamicBody
		defB.Position = box2d.Vec2{X: 1.9, Y: 1.9}
		bodyB, err := world.CreateBody(defB)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeA, err := bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())
		shapeB, err := bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		contactEvents, err := world.GetContactEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEvents.Count).To(Equal(0))
		Expect(contactEvents.BeginCount).To(Equal(0))

		capacity, err := shapeA.GetContactCapacity()
		Expect(err).NotTo(HaveOccurred())
		Expect(capacity).To(Equal(0))

		data, err := shapeA.GetContactData(4)
		Expect(err).NotTo(HaveOccurred())
		Expect(data).To(BeEmpty())

		capacityB, err := shapeB.GetContactCapacity()
		Expect(err).NotTo(HaveOccurred())
		Expect(capacityB).To(Equal(0))
	})

	It("tracks contact and sensor begin/end transitions", func() {
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
		_, err = bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		contactEvents, err := world.GetContactEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEvents.BeginCount).To(Equal(1))
		Expect(contactEvents.EndCount).To(Equal(0))
		Expect(contactEvents.Count).To(Equal(1))

		contactEvents, err = world.GetContactEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEvents.BeginCount).To(Equal(0))
		Expect(contactEvents.EndCount).To(Equal(0))
		Expect(contactEvents.Count).To(Equal(1))

		Expect(bodyB.SetTransform(box2d.Vec2{X: 5, Y: 0}, box2d.RotIdentity)).To(Succeed())
		contactEvents, err = world.GetContactEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEvents.BeginCount).To(Equal(0))
		Expect(contactEvents.EndCount).To(Equal(1))
		Expect(contactEvents.Count).To(Equal(0))

		sensorDef := box2d.DefaultShapeDef()
		sensorDef.IsSensor = true
		Expect(bodyB.SetTransform(box2d.Vec2{X: 0.5, Y: 0}, box2d.RotIdentity)).To(Succeed())
		_, err = bodyA.CreateCircleShape(sensorDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		sensorEvents, err := world.GetSensorEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(sensorEvents.BeginCount).To(Equal(1))
		Expect(sensorEvents.EndCount).To(Equal(0))
		Expect(sensorEvents.Count).To(Equal(1))

		Expect(bodyB.SetTransform(box2d.Vec2{X: 5, Y: 0}, box2d.RotIdentity)).To(Succeed())
		sensorEvents, err = world.GetSensorEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(sensorEvents.BeginCount).To(Equal(0))
		Expect(sensorEvents.EndCount).To(Equal(1))
		Expect(sensorEvents.Count).To(Equal(0))
	})

	It("reports hit events when enabled and approach speed exceeds threshold", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.HitEventThreshold = 1.0
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		defA := box2d.DefaultBodyDef()
		defA.Type = box2d.DynamicBody
		defA.Position = box2d.Vec2{X: -0.9, Y: 0}
		defA.LinearVelocity = box2d.Vec2{X: 5, Y: 0}
		bodyA, err := world.CreateBody(defA)
		Expect(err).NotTo(HaveOccurred())

		defB := box2d.DefaultBodyDef()
		defB.Type = box2d.DynamicBody
		defB.Position = box2d.Vec2{X: 0.9, Y: 0}
		defB.LinearVelocity = box2d.Vec2{X: -5, Y: 0}
		bodyB, err := world.CreateBody(defB)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.EnableHitEvents = true
		_, err = bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(box2d.DefaultShapeDef(), box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		contactEvents, err := world.GetContactEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEvents.HitCount).To(Equal(1))
		Expect(contactEvents.HitEvents).To(HaveLen(1))
		Expect(contactEvents.HitEvents[0].ApproachSpeed).To(BeNumerically(">", 1.0))
		Expect(contactEvents.HitEvents[0].Normal.X).To(BeNumerically("~", 1.0, 1e-5))
	})

	It("suppresses hit events below threshold or when disabled", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.HitEventThreshold = 20.0
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		defA := box2d.DefaultBodyDef()
		defA.Type = box2d.DynamicBody
		defA.Position = box2d.Vec2{X: -0.9, Y: 0}
		defA.LinearVelocity = box2d.Vec2{X: 5, Y: 0}
		bodyA, err := world.CreateBody(defA)
		Expect(err).NotTo(HaveOccurred())

		defB := box2d.DefaultBodyDef()
		defB.Type = box2d.DynamicBody
		defB.Position = box2d.Vec2{X: 0.9, Y: 0}
		defB.LinearVelocity = box2d.Vec2{X: -5, Y: 0}
		bodyB, err := world.CreateBody(defB)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.EnableHitEvents = true
		shapeA, err := bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(box2d.DefaultShapeDef(), box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		contactEvents, err := world.GetContactEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEvents.HitCount).To(Equal(0))

		Expect(shapeA.EnableHitEvents(false)).To(Succeed())
		Expect(world.SetHitEventThreshold(1.0)).To(Succeed())
		contactEvents, err = world.GetContactEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEvents.HitCount).To(Equal(0))
	})

	It("does not emit hit events when relative motion is separating", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.HitEventThreshold = -1.0
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		defA := box2d.DefaultBodyDef()
		defA.Type = box2d.DynamicBody
		defA.Position = box2d.Vec2{X: -0.9, Y: 0}
		defA.LinearVelocity = box2d.Vec2{X: -5, Y: 0}
		bodyA, err := world.CreateBody(defA)
		Expect(err).NotTo(HaveOccurred())

		defB := box2d.DefaultBodyDef()
		defB.Type = box2d.DynamicBody
		defB.Position = box2d.Vec2{X: 0.9, Y: 0}
		defB.LinearVelocity = box2d.Vec2{X: 5, Y: 0}
		bodyB, err := world.CreateBody(defB)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.EnableHitEvents = true
		_, err = bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(box2d.DefaultShapeDef(), box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		contactEvents, err := world.GetContactEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEvents.HitCount).To(Equal(0))
	})

	It("applies custom filter callback when custom filtering is enabled", func() {
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
		shapeDef.EnableCustomFiltering = true
		_, err = bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		Expect(world.SetCustomFilterCallback(func(shapeA, shapeB box2d.ShapeID) bool { return false })).To(Succeed())
		contactEvents, err := world.GetContactEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEvents.Count).To(Equal(0))

		Expect(world.SetCustomFilterCallback(func(shapeA, shapeB box2d.ShapeID) bool { return true })).To(Succeed())
		contactEvents, err = world.GetContactEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEvents.Count).To(Equal(1))
	})

	It("treats chain segments as one-sided for contact generation", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		ground, err := world.CreateBody(box2d.DefaultBodyDef())
		Expect(err).NotTo(HaveOccurred())

		chainDef := box2d.DefaultChainDef()
		chainDef.Points = []box2d.Vec2{
			{X: -2, Y: 0},
			{X: -1, Y: 0},
			{X: 1, Y: 0},
			{X: 2, Y: 0},
		}
		chainDef.Count = len(chainDef.Points)
		_, err = world.CreateChain(ground.ID(), chainDef)
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		def.Position = box2d.Vec2{X: 0, Y: 0.5}
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())
		_, err = body.CreateCircleShape(box2d.DefaultShapeDef(), box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		contactEvents, err := world.GetContactEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEvents.Count).To(Equal(0))

		Expect(body.SetTransform(box2d.Vec2{X: 0, Y: -0.5}, box2d.RotIdentity)).To(Succeed())
		contactEvents, err = world.GetContactEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEvents.Count).To(Equal(1))
	})

	It("suppresses inside-corner ghost contacts for chain segments", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		ground, err := world.CreateBody(box2d.DefaultBodyDef())
		Expect(err).NotTo(HaveOccurred())

		chainDef := box2d.DefaultChainDef()
		chainDef.Points = []box2d.Vec2{
			{X: -2, Y: 0},
			{X: -1, Y: 0},
			{X: 0, Y: 0},
			{X: 0, Y: 1},
			{X: 0, Y: 2},
		}
		chainDef.Count = len(chainDef.Points)
		_, err = world.CreateChain(ground.ID(), chainDef)
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		def.Position = box2d.Vec2{X: -0.2, Y: 0.2}
		body, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())
		_, err = body.CreateCircleShape(box2d.DefaultShapeDef(), box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.35})
		Expect(err).NotTo(HaveOccurred())

		contactEvents, err := world.GetContactEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEvents.Count).To(Equal(0))

		Expect(body.SetTransform(box2d.Vec2{X: -0.2, Y: -0.2}, box2d.RotIdentity)).To(Succeed())
		contactEvents, err = world.GetContactEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEvents.Count).To(Equal(1))
	})

	It("does not invoke pre-solve callback unless enabled on at least one shape", func() {
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
		_, err = bodyA.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		callbackCount := 0
		Expect(world.SetPreSolveCallback(func(shapeA, shapeB box2d.ShapeID) bool {
			callbackCount++
			return false
		})).To(Succeed())

		contactEvents, err := world.GetContactEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(callbackCount).To(Equal(0))
		Expect(contactEvents.Count).To(Equal(1))
	})

	It("invokes pre-solve callback when enabled on one touching shape", func() {
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
		_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		Expect(shapeA.EnablePreSolveEvents(true)).To(Succeed())

		callbackCount := 0
		Expect(world.SetPreSolveCallback(func(shapeA, shapeB box2d.ShapeID) bool {
			callbackCount++
			return true
		})).To(Succeed())

		contactEvents, err := world.GetContactEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(callbackCount).To(Equal(1))
		Expect(contactEvents.Count).To(Equal(1))
	})

	It("applies pre-solve callback to suppress contacts when enabled", func() {
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
		_, err = bodyB.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		Expect(shapeA.EnablePreSolveEvents(true)).To(Succeed())
		Expect(world.SetPreSolveCallback(func(shapeA, shapeB box2d.ShapeID) bool { return false })).To(Succeed())
		contactEvents, err := world.GetContactEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEvents.Count).To(Equal(0))

		Expect(shapeA.EnablePreSolveEvents(false)).To(Succeed())
		Expect(world.SetPreSolveCallback(func(shapeA, shapeB box2d.ShapeID) bool { return true })).To(Succeed())
		contactEvents, err = world.GetContactEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEvents.Count).To(Equal(1))
	})

	It("does not suppress sensor events via pre-solve callback", func() {
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

		sensorDef := box2d.DefaultShapeDef()
		sensorDef.IsSensor = true
		_, err = bodyA.CreateCircleShape(sensorDef, box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCircleShape(box2d.DefaultShapeDef(), box2d.Circle{Center: box2d.Vec2Zero, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		Expect(world.SetPreSolveCallback(func(shapeA, shapeB box2d.ShapeID) bool { return false })).To(Succeed())
		sensorEvents, err := world.GetSensorEvents()
		Expect(err).NotTo(HaveOccurred())
		Expect(sensorEvents.Count).To(Equal(1))
	})
})
