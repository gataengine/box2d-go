package box2d_test

import (
	"github.com/gataengine/box2d-go"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

var _ = Describe("Joint Lifecycle", func() {
	It("creates, queries, and destroys a distance joint", func() {
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
		Expect(joint.IsValid()).To(BeTrue())
		Expect(box2d.IsJointIDValid(joint.ID())).To(BeTrue())

		jointType, err := joint.GetType()
		Expect(err).NotTo(HaveOccurred())
		Expect(jointType).To(Equal(box2d.DistanceJoint))

		a, err := joint.GetBodyA()
		Expect(err).NotTo(HaveOccurred())
		Expect(box2d.BodyIDEqual(a, bodyA.ID())).To(BeTrue())
		b, err := joint.GetBodyB()
		Expect(err).NotTo(HaveOccurred())
		Expect(box2d.BodyIDEqual(b, bodyB.ID())).To(BeTrue())

		worldID, err := joint.GetWorld()
		Expect(err).NotTo(HaveOccurred())
		Expect(worldID).To(Equal(world.ID()))

		Expect(joint.SetUserData("link")).To(Succeed())
		userData, err := joint.UserData()
		Expect(err).NotTo(HaveOccurred())
		Expect(userData).To(Equal("link"))
		userData, err = joint.GetUserData()
		Expect(err).NotTo(HaveOccurred())
		Expect(userData).To(Equal("link"))

		frameA := box2d.Transform{P: box2d.Vec2{X: 0.25, Y: 0}, Q: box2d.MakeRot(0.1)}
		frameB := box2d.Transform{P: box2d.Vec2{X: -0.25, Y: 0}, Q: box2d.MakeRot(-0.2)}
		Expect(joint.SetLocalFrameA(frameA)).To(Succeed())
		Expect(joint.SetLocalFrameB(frameB)).To(Succeed())
		gotFrameA, err := joint.GetLocalFrameA()
		Expect(err).NotTo(HaveOccurred())
		Expect(gotFrameA).To(Equal(frameA))
		gotFrameB, err := joint.GetLocalFrameB()
		Expect(err).NotTo(HaveOccurred())
		Expect(gotFrameB).To(Equal(frameB))

		Expect(joint.SetCollideConnected(true)).To(Succeed())
		collideConnected, err := joint.GetCollideConnected()
		Expect(err).NotTo(HaveOccurred())
		Expect(collideConnected).To(BeTrue())

		Expect(joint.SetConstraintTuning(5, 0.7)).To(Succeed())
		hz, damping, err := joint.GetConstraintTuning()
		Expect(err).NotTo(HaveOccurred())
		Expect(hz).To(BeNumerically("~", 5.0, 1e-6))
		Expect(damping).To(BeNumerically("~", 0.7, 1e-6))

		Expect(joint.SetForceThreshold(10)).To(Succeed())
		forceThreshold, err := joint.GetForceThreshold()
		Expect(err).NotTo(HaveOccurred())
		Expect(forceThreshold).To(BeNumerically("~", 10.0, 1e-6))

		Expect(joint.SetTorqueThreshold(3)).To(Succeed())
		torqueThreshold, err := joint.GetTorqueThreshold()
		Expect(err).NotTo(HaveOccurred())
		Expect(torqueThreshold).To(BeNumerically("~", 3.0, 1e-6))

		Expect(joint.SetLength(2.5)).To(Succeed())
		length, err := joint.GetLength()
		Expect(err).NotTo(HaveOccurred())
		Expect(length).To(BeNumerically("~", 2.5, 1e-6))

		Expect(joint.EnableSpring(true)).To(Succeed())
		springEnabled, err := joint.IsSpringEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(springEnabled).To(BeTrue())

		Expect(joint.SetSpringForceRange(9, -3)).To(Succeed())
		lowerSpringForce, upperSpringForce, err := joint.GetSpringForceRange()
		Expect(err).NotTo(HaveOccurred())
		Expect(lowerSpringForce).To(BeNumerically("~", -3.0, 1e-6))
		Expect(upperSpringForce).To(BeNumerically("~", 9.0, 1e-6))

		Expect(joint.SetSpringHertz(8)).To(Succeed())
		springHertz, err := joint.GetSpringHertz()
		Expect(err).NotTo(HaveOccurred())
		Expect(springHertz).To(BeNumerically("~", 8.0, 1e-6))

		Expect(joint.SetSpringDampingRatio(0.4)).To(Succeed())
		springDamping, err := joint.GetSpringDampingRatio()
		Expect(err).NotTo(HaveOccurred())
		Expect(springDamping).To(BeNumerically("~", 0.4, 1e-6))

		Expect(joint.EnableLimit(true)).To(Succeed())
		limitEnabled, err := joint.IsLimitEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(limitEnabled).To(BeTrue())

		Expect(joint.SetLengthRange(5, 2)).To(Succeed())
		minLength, err := joint.GetMinLength()
		Expect(err).NotTo(HaveOccurred())
		Expect(minLength).To(BeNumerically("~", 2.0, 1e-6))
		maxLength, err := joint.GetMaxLength()
		Expect(err).NotTo(HaveOccurred())
		Expect(maxLength).To(BeNumerically("~", 5.0, 1e-6))

		currentLength, err := joint.GetCurrentLength()
		Expect(err).NotTo(HaveOccurred())
		Expect(currentLength).To(BeNumerically(">", 0.0))

		Expect(joint.EnableMotor(true)).To(Succeed())
		motorEnabled, err := joint.IsMotorEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(motorEnabled).To(BeTrue())

		Expect(joint.SetMotorSpeed(1.25)).To(Succeed())
		motorSpeed, err := joint.GetMotorSpeed()
		Expect(err).NotTo(HaveOccurred())
		Expect(motorSpeed).To(BeNumerically("~", 1.25, 1e-6))

		Expect(joint.SetMaxMotorForce(22)).To(Succeed())
		maxMotorForce, err := joint.GetMaxMotorForce()
		Expect(err).NotTo(HaveOccurred())
		Expect(maxMotorForce).To(BeNumerically("~", 22.0, 1e-6))

		motorForce, err := joint.GetMotorForce()
		Expect(err).NotTo(HaveOccurred())
		Expect(motorForce).To(BeNumerically("~", 0.0, 1e-6))
		Expect(joint.SetLinearVelocity(box2d.Vec2{X: 1, Y: 0})).To(MatchError(box2d.ErrJointTypeMismatch))

		constraintForce, err := joint.GetConstraintForce()
		Expect(err).NotTo(HaveOccurred())
		Expect(box2d.Length(constraintForce)).To(BeNumerically(">", 0.0))
		constraintTorque, err := joint.GetConstraintTorque()
		Expect(err).NotTo(HaveOccurred())
		Expect(constraintTorque).To(BeNumerically(">", 0.0))

		linearSep, err := joint.GetLinearSeparation()
		Expect(err).NotTo(HaveOccurred())
		Expect(linearSep).To(BeNumerically(">", 0.0))
		angularSep, err := joint.GetAngularSeparation()
		Expect(err).NotTo(HaveOccurred())
		Expect(angularSep).To(BeNumerically("~", -0.3, 1e-5))

		Expect(joint.Destroy()).To(Succeed())
		Expect(joint.IsValid()).To(BeFalse())
		Expect(box2d.IsJointIDValid(joint.ID())).To(BeFalse())
		Expect(joint.Destroy()).To(MatchError(box2d.ErrJointInvalid))
		_, err = joint.GetConstraintForce()
		Expect(err).To(MatchError(box2d.ErrJointInvalid))
		_, err = joint.GetConstraintTorque()
		Expect(err).To(MatchError(box2d.ErrJointInvalid))
	})

	It("creates a distance joint from definition", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())
		_, err = world.CreateDistanceJointWithDef(box2d.DistanceJointDef{})
		Expect(err).To(MatchError(box2d.ErrInvalidDistanceJointDef))

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		def.Position = box2d.Vec2{X: 0, Y: 0}
		bodyA, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		def.Position = box2d.Vec2{X: 3, Y: 4}
		bodyB, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		jointDef := box2d.DefaultDistanceJointDef()
		jointDef.Base.BodyIDA = bodyA.ID()
		jointDef.Base.BodyIDB = bodyB.ID()
		jointDef.Base.LocalFrameA = box2d.Transform{P: box2d.Vec2{X: 1, Y: 0}, Q: box2d.RotIdentity}
		jointDef.Base.LocalFrameB = box2d.Transform{P: box2d.Vec2{X: -1, Y: 0}, Q: box2d.RotIdentity}
		jointDef.Base.CollideConnected = true
		jointDef.Base.ConstraintHertz = 7
		jointDef.Base.ConstraintDampingRatio = 0.5
		jointDef.Base.ForceThreshold = 15
		jointDef.Base.TorqueThreshold = 5
		jointDef.Length = 2.5
		jointDef.EnableSpring = true
		jointDef.LowerSpringForce = -3
		jointDef.UpperSpringForce = 11
		jointDef.Hertz = 8
		jointDef.DampingRatio = 0.4
		jointDef.EnableLimit = true
		jointDef.MinLength = 1.2
		jointDef.MaxLength = 6.8
		jointDef.EnableMotor = true
		jointDef.MaxMotorForce = 30
		jointDef.MotorSpeed = 1.5

		joint, err := world.CreateDistanceJointWithDef(jointDef)
		Expect(err).NotTo(HaveOccurred())

		frameA, err := joint.GetLocalFrameA()
		Expect(err).NotTo(HaveOccurred())
		Expect(frameA).To(Equal(jointDef.Base.LocalFrameA))
		frameB, err := joint.GetLocalFrameB()
		Expect(err).NotTo(HaveOccurred())
		Expect(frameB).To(Equal(jointDef.Base.LocalFrameB))

		collideConnected, err := joint.GetCollideConnected()
		Expect(err).NotTo(HaveOccurred())
		Expect(collideConnected).To(BeTrue())

		hz, damping, err := joint.GetConstraintTuning()
		Expect(err).NotTo(HaveOccurred())
		Expect(hz).To(BeNumerically("~", 7.0, 1e-6))
		Expect(damping).To(BeNumerically("~", 0.5, 1e-6))

		forceThreshold, err := joint.GetForceThreshold()
		Expect(err).NotTo(HaveOccurred())
		Expect(forceThreshold).To(BeNumerically("~", 15.0, 1e-6))
		torqueThreshold, err := joint.GetTorqueThreshold()
		Expect(err).NotTo(HaveOccurred())
		Expect(torqueThreshold).To(BeNumerically("~", 5.0, 1e-6))

		length, err := joint.GetLength()
		Expect(err).NotTo(HaveOccurred())
		Expect(length).To(BeNumerically("~", 2.5, 1e-6))

		springEnabled, err := joint.IsSpringEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(springEnabled).To(BeTrue())
		lowerSpringForce, upperSpringForce, err := joint.GetSpringForceRange()
		Expect(err).NotTo(HaveOccurred())
		Expect(lowerSpringForce).To(BeNumerically("~", -3.0, 1e-6))
		Expect(upperSpringForce).To(BeNumerically("~", 11.0, 1e-6))
		springHertz, err := joint.GetSpringHertz()
		Expect(err).NotTo(HaveOccurred())
		Expect(springHertz).To(BeNumerically("~", 8.0, 1e-6))
		springDamping, err := joint.GetSpringDampingRatio()
		Expect(err).NotTo(HaveOccurred())
		Expect(springDamping).To(BeNumerically("~", 0.4, 1e-6))

		limitEnabled, err := joint.IsLimitEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(limitEnabled).To(BeTrue())
		minLength, err := joint.GetMinLength()
		Expect(err).NotTo(HaveOccurred())
		Expect(minLength).To(BeNumerically("~", 1.2, 1e-6))
		maxLength, err := joint.GetMaxLength()
		Expect(err).NotTo(HaveOccurred())
		Expect(maxLength).To(BeNumerically("~", 6.8, 1e-6))

		currentLength, err := joint.GetCurrentLength()
		Expect(err).NotTo(HaveOccurred())
		Expect(currentLength).To(BeNumerically("~", 4.1231055, 1e-5))

		motorEnabled, err := joint.IsMotorEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(motorEnabled).To(BeTrue())
		motorSpeed, err := joint.GetMotorSpeed()
		Expect(err).NotTo(HaveOccurred())
		Expect(motorSpeed).To(BeNumerically("~", 1.5, 1e-6))
		maxMotorForce, err := joint.GetMaxMotorForce()
		Expect(err).NotTo(HaveOccurred())
		Expect(maxMotorForce).To(BeNumerically("~", 30.0, 1e-6))
	})

	It("creates a motor joint from definition", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())
		_, err = world.CreateMotorJointWithDef(box2d.MotorJointDef{})
		Expect(err).To(MatchError(box2d.ErrInvalidMotorJointDef))

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		bodyA, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())
		bodyB, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		jointDef := box2d.DefaultMotorJointDef()
		jointDef.Base.BodyIDA = bodyA.ID()
		jointDef.Base.BodyIDB = bodyB.ID()
		jointDef.Base.LocalFrameA = box2d.Transform{P: box2d.Vec2{X: 0.2, Y: 0.3}, Q: box2d.MakeRot(0.1)}
		jointDef.Base.LocalFrameB = box2d.Transform{P: box2d.Vec2{X: -0.4, Y: 0.5}, Q: box2d.MakeRot(-0.2)}
		jointDef.Base.ConstraintHertz = 9
		jointDef.Base.ConstraintDampingRatio = 0.6
		jointDef.Base.ForceThreshold = 17
		jointDef.Base.TorqueThreshold = 4
		jointDef.Base.CollideConnected = true
		jointDef.LinearVelocity = box2d.Vec2{X: 2, Y: -1}
		jointDef.MaxVelocityForce = 13
		jointDef.AngularVelocity = 1.5
		jointDef.MaxVelocityTorque = 8
		jointDef.LinearHertz = 5
		jointDef.LinearDampingRatio = 0.4
		jointDef.MaxSpringForce = 19
		jointDef.AngularHertz = 6
		jointDef.AngularDampingRatio = 0.3
		jointDef.MaxSpringTorque = 11

		joint, err := world.CreateMotorJointWithDef(jointDef)
		Expect(err).NotTo(HaveOccurred())

		jointType, err := joint.GetType()
		Expect(err).NotTo(HaveOccurred())
		Expect(jointType).To(Equal(box2d.MotorJoint))

		frameA, err := joint.GetLocalFrameA()
		Expect(err).NotTo(HaveOccurred())
		Expect(frameA).To(Equal(jointDef.Base.LocalFrameA))
		frameB, err := joint.GetLocalFrameB()
		Expect(err).NotTo(HaveOccurred())
		Expect(frameB).To(Equal(jointDef.Base.LocalFrameB))

		hz, damping, err := joint.GetConstraintTuning()
		Expect(err).NotTo(HaveOccurred())
		Expect(hz).To(BeNumerically("~", 9.0, 1e-6))
		Expect(damping).To(BeNumerically("~", 0.6, 1e-6))

		forceThreshold, err := joint.GetForceThreshold()
		Expect(err).NotTo(HaveOccurred())
		Expect(forceThreshold).To(BeNumerically("~", 17.0, 1e-6))
		torqueThreshold, err := joint.GetTorqueThreshold()
		Expect(err).NotTo(HaveOccurred())
		Expect(torqueThreshold).To(BeNumerically("~", 4.0, 1e-6))

		collideConnected, err := joint.GetCollideConnected()
		Expect(err).NotTo(HaveOccurred())
		Expect(collideConnected).To(BeTrue())

		linearVelocity, err := joint.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(linearVelocity).To(Equal(box2d.Vec2{X: 2, Y: -1}))
		angularVelocity, err := joint.GetAngularVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(angularVelocity).To(BeNumerically("~", 1.5, 1e-6))
		maxVelocityForce, err := joint.GetMaxVelocityForce()
		Expect(err).NotTo(HaveOccurred())
		Expect(maxVelocityForce).To(BeNumerically("~", 13.0, 1e-6))
		maxVelocityTorque, err := joint.GetMaxVelocityTorque()
		Expect(err).NotTo(HaveOccurred())
		Expect(maxVelocityTorque).To(BeNumerically("~", 8.0, 1e-6))
		linearHertz, err := joint.GetLinearHertz()
		Expect(err).NotTo(HaveOccurred())
		Expect(linearHertz).To(BeNumerically("~", 5.0, 1e-6))
		linearDamping, err := joint.GetLinearDampingRatio()
		Expect(err).NotTo(HaveOccurred())
		Expect(linearDamping).To(BeNumerically("~", 0.4, 1e-6))
		angularHertz, err := joint.GetAngularHertz()
		Expect(err).NotTo(HaveOccurred())
		Expect(angularHertz).To(BeNumerically("~", 6.0, 1e-6))
		angularDamping, err := joint.GetAngularDampingRatio()
		Expect(err).NotTo(HaveOccurred())
		Expect(angularDamping).To(BeNumerically("~", 0.3, 1e-6))
		maxSpringForce, err := joint.GetMaxSpringForce()
		Expect(err).NotTo(HaveOccurred())
		Expect(maxSpringForce).To(BeNumerically("~", 19.0, 1e-6))
		maxSpringTorque, err := joint.GetMaxSpringTorque()
		Expect(err).NotTo(HaveOccurred())
		Expect(maxSpringTorque).To(BeNumerically("~", 11.0, 1e-6))

		Expect(joint.SetLinearVelocity(box2d.Vec2{X: -3, Y: 7})).To(Succeed())
		Expect(joint.SetAngularVelocity(-2.25)).To(Succeed())
		Expect(joint.SetMaxVelocityForce(21)).To(Succeed())
		Expect(joint.SetMaxVelocityTorque(18)).To(Succeed())
		Expect(joint.SetLinearHertz(12)).To(Succeed())
		Expect(joint.SetLinearDampingRatio(0.9)).To(Succeed())
		Expect(joint.SetAngularHertz(14)).To(Succeed())
		Expect(joint.SetAngularDampingRatio(0.8)).To(Succeed())
		Expect(joint.SetMaxSpringForce(-3)).To(Succeed())
		Expect(joint.SetMaxSpringTorque(-2)).To(Succeed())

		linearVelocity, err = joint.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(linearVelocity).To(Equal(box2d.Vec2{X: -3, Y: 7}))
		angularVelocity, err = joint.GetAngularVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(angularVelocity).To(BeNumerically("~", -2.25, 1e-6))
		maxVelocityForce, err = joint.GetMaxVelocityForce()
		Expect(err).NotTo(HaveOccurred())
		Expect(maxVelocityForce).To(BeNumerically("~", 21.0, 1e-6))
		maxVelocityTorque, err = joint.GetMaxVelocityTorque()
		Expect(err).NotTo(HaveOccurred())
		Expect(maxVelocityTorque).To(BeNumerically("~", 18.0, 1e-6))
		linearHertz, err = joint.GetLinearHertz()
		Expect(err).NotTo(HaveOccurred())
		Expect(linearHertz).To(BeNumerically("~", 12.0, 1e-6))
		linearDamping, err = joint.GetLinearDampingRatio()
		Expect(err).NotTo(HaveOccurred())
		Expect(linearDamping).To(BeNumerically("~", 0.9, 1e-6))
		angularHertz, err = joint.GetAngularHertz()
		Expect(err).NotTo(HaveOccurred())
		Expect(angularHertz).To(BeNumerically("~", 14.0, 1e-6))
		angularDamping, err = joint.GetAngularDampingRatio()
		Expect(err).NotTo(HaveOccurred())
		Expect(angularDamping).To(BeNumerically("~", 0.8, 1e-6))
		maxSpringForce, err = joint.GetMaxSpringForce()
		Expect(err).NotTo(HaveOccurred())
		Expect(maxSpringForce).To(BeNumerically("~", 0.0, 1e-6))
		maxSpringTorque, err = joint.GetMaxSpringTorque()
		Expect(err).NotTo(HaveOccurred())
		Expect(maxSpringTorque).To(BeNumerically("~", 0.0, 1e-6))
		constraintForce, err := joint.GetConstraintForce()
		Expect(err).NotTo(HaveOccurred())
		Expect(box2d.Length(constraintForce)).To(BeNumerically(">", 0.0))
		constraintTorque, err := joint.GetConstraintTorque()
		Expect(err).NotTo(HaveOccurred())
		Expect(constraintTorque).To(BeNumerically(">=", 0.0))
		Expect(joint.SetLength(2.0)).To(MatchError(box2d.ErrJointTypeMismatch))
	})

	It("creates a filter joint from definition", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())
		_, err = world.CreateFilterJointWithDef(box2d.FilterJointDef{})
		Expect(err).To(MatchError(box2d.ErrInvalidFilterJointDef))

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		bodyA, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())
		bodyB, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		jointDef := box2d.DefaultFilterJointDef()
		jointDef.Base.BodyIDA = bodyA.ID()
		jointDef.Base.BodyIDB = bodyB.ID()
		jointDef.Base.UserData = "filter"
		jointDef.Base.CollideConnected = true
		jointDef.Base.ConstraintHertz = 12
		jointDef.Base.ConstraintDampingRatio = 0.75
		jointDef.Base.ForceThreshold = 9
		jointDef.Base.TorqueThreshold = 2
		jointDef.Base.LocalFrameA = box2d.Transform{P: box2d.Vec2{X: 0.1, Y: 0.2}, Q: box2d.MakeRot(0.15)}
		jointDef.Base.LocalFrameB = box2d.Transform{P: box2d.Vec2{X: -0.2, Y: -0.1}, Q: box2d.MakeRot(-0.25)}

		joint, err := world.CreateFilterJointWithDef(jointDef)
		Expect(err).NotTo(HaveOccurred())

		jointType, err := joint.GetType()
		Expect(err).NotTo(HaveOccurred())
		Expect(jointType).To(Equal(box2d.FilterJoint))

		userData, err := joint.GetUserData()
		Expect(err).NotTo(HaveOccurred())
		Expect(userData).To(Equal("filter"))

		frameA, err := joint.GetLocalFrameA()
		Expect(err).NotTo(HaveOccurred())
		Expect(frameA).To(Equal(jointDef.Base.LocalFrameA))
		frameB, err := joint.GetLocalFrameB()
		Expect(err).NotTo(HaveOccurred())
		Expect(frameB).To(Equal(jointDef.Base.LocalFrameB))

		collideConnected, err := joint.GetCollideConnected()
		Expect(err).NotTo(HaveOccurred())
		Expect(collideConnected).To(BeTrue())

		hz, damping, err := joint.GetConstraintTuning()
		Expect(err).NotTo(HaveOccurred())
		Expect(hz).To(BeNumerically("~", 12.0, 1e-6))
		Expect(damping).To(BeNumerically("~", 0.75, 1e-6))

		forceThreshold, err := joint.GetForceThreshold()
		Expect(err).NotTo(HaveOccurred())
		Expect(forceThreshold).To(BeNumerically("~", 9.0, 1e-6))
		torqueThreshold, err := joint.GetTorqueThreshold()
		Expect(err).NotTo(HaveOccurred())
		Expect(torqueThreshold).To(BeNumerically("~", 2.0, 1e-6))
	})

	It("estimates motor constraint force from velocity mismatch when anchors coincide", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		bodyA, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())
		bodyB, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		joint, err := world.CreateMotorJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		Expect(joint.SetLinearVelocity(box2d.Vec2{X: 4, Y: 0})).To(Succeed())

		force, err := joint.GetConstraintForce()
		Expect(err).NotTo(HaveOccurred())
		Expect(box2d.Length(force)).To(BeNumerically(">", 0.0))
	})

	It("creates a prismatic joint from definition", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())
		_, err = world.CreatePrismaticJointWithDef(box2d.PrismaticJointDef{})
		Expect(err).To(MatchError(box2d.ErrInvalidPrismaticJointDef))

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		def.Position = box2d.Vec2{X: 0, Y: 0}
		def.LinearVelocity = box2d.Vec2{X: 1, Y: 0}
		bodyA, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		def.Position = box2d.Vec2{X: 3, Y: 1}
		def.LinearVelocity = box2d.Vec2{X: 4, Y: 0}
		bodyB, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		jointDef := box2d.DefaultPrismaticJointDef()
		jointDef.Base.BodyIDA = bodyA.ID()
		jointDef.Base.BodyIDB = bodyB.ID()
		jointDef.Base.LocalFrameA = box2d.Transform{P: box2d.Vec2{X: 0, Y: 0}, Q: box2d.RotIdentity}
		jointDef.Base.LocalFrameB = box2d.Transform{P: box2d.Vec2{X: 0, Y: 0}, Q: box2d.RotIdentity}
		jointDef.EnableSpring = true
		jointDef.Hertz = 6
		jointDef.DampingRatio = 0.2
		jointDef.TargetTranslation = 1.75
		jointDef.EnableLimit = true
		jointDef.LowerTranslation = -2
		jointDef.UpperTranslation = 4
		jointDef.EnableMotor = true
		jointDef.MaxMotorForce = 15
		jointDef.MotorSpeed = 2.25

		joint, err := world.CreatePrismaticJointWithDef(jointDef)
		Expect(err).NotTo(HaveOccurred())

		jointType, err := joint.GetType()
		Expect(err).NotTo(HaveOccurred())
		Expect(jointType).To(Equal(box2d.PrismaticJoint))

		springEnabled, err := joint.IsSpringEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(springEnabled).To(BeTrue())
		springHertz, err := joint.GetSpringHertz()
		Expect(err).NotTo(HaveOccurred())
		Expect(springHertz).To(BeNumerically("~", 6.0, 1e-6))
		springDamping, err := joint.GetSpringDampingRatio()
		Expect(err).NotTo(HaveOccurred())
		Expect(springDamping).To(BeNumerically("~", 0.2, 1e-6))
		targetTranslation, err := joint.GetTargetTranslation()
		Expect(err).NotTo(HaveOccurred())
		Expect(targetTranslation).To(BeNumerically("~", 1.75, 1e-6))

		limitEnabled, err := joint.IsLimitEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(limitEnabled).To(BeTrue())
		lowerLimit, err := joint.GetLowerLimit()
		Expect(err).NotTo(HaveOccurred())
		Expect(lowerLimit).To(BeNumerically("~", -2.0, 1e-6))
		upperLimit, err := joint.GetUpperLimit()
		Expect(err).NotTo(HaveOccurred())
		Expect(upperLimit).To(BeNumerically("~", 4.0, 1e-6))

		motorEnabled, err := joint.IsMotorEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(motorEnabled).To(BeTrue())
		motorSpeed, err := joint.GetMotorSpeed()
		Expect(err).NotTo(HaveOccurred())
		Expect(motorSpeed).To(BeNumerically("~", 2.25, 1e-6))
		maxMotorForce, err := joint.GetMaxMotorForce()
		Expect(err).NotTo(HaveOccurred())
		Expect(maxMotorForce).To(BeNumerically("~", 15.0, 1e-6))
		motorForce, err := joint.GetMotorForce()
		Expect(err).NotTo(HaveOccurred())
		Expect(motorForce).To(BeNumerically("~", 0.0, 1e-6))

		translation, err := joint.GetTranslation()
		Expect(err).NotTo(HaveOccurred())
		Expect(translation).To(BeNumerically("~", 3.0, 1e-6))
		speed, err := joint.GetSpeed()
		Expect(err).NotTo(HaveOccurred())
		Expect(speed).To(BeNumerically("~", 3.0, 1e-5))

		Expect(joint.EnableSpring(false)).To(Succeed())
		Expect(joint.SetSpringHertz(7.5)).To(Succeed())
		Expect(joint.SetSpringDampingRatio(0.55)).To(Succeed())
		Expect(joint.SetTargetTranslation(-1.25)).To(Succeed())
		Expect(joint.EnableLimit(false)).To(Succeed())
		Expect(joint.SetLimits(5, -3)).To(Succeed())
		Expect(joint.EnableMotor(false)).To(Succeed())
		Expect(joint.SetMotorSpeed(-4)).To(Succeed())
		Expect(joint.SetMaxMotorForce(9)).To(Succeed())

		springEnabled, err = joint.IsSpringEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(springEnabled).To(BeFalse())
		springHertz, err = joint.GetSpringHertz()
		Expect(err).NotTo(HaveOccurred())
		Expect(springHertz).To(BeNumerically("~", 7.5, 1e-6))
		springDamping, err = joint.GetSpringDampingRatio()
		Expect(err).NotTo(HaveOccurred())
		Expect(springDamping).To(BeNumerically("~", 0.55, 1e-6))
		targetTranslation, err = joint.GetTargetTranslation()
		Expect(err).NotTo(HaveOccurred())
		Expect(targetTranslation).To(BeNumerically("~", -1.25, 1e-6))
		limitEnabled, err = joint.IsLimitEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(limitEnabled).To(BeFalse())
		lowerLimit, err = joint.GetLowerLimit()
		Expect(err).NotTo(HaveOccurred())
		Expect(lowerLimit).To(BeNumerically("~", -3.0, 1e-6))
		upperLimit, err = joint.GetUpperLimit()
		Expect(err).NotTo(HaveOccurred())
		Expect(upperLimit).To(BeNumerically("~", 5.0, 1e-6))
		motorEnabled, err = joint.IsMotorEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(motorEnabled).To(BeFalse())
		motorSpeed, err = joint.GetMotorSpeed()
		Expect(err).NotTo(HaveOccurred())
		Expect(motorSpeed).To(BeNumerically("~", -4.0, 1e-6))
		maxMotorForce, err = joint.GetMaxMotorForce()
		Expect(err).NotTo(HaveOccurred())
		Expect(maxMotorForce).To(BeNumerically("~", 9.0, 1e-6))

		Expect(joint.SetLength(1.0)).To(MatchError(box2d.ErrJointTypeMismatch))
	})

	It("creates a revolute joint from definition", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())
		_, err = world.CreateRevoluteJointWithDef(box2d.RevoluteJointDef{})
		Expect(err).To(MatchError(box2d.ErrInvalidRevoluteJointDef))

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		def.Rotation = box2d.MakeRot(0.2)
		bodyA, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		def.Rotation = box2d.MakeRot(0.7)
		bodyB, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		jointDef := box2d.DefaultRevoluteJointDef()
		jointDef.Base.BodyIDA = bodyA.ID()
		jointDef.Base.BodyIDB = bodyB.ID()
		jointDef.Base.LocalFrameA = box2d.Transform{P: box2d.Vec2{X: 0, Y: 0}, Q: box2d.MakeRot(0.1)}
		jointDef.Base.LocalFrameB = box2d.Transform{P: box2d.Vec2{X: 0, Y: 0}, Q: box2d.MakeRot(-0.2)}
		jointDef.TargetAngle = 0.5
		jointDef.EnableSpring = true
		jointDef.Hertz = 6
		jointDef.DampingRatio = 0.25
		jointDef.EnableLimit = true
		jointDef.LowerAngle = -1.2
		jointDef.UpperAngle = 1.1
		jointDef.EnableMotor = true
		jointDef.MaxMotorTorque = 10
		jointDef.MotorSpeed = 2.5

		joint, err := world.CreateRevoluteJointWithDef(jointDef)
		Expect(err).NotTo(HaveOccurred())

		jointType, err := joint.GetType()
		Expect(err).NotTo(HaveOccurred())
		Expect(jointType).To(Equal(box2d.RevoluteJoint))

		targetAngle, err := joint.GetTargetAngle()
		Expect(err).NotTo(HaveOccurred())
		Expect(targetAngle).To(BeNumerically("~", 0.5, 1e-6))
		angle, err := joint.GetAngle()
		Expect(err).NotTo(HaveOccurred())
		Expect(angle).To(BeNumerically("~", 0.2, 1e-5))

		springEnabled, err := joint.IsSpringEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(springEnabled).To(BeTrue())
		springHertz, err := joint.GetSpringHertz()
		Expect(err).NotTo(HaveOccurred())
		Expect(springHertz).To(BeNumerically("~", 6.0, 1e-6))
		springDamping, err := joint.GetSpringDampingRatio()
		Expect(err).NotTo(HaveOccurred())
		Expect(springDamping).To(BeNumerically("~", 0.25, 1e-6))

		limitEnabled, err := joint.IsLimitEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(limitEnabled).To(BeTrue())
		lowerLimit, err := joint.GetLowerLimit()
		Expect(err).NotTo(HaveOccurred())
		Expect(lowerLimit).To(BeNumerically("~", -1.2, 1e-6))
		upperLimit, err := joint.GetUpperLimit()
		Expect(err).NotTo(HaveOccurred())
		Expect(upperLimit).To(BeNumerically("~", 1.1, 1e-6))

		motorEnabled, err := joint.IsMotorEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(motorEnabled).To(BeTrue())
		motorSpeed, err := joint.GetMotorSpeed()
		Expect(err).NotTo(HaveOccurred())
		Expect(motorSpeed).To(BeNumerically("~", 2.5, 1e-6))
		maxMotorTorque, err := joint.GetMaxMotorTorque()
		Expect(err).NotTo(HaveOccurred())
		Expect(maxMotorTorque).To(BeNumerically("~", 10.0, 1e-6))
		motorTorque, err := joint.GetMotorTorque()
		Expect(err).NotTo(HaveOccurred())
		Expect(motorTorque).To(BeNumerically("~", 0.0, 1e-6))

		Expect(joint.SetTargetAngle(-0.75)).To(Succeed())
		Expect(joint.EnableSpring(false)).To(Succeed())
		Expect(joint.SetSpringHertz(12)).To(Succeed())
		Expect(joint.SetSpringDampingRatio(0.7)).To(Succeed())
		Expect(joint.EnableLimit(false)).To(Succeed())
		Expect(joint.SetLimits(4, -4)).To(Succeed())
		Expect(joint.EnableMotor(false)).To(Succeed())
		Expect(joint.SetMotorSpeed(-3)).To(Succeed())
		Expect(joint.SetMaxMotorTorque(22)).To(Succeed())

		targetAngle, err = joint.GetTargetAngle()
		Expect(err).NotTo(HaveOccurred())
		Expect(targetAngle).To(BeNumerically("~", -0.75, 1e-6))
		springEnabled, err = joint.IsSpringEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(springEnabled).To(BeFalse())
		springHertz, err = joint.GetSpringHertz()
		Expect(err).NotTo(HaveOccurred())
		Expect(springHertz).To(BeNumerically("~", 12.0, 1e-6))
		springDamping, err = joint.GetSpringDampingRatio()
		Expect(err).NotTo(HaveOccurred())
		Expect(springDamping).To(BeNumerically("~", 0.7, 1e-6))
		limitEnabled, err = joint.IsLimitEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(limitEnabled).To(BeFalse())
		lowerLimit, err = joint.GetLowerLimit()
		Expect(err).NotTo(HaveOccurred())
		Expect(lowerLimit).To(BeNumerically("~", -3.1101767, 1e-4))
		upperLimit, err = joint.GetUpperLimit()
		Expect(err).NotTo(HaveOccurred())
		Expect(upperLimit).To(BeNumerically("~", 3.1101767, 1e-4))
		motorEnabled, err = joint.IsMotorEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(motorEnabled).To(BeFalse())
		motorSpeed, err = joint.GetMotorSpeed()
		Expect(err).NotTo(HaveOccurred())
		Expect(motorSpeed).To(BeNumerically("~", -3.0, 1e-6))
		maxMotorTorque, err = joint.GetMaxMotorTorque()
		Expect(err).NotTo(HaveOccurred())
		Expect(maxMotorTorque).To(BeNumerically("~", 22.0, 1e-6))

		Expect(joint.SetMaxMotorForce(1.0)).To(MatchError(box2d.ErrJointTypeMismatch))
	})

	It("creates a weld joint from definition", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())
		_, err = world.CreateWeldJointWithDef(box2d.WeldJointDef{})
		Expect(err).To(MatchError(box2d.ErrInvalidWeldJointDef))

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		bodyA, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())
		bodyB, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		jointDef := box2d.DefaultWeldJointDef()
		jointDef.Base.BodyIDA = bodyA.ID()
		jointDef.Base.BodyIDB = bodyB.ID()
		jointDef.LinearHertz = 3
		jointDef.LinearDampingRatio = 0.4
		jointDef.AngularHertz = 5
		jointDef.AngularDampingRatio = 0.6

		joint, err := world.CreateWeldJointWithDef(jointDef)
		Expect(err).NotTo(HaveOccurred())

		jointType, err := joint.GetType()
		Expect(err).NotTo(HaveOccurred())
		Expect(jointType).To(Equal(box2d.WeldJoint))

		linearHertz, err := joint.GetLinearHertz()
		Expect(err).NotTo(HaveOccurred())
		Expect(linearHertz).To(BeNumerically("~", 3.0, 1e-6))
		linearDamping, err := joint.GetLinearDampingRatio()
		Expect(err).NotTo(HaveOccurred())
		Expect(linearDamping).To(BeNumerically("~", 0.4, 1e-6))
		angularHertz, err := joint.GetAngularHertz()
		Expect(err).NotTo(HaveOccurred())
		Expect(angularHertz).To(BeNumerically("~", 5.0, 1e-6))
		angularDamping, err := joint.GetAngularDampingRatio()
		Expect(err).NotTo(HaveOccurred())
		Expect(angularDamping).To(BeNumerically("~", 0.6, 1e-6))

		Expect(joint.SetLinearHertz(7)).To(Succeed())
		Expect(joint.SetLinearDampingRatio(0.2)).To(Succeed())
		Expect(joint.SetAngularHertz(9)).To(Succeed())
		Expect(joint.SetAngularDampingRatio(0.1)).To(Succeed())

		linearHertz, err = joint.GetLinearHertz()
		Expect(err).NotTo(HaveOccurred())
		Expect(linearHertz).To(BeNumerically("~", 7.0, 1e-6))
		linearDamping, err = joint.GetLinearDampingRatio()
		Expect(err).NotTo(HaveOccurred())
		Expect(linearDamping).To(BeNumerically("~", 0.2, 1e-6))
		angularHertz, err = joint.GetAngularHertz()
		Expect(err).NotTo(HaveOccurred())
		Expect(angularHertz).To(BeNumerically("~", 9.0, 1e-6))
		angularDamping, err = joint.GetAngularDampingRatio()
		Expect(err).NotTo(HaveOccurred())
		Expect(angularDamping).To(BeNumerically("~", 0.1, 1e-6))

		Expect(joint.SetLength(1.0)).To(MatchError(box2d.ErrJointTypeMismatch))
	})

	It("creates a wheel joint from definition", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())
		_, err = world.CreateWheelJointWithDef(box2d.WheelJointDef{})
		Expect(err).To(MatchError(box2d.ErrInvalidWheelJointDef))

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		bodyA, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())
		bodyB, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		defaultJoint, err := world.CreateWheelJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		springEnabled, err := defaultJoint.IsSpringEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(springEnabled).To(BeTrue())
		defaultHertz, err := defaultJoint.GetSpringHertz()
		Expect(err).NotTo(HaveOccurred())
		Expect(defaultHertz).To(BeNumerically("~", 1.0, 1e-6))
		defaultDamping, err := defaultJoint.GetSpringDampingRatio()
		Expect(err).NotTo(HaveOccurred())
		Expect(defaultDamping).To(BeNumerically("~", 0.7, 1e-6))

		jointDef := box2d.DefaultWheelJointDef()
		jointDef.Base.BodyIDA = bodyA.ID()
		jointDef.Base.BodyIDB = bodyB.ID()
		jointDef.EnableSpring = true
		jointDef.Hertz = 4
		jointDef.DampingRatio = 0.3
		jointDef.EnableLimit = true
		jointDef.LowerTranslation = -1
		jointDef.UpperTranslation = 2
		jointDef.EnableMotor = true
		jointDef.MaxMotorTorque = 12
		jointDef.MotorSpeed = 6

		joint, err := world.CreateWheelJointWithDef(jointDef)
		Expect(err).NotTo(HaveOccurred())

		jointType, err := joint.GetType()
		Expect(err).NotTo(HaveOccurred())
		Expect(jointType).To(Equal(box2d.WheelJoint))

		springEnabled, err = joint.IsSpringEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(springEnabled).To(BeTrue())
		springHertz, err := joint.GetSpringHertz()
		Expect(err).NotTo(HaveOccurred())
		Expect(springHertz).To(BeNumerically("~", 4.0, 1e-6))
		springDamping, err := joint.GetSpringDampingRatio()
		Expect(err).NotTo(HaveOccurred())
		Expect(springDamping).To(BeNumerically("~", 0.3, 1e-6))
		limitEnabled, err := joint.IsLimitEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(limitEnabled).To(BeTrue())
		lowerLimit, err := joint.GetLowerLimit()
		Expect(err).NotTo(HaveOccurred())
		Expect(lowerLimit).To(BeNumerically("~", -1.0, 1e-6))
		upperLimit, err := joint.GetUpperLimit()
		Expect(err).NotTo(HaveOccurred())
		Expect(upperLimit).To(BeNumerically("~", 2.0, 1e-6))
		motorEnabled, err := joint.IsMotorEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(motorEnabled).To(BeTrue())
		motorSpeed, err := joint.GetMotorSpeed()
		Expect(err).NotTo(HaveOccurred())
		Expect(motorSpeed).To(BeNumerically("~", 6.0, 1e-6))
		maxMotorTorque, err := joint.GetMaxMotorTorque()
		Expect(err).NotTo(HaveOccurred())
		Expect(maxMotorTorque).To(BeNumerically("~", 12.0, 1e-6))
		motorTorque, err := joint.GetMotorTorque()
		Expect(err).NotTo(HaveOccurred())
		Expect(motorTorque).To(BeNumerically("~", 0.0, 1e-6))

		Expect(joint.SetSpringHertz(8)).To(Succeed())
		Expect(joint.SetSpringDampingRatio(0.9)).To(Succeed())
		Expect(joint.EnableLimit(false)).To(Succeed())
		Expect(joint.SetLimits(5, -2)).To(Succeed())
		Expect(joint.EnableMotor(false)).To(Succeed())
		Expect(joint.SetMotorSpeed(-1.5)).To(Succeed())
		Expect(joint.SetMaxMotorTorque(3)).To(Succeed())

		springHertz, err = joint.GetSpringHertz()
		Expect(err).NotTo(HaveOccurred())
		Expect(springHertz).To(BeNumerically("~", 8.0, 1e-6))
		springDamping, err = joint.GetSpringDampingRatio()
		Expect(err).NotTo(HaveOccurred())
		Expect(springDamping).To(BeNumerically("~", 0.9, 1e-6))
		limitEnabled, err = joint.IsLimitEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(limitEnabled).To(BeFalse())
		lowerLimit, err = joint.GetLowerLimit()
		Expect(err).NotTo(HaveOccurred())
		Expect(lowerLimit).To(BeNumerically("~", -2.0, 1e-6))
		upperLimit, err = joint.GetUpperLimit()
		Expect(err).NotTo(HaveOccurred())
		Expect(upperLimit).To(BeNumerically("~", 5.0, 1e-6))
		motorEnabled, err = joint.IsMotorEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(motorEnabled).To(BeFalse())
		motorSpeed, err = joint.GetMotorSpeed()
		Expect(err).NotTo(HaveOccurred())
		Expect(motorSpeed).To(BeNumerically("~", -1.5, 1e-6))
		maxMotorTorque, err = joint.GetMaxMotorTorque()
		Expect(err).NotTo(HaveOccurred())
		Expect(maxMotorTorque).To(BeNumerically("~", 3.0, 1e-6))

		Expect(joint.SetMaxMotorForce(1.0)).To(MatchError(box2d.ErrJointTypeMismatch))
	})

	It("destroys attached joints when a body is destroyed", func() {
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
		jointID := joint.ID()
		Expect(bodyB.SetAwake(false)).To(Succeed())
		Expect(joint.WakeBodies()).To(Succeed())
		awakeB, err := bodyB.IsAwake()
		Expect(err).NotTo(HaveOccurred())
		Expect(awakeB).To(BeTrue())

		Expect(bodyA.Destroy()).To(Succeed())
		Expect(box2d.IsJointIDValid(jointID)).To(BeFalse())

		counters, err := world.Counters()
		Expect(err).NotTo(HaveOccurred())
		Expect(counters.JointCount).To(Equal(0))
	})

	It("clamps negative max force and torque inputs to zero", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())
		defer func() { _ = world.Destroy() }()

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		bodyA, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())
		bodyB, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		distanceJoint, err := world.CreateDistanceJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		Expect(distanceJoint.SetMaxMotorForce(-10)).To(Succeed())
		maxMotorForce, err := distanceJoint.GetMaxMotorForce()
		Expect(err).NotTo(HaveOccurred())
		Expect(maxMotorForce).To(BeNumerically("~", 0.0, 1e-6))

		motorJoint, err := world.CreateMotorJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		Expect(motorJoint.SetMaxVelocityForce(-8)).To(Succeed())
		Expect(motorJoint.SetMaxVelocityTorque(-6)).To(Succeed())
		maxVelocityForce, err := motorJoint.GetMaxVelocityForce()
		Expect(err).NotTo(HaveOccurred())
		Expect(maxVelocityForce).To(BeNumerically("~", 0.0, 1e-6))
		maxVelocityTorque, err := motorJoint.GetMaxVelocityTorque()
		Expect(err).NotTo(HaveOccurred())
		Expect(maxVelocityTorque).To(BeNumerically("~", 0.0, 1e-6))

		revoluteJoint, err := world.CreateRevoluteJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		Expect(revoluteJoint.SetMaxMotorTorque(-4)).To(Succeed())
		maxRevoluteTorque, err := revoluteJoint.GetMaxMotorTorque()
		Expect(err).NotTo(HaveOccurred())
		Expect(maxRevoluteTorque).To(BeNumerically("~", 0.0, 1e-6))

		wheelJoint, err := world.CreateWheelJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		Expect(wheelJoint.SetMaxMotorTorque(-3)).To(Succeed())
		maxWheelTorque, err := wheelJoint.GetMaxMotorTorque()
		Expect(err).NotTo(HaveOccurred())
		Expect(maxWheelTorque).To(BeNumerically("~", 0.0, 1e-6))
	})

	It("clamps negative hertz and damping inputs to zero", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())
		defer func() { _ = world.Destroy() }()

		def := box2d.DefaultBodyDef()
		def.Type = box2d.DynamicBody
		bodyA, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())
		bodyB, err := world.CreateBody(def)
		Expect(err).NotTo(HaveOccurred())

		distanceJoint, err := world.CreateDistanceJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		Expect(distanceJoint.SetSpringHertz(-5)).To(Succeed())
		Expect(distanceJoint.SetSpringDampingRatio(-0.5)).To(Succeed())
		springHertz, err := distanceJoint.GetSpringHertz()
		Expect(err).NotTo(HaveOccurred())
		Expect(springHertz).To(BeNumerically("~", 0.0, 1e-6))
		springDamping, err := distanceJoint.GetSpringDampingRatio()
		Expect(err).NotTo(HaveOccurred())
		Expect(springDamping).To(BeNumerically("~", 0.0, 1e-6))

		motorJoint, err := world.CreateMotorJoint(bodyA.ID(), bodyB.ID())
		Expect(err).NotTo(HaveOccurred())
		Expect(motorJoint.SetLinearHertz(-7)).To(Succeed())
		Expect(motorJoint.SetLinearDampingRatio(-0.4)).To(Succeed())
		Expect(motorJoint.SetAngularHertz(-9)).To(Succeed())
		Expect(motorJoint.SetAngularDampingRatio(-0.6)).To(Succeed())
		linearHertz, err := motorJoint.GetLinearHertz()
		Expect(err).NotTo(HaveOccurred())
		Expect(linearHertz).To(BeNumerically("~", 0.0, 1e-6))
		linearDamping, err := motorJoint.GetLinearDampingRatio()
		Expect(err).NotTo(HaveOccurred())
		Expect(linearDamping).To(BeNumerically("~", 0.0, 1e-6))
		angularHertz, err := motorJoint.GetAngularHertz()
		Expect(err).NotTo(HaveOccurred())
		Expect(angularHertz).To(BeNumerically("~", 0.0, 1e-6))
		angularDamping, err := motorJoint.GetAngularDampingRatio()
		Expect(err).NotTo(HaveOccurred())
		Expect(angularDamping).To(BeNumerically("~", 0.0, 1e-6))
	})
})
