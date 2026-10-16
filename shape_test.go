package box2d_test

import (
	"github.com/gataengine/box2d-go"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

var _ = Describe("Shape Lifecycle", func() {
	It("creates and destroys shapes while updating counters", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		body, err := world.CreateBody(box2d.DefaultBodyDef())
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		circle := box2d.Circle{
			Center: box2d.Vec2{X: 1.0, Y: 2.0},
			Radius: 0.5,
		}
		shape, err := body.CreateCircleShape(shapeDef, circle)
		Expect(err).NotTo(HaveOccurred())
		Expect(shape.IsValid()).To(BeTrue())
		Expect(box2d.IsShapeIDValid(shape.ID())).To(BeTrue())

		shapeType, err := shape.GetType()
		Expect(err).NotTo(HaveOccurred())
		Expect(shapeType).To(Equal(box2d.CircleShape))

		bodyID, err := shape.GetBody()
		Expect(err).NotTo(HaveOccurred())
		Expect(box2d.BodyIDEqual(bodyID, body.ID())).To(BeTrue())

		bodyShapeCount, err := body.ShapeCount()
		Expect(err).NotTo(HaveOccurred())
		Expect(bodyShapeCount).To(Equal(1))

		counters, err := world.Counters()
		Expect(err).NotTo(HaveOccurred())
		Expect(counters.ShapeCount).To(Equal(1))

		Expect(shape.Destroy(true)).To(Succeed())
		Expect(shape.IsValid()).To(BeFalse())

		bodyShapeCount, err = body.ShapeCount()
		Expect(err).NotTo(HaveOccurred())
		Expect(bodyShapeCount).To(Equal(0))

		counters, err = world.Counters()
		Expect(err).NotTo(HaveOccurred())
		Expect(counters.ShapeCount).To(Equal(0))
	})

	It("roundtrips mutable shape properties", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		body, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.IsSensor = true
		polygon := box2d.MakeBox(1.0, 2.0)
		shape, err := body.CreatePolygonShape(shapeDef, polygon)
		Expect(err).NotTo(HaveOccurred())

		isSensor, err := shape.IsSensor()
		Expect(err).NotTo(HaveOccurred())
		Expect(isSensor).To(BeTrue())

		Expect(shape.SetDensity(2.5, true)).To(Succeed())
		density, err := shape.Density()
		Expect(err).NotTo(HaveOccurred())
		Expect(density).To(BeNumerically("==", float32(2.5)))

		Expect(shape.SetFriction(0.2)).To(Succeed())
		friction, err := shape.Friction()
		Expect(err).NotTo(HaveOccurred())
		Expect(friction).To(BeNumerically("==", float32(0.2)))

		Expect(shape.SetRestitution(0.7)).To(Succeed())
		restitution, err := shape.Restitution()
		Expect(err).NotTo(HaveOccurred())
		Expect(restitution).To(BeNumerically("==", float32(0.7)))

		Expect(shape.SetUserMaterial(42)).To(Succeed())
		materialID, err := shape.UserMaterial()
		Expect(err).NotTo(HaveOccurred())
		Expect(materialID).To(Equal(uint64(42)))

		filter := box2d.Filter{CategoryBits: 2, MaskBits: 4, GroupIndex: -1}
		Expect(shape.SetFilter(filter)).To(Succeed())
		gotFilter, err := shape.Filter()
		Expect(err).NotTo(HaveOccurred())
		Expect(gotFilter).To(Equal(filter))

		Expect(shape.EnableSensorEvents(true)).To(Succeed())
		Expect(shape.EnableContactEvents(true)).To(Succeed())
		Expect(shape.EnablePreSolveEvents(true)).To(Succeed())
		Expect(shape.EnableHitEvents(true)).To(Succeed())

		sensorEvents, err := shape.SensorEventsEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(sensorEvents).To(BeTrue())
		contactEvents, err := shape.ContactEventsEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEvents).To(BeTrue())
		preSolveEvents, err := shape.PreSolveEventsEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(preSolveEvents).To(BeTrue())
		hitEvents, err := shape.HitEventsEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(hitEvents).To(BeTrue())
	})

	It("supports extended shape query and alias APIs", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyDef.Position = box2d.Vec2{X: 1, Y: 2}
		bodyDef.Rotation = box2d.MakeRot(box2d.Pi / 4)
		body, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.EnableSensorEvents = true
		shapeDef.EnableContactEvents = true
		shapeDef.EnablePreSolveEvents = true
		shapeDef.EnableHitEvents = true
		circle := box2d.Circle{Center: box2d.Vec2Zero, Radius: 0.5}
		shape, err := body.CreateCircleShape(shapeDef, circle)
		Expect(err).NotTo(HaveOccurred())

		Expect(shape.SetUserData("tag")).To(Succeed())
		userData, err := shape.GetUserData()
		Expect(err).NotTo(HaveOccurred())
		Expect(userData).To(Equal("tag"))

		density, err := shape.GetDensity()
		Expect(err).NotTo(HaveOccurred())
		Expect(density).To(BeNumerically("~", shapeDef.Density, 1e-6))

		friction, err := shape.GetFriction()
		Expect(err).NotTo(HaveOccurred())
		Expect(friction).To(BeNumerically("~", shapeDef.Material.Friction, 1e-6))

		restitution, err := shape.GetRestitution()
		Expect(err).NotTo(HaveOccurred())
		Expect(restitution).To(BeNumerically("~", shapeDef.Material.Restitution, 1e-6))

		sensorEvents, err := shape.AreSensorEventsEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(sensorEvents).To(BeTrue())
		contactEvents, err := shape.AreContactEventsEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactEvents).To(BeTrue())
		preSolveEvents, err := shape.ArePreSolveEventsEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(preSolveEvents).To(BeTrue())
		hitEvents, err := shape.AreHitEventsEnabled()
		Expect(err).NotTo(HaveOccurred())
		Expect(hitEvents).To(BeTrue())

		newCircle := box2d.Circle{Center: box2d.Vec2{X: 0.25, Y: 0}, Radius: 0.75}
		Expect(shape.SetCircle(newCircle)).To(Succeed())

		md, err := shape.ComputeMassData()
		Expect(err).NotTo(HaveOccurred())
		expectedMD := box2d.ComputeCircleMass(&newCircle, shapeDef.Density)
		Expect(md.Mass).To(BeNumerically("~", expectedMD.Mass, 1e-5))
		Expect(md.Center.X).To(BeNumerically("~", expectedMD.Center.X, 1e-5))
		Expect(md.Center.Y).To(BeNumerically("~", expectedMD.Center.Y, 1e-5))

		transform := box2d.Transform{P: bodyDef.Position, Q: bodyDef.Rotation}
		expectedAABB := box2d.ComputeCircleAABB(&newCircle, transform)
		gotAABB, err := shape.GetAABB()
		Expect(err).NotTo(HaveOccurred())
		Expect(gotAABB.LowerBound.X).To(BeNumerically("~", expectedAABB.LowerBound.X, 1e-5))
		Expect(gotAABB.LowerBound.Y).To(BeNumerically("~", expectedAABB.LowerBound.Y, 1e-5))
		Expect(gotAABB.UpperBound.X).To(BeNumerically("~", expectedAABB.UpperBound.X, 1e-5))
		Expect(gotAABB.UpperBound.Y).To(BeNumerically("~", expectedAABB.UpperBound.Y, 1e-5))

		worldCenter := box2d.TransformPoint(transform, newCircle.Center)
		inside, err := shape.TestPoint(worldCenter)
		Expect(err).NotTo(HaveOccurred())
		Expect(inside).To(BeTrue())

		outsidePoint := box2d.Add(worldCenter, box2d.Vec2{X: 2, Y: 0})
		inside, err = shape.TestPoint(outsidePoint)
		Expect(err).NotTo(HaveOccurred())
		Expect(inside).To(BeFalse())

		rayInput := box2d.RayCastInput{
			Origin:      box2d.Add(worldCenter, box2d.Vec2{X: -2, Y: 0}),
			Translation: box2d.Vec2{X: 4, Y: 0},
			MaxFraction: 1.0,
		}
		rayOutput, err := shape.RayCast(rayInput)
		Expect(err).NotTo(HaveOccurred())
		Expect(rayOutput.Hit).To(BeTrue())

		closest, err := shape.GetClosestPoint(outsidePoint)
		Expect(err).NotTo(HaveOccurred())
		Expect(box2d.Distance(closest, outsidePoint)).To(BeNumerically("<", 2.0))

		chain, err := shape.GetParentChain()
		Expect(err).NotTo(HaveOccurred())
		Expect(chain.IsNull()).To(BeTrue())

		contactCap, err := shape.GetContactCapacity()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactCap).To(Equal(0))
		contactData, err := shape.GetContactData(8)
		Expect(err).NotTo(HaveOccurred())
		Expect(contactData).To(BeEmpty())

		sensorCap, err := shape.GetSensorCapacity()
		Expect(err).NotTo(HaveOccurred())
		Expect(sensorCap).To(Equal(0))
		sensorData, err := shape.GetSensorData(8)
		Expect(err).NotTo(HaveOccurred())
		Expect(sensorData).To(BeEmpty())

		Expect(shape.ApplyWind(box2d.Vec2{X: 1, Y: 0}, 0.2, 0.1, true)).To(Succeed())
		Expect(shape.SetPolygon(box2d.MakeSquare(1))).To(MatchError(box2d.ErrShapeInvalid))
	})

	It("updates body mass from density changes only when requested", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		body, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1.0
		shape, err := body.CreateCircleShape(shapeDef, box2d.Circle{
			Center: box2d.Vec2Zero,
			Radius: 1.0,
		})
		Expect(err).NotTo(HaveOccurred())

		mass1, err := body.GetMass()
		Expect(err).NotTo(HaveOccurred())
		Expect(mass1).To(BeNumerically("~", box2d.Pi, 1e-5))

		Expect(shape.SetDensity(2.0, false)).To(Succeed())
		mass2, err := body.GetMass()
		Expect(err).NotTo(HaveOccurred())
		Expect(mass2).To(BeNumerically("~", mass1, 1e-5))

		Expect(shape.SetDensity(2.0, true)).To(Succeed())
		mass3, err := body.GetMass()
		Expect(err).NotTo(HaveOccurred())
		Expect(mass3).To(BeNumerically("~", 2.0*box2d.Pi, 1e-5))
	})

	It("reports contact and sensor overlap data for touching shapes", func() {
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

		contactCap, err := shapeA.GetContactCapacity()
		Expect(err).NotTo(HaveOccurred())
		Expect(contactCap).To(Equal(1))

		contactData, err := shapeA.GetContactData(4)
		Expect(err).NotTo(HaveOccurred())
		Expect(contactData).To(HaveLen(1))
		Expect(box2d.ShapeIDEqual(contactData[0].ShapeIDA, shapeA.ID()) || box2d.ShapeIDEqual(contactData[0].ShapeIDB, shapeA.ID())).To(BeTrue())
		Expect(box2d.ShapeIDEqual(contactData[0].ShapeIDA, shapeB.ID()) || box2d.ShapeIDEqual(contactData[0].ShapeIDB, shapeB.ID())).To(BeTrue())
		Expect(contactData[0].Manifold.PointCount).To(Equal(1))
		Expect(contactData[0].Manifold.Point1.Separation).To(BeNumerically("<", 0.0))
		Expect(contactData[0].Manifold.Normal.X).To(BeNumerically("~", 1.0, 1e-5))
		Expect(contactData[0].Manifold.Normal.Y).To(BeNumerically("~", 0.0, 1e-5))

		sensorDef := box2d.DefaultShapeDef()
		sensorDef.IsSensor = true
		sensorShape, err := bodyA.CreateCircleShape(sensorDef, box2d.Circle{Center: box2d.Vec2{X: 0, Y: 0}, Radius: 1})
		Expect(err).NotTo(HaveOccurred())

		sensorCap, err := sensorShape.GetSensorCapacity()
		Expect(err).NotTo(HaveOccurred())
		Expect(sensorCap).To(Equal(1))

		sensorData, err := sensorShape.GetSensorData(4)
		Expect(err).NotTo(HaveOccurred())
		Expect(sensorData).To(HaveLen(1))
		Expect(box2d.ShapeIDEqual(sensorData[0], shapeB.ID())).To(BeTrue())

		sensorOverlaps, err := sensorShape.GetSensorOverlaps(4)
		Expect(err).NotTo(HaveOccurred())
		Expect(sensorOverlaps).To(HaveLen(1))
		Expect(box2d.ShapeIDEqual(sensorOverlaps[0], shapeB.ID())).To(BeTrue())
	})

	It("populates manifold data for circle-capsule and circle-segment contacts", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		circleBodyDef := box2d.DefaultBodyDef()
		circleBodyDef.Type = box2d.DynamicBody
		circleBody, err := world.CreateBody(circleBodyDef)
		Expect(err).NotTo(HaveOccurred())

		otherBodyDef := box2d.DefaultBodyDef()
		otherBodyDef.Type = box2d.DynamicBody
		otherBodyDef.Position = box2d.Vec2{X: 0.0, Y: 0.5}
		otherBody, err := world.CreateBody(otherBodyDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		circleShape, err := circleBody.CreateCircleShape(shapeDef, box2d.Circle{
			Center: box2d.Vec2Zero,
			Radius: 1.0,
		})
		Expect(err).NotTo(HaveOccurred())

		_, err = otherBody.CreateCapsuleShape(shapeDef, box2d.Capsule{
			Center1: box2d.Vec2{X: -1.0, Y: 0.0},
			Center2: box2d.Vec2{X: 1.0, Y: 0.0},
			Radius:  0.25,
		})
		Expect(err).NotTo(HaveOccurred())

		data, err := circleShape.GetContactData(8)
		Expect(err).NotTo(HaveOccurred())
		Expect(data).NotTo(BeEmpty())
		Expect(data[0].Manifold.PointCount).To(Equal(1))
		Expect(data[0].Manifold.Point1.Separation).To(BeNumerically("<=", 0.0))

		Expect(otherBody.SetTransform(box2d.Vec2{X: 0.0, Y: 0.75}, box2d.RotIdentity)).To(Succeed())
		segmentShape, err := otherBody.CreateSegmentShape(shapeDef, box2d.Segment{
			Point1: box2d.Vec2{X: -1.0, Y: 0.0},
			Point2: box2d.Vec2{X: 1.0, Y: 0.0},
		})
		Expect(err).NotTo(HaveOccurred())
		Expect(segmentShape).NotTo(BeNil())

		data, err = circleShape.GetContactData(8)
		Expect(err).NotTo(HaveOccurred())
		Expect(data).NotTo(BeEmpty())
		foundSegment := false
		for _, item := range data {
			if box2d.ShapeIDEqual(item.ShapeIDA, segmentShape.ID()) || box2d.ShapeIDEqual(item.ShapeIDB, segmentShape.ID()) {
				foundSegment = true
				Expect(item.Manifold.PointCount).To(BeNumerically(">=", 1))
				Expect(item.Manifold.Point1.Separation).To(BeNumerically("<=", 0.0))
			}
		}
		Expect(foundSegment).To(BeTrue())
	})

	It("populates manifold data for circle-polygon contacts", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		circleBodyDef := box2d.DefaultBodyDef()
		circleBodyDef.Type = box2d.DynamicBody
		circleBody, err := world.CreateBody(circleBodyDef)
		Expect(err).NotTo(HaveOccurred())

		polyBodyDef := box2d.DefaultBodyDef()
		polyBodyDef.Type = box2d.DynamicBody
		polyBodyDef.Position = box2d.Vec2{X: 0.0, Y: 1.25}
		polyBody, err := world.CreateBody(polyBodyDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		circleShape, err := circleBody.CreateCircleShape(shapeDef, box2d.Circle{
			Center: box2d.Vec2Zero,
			Radius: 1.0,
		})
		Expect(err).NotTo(HaveOccurred())

		_, err = polyBody.CreatePolygonShape(shapeDef, box2d.MakeBox(1.0, 0.5))
		Expect(err).NotTo(HaveOccurred())

		data, err := circleShape.GetContactData(8)
		Expect(err).NotTo(HaveOccurred())
		Expect(data).NotTo(BeEmpty())
		Expect(data[0].Manifold.PointCount).To(Equal(1))
		Expect(data[0].Manifold.Point1.Separation).To(BeNumerically("<=", 0.0))
	})

	It("populates manifold data for capsule-capsule and capsule-segment contacts", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		bodyADef := box2d.DefaultBodyDef()
		bodyADef.Type = box2d.DynamicBody
		bodyA, err := world.CreateBody(bodyADef)
		Expect(err).NotTo(HaveOccurred())

		bodyBDef := box2d.DefaultBodyDef()
		bodyBDef.Type = box2d.DynamicBody
		bodyBDef.Position = box2d.Vec2{X: 0.0, Y: 0.5}
		bodyB, err := world.CreateBody(bodyBDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		capsuleA, err := bodyA.CreateCapsuleShape(shapeDef, box2d.Capsule{
			Center1: box2d.Vec2{X: -1.0, Y: 0.0},
			Center2: box2d.Vec2{X: 1.0, Y: 0.0},
			Radius:  0.5,
		})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCapsuleShape(shapeDef, box2d.Capsule{
			Center1: box2d.Vec2{X: -1.0, Y: 0.0},
			Center2: box2d.Vec2{X: 1.0, Y: 0.0},
			Radius:  0.5,
		})
		Expect(err).NotTo(HaveOccurred())

		data, err := capsuleA.GetContactData(8)
		Expect(err).NotTo(HaveOccurred())
		Expect(data).NotTo(BeEmpty())
		Expect(data[0].Manifold.PointCount).To(BeNumerically(">=", 1))
		Expect(data[0].Manifold.Point1.Separation).To(BeNumerically("<", 0.0))

		segmentShape, err := bodyB.CreateSegmentShape(shapeDef, box2d.Segment{
			Point1: box2d.Vec2{X: -1.0, Y: 0.0},
			Point2: box2d.Vec2{X: 1.0, Y: 0.0},
		})
		Expect(err).NotTo(HaveOccurred())

		data, err = capsuleA.GetContactData(16)
		Expect(err).NotTo(HaveOccurred())
		foundSegment := false
		for _, item := range data {
			if box2d.ShapeIDEqual(item.ShapeIDA, segmentShape.ID()) || box2d.ShapeIDEqual(item.ShapeIDB, segmentShape.ID()) {
				foundSegment = true
				Expect(item.Manifold.PointCount).To(BeNumerically(">=", 1))
				Expect(item.Manifold.Point1.Separation).To(BeNumerically("<=", 0.0))
			}
		}
		Expect(foundSegment).To(BeTrue())
	})

	It("produces two manifold points for overlapping parallel capsules", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		bodyADef := box2d.DefaultBodyDef()
		bodyADef.Type = box2d.DynamicBody
		bodyA, err := world.CreateBody(bodyADef)
		Expect(err).NotTo(HaveOccurred())

		bodyBDef := box2d.DefaultBodyDef()
		bodyBDef.Type = box2d.DynamicBody
		bodyBDef.Position = box2d.Vec2{X: 0.25, Y: 0.4}
		bodyB, err := world.CreateBody(bodyBDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		capsuleA, err := bodyA.CreateCapsuleShape(shapeDef, box2d.Capsule{
			Center1: box2d.Vec2{X: -1.5, Y: 0.0},
			Center2: box2d.Vec2{X: 1.5, Y: 0.0},
			Radius:  0.5,
		})
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreateCapsuleShape(shapeDef, box2d.Capsule{
			Center1: box2d.Vec2{X: -1.0, Y: 0.0},
			Center2: box2d.Vec2{X: 1.0, Y: 0.0},
			Radius:  0.5,
		})
		Expect(err).NotTo(HaveOccurred())

		data, err := capsuleA.GetContactData(8)
		Expect(err).NotTo(HaveOccurred())
		Expect(data).NotTo(BeEmpty())
		Expect(data[0].Manifold.PointCount).To(Equal(2))
		Expect(data[0].Manifold.Point1.Separation).To(BeNumerically("<=", 0.0))
		Expect(data[0].Manifold.Point2.Separation).To(BeNumerically("<=", 0.0))
		Expect(box2d.DistanceSquared(data[0].Manifold.Point1.Point, data[0].Manifold.Point2.Point)).To(BeNumerically(">", 1e-4))
	})

	It("populates manifold data for polygon-polygon and capsule-polygon contacts", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		bodyADef := box2d.DefaultBodyDef()
		bodyADef.Type = box2d.DynamicBody
		bodyA, err := world.CreateBody(bodyADef)
		Expect(err).NotTo(HaveOccurred())

		bodyBDef := box2d.DefaultBodyDef()
		bodyBDef.Type = box2d.DynamicBody
		bodyBDef.Position = box2d.Vec2{X: 0.75, Y: 0.0}
		bodyB, err := world.CreateBody(bodyBDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		polyA, err := bodyA.CreatePolygonShape(shapeDef, box2d.MakeBox(1.0, 1.0))
		Expect(err).NotTo(HaveOccurred())
		_, err = bodyB.CreatePolygonShape(shapeDef, box2d.MakeBox(1.0, 1.0))
		Expect(err).NotTo(HaveOccurred())

		data, err := polyA.GetContactData(8)
		Expect(err).NotTo(HaveOccurred())
		Expect(data).NotTo(BeEmpty())
		Expect(data[0].Manifold.PointCount).To(Equal(1))
		Expect(data[0].Manifold.Point1.Separation).To(BeNumerically("<=", 0.0))

		capsuleShape, err := bodyB.CreateCapsuleShape(shapeDef, box2d.Capsule{
			Center1: box2d.Vec2{X: -1.0, Y: 0.0},
			Center2: box2d.Vec2{X: 1.0, Y: 0.0},
			Radius:  0.5,
		})
		Expect(err).NotTo(HaveOccurred())

		data, err = polyA.GetContactData(16)
		Expect(err).NotTo(HaveOccurred())
		foundCapsule := false
		for _, item := range data {
			if box2d.ShapeIDEqual(item.ShapeIDA, capsuleShape.ID()) || box2d.ShapeIDEqual(item.ShapeIDB, capsuleShape.ID()) {
				foundCapsule = true
				Expect(item.Manifold.PointCount).To(Equal(1))
				Expect(item.Manifold.Point1.Separation).To(BeNumerically("<=", 0.0))
			}
		}
		Expect(foundCapsule).To(BeTrue())
	})

	It("populates manifold data for segment-segment contacts", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		bodyADef := box2d.DefaultBodyDef()
		bodyADef.Type = box2d.DynamicBody
		bodyA, err := world.CreateBody(bodyADef)
		Expect(err).NotTo(HaveOccurred())

		bodyBDef := box2d.DefaultBodyDef()
		bodyBDef.Type = box2d.DynamicBody
		bodyB, err := world.CreateBody(bodyBDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		segmentA, err := bodyA.CreateSegmentShape(shapeDef, box2d.Segment{
			Point1: box2d.Vec2{X: -1.0, Y: 0.0},
			Point2: box2d.Vec2{X: 1.0, Y: 0.0},
		})
		Expect(err).NotTo(HaveOccurred())

		_, err = bodyB.CreateSegmentShape(shapeDef, box2d.Segment{
			Point1: box2d.Vec2{X: 0.0, Y: -1.0},
			Point2: box2d.Vec2{X: 0.0, Y: 1.0},
		})
		Expect(err).NotTo(HaveOccurred())

		data, err := segmentA.GetContactData(8)
		Expect(err).NotTo(HaveOccurred())
		Expect(data).NotTo(BeEmpty())
		Expect(data[0].Manifold.PointCount).To(Equal(1))
		Expect(data[0].Manifold.Point1.Separation).To(BeNumerically("<=", 0.0))
	})

	It("ray casts capsule shapes", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		body, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		capsule := box2d.Capsule{
			Center1: box2d.Vec2{X: -1, Y: 0},
			Center2: box2d.Vec2{X: 1, Y: 0},
			Radius:  0.5,
		}
		shape, err := body.CreateCapsuleShape(shapeDef, capsule)
		Expect(err).NotTo(HaveOccurred())

		ray := box2d.RayCastInput{
			Origin:      box2d.Vec2{X: -3, Y: 0},
			Translation: box2d.Vec2{X: 6, Y: 0},
			MaxFraction: 1.0,
		}
		output, err := shape.RayCast(ray)
		Expect(err).NotTo(HaveOccurred())
		Expect(output.Hit).To(BeTrue())
		Expect(output.Fraction).To(BeNumerically("~", 0.25, 1e-5))
	})

	It("applies wind forces to awake dynamic bodies", func() {
		worldDef := box2d.DefaultWorldDef()
		worldDef.Gravity = box2d.Vec2Zero
		world, err := box2d.NewWorld(worldDef)
		Expect(err).NotTo(HaveOccurred())

		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		body, err := world.CreateBody(bodyDef)
		Expect(err).NotTo(HaveOccurred())

		shape, err := body.CreateCircleShape(box2d.DefaultShapeDef(), box2d.Circle{
			Center: box2d.Vec2Zero,
			Radius: 1.0,
		})
		Expect(err).NotTo(HaveOccurred())

		Expect(shape.ApplyWind(box2d.Vec2{X: 10, Y: 0}, 0.2, 0.0, true)).To(Succeed())
		Expect(world.Step(1.0/60.0, 1)).To(Succeed())
		velocity, err := body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(velocity.X).To(BeNumerically(">", 0.0))

		Expect(body.SetLinearVelocity(box2d.Vec2Zero)).To(Succeed())
		Expect(body.SetAwake(false)).To(Succeed())
		Expect(shape.ApplyWind(box2d.Vec2{X: 10, Y: 0}, 0.2, 0.0, false)).To(Succeed())
		Expect(world.Step(1.0/60.0, 1)).To(Succeed())
		velocity, err = body.GetLinearVelocity()
		Expect(err).NotTo(HaveOccurred())
		Expect(velocity.X).To(BeNumerically("~", 0.0, 1e-6))
	})

	It("destroys attached shapes when a body is destroyed", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		body, err := world.CreateBody(box2d.DefaultBodyDef())
		Expect(err).NotTo(HaveOccurred())

		shapeDef := box2d.DefaultShapeDef()
		shape1, err := body.CreateCircleShape(shapeDef, box2d.Circle{Center: box2d.Vec2{}, Radius: 0.5})
		Expect(err).NotTo(HaveOccurred())
		shape2, err := body.CreateSegmentShape(shapeDef, box2d.Segment{
			Point1: box2d.Vec2{X: -1, Y: 0},
			Point2: box2d.Vec2{X: 1, Y: 0},
		})
		Expect(err).NotTo(HaveOccurred())

		counters, err := world.Counters()
		Expect(err).NotTo(HaveOccurred())
		Expect(counters.ShapeCount).To(Equal(2))

		Expect(body.Destroy()).To(Succeed())
		Expect(shape1.IsValid()).To(BeFalse())
		Expect(shape2.IsValid()).To(BeFalse())

		counters, err = world.Counters()
		Expect(err).NotTo(HaveOccurred())
		Expect(counters.ShapeCount).To(Equal(0))
	})

	It("rejects uninitialized shape definitions", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())
		body, err := world.CreateBody(box2d.DefaultBodyDef())
		Expect(err).NotTo(HaveOccurred())

		_, err = body.CreateCircleShape(box2d.ShapeDef{}, box2d.Circle{Center: box2d.Vec2{}, Radius: 1})
		Expect(err).To(MatchError(box2d.ErrInvalidShapeDef))
	})
})
