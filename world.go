package box2d

import (
	"cmp"
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"slices"
	"sync"
)

const MaxWorlds = 128

var (
	ErrInvalidWorldDef     = errors.New("box2d: invalid world definition (use DefaultWorldDef)")
	ErrInvalidExplosionDef = errors.New("box2d: invalid explosion definition (use DefaultExplosionDef)")
	ErrWorldLimitReached   = errors.New("box2d: maximum world count reached")
	ErrWorldInvalid        = errors.New("box2d: world is invalid")
	ErrInvalidTimeStep     = errors.New("box2d: timestep must be > 0")
	ErrInvalidSubStep      = errors.New("box2d: sub-step count must be >= 1")
	worldRegistryMu        sync.Mutex
	worldRegistry          [MaxWorlds]worldRegistrySlot
)

type worldRegistrySlot struct {
	generation uint16
	world      *World
}

type Profile struct {
	StepCount      int
	SubStepCount   int
	LastTimeStep   float32
	DrawCount      int
	BodyCount      int
	ShapeCount     int
	ContactCount   int
	SensorCount    int
	JointCount     int
	AwakeBodyCount int
}

type BodyMoveEvent struct {
	UserData   any
	Transform  Transform
	BodyID     BodyID
	FellAsleep bool
}

type BodyEvents struct {
	MoveEvents []BodyMoveEvent
	MoveCount  int
	Count      int
}

type SensorBeginTouchEvent struct {
	SensorShapeID  ShapeID
	VisitorShapeID ShapeID
}

type SensorEndTouchEvent struct {
	SensorShapeID  ShapeID
	VisitorShapeID ShapeID
}

type ContactEvents struct {
	BeginEvents []ContactBeginTouchEvent
	EndEvents   []ContactEndTouchEvent
	HitEvents   []ContactHitEvent
	BeginCount  int
	EndCount    int
	HitCount    int
	Count       int
}

type SensorEvents struct {
	BeginEvents []SensorBeginTouchEvent
	EndEvents   []SensorEndTouchEvent
	BeginCount  int
	EndCount    int
	Count       int
}

type JointEvents struct {
	Events []JointEvent
	Count  int
}

type JointEvent struct {
	JointID  JointID
	UserData any
}

type ContactBeginTouchEvent struct {
	ShapeIDA  ShapeID
	ShapeIDB  ShapeID
	ContactID ContactID
}

type ContactEndTouchEvent struct {
	ShapeIDA  ShapeID
	ShapeIDB  ShapeID
	ContactID ContactID
}

type ContactHitEvent struct {
	ShapeIDA      ShapeID
	ShapeIDB      ShapeID
	ContactID     ContactID
	Point         Vec2
	Normal        Vec2
	ApproachSpeed float32
}

type activeContact struct {
	contactID           ContactID
	shapeIDA            ShapeID
	shapeIDB            ShapeID
	manifold            Manifold
	normalImpulse       [2]float32
	tangentImpulse      [2]float32
	nextManifoldPointID uint16
	friction            float32
	restitution         float32
}

type sensorTouchKey struct {
	sensor  int
	visitor int
}

type activeSensorTouch struct {
	sensorShapeID  ShapeID
	visitorShapeID ShapeID
}

type CustomFilterCallback func(shapeA, shapeB ShapeID) bool
type FrictionCallback func(shapeA, shapeB ShapeID, defaultFriction float32) float32
type RestitutionCallback func(shapeA, shapeB ShapeID, defaultRestitution float32) float32
type PreSolveCallback func(shapeA, shapeB ShapeID) bool

// World is the OOP root simulation object.
// Additional systems (bodies, shapes, joints) are ported in later phases.
type World struct {
	mu                   sync.RWMutex
	id                   WorldID
	gravity              Vec2
	restitutionThreshold float32
	hitEventThreshold    float32
	contactHertz         float32
	contactDampingRatio  float32
	contactSpeed         float32
	maximumLinearSpeed   float32
	enableSleep          bool
	enableContinuous     bool
	enableWarmStarting   bool
	enableSpeculative    bool
	customFilterCallback CustomFilterCallback
	frictionCallback     FrictionCallback
	restitutionCallback  RestitutionCallback
	preSolveCallback     PreSolveCallback
	nextContactIndex     int32
	activeContactPairs   []shapePair
	activeContactData    []activeContact
	activeSensorKeys     []sensorTouchKey
	activeSensorData     []activeSensorTouch
	bodyEvents           BodyEvents
	contactEvents        ContactEvents
	sensorEvents         SensorEvents
	jointEvents          JointEvents
	profile              Profile
	userData             any
	bodies               []bodyState
	freeBodyIndices      []int
	bodyCount            int
	joints               []jointState
	freeJointIndices     []int
	jointCount           int
	chains               []chainState
	freeChainIndices     []int
	chainCount           int
	broadPhaseCandidates int
	staticTreeRebuilds   int
	lastMoverMatches     int
	shapes               []shapeState
	freeShapeIndices     []int
	shapeCount           int

	// scratch buffers reused across Step() calls to avoid per-frame allocations
	scratchPrevPositions  []Vec2
	scratchPrevRotations  []Rot
	scratchPrevAwake      []bool
	scratchBeginContacts  []ContactBeginTouchEvent
	scratchEndContacts    []ContactEndTouchEvent
	scratchHitEvents      []ContactHitEvent
	scratchCurrSensorKeys []sensorTouchKey
	scratchCurrSensorData []activeSensorTouch
	scratchBeginSensors   []SensorBeginTouchEvent
	scratchEndSensors     []SensorEndTouchEvent
	scratchBPShapes       []broadPhaseShape
	scratchBPPairs        []shapePair
	scratchBPActive       []int
	scratchMoveEvents     []BodyMoveEvent
	scratchJointEvents    []JointEvent
	scratchTouchPairs     []shapePair
	scratchTouchingPairs  []touchingPair
	scratchMergedPairs    []shapePair
	scratchMergedData     []activeContact
	scratchSensorPairs    []shapePair
}

// findActiveContact returns the index into activeContactPairs/activeContactData
// where pair is (or would be inserted), and whether it was found.
func (w *World) findActiveContact(pair shapePair) (int, bool) {
	return slices.BinarySearchFunc(w.activeContactPairs, pair, compareShapePairs)
}

func (w *World) insertActiveContact(pair shapePair, entry activeContact) {
	idx, found := w.findActiveContact(pair)
	if found {
		w.activeContactData[idx] = entry
		return
	}
	w.activeContactPairs = slices.Insert(w.activeContactPairs, idx, pair)
	w.activeContactData = slices.Insert(w.activeContactData, idx, entry)
}

func (w *World) removeActiveContactAt(idx int) {
	w.activeContactPairs = slices.Delete(w.activeContactPairs, idx, idx+1)
	w.activeContactData = slices.Delete(w.activeContactData, idx, idx+1)
}

func compareShapePairs(a, b shapePair) int {
	if c := cmp.Compare(a.a, b.a); c != 0 {
		return c
	}
	return cmp.Compare(a.b, b.b)
}

func compareSensorTouchKeys(a, b sensorTouchKey) int {
	if c := cmp.Compare(a.sensor, b.sensor); c != 0 {
		return c
	}
	return cmp.Compare(a.visitor, b.visitor)
}

// NewWorld creates a new physics world.
func NewWorld(def WorldDef) (*World, error) {
	if !def.isInitialized() {
		return nil, ErrInvalidWorldDef
	}

	worldRegistryMu.Lock()
	defer worldRegistryMu.Unlock()

	for i := range worldRegistry {
		slot := &worldRegistry[i]
		if slot.world != nil {
			continue
		}

		slot.generation++
		if slot.generation == 0 {
			slot.generation = 1
		}

		id := WorldID{
			Index1:     uint16(i + 1),
			Generation: slot.generation,
		}
		w := &World{
			id:                   id,
			gravity:              def.Gravity,
			restitutionThreshold: def.RestitutionThreshold,
			hitEventThreshold:    def.HitEventThreshold,
			contactHertz:         def.ContactHertz,
			contactDampingRatio:  def.ContactDampingRatio,
			contactSpeed:         def.ContactSpeed,
			maximumLinearSpeed:   def.MaximumLinearSpeed,
			enableSleep:          def.EnableSleep,
			enableContinuous:     def.EnableContinuous,
			enableWarmStarting:   true,
			enableSpeculative:    true,
			nextContactIndex:     0,
			activeContactPairs:   nil,
			activeContactData:    nil,
			activeSensorKeys:     nil,
			activeSensorData:     nil,
			userData:             def.UserData,
		}
		slot.world = w
		return w, nil
	}

	return nil, ErrWorldLimitReached
}

// IsWorldIDValid reports whether an ID refers to a currently alive world.
func IsWorldIDValid(id WorldID) bool {
	if id.Index1 == 0 {
		return false
	}

	index := int(id.Index1) - 1
	if index < 0 || index >= MaxWorlds {
		return false
	}

	worldRegistryMu.Lock()
	defer worldRegistryMu.Unlock()

	slot := worldRegistry[index]
	return slot.world != nil && slot.generation == id.Generation
}

// ID returns the world handle.
func (w *World) ID() WorldID {
	if w == nil {
		return NullWorldID
	}

	w.mu.RLock()
	defer w.mu.RUnlock()
	return w.id
}

// IsValid reports whether this world is still alive.
func (w *World) IsValid() bool {
	if w == nil {
		return false
	}
	return IsWorldIDValid(w.ID())
}

// Destroy destroys the world and invalidates this handle.
func (w *World) Destroy() error {
	if w == nil {
		return ErrWorldInvalid
	}

	id := w.ID()
	if id.IsNull() {
		return ErrWorldInvalid
	}

	index := int(id.Index1) - 1
	if index < 0 || index >= MaxWorlds {
		return ErrWorldInvalid
	}

	worldRegistryMu.Lock()
	defer worldRegistryMu.Unlock()

	slot := &worldRegistry[index]
	if slot.world != w || slot.generation != id.Generation {
		return ErrWorldInvalid
	}

	slot.world = nil
	slot.generation++
	if slot.generation == 0 {
		slot.generation = 1
	}

	w.mu.Lock()
	for i := range w.bodies {
		w.bodies[i].alive = false
		w.bodies[i].generation++
		if w.bodies[i].generation == 0 {
			w.bodies[i].generation = 1
		}
	}
	w.bodies = nil
	w.freeBodyIndices = nil
	w.bodyCount = 0
	for i := range w.shapes {
		w.shapes[i].alive = false
		w.shapes[i].generation++
		if w.shapes[i].generation == 0 {
			w.shapes[i].generation = 1
		}
	}
	for i := range w.joints {
		w.joints[i].alive = false
		w.joints[i].generation++
		if w.joints[i].generation == 0 {
			w.joints[i].generation = 1
		}
	}
	for i := range w.chains {
		w.chains[i].alive = false
		w.chains[i].generation++
		if w.chains[i].generation == 0 {
			w.chains[i].generation = 1
		}
		w.chains[i].shapeIndices = nil
		w.chains[i].materials = nil
	}
	w.shapes = nil
	w.freeShapeIndices = nil
	w.shapeCount = 0
	w.joints = nil
	w.freeJointIndices = nil
	w.jointCount = 0
	w.chains = nil
	w.freeChainIndices = nil
	w.chainCount = 0
	w.activeContactPairs = nil
	w.activeContactData = nil
	w.activeSensorKeys = nil
	w.activeSensorData = nil
	w.bodyEvents = BodyEvents{}
	w.contactEvents = ContactEvents{}
	w.sensorEvents = SensorEvents{}
	w.jointEvents = JointEvents{}
	w.id = NullWorldID
	w.mu.Unlock()
	return nil
}

// Step advances the world by one frame.
// This phase integrates body velocities/poses with deterministic sub-steps.
func (w *World) Step(timeStep float32, subStepCount int) error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	if timeStep <= 0 {
		return ErrInvalidTimeStep
	}
	if subStepCount < 1 {
		return ErrInvalidSubStep
	}

	h := timeStep / float32(subStepCount)

	w.mu.Lock()
	defer w.mu.Unlock()
	if w.id.IsNull() {
		return ErrWorldInvalid
	}

	n := len(w.bodies)
	if cap(w.scratchPrevPositions) < n {
		w.scratchPrevPositions = make([]Vec2, n)
		w.scratchPrevRotations = make([]Rot, n)
		w.scratchPrevAwake = make([]bool, n)
	}
	prevPositions := w.scratchPrevPositions[:n]
	prevRotations := w.scratchPrevRotations[:n]
	prevAwake := w.scratchPrevAwake[:n]
	for i := range w.bodies {
		prevPositions[i] = w.bodies[i].position
		prevRotations[i] = w.bodies[i].rotation
		prevAwake[i] = w.bodies[i].isAwake
	}

	for range subStepCount {
		scaledGravity := MulSV(h, w.gravity)
		for i := range w.bodies {
			body := &w.bodies[i]
			if !body.alive || !body.isEnabled || !body.isAwake {
				continue
			}
			if body.bodyType == StaticBody {
				continue
			}

			linearDamping := float32(1.0) / (1.0 + h*body.linearDamping)
			angularDamping := float32(1.0) / (1.0 + h*body.angularDamping)

			if body.bodyType == DynamicBody {
				forceDelta := MulSV(h*body.invMass, body.force)
				gravityScale := body.gravityScale
				if body.invMass == 0 {
					gravityScale = 0
				}
				gravityDelta := MulSV(gravityScale, scaledGravity)
				linearVelocityDelta := Add(forceDelta, gravityDelta)
				angularVelocityDelta := h * body.invInertia * body.torque
				body.linearVelocity = MulAdd(linearVelocityDelta, linearDamping, body.linearVelocity)
				body.angularVelocity = angularVelocityDelta + angularDamping*body.angularVelocity
			} else {
				body.linearVelocity = MulSV(linearDamping, body.linearVelocity)
				body.angularVelocity *= angularDamping
			}

			if lf := body.lockFlags; lf != 0 {
				if lf&lockFlagLinearX != 0 {
					body.linearVelocity.X = 0
				}
				if lf&lockFlagLinearY != 0 {
					body.linearVelocity.Y = 0
				}
				if lf&lockFlagAngularZ != 0 {
					body.angularVelocity = 0
				}
			}

			oldCenter := Add(body.position, RotateVector(body.rotation, body.localCenter))
			body.rotation = integrateRotation(body.rotation, h*body.angularVelocity)
			newCenter := MulAdd(oldCenter, h, body.linearVelocity)
			body.position = Sub(newCenter, RotateVector(body.rotation, body.localCenter))
		}
	}
	for i := range w.bodies {
		w.bodies[i].force = Vec2Zero
		w.bodies[i].torque = 0
	}
	w.refreshContactAndSensorStateLocked()
	w.solveActiveContactsLocked(h, subStepCount)
	w.solveJointsLocked(h, subStepCount)
	w.updateSleepStateLocked(timeStep)
	w.refreshManifoldsAndEventsLocked()
	w.refreshBodyEventsLocked(prevPositions, prevRotations, prevAwake)
	w.refreshJointEventsLocked()
	w.profile.StepCount++
	w.profile.SubStepCount = subStepCount
	w.profile.LastTimeStep = timeStep
	w.profile.BodyCount = w.bodyCount
	w.profile.ShapeCount = w.shapeCount
	w.profile.ContactCount = len(w.activeContactPairs)
	w.profile.SensorCount = len(w.activeSensorKeys)
	w.profile.JointCount = w.jointCount
	awakeCount := 0
	for i := range w.bodies {
		body := w.bodies[i]
		if body.alive && body.isEnabled && body.isAwake {
			awakeCount++
		}
	}
	w.profile.AwakeBodyCount = awakeCount

	return nil
}

func clampFloat(v, lo, hi float32) float32 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

func mixFrictionValue(frictionA, frictionB float32) float32 {
	return float32(math.Sqrt(float64(MaxFloat(0, frictionA) * MaxFloat(0, frictionB))))
}

func (w *World) effectiveContactMaterialLocked(shapeAID, shapeBID ShapeID, shapeA, shapeB shapeState) (float32, float32) {
	friction := mixFrictionValue(shapeA.material.Friction, shapeB.material.Friction)
	if w.frictionCallback != nil {
		friction = w.frictionCallback(shapeAID, shapeBID, friction)
	}
	if friction < 0 {
		friction = 0
	}

	restitution := MaxFloat(shapeA.material.Restitution, shapeB.material.Restitution)
	if w.restitutionCallback != nil {
		restitution = w.restitutionCallback(shapeAID, shapeBID, restitution)
	}
	if restitution < 0 {
		restitution = 0
	}
	return friction, restitution
}

func applyContactImpulse(body *bodyState, impulse Vec2, r Vec2, sign float32) {
	body.linearVelocity = MulAdd(body.linearVelocity, sign*body.invMass, impulse)
	body.angularVelocity += sign * body.invInertia * Cross(r, impulse)
}

func (w *World) solveActiveContactsLocked(dt float32, iterations int) {
	if iterations < 1 {
		iterations = 1
	}
	pairs := w.sortedActiveContactPairsLocked()
	if len(pairs) == 0 {
		return
	}

	correctionScale := clampFloat(w.contactSpeed*dt, 0, 0.8)
	if correctionScale == 0 {
		correctionScale = 0.2
	}
	const penetrationSlop = float32(0.005)

	for iter := range iterations {
		for ci, pair := range pairs {
			entry := w.activeContactData[ci]
			if entry.manifold.PointCount == 0 {
				continue
			}

			shapeA := w.shapes[pair.a]
			shapeB := w.shapes[pair.b]
			if !shapeA.alive || !shapeB.alive {
				continue
			}
			bodyAIdx := shapeA.bodyIndex
			bodyBIdx := shapeB.bodyIndex
			if bodyAIdx < 0 || bodyAIdx >= len(w.bodies) || bodyBIdx < 0 || bodyBIdx >= len(w.bodies) {
				continue
			}
			bodyA := &w.bodies[bodyAIdx]
			bodyB := &w.bodies[bodyBIdx]
			if !bodyA.alive || !bodyB.alive || !bodyA.isEnabled || !bodyB.isEnabled {
				continue
			}

			dynamicA := bodyA.bodyType == DynamicBody && bodyA.isAwake
			dynamicB := bodyB.bodyType == DynamicBody && bodyB.isAwake
			if !dynamicA && !dynamicB {
				continue
			}

			invMassA := float32(0)
			invMassB := float32(0)
			invInertiaA := float32(0)
			invInertiaB := float32(0)
			if dynamicA {
				invMassA = bodyA.invMass
				invInertiaA = bodyA.invInertia
			}
			if dynamicB {
				invMassB = bodyB.invMass
				invInertiaB = bodyB.invInertia
			}
			if invMassA+invMassB == 0 {
				continue
			}

			normal := entry.manifold.Normal
			if LengthSquared(normal) <= floatEpsilon {
				continue
			}
			normal = Normalize(normal)

			friction, restitution := entry.friction, entry.restitution
			points := [2]ManifoldPoint{entry.manifold.Point1, entry.manifold.Point2}
			if entry.manifold.PointCount > len(points) {
				entry.manifold.PointCount = len(points)
			}
			if entry.manifold.PointCount < len(entry.normalImpulse) {
				for pi := entry.manifold.PointCount; pi < len(entry.normalImpulse); pi++ {
					entry.normalImpulse[pi] = 0
					entry.tangentImpulse[pi] = 0
				}
			}
			centerA := Add(bodyA.position, RotateVector(bodyA.rotation, bodyA.localCenter))
			centerB := Add(bodyB.position, RotateVector(bodyB.rotation, bodyB.localCenter))
			for pi := 0; pi < entry.manifold.PointCount; pi++ {
				contactPoint := points[pi].Point
				ra := Sub(contactPoint, centerA)
				rb := Sub(contactPoint, centerB)

				velA := Add(bodyA.linearVelocity, CrossSV(bodyA.angularVelocity, ra))
				velB := Add(bodyB.linearVelocity, CrossSV(bodyB.angularVelocity, rb))
				relVel := Sub(velB, velA)
				normalSpeed := Dot(relVel, normal)
				effectiveRestitution := restitution
				if -normalSpeed < w.restitutionThreshold {
					effectiveRestitution = 0
				}

				raCrossN := Cross(ra, normal)
				rbCrossN := Cross(rb, normal)
				normalMass := invMassA + invMassB + invInertiaA*raCrossN*raCrossN + invInertiaB*rbCrossN*rbCrossN
				if normalMass > 0 {
					if w.enableWarmStarting && iter == 0 && entry.normalImpulse[pi] != 0 {
						warmImpulse := MulSV(entry.normalImpulse[pi], normal)
						if dynamicA {
							applyContactImpulse(bodyA, warmImpulse, ra, -1)
						}
						if dynamicB {
							applyContactImpulse(bodyB, warmImpulse, rb, 1)
						}
					}
					if w.enableWarmStarting && iter == 0 && entry.tangentImpulse[pi] != 0 {
						tangent := Vec2{X: -normal.Y, Y: normal.X}
						warmTangentImpulse := MulSV(entry.tangentImpulse[pi], tangent)
						if dynamicA {
							applyContactImpulse(bodyA, warmTangentImpulse, ra, -1)
						}
						if dynamicB {
							applyContactImpulse(bodyB, warmTangentImpulse, rb, 1)
						}
					}
					normalImpulseMag := -(1 + effectiveRestitution) * normalSpeed / normalMass
					if normalImpulseMag > 0 {
						impulse := MulSV(normalImpulseMag, normal)
						if dynamicA {
							applyContactImpulse(bodyA, impulse, ra, -1)
						}
						if dynamicB {
							applyContactImpulse(bodyB, impulse, rb, 1)
						}

						velA = Add(bodyA.linearVelocity, CrossSV(bodyA.angularVelocity, ra))
						velB = Add(bodyB.linearVelocity, CrossSV(bodyB.angularVelocity, rb))
						relVel = Sub(velB, velA)

						tangent := Sub(relVel, MulSV(Dot(relVel, normal), normal))
						if LengthSquared(tangent) > floatEpsilon {
							tangent = Normalize(tangent)
							raCrossT := Cross(ra, tangent)
							rbCrossT := Cross(rb, tangent)
							tangentMass := invMassA + invMassB + invInertiaA*raCrossT*raCrossT + invInertiaB*rbCrossT*rbCrossT
							if tangentMass > 0 {
								tangentImpulseMag := -Dot(relVel, tangent) / tangentMass
								maxFriction := friction * normalImpulseMag
								tangentImpulseMag = clampFloat(tangentImpulseMag, -maxFriction, maxFriction)
								if tangentImpulseMag != 0 {
									tangentImpulse := MulSV(tangentImpulseMag, tangent)
									if dynamicA {
										applyContactImpulse(bodyA, tangentImpulse, ra, -1)
									}
									if dynamicB {
										applyContactImpulse(bodyB, tangentImpulse, rb, 1)
									}
									entry.tangentImpulse[pi] = tangentImpulseMag
								}
							}
						}
						entry.normalImpulse[pi] = normalImpulseMag
					} else {
						entry.normalImpulse[pi] = 0
						entry.tangentImpulse[pi] = 0
					}
				}

				penetration := -points[pi].Separation
				if penetration > penetrationSlop {
					correction := correctionScale * (penetration - penetrationSlop) / (invMassA + invMassB)
					offset := MulSV(correction, normal)
					if dynamicA {
						bodyA.position = MulAdd(bodyA.position, -invMassA, offset)
					}
					if dynamicB {
						bodyB.position = MulAdd(bodyB.position, invMassB, offset)
					}
				}
			}

			if lf := bodyA.lockFlags; lf != 0 {
				if lf&lockFlagLinearX != 0 {
					bodyA.linearVelocity.X = 0
				}
				if lf&lockFlagLinearY != 0 {
					bodyA.linearVelocity.Y = 0
				}
				if lf&lockFlagAngularZ != 0 {
					bodyA.angularVelocity = 0
				}
			}
			if lf := bodyB.lockFlags; lf != 0 {
				if lf&lockFlagLinearX != 0 {
					bodyB.linearVelocity.X = 0
				}
				if lf&lockFlagLinearY != 0 {
					bodyB.linearVelocity.Y = 0
				}
				if lf&lockFlagAngularZ != 0 {
					bodyB.angularVelocity = 0
				}
			}
			w.activeContactData[ci] = entry
		}
	}
}

func (w *World) solveJointsLocked(dt float32, iterations int) {
	if iterations < 1 {
		iterations = 1
	}
	for i := range w.joints {
		w.joints[i].distanceMotorForce = 0
		w.joints[i].prismaticMotorForce = 0
		w.joints[i].revoluteMotorTorque = 0
		w.joints[i].wheelMotorTorque = 0
	}
	for range iterations {
		for i := range w.joints {
			joint := &w.joints[i]
			if !joint.alive {
				continue
			}
			switch joint.jointType {
			case DistanceJoint, MotorJoint, PrismaticJoint, RevoluteJoint, WheelJoint, WeldJoint:
			default:
				continue
			}
			if joint.bodyAIndex < 0 || joint.bodyAIndex >= len(w.bodies) || joint.bodyBIndex < 0 || joint.bodyBIndex >= len(w.bodies) {
				continue
			}
			bodyA := &w.bodies[joint.bodyAIndex]
			bodyB := &w.bodies[joint.bodyBIndex]
			if !bodyA.alive || !bodyB.alive || !bodyA.isEnabled || !bodyB.isEnabled {
				continue
			}

			dynamicA := bodyA.bodyType == DynamicBody && bodyA.isAwake
			dynamicB := bodyB.bodyType == DynamicBody && bodyB.isAwake
			if !dynamicA && !dynamicB {
				continue
			}

			invMassA := float32(0)
			invMassB := float32(0)
			if dynamicA {
				invMassA = bodyA.invMass
			}
			if dynamicB {
				invMassB = bodyB.invMass
			}
			if invMassA+invMassB == 0 {
				continue
			}

			switch joint.jointType {
			case PrismaticJoint:
				axis := RotateVector(MulRot(bodyA.rotation, joint.localFrameA.Q), Vec2{X: 1, Y: 0})
				transformA := Transform{P: bodyA.position, Q: bodyA.rotation}
				transformB := Transform{P: bodyB.position, Q: bodyB.rotation}
				pA := TransformPoint(transformA, joint.localFrameA.P)
				pB := TransformPoint(transformB, joint.localFrameB.P)
				translation := Dot(Sub(pB, pA), axis)

				applyConstraint := joint.prismaticSpringEnabled || joint.prismaticLimitEnabled
				if applyConstraint {
					target := translation
					if joint.prismaticSpringEnabled {
						target = joint.prismaticTarget
					}
					if joint.prismaticLimitEnabled {
						target = ClampFloat(target, joint.prismaticLowerLimit, joint.prismaticUpperLimit)
						if !joint.prismaticSpringEnabled {
							target = ClampFloat(translation, joint.prismaticLowerLimit, joint.prismaticUpperLimit)
						}
					}
					error := translation - target
					relVel := Sub(bodyB.linearVelocity, bodyA.linearVelocity)
					speedError := Dot(relVel, axis)
					hertz := joint.prismaticHertz
					if hertz <= 0 {
						hertz = joint.constraintHertz
					}
					impulse := -(speedError + hertz*error) / (invMassA + invMassB)
					impulseVec := MulSV(impulse, axis)
					if dynamicA {
						bodyA.linearVelocity = MulAdd(bodyA.linearVelocity, -invMassA, impulseVec)
					}
					if dynamicB {
						bodyB.linearVelocity = MulAdd(bodyB.linearVelocity, invMassB, impulseVec)
					}
					posCorrection := clampFloat(error, -0.2, 0.2) * 0.2
					correctionVec := MulSV(posCorrection/(invMassA+invMassB), axis)
					if dynamicA {
						bodyA.position = MulAdd(bodyA.position, invMassA, correctionVec)
					}
					if dynamicB {
						bodyB.position = MulAdd(bodyB.position, -invMassB, correctionVec)
					}
				}

				if !joint.prismaticMotorEnabled || joint.prismaticMaxMotorForce <= 0 {
					continue
				}
				relVel := Sub(bodyB.linearVelocity, bodyA.linearVelocity)
				speedError := Dot(relVel, axis) - joint.prismaticMotorSpeed
				motorImpulse := -speedError / (invMassA + invMassB)
				maxImpulse := joint.prismaticMaxMotorForce * dt
				motorImpulse = clampFloat(motorImpulse, -maxImpulse, maxImpulse)
				impulseVec := MulSV(motorImpulse, axis)
				if dynamicA {
					bodyA.linearVelocity = MulAdd(bodyA.linearVelocity, -invMassA, impulseVec)
				}
				if dynamicB {
					bodyB.linearVelocity = MulAdd(bodyB.linearVelocity, invMassB, impulseVec)
				}
				joint.prismaticMotorForce += AbsFloat(motorImpulse / MaxFloat(dt, floatEpsilon))

			case WeldJoint:
				transformA := Transform{P: bodyA.position, Q: bodyA.rotation}
				transformB := Transform{P: bodyB.position, Q: bodyB.rotation}
				anchorA := TransformPoint(transformA, joint.localFrameA.P)
				anchorB := TransformPoint(transformB, joint.localFrameB.P)
				delta := Sub(anchorB, anchorA)
				relVel := Sub(bodyB.linearVelocity, bodyA.linearVelocity)
				hertz := joint.weldLinearHertz
				if hertz <= 0 {
					hertz = joint.constraintHertz
				}
				impulse := MulSV(-1.0/(invMassA+invMassB), Add(relVel, MulSV(hertz, delta)))
				if dynamicA {
					bodyA.linearVelocity = MulAdd(bodyA.linearVelocity, -invMassA, impulse)
				}
				if dynamicB {
					bodyB.linearVelocity = MulAdd(bodyB.linearVelocity, invMassB, impulse)
				}
				correction := MulSV(0.15, delta)
				if dynamicA {
					bodyA.position = MulAdd(bodyA.position, invMassA/(invMassA+invMassB), correction)
				}
				if dynamicB {
					bodyB.position = MulAdd(bodyB.position, -invMassB/(invMassA+invMassB), correction)
				}

				invIA := float32(0)
				invIB := float32(0)
				if dynamicA {
					invIA = bodyA.invInertia
				}
				if dynamicB {
					invIB = bodyB.invInertia
				}
				angularMass := invIA + invIB
				if angularMass > 0 {
					qa := MulRot(bodyA.rotation, joint.localFrameA.Q)
					qb := MulRot(bodyB.rotation, joint.localFrameB.Q)
					angleError := RelativeAngle(qa, qb)
					angHertz := joint.weldAngularHertz
					if angHertz <= 0 {
						angHertz = joint.constraintHertz
					}
					angularImpulse := -((bodyB.angularVelocity - bodyA.angularVelocity) + angHertz*angleError) / angularMass
					if dynamicA {
						bodyA.angularVelocity -= invIA * angularImpulse
					}
					if dynamicB {
						bodyB.angularVelocity += invIB * angularImpulse
					}
					angleCorrection := clampFloat(angleError, -0.2, 0.2) * 0.2
					if dynamicA {
						bodyA.rotation = integrateRotation(bodyA.rotation, invIA*angleCorrection)
					}
					if dynamicB {
						bodyB.rotation = integrateRotation(bodyB.rotation, -invIB*angleCorrection)
					}
				}

			case RevoluteJoint, WheelJoint:
				invIA := float32(0)
				invIB := float32(0)
				if dynamicA {
					invIA = bodyA.invInertia
				}
				if dynamicB {
					invIB = bodyB.invInertia
				}
				angularMass := invIA + invIB
				if angularMass <= 0 {
					continue
				}

				var enabled bool
				var speed float32
				var maxTorque float32
				if joint.jointType == RevoluteJoint {
					if joint.revoluteSpringEnabled || joint.revoluteLimitEnabled {
						qa := MulRot(bodyA.rotation, joint.localFrameA.Q)
						qb := MulRot(bodyB.rotation, joint.localFrameB.Q)
						angle := RelativeAngle(qa, qb)
						target := angle
						if joint.revoluteSpringEnabled {
							target = joint.revoluteTargetAngle
						}
						if joint.revoluteLimitEnabled {
							target = ClampFloat(target, joint.revoluteLowerAngle, joint.revoluteUpperAngle)
							if !joint.revoluteSpringEnabled {
								target = ClampFloat(angle, joint.revoluteLowerAngle, joint.revoluteUpperAngle)
							}
						}
						angleError := angle - target
						hertz := joint.revoluteHertz
						if hertz <= 0 {
							hertz = joint.constraintHertz
						}
						angularImpulse := -((bodyB.angularVelocity - bodyA.angularVelocity) + hertz*angleError) / angularMass
						if dynamicA {
							bodyA.angularVelocity -= invIA * angularImpulse
						}
						if dynamicB {
							bodyB.angularVelocity += invIB * angularImpulse
						}
						angleCorrection := clampFloat(angleError, -0.2, 0.2) * 0.2
						if dynamicA {
							bodyA.rotation = integrateRotation(bodyA.rotation, invIA*angleCorrection)
						}
						if dynamicB {
							bodyB.rotation = integrateRotation(bodyB.rotation, -invIB*angleCorrection)
						}
					}
					enabled = joint.revoluteMotorEnabled
					speed = joint.revoluteMotorSpeed
					maxTorque = joint.revoluteMaxMotorTorque
				} else {
					enabled = joint.wheelMotorEnabled
					speed = joint.wheelMotorSpeed
					maxTorque = joint.wheelMaxMotorTorque
					axis := RotateVector(MulRot(bodyA.rotation, joint.localFrameA.Q), Vec2{X: 1, Y: 0})
					transformA := Transform{P: bodyA.position, Q: bodyA.rotation}
					transformB := Transform{P: bodyB.position, Q: bodyB.rotation}
					pA := TransformPoint(transformA, joint.localFrameA.P)
					pB := TransformPoint(transformB, joint.localFrameB.P)
					translation := Dot(Sub(pB, pA), axis)
					applyConstraint := joint.wheelSpringEnabled || joint.wheelLimitEnabled
					if applyConstraint {
						target := translation
						if joint.wheelSpringEnabled {
							target = 0
						}
						if joint.wheelLimitEnabled {
							target = ClampFloat(target, joint.wheelLowerTranslation, joint.wheelUpperTranslation)
							if !joint.wheelSpringEnabled {
								target = ClampFloat(translation, joint.wheelLowerTranslation, joint.wheelUpperTranslation)
							}
						}
						error := translation - target
						relVel := Sub(bodyB.linearVelocity, bodyA.linearVelocity)
						speedError := Dot(relVel, axis)
						hertz := joint.wheelHertz
						if hertz <= 0 {
							hertz = joint.constraintHertz
						}
						impulse := -(speedError + hertz*error) / (invMassA + invMassB)
						impulseVec := MulSV(impulse, axis)
						if dynamicA {
							bodyA.linearVelocity = MulAdd(bodyA.linearVelocity, -invMassA, impulseVec)
						}
						if dynamicB {
							bodyB.linearVelocity = MulAdd(bodyB.linearVelocity, invMassB, impulseVec)
						}
						posCorrection := clampFloat(error, -0.2, 0.2) * 0.2
						correctionVec := MulSV(posCorrection/(invMassA+invMassB), axis)
						if dynamicA {
							bodyA.position = MulAdd(bodyA.position, invMassA, correctionVec)
						}
						if dynamicB {
							bodyB.position = MulAdd(bodyB.position, -invMassB, correctionVec)
						}
					}
				}
				if !enabled || maxTorque <= 0 {
					continue
				}
				speedError := (bodyB.angularVelocity - bodyA.angularVelocity) - speed
				motorImpulse := -speedError / angularMass
				maxAngularImpulse := maxTorque * dt
				motorImpulse = clampFloat(motorImpulse, -maxAngularImpulse, maxAngularImpulse)
				if dynamicA {
					bodyA.angularVelocity -= invIA * motorImpulse
				}
				if dynamicB {
					bodyB.angularVelocity += invIB * motorImpulse
				}
				torque := AbsFloat(motorImpulse / MaxFloat(dt, floatEpsilon))
				if joint.jointType == RevoluteJoint {
					joint.revoluteMotorTorque += torque
				} else {
					joint.wheelMotorTorque += torque
				}

			case MotorJoint:
				transformA := Transform{P: bodyA.position, Q: bodyA.rotation}
				transformB := Transform{P: bodyB.position, Q: bodyB.rotation}
				relVel := Sub(bodyB.linearVelocity, bodyA.linearVelocity)
				velocityError := Sub(relVel, joint.motorLinearVelocity)
				mass := invMassA + invMassB
				if mass > 0 {
					impulse := MulSV(-1.0/mass, velocityError)
					maxImpulse := joint.motorMaxVelocityForce * dt
					if maxImpulse > 0 {
						impulseMag := Length(impulse)
						if impulseMag > maxImpulse {
							impulse = MulSV(maxImpulse/impulseMag, impulse)
						}
						if dynamicA {
							bodyA.linearVelocity = MulAdd(bodyA.linearVelocity, -invMassA, impulse)
						}
						if dynamicB {
							bodyB.linearVelocity = MulAdd(bodyB.linearVelocity, invMassB, impulse)
						}
					}
				}
				if joint.motorLinearHertz > 0 && joint.motorMaxSpringForce > 0 {
					anchorA := TransformPoint(transformA, joint.localFrameA.P)
					anchorB := TransformPoint(transformB, joint.localFrameB.P)
					positionError := Sub(anchorB, anchorA)
					springImpulse := MulSV(-joint.motorLinearHertz/mass, positionError)
					maxSpringImpulse := joint.motorMaxSpringForce * dt
					impulseMag := Length(springImpulse)
					if impulseMag > maxSpringImpulse && impulseMag > floatEpsilon {
						springImpulse = MulSV(maxSpringImpulse/impulseMag, springImpulse)
					}
					if dynamicA {
						bodyA.linearVelocity = MulAdd(bodyA.linearVelocity, -invMassA, springImpulse)
					}
					if dynamicB {
						bodyB.linearVelocity = MulAdd(bodyB.linearVelocity, invMassB, springImpulse)
					}
					correction := MulSV(0.15, positionError)
					if dynamicA {
						bodyA.position = MulAdd(bodyA.position, invMassA/mass, correction)
					}
					if dynamicB {
						bodyB.position = MulAdd(bodyB.position, -invMassB/mass, correction)
					}
				}

				invIA := float32(0)
				invIB := float32(0)
				if dynamicA {
					invIA = bodyA.invInertia
				}
				if dynamicB {
					invIB = bodyB.invInertia
				}
				angularMass := invIA + invIB
				if angularMass > 0 {
					angularError := (bodyB.angularVelocity - bodyA.angularVelocity) - joint.motorAngularVelocity
					angularImpulse := -angularError / angularMass
					maxAngularImpulse := joint.motorMaxVelocityTorque * dt
					if maxAngularImpulse > 0 {
						angularImpulse = clampFloat(angularImpulse, -maxAngularImpulse, maxAngularImpulse)
						if dynamicA {
							bodyA.angularVelocity -= invIA * angularImpulse
						}
						if dynamicB {
							bodyB.angularVelocity += invIB * angularImpulse
						}
					}
				}
				if joint.motorAngularHertz > 0 && joint.motorMaxSpringTorque > 0 {
					qa := MulRot(bodyA.rotation, joint.localFrameA.Q)
					qb := MulRot(bodyB.rotation, joint.localFrameB.Q)
					angularSeparation := RelativeAngle(qa, qb)
					springAngularImpulse := -joint.motorAngularHertz * angularSeparation / MaxFloat(angularMass, floatEpsilon)
					maxSpringAngularImpulse := joint.motorMaxSpringTorque * dt
					springAngularImpulse = clampFloat(springAngularImpulse, -maxSpringAngularImpulse, maxSpringAngularImpulse)
					if dynamicA {
						bodyA.angularVelocity -= invIA * springAngularImpulse
					}
					if dynamicB {
						bodyB.angularVelocity += invIB * springAngularImpulse
					}
				}

			default: // DistanceJoint
				transformA := Transform{P: bodyA.position, Q: bodyA.rotation}
				transformB := Transform{P: bodyB.position, Q: bodyB.rotation}
				anchorA := TransformPoint(transformA, joint.localFrameA.P)
				anchorB := TransformPoint(transformB, joint.localFrameB.P)
				delta := Sub(anchorB, anchorA)
				length := Length(delta)
				if length <= floatEpsilon {
					continue
				}
				axis := MulSV(1.0/length, delta)

				targetLength := joint.distanceLength
				if joint.distanceLimitEnabled {
					if length < joint.distanceMinLength {
						targetLength = joint.distanceMinLength
					} else if length > joint.distanceMaxLength {
						targetLength = joint.distanceMaxLength
					} else {
						targetLength = length
					}
				}
				if targetLength < linearSlop() {
					targetLength = linearSlop()
				}

				relVel := Sub(bodyB.linearVelocity, bodyA.linearVelocity)
				relativeSpeed := Dot(relVel, axis)

				if joint.distanceMotorEnabled && joint.distanceMaxMotorForce > 0 {
					motorImpulse := -(relativeSpeed - joint.distanceMotorSpeed) / (invMassA + invMassB)
					maxImpulse := joint.distanceMaxMotorForce * dt
					motorImpulse = clampFloat(motorImpulse, -maxImpulse, maxImpulse)
					impulseVec := MulSV(motorImpulse, axis)
					if dynamicA {
						bodyA.linearVelocity = MulAdd(bodyA.linearVelocity, -invMassA, impulseVec)
					}
					if dynamicB {
						bodyB.linearVelocity = MulAdd(bodyB.linearVelocity, invMassB, impulseVec)
					}
					joint.distanceMotorForce += AbsFloat(motorImpulse / MaxFloat(dt, floatEpsilon))
				}

				relVel = Sub(bodyB.linearVelocity, bodyA.linearVelocity)
				relativeSpeed = Dot(relVel, axis)
				distanceError := length - targetLength
				stiffness := joint.constraintHertz
				if joint.distanceSpringEnabled && joint.distanceSpringHertz > 0 {
					stiffness = joint.distanceSpringHertz
				}
				if stiffness < 0 {
					stiffness = 0
				}
				bias := stiffness * distanceError
				constraintImpulse := -(relativeSpeed + bias) / (invMassA + invMassB)
				impulseVec := MulSV(constraintImpulse, axis)
				if dynamicA {
					bodyA.linearVelocity = MulAdd(bodyA.linearVelocity, -invMassA, impulseVec)
				}
				if dynamicB {
					bodyB.linearVelocity = MulAdd(bodyB.linearVelocity, invMassB, impulseVec)
				}

				posCorrection := clampFloat(distanceError, -0.2, 0.2) * 0.2
				correctionVec := MulSV(posCorrection/(invMassA+invMassB), axis)
				if dynamicA {
					bodyA.position = MulAdd(bodyA.position, -invMassA, correctionVec)
				}
				if dynamicB {
					bodyB.position = MulAdd(bodyB.position, invMassB, correctionVec)
				}
			}
		}
	}
}

func (w *World) updateSleepStateLocked(dt float32) {
	if !w.enableSleep {
		for i := range w.bodies {
			body := &w.bodies[i]
			if !body.alive || !body.isEnabled || body.bodyType != DynamicBody {
				continue
			}
			body.isAwake = true
			body.sleepTime = 0
		}
		return
	}

	const timeToSleep = float32(0.5)
	for i := range w.bodies {
		body := &w.bodies[i]
		if !body.alive || !body.isEnabled || body.bodyType != DynamicBody {
			continue
		}
		if !body.enableSleep {
			body.isAwake = true
			body.sleepTime = 0
			continue
		}
		if !body.isAwake {
			continue
		}

		threshold := body.sleepThreshold
		if threshold < 0 {
			threshold = 0
		}
		linearSleep := LengthSquared(body.linearVelocity) <= threshold*threshold
		angularSleep := AbsFloat(body.angularVelocity) <= threshold
		if linearSleep && angularSleep {
			body.sleepTime += dt
			if body.sleepTime >= timeToSleep {
				body.isAwake = false
				body.sleepTime = 0
				body.linearVelocity = Vec2Zero
				body.angularVelocity = 0
			}
		} else {
			body.sleepTime = 0
		}
	}
}

func integrateRotation(q Rot, deltaAngle float32) Rot {
	q2 := Rot{
		C: q.C - deltaAngle*q.S,
		S: q.S + deltaAngle*q.C,
	}
	return NormalizeRot(q2)
}

func (w *World) refreshBodyEventsLocked(prevPositions []Vec2, prevRotations []Rot, prevAwake []bool) {
	moveEvents := w.scratchMoveEvents[:0]
	for i := range w.bodies {
		body := w.bodies[i]
		if !body.alive || !body.isEnabled {
			continue
		}
		if body.bodyType == StaticBody {
			continue
		}
		if i >= len(prevPositions) || i >= len(prevRotations) || i >= len(prevAwake) {
			continue
		}

		moved := body.position != prevPositions[i] || body.rotation != prevRotations[i]
		fellAsleep := prevAwake[i] && !body.isAwake
		if !moved && !fellAsleep {
			continue
		}

		moveEvents = append(moveEvents, BodyMoveEvent{
			UserData:   body.userData,
			Transform:  Transform{P: body.position, Q: body.rotation},
			BodyID:     w.bodyIDFromIndexLocked(i),
			FellAsleep: fellAsleep,
		})
	}

	slices.SortFunc(moveEvents, func(a, b BodyMoveEvent) int {
		return cmp.Compare(a.BodyID.Index1, b.BodyID.Index1)
	})

	w.scratchMoveEvents = moveEvents
	w.bodyEvents = BodyEvents{
		MoveEvents: moveEvents,
		MoveCount:  len(moveEvents),
		Count:      len(moveEvents),
	}
}

func (w *World) refreshJointEventsLocked() {
	events := w.scratchJointEvents[:0]
	for jointIdx := range w.joints {
		joint := w.joints[jointIdx]
		if !joint.alive {
			continue
		}
		if joint.bodyAIndex < 0 || joint.bodyAIndex >= len(w.bodies) || joint.bodyBIndex < 0 || joint.bodyBIndex >= len(w.bodies) {
			continue
		}
		bodyA := w.bodies[joint.bodyAIndex]
		bodyB := w.bodies[joint.bodyBIndex]
		if !bodyA.alive || !bodyB.alive || !bodyA.isEnabled || !bodyB.isEnabled {
			continue
		}
		if !bodyA.isAwake && !bodyB.isAwake {
			continue
		}

		force, torque := estimateJointForceTorque(joint, bodyA, bodyB)
		if force >= joint.forceThreshold || torque >= joint.torqueThreshold {
			events = append(events, JointEvent{
				JointID: JointID{
					Index1:     int32(jointIdx + 1),
					World0:     w.id.Index1 - 1,
					Generation: joint.generation,
				},
				UserData: joint.userData,
			})
		}
	}

	slices.SortFunc(events, func(a, b JointEvent) int {
		return cmp.Compare(a.JointID.Index1, b.JointID.Index1)
	})

	w.scratchJointEvents = events
	w.jointEvents = JointEvents{
		Events: events,
		Count:  len(events),
	}
}

func estimateJointForceTorque(joint jointState, bodyA, bodyB bodyState) (float32, float32) {
	xfA := Transform{P: bodyA.position, Q: bodyA.rotation}
	xfB := Transform{P: bodyB.position, Q: bodyB.rotation}
	worldAnchorA := TransformPoint(xfA, joint.localFrameA.P)
	worldAnchorB := TransformPoint(xfB, joint.localFrameB.P)

	linearError := Length(Sub(worldAnchorB, worldAnchorA))
	if joint.jointType == DistanceJoint {
		linearError = AbsFloat(linearError - joint.distanceLength)
	}
	if joint.jointType == MotorJoint {
		linearError = Length(Sub(Sub(bodyB.linearVelocity, bodyA.linearVelocity), joint.motorLinearVelocity))
	}

	qa := MulRot(bodyA.rotation, joint.localFrameA.Q)
	qb := MulRot(bodyB.rotation, joint.localFrameB.Q)
	angularError := AbsFloat(RelativeAngle(qa, qb))
	if joint.jointType == RevoluteJoint {
		angularError = AbsFloat(angularError - joint.revoluteTargetAngle)
	}
	if joint.jointType == MotorJoint {
		angularError = AbsFloat((bodyB.angularVelocity - bodyA.angularVelocity) - joint.motorAngularVelocity)
	}

	linearHertz := joint.constraintHertz
	angularHertz := joint.constraintHertz
	if joint.jointType == MotorJoint {
		if joint.motorLinearHertz > 0 {
			linearHertz = joint.motorLinearHertz
		}
		if joint.motorAngularHertz > 0 {
			angularHertz = joint.motorAngularHertz
		}
	}
	if linearHertz < 0 {
		linearHertz = 0
	}
	if angularHertz < 0 {
		angularHertz = 0
	}
	force := linearError * linearHertz * linearHertz
	torque := angularError * angularHertz * angularHertz

	switch joint.jointType {
	case DistanceJoint:
		if joint.distanceMotorForce > force {
			force = joint.distanceMotorForce
		}
	case MotorJoint:
		if joint.motorMaxSpringForce > force {
			force = joint.motorMaxSpringForce
		}
		if joint.motorMaxSpringTorque > torque {
			torque = joint.motorMaxSpringTorque
		}
	case PrismaticJoint:
		if joint.prismaticMotorForce > force {
			force = joint.prismaticMotorForce
		}
	case RevoluteJoint:
		if joint.revoluteMotorTorque > torque {
			torque = joint.revoluteMotorTorque
		}
	case WheelJoint:
		if joint.wheelMotorTorque > torque {
			torque = joint.wheelMotorTorque
		}
	}
	return force, torque
}

func shapeAABBWithBody(shape *shapeState, body *bodyState) AABB {
	transform := Transform{P: body.position, Q: body.rotation}
	switch shape.shapeType {
	case CircleShape:
		return ComputeCircleAABB(&shape.circle, transform)
	case CapsuleShape:
		return ComputeCapsuleAABB(&shape.capsule, transform)
	case PolygonShape:
		return ComputePolygonAABB(&shape.polygon, transform)
	case SegmentShape, ChainSegmentShape:
		return ComputeSegmentAABB(&shape.segment, transform)
	default:
		return AABB{}
	}
}

func rayCastShapeWithBody(shape shapeState, body bodyState, input RayCastInput) CastOutput {
	transform := Transform{P: body.position, Q: body.rotation}
	localInput := RayCastInput{
		Origin:      InvTransformPoint(transform, input.Origin),
		Translation: InvRotateVector(transform.Q, input.Translation),
		MaxFraction: input.MaxFraction,
	}

	var output CastOutput
	switch shape.shapeType {
	case CircleShape:
		output = RayCastCircle(&shape.circle, &localInput)
	case PolygonShape:
		output = RayCastPolygon(&shape.polygon, &localInput)
	case SegmentShape:
		output = RayCastSegment(&shape.segment, &localInput, false)
	case ChainSegmentShape:
		output = RayCastSegment(&shape.segment, &localInput, true)
	case CapsuleShape:
		output = RayCastCapsule(&shape.capsule, &localInput)
	default:
		output = CastOutput{}
	}
	if output.Hit {
		output.Point = TransformPoint(transform, output.Point)
		output.Normal = RotateVector(transform.Q, output.Normal)
	}
	return output
}

func shapeWorldTransform(body *bodyState) Transform {
	return Transform{P: body.position, Q: body.rotation}
}

func shapeWorldCircle(shape *shapeState, body *bodyState) Circle {
	xf := shapeWorldTransform(body)
	return Circle{
		Center: TransformPoint(xf, shape.circle.Center),
		Radius: shape.circle.Radius,
	}
}

func circleCircleManifold(circleA, circleB Circle) Manifold {
	delta := Sub(circleB.Center, circleA.Center)
	distance := Length(delta)
	radiusSum := circleA.Radius + circleB.Radius

	if distance > radiusSum {
		return Manifold{}
	}

	normal := Vec2{X: 1, Y: 0}
	if distance > floatEpsilon {
		normal = MulSV(1.0/distance, delta)
	}

	pA := MulAdd(circleA.Center, circleA.Radius, normal)
	pB := MulAdd(circleB.Center, -circleB.Radius, normal)
	separation := distance - radiusSum
	return Manifold{
		Normal: normal,
		Point1: ManifoldPoint{
			Point:      MulSV(0.5, Add(pA, pB)),
			Separation: separation,
		},
		PointCount: 1,
	}
}

func worldSegment(shape *shapeState, body *bodyState) Segment {
	xf := shapeWorldTransform(body)
	return Segment{
		Point1: TransformPoint(xf, shape.segment.Point1),
		Point2: TransformPoint(xf, shape.segment.Point2),
	}
}

func worldCapsule(shape *shapeState, body *bodyState) Capsule {
	xf := shapeWorldTransform(body)
	return Capsule{
		Center1: TransformPoint(xf, shape.capsule.Center1),
		Center2: TransformPoint(xf, shape.capsule.Center2),
		Radius:  shape.capsule.Radius,
	}
}

func worldPolygon(shape *shapeState, body *bodyState) Polygon {
	xf := shapeWorldTransform(body)
	out := shape.polygon
	out.Centroid = TransformPoint(xf, out.Centroid)
	for i := 0; i < out.Count; i++ {
		out.Vertices[i] = TransformPoint(xf, out.Vertices[i])
		out.Normals[i] = RotateVector(xf.Q, out.Normals[i])
	}
	return out
}

func shapeRepresentativePoint(shape *shapeState, body *bodyState) Vec2 {
	switch shape.shapeType {
	case CircleShape:
		circle := shapeWorldCircle(shape, body)
		return circle.Center
	case CapsuleShape:
		capsule := worldCapsule(shape, body)
		return MulSV(0.5, Add(capsule.Center1, capsule.Center2))
	case SegmentShape, ChainSegmentShape:
		segment := worldSegment(shape, body)
		return MulSV(0.5, Add(segment.Point1, segment.Point2))
	case PolygonShape:
		polygon := worldPolygon(shape, body)
		return polygon.Centroid
	default:
		return body.position
	}
}

func chainSegmentAllowsContact(chainShape *shapeState, chainBody *bodyState, otherShape *shapeState, otherBody *bodyState) bool {
	segment := worldSegment(chainShape, chainBody)
	edge := Sub(segment.Point2, segment.Point1)
	edgeLen2 := LengthSquared(edge)
	if edgeLen2 <= floatEpsilon {
		return true
	}
	target := shapeRepresentativePoint(otherShape, otherBody)
	side := Cross(edge, Sub(target, segment.Point1))
	if side > 0 {
		return false
	}

	xf := shapeWorldTransform(chainBody)
	ghost1 := TransformPoint(xf, chainShape.chainGhost1)
	ghost2 := TransformPoint(xf, chainShape.chainGhost2)
	edgeFraction := Dot(Sub(target, segment.Point1), edge) / edgeLen2

	if edgeFraction < 0 {
		prevEdge := Sub(segment.Point1, ghost1)
		if LengthSquared(prevEdge) > floatEpsilon {
			prevSide := Cross(prevEdge, Sub(target, ghost1))
			if prevSide > 0 {
				return false
			}
		}
	} else if edgeFraction > 1 {
		nextEdge := Sub(ghost2, segment.Point2)
		if LengthSquared(nextEdge) > floatEpsilon {
			nextSide := Cross(nextEdge, Sub(target, segment.Point2))
			if nextSide > 0 {
				return false
			}
		}
	}

	return true
}

func circleSegmentLikeManifold(circle Circle, point1, point2 Vec2, otherRadius float32) Manifold {
	closest := closestPointOnSegment(circle.Center, point1, point2)
	delta := Sub(closest, circle.Center)
	distance := Length(delta)
	radiusSum := circle.Radius + otherRadius
	if distance > radiusSum {
		return Manifold{}
	}

	normal := Vec2{X: 1, Y: 0}
	if distance > floatEpsilon {
		normal = MulSV(1.0/distance, delta)
	}

	pA := MulAdd(circle.Center, circle.Radius, normal)
	pB := MulAdd(closest, -otherRadius, normal)
	return Manifold{
		Normal: normal,
		Point1: ManifoldPoint{
			Point:      MulSV(0.5, Add(pA, pB)),
			Separation: distance - radiusSum,
		},
		PointCount: 1,
	}
}

func flipManifold(manifold Manifold) Manifold {
	manifold.Normal = Neg(manifold.Normal)
	return manifold
}

func assignManifoldPointMetadata(previous, current Manifold, nextPointID *uint16) Manifold {
	if nextPointID == nil {
		return current
	}

	current.Point1.ID = 0
	current.Point1.Persisted = false
	current.Point2.ID = 0
	current.Point2.Persisted = false

	nextID := *nextPointID
	if nextID == 0 {
		nextID = 1
	}
	allocateNew := func(point *ManifoldPoint) {
		point.ID = nextID
		point.Persisted = false
		nextID++
	}
	assignMatched := func(point *ManifoldPoint, previousPoint ManifoldPoint) {
		point.ID = previousPoint.ID
		point.Persisted = true
	}

	switch current.PointCount {
	case 0:
		*nextPointID = nextID
		return current
	case 1:
		if previous.PointCount == 0 {
			allocateNew(&current.Point1)
			break
		}

		bestPrevious := previous.Point1
		bestDistance := DistanceSquared(current.Point1.Point, previous.Point1.Point)
		if previous.PointCount > 1 {
			distance := DistanceSquared(current.Point1.Point, previous.Point2.Point)
			if distance < bestDistance {
				bestPrevious = previous.Point2
				bestDistance = distance
			}
		}
		assignMatched(&current.Point1, bestPrevious)
	case 2:
		if previous.PointCount == 0 {
			allocateNew(&current.Point1)
			allocateNew(&current.Point2)
			break
		}

		if previous.PointCount == 1 {
			d0 := DistanceSquared(current.Point1.Point, previous.Point1.Point)
			d1 := DistanceSquared(current.Point2.Point, previous.Point1.Point)
			if d1 < d0 {
				assignMatched(&current.Point2, previous.Point1)
				allocateNew(&current.Point1)
			} else {
				assignMatched(&current.Point1, previous.Point1)
				allocateNew(&current.Point2)
			}
			break
		}

		sameOrder := DistanceSquared(current.Point1.Point, previous.Point1.Point) + DistanceSquared(current.Point2.Point, previous.Point2.Point)
		swappedOrder := DistanceSquared(current.Point1.Point, previous.Point2.Point) + DistanceSquared(current.Point2.Point, previous.Point1.Point)
		if swappedOrder < sameOrder {
			assignMatched(&current.Point1, previous.Point2)
			assignMatched(&current.Point2, previous.Point1)
		} else {
			assignMatched(&current.Point1, previous.Point1)
			assignMatched(&current.Point2, previous.Point2)
		}
	default:
		for i := 0; i < current.PointCount; i++ {
			switch i {
			case 0:
				allocateNew(&current.Point1)
			case 1:
				allocateNew(&current.Point2)
			}
		}
	}

	*nextPointID = nextID
	return current
}

func circlePolygonManifold(circle Circle, polygon Polygon) Manifold {
	combinedRadius := circle.Radius + polygon.Radius
	center := circle.Center

	maxSep := -float32(1e30)
	bestIndex := 0
	for i := 0; i < polygon.Count; i++ {
		sep := Dot(polygon.Normals[i], Sub(center, polygon.Vertices[i]))
		if sep > maxSep {
			maxSep = sep
			bestIndex = i
		}
	}
	if maxSep > combinedRadius {
		return Manifold{}
	}

	var closest Vec2
	var normal Vec2
	if PointInPolygon(&polygon, center) {
		normal = polygon.Normals[bestIndex]
		closest = MulAdd(center, -maxSep, normal)
	} else {
		closest = closestPointOnPolygon(polygon, center)
		delta := Sub(closest, center)
		dist := Length(delta)
		if dist > combinedRadius {
			return Manifold{}
		}
		normal = Vec2{X: 1, Y: 0}
		if dist > floatEpsilon {
			normal = MulSV(1.0/dist, delta)
		} else {
			normal = polygon.Normals[bestIndex]
		}
	}

	distance := Dot(Sub(closest, center), normal)
	pA := MulAdd(center, circle.Radius, normal)
	pB := MulAdd(closest, -polygon.Radius, normal)
	return Manifold{
		Normal: normal,
		Point1: ManifoldPoint{
			Point:      MulSV(0.5, Add(pA, pB)),
			Separation: distance - combinedRadius,
		},
		PointCount: 1,
	}
}

func projectPolygonOnAxis(polygon Polygon, axis Vec2) (float32, float32) {
	proj := Dot(axis, polygon.Vertices[0])
	minProj := proj
	maxProj := proj
	for i := 1; i < polygon.Count; i++ {
		proj = Dot(axis, polygon.Vertices[i])
		if proj < minProj {
			minProj = proj
		}
		if proj > maxProj {
			maxProj = proj
		}
	}
	return minProj - polygon.Radius, maxProj + polygon.Radius
}

func polygonBestNormalAtPoint(polygon Polygon, point Vec2) (Vec2, float32) {
	maxSep := -float32(1e30)
	bestIndex := 0
	for i := 0; i < polygon.Count; i++ {
		sep := Dot(polygon.Normals[i], Sub(point, polygon.Vertices[i]))
		if sep > maxSep {
			maxSep = sep
			bestIndex = i
		}
	}
	return polygon.Normals[bestIndex], maxSep
}

func polygonPolygonManifold(polygonA, polygonB Polygon) Manifold {
	bestOverlap := float32(1e30)
	bestNormal := Vec2{X: 1, Y: 0}

	checkAxes := func(poly Polygon) bool {
		for i := 0; i < poly.Count; i++ {
			axis := poly.Normals[i]
			minA, maxA := projectPolygonOnAxis(polygonA, axis)
			minB, maxB := projectPolygonOnAxis(polygonB, axis)
			overlap := MinFloat(maxA, maxB) - MaxFloat(minA, minB)
			if overlap < 0 {
				return false
			}
			if overlap < bestOverlap {
				bestOverlap = overlap
				bestNormal = axis
			}
		}
		return true
	}

	if !checkAxes(polygonA) || !checkAxes(polygonB) {
		return Manifold{}
	}

	centerDelta := Sub(polygonB.Centroid, polygonA.Centroid)
	if Dot(centerDelta, bestNormal) < 0 {
		bestNormal = Neg(bestNormal)
	}

	closestA := closestPointOnPolygon(polygonA, polygonB.Centroid)
	closestB := closestPointOnPolygon(polygonB, closestA)
	distance := Dot(Sub(closestB, closestA), bestNormal)
	pA := MulAdd(closestA, polygonA.Radius, bestNormal)
	pB := MulAdd(closestB, -polygonB.Radius, bestNormal)
	return Manifold{
		Normal: bestNormal,
		Point1: ManifoldPoint{
			Point:      MulSV(0.5, Add(pA, pB)),
			Separation: MinFloat(distance-(polygonA.Radius+polygonB.Radius), -bestOverlap),
		},
		PointCount: 1,
	}
}

func closestPointsBetweenSegments(a1, a2, b1, b2 Vec2) (Vec2, Vec2) {
	u := Sub(a2, a1)
	v := Sub(b2, b1)
	w0 := Sub(a1, b1)

	a := Dot(u, u)
	b := Dot(u, v)
	c := Dot(v, v)
	d := Dot(u, w0)
	e := Dot(v, w0)

	const eps = float32(1e-6)
	den := a*c - b*b

	var sN, sD float32 = den, den
	var tN, tD float32 = den, den

	if den < eps {
		sN = 0
		sD = 1
		tN = e
		tD = c
	} else {
		sN = b*e - c*d
		tN = a*e - b*d
		if sN < 0 {
			sN = 0
			tN = e
			tD = c
		} else if sN > sD {
			sN = sD
			tN = e + b
			tD = c
		}
	}

	if tN < 0 {
		tN = 0
		if -d < 0 {
			sN = 0
		} else if -d > a {
			sN = sD
		} else {
			sN = -d
			sD = a
		}
	} else if tN > tD {
		tN = tD
		if -d+b < 0 {
			sN = 0
		} else if -d+b > a {
			sN = sD
		} else {
			sN = -d + b
			sD = a
		}
	}

	sc := float32(0)
	if AbsFloat(sN) > eps {
		sc = sN / sD
	}
	tc := float32(0)
	if AbsFloat(tN) > eps {
		tc = tN / tD
	}

	pA := MulAdd(a1, sc, u)
	pB := MulAdd(b1, tc, v)
	return pA, pB
}

func segmentSegmentLikeManifold(a1, a2 Vec2, radiusA float32, b1, b2 Vec2, radiusB float32) Manifold {
	closestA, closestB := closestPointsBetweenSegments(a1, a2, b1, b2)
	delta := Sub(closestB, closestA)
	distance := Length(delta)
	radiusSum := radiusA + radiusB
	if distance > radiusSum {
		return Manifold{}
	}

	normal := Vec2{X: 1, Y: 0}
	if distance > floatEpsilon {
		normal = MulSV(1.0/distance, delta)
	} else {
		edge := Sub(a2, a1)
		edgeLen2 := LengthSquared(edge)
		if edgeLen2 > floatEpsilon {
			tangent := MulSV(1.0/float32(math.Sqrt(float64(edgeLen2))), edge)
			normal = Vec2{X: -tangent.Y, Y: tangent.X}
			if Dot(normal, Sub(b1, a1)) < 0 {
				normal = Neg(normal)
			}
		}
	}

	edgeA := Sub(a2, a1)
	edgeB := Sub(b2, b1)
	lenA := Length(edgeA)
	lenB := Length(edgeB)
	if lenA > floatEpsilon && lenB > floatEpsilon {
		dirA := MulSV(1.0/lenA, edgeA)
		dirB := MulSV(1.0/lenB, edgeB)
		parallel := AbsFloat(Cross(dirA, dirB)) < 0.05
		if parallel {
			bStart := Dot(Sub(b1, a1), dirA)
			bEnd := Dot(Sub(b2, a1), dirA)
			if bStart > bEnd {
				bStart, bEnd = bEnd, bStart
			}
			overlapStart := MaxFloat(0, bStart)
			overlapEnd := MinFloat(lenA, bEnd)
			if overlapEnd-overlapStart > 1e-3 {
				aPoint1 := MulAdd(a1, overlapStart, dirA)
				aPoint2 := MulAdd(a1, overlapEnd, dirA)
				bPoint1 := closestPointOnSegment(aPoint1, b1, b2)
				bPoint2 := closestPointOnSegment(aPoint2, b1, b2)
				pA1 := MulAdd(aPoint1, radiusA, normal)
				pB1 := MulAdd(bPoint1, -radiusB, normal)
				pA2 := MulAdd(aPoint2, radiusA, normal)
				pB2 := MulAdd(bPoint2, -radiusB, normal)
				sep1 := Dot(Sub(bPoint1, aPoint1), normal) - radiusSum
				sep2 := Dot(Sub(bPoint2, aPoint2), normal) - radiusSum
				contact1 := MulSV(0.5, Add(pA1, pB1))
				contact2 := MulSV(0.5, Add(pA2, pB2))
				if DistanceSquared(contact1, contact2) > 1e-6 {
					return Manifold{
						Normal: normal,
						Point1: ManifoldPoint{
							Point:      contact1,
							Separation: sep1,
						},
						Point2: ManifoldPoint{
							Point:      contact2,
							Separation: sep2,
						},
						PointCount: 2,
					}
				}
			}
		}
	}

	pA := MulAdd(closestA, radiusA, normal)
	pB := MulAdd(closestB, -radiusB, normal)
	return Manifold{
		Normal: normal,
		Point1: ManifoldPoint{
			Point:      MulSV(0.5, Add(pA, pB)),
			Separation: distance - radiusSum,
		},
		PointCount: 1,
	}
}

func capsulePolygonManifold(capsule Capsule, polygon Polygon) Manifold {
	bestSeg := capsule.Center1
	bestPoly := closestPointOnPolygon(polygon, capsule.Center1)
	bestDist2 := DistanceSquared(bestSeg, bestPoly)

	checkCandidate := func(segPoint, polyPoint Vec2) {
		d2 := DistanceSquared(segPoint, polyPoint)
		if d2 < bestDist2 {
			bestDist2 = d2
			bestSeg = segPoint
			bestPoly = polyPoint
		}
	}

	checkCandidate(capsule.Center2, closestPointOnPolygon(polygon, capsule.Center2))
	mid := MulSV(0.5, Add(capsule.Center1, capsule.Center2))
	checkCandidate(mid, closestPointOnPolygon(polygon, mid))

	for i := 0; i < polygon.Count; i++ {
		j := i + 1
		if j == polygon.Count {
			j = 0
		}
		edgeA := polygon.Vertices[i]
		edgeB := polygon.Vertices[j]
		segPoint, polyPoint := closestPointsBetweenSegments(capsule.Center1, capsule.Center2, edgeA, edgeB)
		checkCandidate(segPoint, polyPoint)
	}

	delta := Sub(bestPoly, bestSeg)
	distance := Length(delta)
	radiusSum := capsule.Radius + polygon.Radius
	if distance > radiusSum {
		return Manifold{}
	}

	normal := Vec2{X: 1, Y: 0}
	if distance > floatEpsilon {
		normal = MulSV(1.0/distance, delta)
	} else {
		normal, _ = polygonBestNormalAtPoint(polygon, mid)
	}

	pA := MulAdd(bestSeg, capsule.Radius, normal)
	pB := MulAdd(bestPoly, -polygon.Radius, normal)
	return Manifold{
		Normal: normal,
		Point1: ManifoldPoint{
			Point:      MulSV(0.5, Add(pA, pB)),
			Separation: distance - radiusSum,
		},
		PointCount: 1,
	}
}

// manifoldFunc computes a contact manifold for a shape pair.
type manifoldFunc func(shapeA *shapeState, bodyA *bodyState, shapeB *shapeState, bodyB *bodyState) Manifold

const shapeTypeCount = 5 // CircleShape(0) through ChainSegmentShape(4)

// manifoldDispatch is a [shapeTypeA][shapeTypeB] lookup table for manifold computation.
// nil entries mean no manifold support for that pair.
var manifoldDispatch [shapeTypeCount][shapeTypeCount]manifoldFunc

func init() {
	// Circle vs *
	manifoldDispatch[CircleShape][CircleShape] = func(sA *shapeState, bA *bodyState, sB *shapeState, bB *bodyState) Manifold {
		return circleCircleManifold(shapeWorldCircle(sA, bA), shapeWorldCircle(sB, bB))
	}
	manifoldDispatch[CircleShape][CapsuleShape] = func(sA *shapeState, bA *bodyState, sB *shapeState, bB *bodyState) Manifold {
		c := shapeWorldCircle(sA, bA)
		cap := worldCapsule(sB, bB)
		return circleSegmentLikeManifold(c, cap.Center1, cap.Center2, cap.Radius)
	}
	manifoldDispatch[CapsuleShape][CircleShape] = func(sA *shapeState, bA *bodyState, sB *shapeState, bB *bodyState) Manifold {
		c := shapeWorldCircle(sB, bB)
		cap := worldCapsule(sA, bA)
		return flipManifold(circleSegmentLikeManifold(c, cap.Center1, cap.Center2, cap.Radius))
	}
	circleSegment := func(sA *shapeState, bA *bodyState, sB *shapeState, bB *bodyState) Manifold {
		c := shapeWorldCircle(sA, bA)
		seg := worldSegment(sB, bB)
		return circleSegmentLikeManifold(c, seg.Point1, seg.Point2, 0)
	}
	segmentCircle := func(sA *shapeState, bA *bodyState, sB *shapeState, bB *bodyState) Manifold {
		c := shapeWorldCircle(sB, bB)
		seg := worldSegment(sA, bA)
		return flipManifold(circleSegmentLikeManifold(c, seg.Point1, seg.Point2, 0))
	}
	manifoldDispatch[CircleShape][SegmentShape] = circleSegment
	manifoldDispatch[CircleShape][ChainSegmentShape] = circleSegment
	manifoldDispatch[SegmentShape][CircleShape] = segmentCircle
	manifoldDispatch[ChainSegmentShape][CircleShape] = segmentCircle
	manifoldDispatch[CircleShape][PolygonShape] = func(sA *shapeState, bA *bodyState, sB *shapeState, bB *bodyState) Manifold {
		return circlePolygonManifold(shapeWorldCircle(sA, bA), worldPolygon(sB, bB))
	}
	manifoldDispatch[PolygonShape][CircleShape] = func(sA *shapeState, bA *bodyState, sB *shapeState, bB *bodyState) Manifold {
		return flipManifold(circlePolygonManifold(shapeWorldCircle(sB, bB), worldPolygon(sA, bA)))
	}

	// Capsule vs *
	manifoldDispatch[CapsuleShape][CapsuleShape] = func(sA *shapeState, bA *bodyState, sB *shapeState, bB *bodyState) Manifold {
		cA := worldCapsule(sA, bA)
		cB := worldCapsule(sB, bB)
		return segmentSegmentLikeManifold(cA.Center1, cA.Center2, cA.Radius, cB.Center1, cB.Center2, cB.Radius)
	}
	capsuleSegment := func(sA *shapeState, bA *bodyState, sB *shapeState, bB *bodyState) Manifold {
		cA := worldCapsule(sA, bA)
		sB2 := worldSegment(sB, bB)
		return segmentSegmentLikeManifold(cA.Center1, cA.Center2, cA.Radius, sB2.Point1, sB2.Point2, 0)
	}
	segmentCapsule := func(sA *shapeState, bA *bodyState, sB *shapeState, bB *bodyState) Manifold {
		sA2 := worldSegment(sA, bA)
		cB := worldCapsule(sB, bB)
		return segmentSegmentLikeManifold(sA2.Point1, sA2.Point2, 0, cB.Center1, cB.Center2, cB.Radius)
	}
	manifoldDispatch[CapsuleShape][SegmentShape] = capsuleSegment
	manifoldDispatch[CapsuleShape][ChainSegmentShape] = capsuleSegment
	manifoldDispatch[SegmentShape][CapsuleShape] = segmentCapsule
	manifoldDispatch[ChainSegmentShape][CapsuleShape] = segmentCapsule
	manifoldDispatch[CapsuleShape][PolygonShape] = func(sA *shapeState, bA *bodyState, sB *shapeState, bB *bodyState) Manifold {
		return capsulePolygonManifold(worldCapsule(sA, bA), worldPolygon(sB, bB))
	}
	manifoldDispatch[PolygonShape][CapsuleShape] = func(sA *shapeState, bA *bodyState, sB *shapeState, bB *bodyState) Manifold {
		return flipManifold(capsulePolygonManifold(worldCapsule(sB, bB), worldPolygon(sA, bA)))
	}

	// Polygon vs Polygon
	manifoldDispatch[PolygonShape][PolygonShape] = func(sA *shapeState, bA *bodyState, sB *shapeState, bB *bodyState) Manifold {
		return polygonPolygonManifold(worldPolygon(sA, bA), worldPolygon(sB, bB))
	}

	// Segment vs Polygon
	segmentPolygon := func(sA *shapeState, bA *bodyState, sB *shapeState, bB *bodyState) Manifold {
		seg := worldSegment(sA, bA)
		return capsulePolygonManifold(Capsule{Center1: seg.Point1, Center2: seg.Point2, Radius: 0}, worldPolygon(sB, bB))
	}
	polygonSegment := func(sA *shapeState, bA *bodyState, sB *shapeState, bB *bodyState) Manifold {
		seg := worldSegment(sB, bB)
		return flipManifold(capsulePolygonManifold(Capsule{Center1: seg.Point1, Center2: seg.Point2, Radius: 0}, worldPolygon(sA, bA)))
	}
	manifoldDispatch[SegmentShape][PolygonShape] = segmentPolygon
	manifoldDispatch[ChainSegmentShape][PolygonShape] = segmentPolygon
	manifoldDispatch[PolygonShape][SegmentShape] = polygonSegment
	manifoldDispatch[PolygonShape][ChainSegmentShape] = polygonSegment

	// Segment vs Segment
	segmentSegment := func(sA *shapeState, bA *bodyState, sB *shapeState, bB *bodyState) Manifold {
		sA2 := worldSegment(sA, bA)
		sB2 := worldSegment(sB, bB)
		return segmentSegmentLikeManifold(sA2.Point1, sA2.Point2, 0, sB2.Point1, sB2.Point2, 0)
	}
	manifoldDispatch[SegmentShape][SegmentShape] = segmentSegment
	manifoldDispatch[SegmentShape][ChainSegmentShape] = segmentSegment
	manifoldDispatch[ChainSegmentShape][SegmentShape] = segmentSegment
	manifoldDispatch[ChainSegmentShape][ChainSegmentShape] = segmentSegment
}

func hasManifoldSupport(shapeAType, shapeBType ShapeType) bool {
	return manifoldDispatch[shapeAType][shapeBType] != nil
}

func contactManifoldFromPair(shapeA *shapeState, bodyA *bodyState, shapeB *shapeState, bodyB *bodyState) Manifold {
	if shapeA.shapeType == ChainSegmentShape && !chainSegmentAllowsContact(shapeA, bodyA, shapeB, bodyB) {
		return Manifold{}
	}
	if shapeB.shapeType == ChainSegmentShape && !chainSegmentAllowsContact(shapeB, bodyB, shapeA, bodyA) {
		return Manifold{}
	}
	fn := manifoldDispatch[shapeA.shapeType][shapeB.shapeType]
	if fn == nil {
		return Manifold{}
	}
	return fn(shapeA, bodyA, shapeB, bodyB)
}

func shapesTouchingForContact(shapeA *shapeState, bodyA *bodyState, shapeB *shapeState, bodyB *bodyState) bool {
	if !hasManifoldSupport(shapeA.shapeType, shapeB.shapeType) {
		return true
	}
	manifold := contactManifoldFromPair(shapeA, bodyA, shapeB, bodyB)
	return manifold.PointCount > 0
}

type shapePair struct {
	a int
	b int
}

type touchingPair struct {
	pair     shapePair
	manifold Manifold
}

type broadPhaseShape struct {
	idx       int
	bodyIndex int
	aabb      AABB
}

func (w *World) shapeIDFromIndexLocked(shapeIdx int) ShapeID {
	return ShapeID{
		Index1:     int32(shapeIdx + 1),
		World0:     w.id.Index1 - 1,
		Generation: w.shapes[shapeIdx].generation,
	}
}

func (w *World) bodyIDFromIndexLocked(bodyIdx int) BodyID {
	return BodyID{
		Index1:     int32(bodyIdx + 1),
		World0:     w.id.Index1 - 1,
		Generation: w.bodies[bodyIdx].generation,
	}
}

func shapesPassCollisionFilter(shapeA, shapeB shapeState) bool {
	filterA := shapeA.filter
	filterB := shapeB.filter
	if filterA.GroupIndex == filterB.GroupIndex && filterA.GroupIndex != 0 {
		return filterA.GroupIndex > 0
	}
	if (filterA.MaskBits & filterB.CategoryBits) == 0 {
		return false
	}
	if (filterB.MaskBits & filterA.CategoryBits) == 0 {
		return false
	}
	return true
}

func (w *World) shouldBodiesCollideLocked(bodyAIdx, bodyBIdx int) bool {
	for i := range w.joints {
		joint := w.joints[i]
		if !joint.alive {
			continue
		}
		if joint.collideConnected {
			continue
		}
		matchAB := joint.bodyAIndex == bodyAIdx && joint.bodyBIndex == bodyBIdx
		matchBA := joint.bodyAIndex == bodyBIdx && joint.bodyBIndex == bodyAIdx
		if matchAB || matchBA {
			return false
		}
	}
	return true
}

func (w *World) shouldShapesCollideLocked(shapeA shapeState, shapeAIdx int, shapeB shapeState, shapeBIdx int) bool {
	if !shapesPassCollisionFilter(shapeA, shapeB) {
		return false
	}
	if !w.shouldBodiesCollideLocked(shapeA.bodyIndex, shapeB.bodyIndex) {
		return false
	}
	if w.customFilterCallback != nil && (shapeA.enableCustomFiltering || shapeB.enableCustomFiltering) {
		shapeIDA := w.shapeIDFromIndexLocked(shapeAIdx)
		shapeIDB := w.shapeIDFromIndexLocked(shapeBIdx)
		if !w.customFilterCallback(shapeIDA, shapeIDB) {
			return false
		}
	}
	return true
}

func (w *World) collectTouchingContactPairsLocked() []shapePair {
	candidates := w.collectBroadPhaseCandidatesLocked()
	return w.collectTouchingContactPairsFromCandidatesLocked(candidates)
}

func (w *World) collectTouchingContactPairsFromCandidatesLocked(candidates []shapePair) []shapePair {
	touching := w.collectTouchingContactPairsWithManifoldsLocked(candidates)
	pairs := w.scratchTouchPairs[:0]
	for _, tp := range touching {
		pairs = append(pairs, tp.pair)
	}
	w.scratchTouchPairs = pairs
	return pairs
}

func (w *World) collectTouchingContactPairsWithManifoldsLocked(candidates []shapePair) []touchingPair {
	result := w.scratchTouchingPairs[:0]
	for _, pair := range candidates {
		i := pair.a
		j := pair.b
		shapeA := &w.shapes[i]
		if !shapeA.alive || shapeA.isSensor {
			continue
		}
		bodyA := &w.bodies[shapeA.bodyIndex]
		if !bodyA.alive || !bodyA.isEnabled {
			continue
		}
		shapeB := &w.shapes[j]
		if !shapeB.alive || shapeB.isSensor {
			continue
		}
		bodyB := &w.bodies[shapeB.bodyIndex]
		if !bodyB.alive || !bodyB.isEnabled {
			continue
		}
		if bodyA.bodyType != DynamicBody && bodyB.bodyType != DynamicBody {
			continue
		}
		if !w.shouldShapesCollideLocked(*shapeA, i, *shapeB, j) {
			continue
		}
		if !hasManifoldSupport(shapeA.shapeType, shapeB.shapeType) {
			// No manifold support means always touching
			if w.preSolveCallback != nil && (shapeA.enablePreSolveEvents || shapeB.enablePreSolveEvents) {
				shapeIDA := w.shapeIDFromIndexLocked(i)
				shapeIDB := w.shapeIDFromIndexLocked(j)
				if !w.preSolveCallback(shapeIDA, shapeIDB) {
					continue
				}
			}
			result = append(result, touchingPair{pair: shapePair{a: i, b: j}, manifold: Manifold{}})
			continue
		}
		manifold := contactManifoldFromPair(shapeA, bodyA, shapeB, bodyB)
		if manifold.PointCount == 0 {
			continue
		}
		if w.preSolveCallback != nil && (shapeA.enablePreSolveEvents || shapeB.enablePreSolveEvents) {
			shapeIDA := w.shapeIDFromIndexLocked(i)
			shapeIDB := w.shapeIDFromIndexLocked(j)
			if !w.preSolveCallback(shapeIDA, shapeIDB) {
				continue
			}
		}
		result = append(result, touchingPair{pair: shapePair{a: i, b: j}, manifold: manifold})
	}
	w.scratchTouchingPairs = result
	return result
}

func (w *World) collectTouchingSensorPairsLocked() []shapePair {
	candidates := w.collectBroadPhaseCandidatesLocked()
	return w.collectTouchingSensorPairsFromCandidatesLocked(candidates)
}

func (w *World) collectTouchingSensorPairsFromCandidatesLocked(candidates []shapePair) []shapePair {
	pairs := w.scratchSensorPairs[:0]
	for _, pair := range candidates {
		i := pair.a
		j := pair.b
		shapeA := &w.shapes[i]
		if !shapeA.alive {
			continue
		}
		bodyA := &w.bodies[shapeA.bodyIndex]
		if !bodyA.alive || !bodyA.isEnabled {
			continue
		}
		shapeB := &w.shapes[j]
		if !shapeB.alive {
			continue
		}
		bodyB := &w.bodies[shapeB.bodyIndex]
		if !bodyB.alive || !bodyB.isEnabled {
			continue
		}
		if !shapeA.isSensor && !shapeB.isSensor {
			continue
		}
		if !w.shouldShapesCollideLocked(*shapeA, i, *shapeB, j) {
			continue
		}
		if !shapesTouchingForContact(shapeA, bodyA, shapeB, bodyB) {
			continue
		}
		pairs = append(pairs, shapePair{a: i, b: j})
	}
	w.scratchSensorPairs = pairs
	return pairs
}

func (w *World) collectBroadPhaseCandidatesLocked() []shapePair {
	shapes := w.scratchBPShapes[:0]
	for i := range w.shapes {
		shape := &w.shapes[i]
		if !shape.alive {
			continue
		}
		bodyIndex := shape.bodyIndex
		if bodyIndex < 0 || bodyIndex >= len(w.bodies) {
			continue
		}
		body := &w.bodies[bodyIndex]
		if !body.alive || !body.isEnabled {
			continue
		}
		shapes = append(shapes, broadPhaseShape{
			idx:       i,
			bodyIndex: bodyIndex,
			aabb:      shapeAABBWithBody(shape, body),
		})
	}

	// Insertion sort: O(n) for nearly-sorted data (shapes move slightly between frames)
	for i := 1; i < len(shapes); i++ {
		key := shapes[i]
		j := i - 1
		for j >= 0 && (shapes[j].aabb.LowerBound.X > key.aabb.LowerBound.X ||
			(shapes[j].aabb.LowerBound.X == key.aabb.LowerBound.X && shapes[j].idx > key.idx)) {
			shapes[j+1] = shapes[j]
			j--
		}
		shapes[j+1] = key
	}

	pairs := w.scratchBPPairs[:0]
	active := w.scratchBPActive[:0]
	for i := range shapes {
		current := shapes[i]
		nextActive := active[:0]
		for _, ai := range active {
			if shapes[ai].aabb.UpperBound.X >= current.aabb.LowerBound.X {
				nextActive = append(nextActive, ai)
			}
		}
		active = nextActive

		for _, ai := range active {
			other := shapes[ai]
			if other.bodyIndex == current.bodyIndex {
				continue
			}
			if !AABBOverlaps(other.aabb, current.aabb) {
				continue
			}
			a := other.idx
			b := current.idx
			if a > b {
				a, b = b, a
			}
			pairs = append(pairs, shapePair{a: a, b: b})
		}
		active = append(active, i)
	}
	w.scratchBPShapes = shapes
	w.scratchBPPairs = pairs
	w.scratchBPActive = active
	return pairs
}

func (w *World) activeContactFromPairLocked(pair shapePair) activeContact {
	if idx, ok := w.findActiveContact(pair); ok {
		return w.activeContactData[idx]
	}
	w.nextContactIndex++
	entry := activeContact{
		contactID: ContactID{
			Index1:     w.nextContactIndex,
			World0:     w.id.Index1 - 1,
			Padding:    0,
			Generation: 1,
		},
		shapeIDA: w.shapeIDFromIndexLocked(pair.a),
		shapeIDB: w.shapeIDFromIndexLocked(pair.b),
		manifold: Manifold{},
	}
	w.insertActiveContact(pair, entry)
	return entry
}

func (w *World) newActiveContactEntry(pair shapePair) activeContact {
	w.nextContactIndex++
	return activeContact{
		contactID: ContactID{
			Index1:     w.nextContactIndex,
			World0:     w.id.Index1 - 1,
			Padding:    0,
			Generation: 1,
		},
		shapeIDA: w.shapeIDFromIndexLocked(pair.a),
		shapeIDB: w.shapeIDFromIndexLocked(pair.b),
		manifold: Manifold{},
	}
}

func (w *World) refreshContactAndSensorStateLocked() {
	// activeContactPairs/activeContactData and activeSensorKeys/activeSensorData are nil-safe (slices)

	candidates := w.collectBroadPhaseCandidatesLocked()
	w.broadPhaseCandidates = len(candidates)
	touching := w.collectTouchingContactPairsWithManifoldsLocked(candidates)

	// Sort touching pairs by shapePair for merge with activeContactPairs (already sorted)
	slices.SortFunc(touching, func(a, b touchingPair) int {
		return compareShapePairs(a.pair, b.pair)
	})

	// Sorted merge: walk activeContactPairs and touching in parallel to detect
	// begin/end/continue contacts without using a map
	beginContacts := w.scratchBeginContacts[:0]
	endContacts := w.scratchEndContacts[:0]
	needed := max(len(touching), len(w.activeContactPairs))
	if cap(w.scratchMergedPairs) < needed {
		w.scratchMergedPairs = make([]shapePair, 0, needed)
		w.scratchMergedData = make([]activeContact, 0, needed)
	}
	newPairs := w.scratchMergedPairs[:0]
	newData := w.scratchMergedData[:0]

	ai, ti := 0, 0
	for ai < len(w.activeContactPairs) && ti < len(touching) {
		ap := w.activeContactPairs[ai]
		tp := touching[ti].pair
		c := compareShapePairs(ap, tp)
		if c < 0 {
			// active contact no longer touching → end event
			entry := w.activeContactData[ai]
			endContacts = append(endContacts, ContactEndTouchEvent{
				ShapeIDA:  entry.shapeIDA,
				ShapeIDB:  entry.shapeIDB,
				ContactID: entry.contactID,
			})
			ai++
		} else if c > 0 {
			// new touching pair → begin event
			pair := tp
			shapeA := &w.shapes[pair.a]
			shapeB := &w.shapes[pair.b]
			friction, restitution := w.effectiveContactMaterialLocked(
				w.shapeIDFromIndexLocked(pair.a), w.shapeIDFromIndexLocked(pair.b), *shapeA, *shapeB)
			entry := w.newActiveContactEntry(pair)
			entry.manifold = assignManifoldPointMetadata(Manifold{}, touching[ti].manifold, &entry.nextManifoldPointID)
			entry.friction = friction
			entry.restitution = restitution
			newPairs = append(newPairs, pair)
			newData = append(newData, entry)
			beginContacts = append(beginContacts, ContactBeginTouchEvent{
				ShapeIDA:  entry.shapeIDA,
				ShapeIDB:  entry.shapeIDB,
				ContactID: entry.contactID,
			})
			ti++
		} else {
			// existing contact still touching → update manifold
			pair := ap
			shapeA := &w.shapes[pair.a]
			shapeB := &w.shapes[pair.b]
			friction, restitution := w.effectiveContactMaterialLocked(
				w.shapeIDFromIndexLocked(pair.a), w.shapeIDFromIndexLocked(pair.b), *shapeA, *shapeB)
			entry := w.activeContactData[ai]
			entry.manifold = assignManifoldPointMetadata(entry.manifold, touching[ti].manifold, &entry.nextManifoldPointID)
			entry.friction = friction
			entry.restitution = restitution
			newPairs = append(newPairs, pair)
			newData = append(newData, entry)
			ai++
			ti++
		}
	}
	// Remaining active contacts that are no longer touching
	for ; ai < len(w.activeContactPairs); ai++ {
		entry := w.activeContactData[ai]
		endContacts = append(endContacts, ContactEndTouchEvent{
			ShapeIDA:  entry.shapeIDA,
			ShapeIDB:  entry.shapeIDB,
			ContactID: entry.contactID,
		})
	}
	// Remaining new touching pairs
	for ; ti < len(touching); ti++ {
		pair := touching[ti].pair
		shapeA := &w.shapes[pair.a]
		shapeB := &w.shapes[pair.b]
		friction, restitution := w.effectiveContactMaterialLocked(
			w.shapeIDFromIndexLocked(pair.a), w.shapeIDFromIndexLocked(pair.b), *shapeA, *shapeB)
		entry := w.newActiveContactEntry(pair)
		entry.manifold = assignManifoldPointMetadata(Manifold{}, touching[ti].manifold, &entry.nextManifoldPointID)
		entry.friction = friction
		entry.restitution = restitution
		newPairs = append(newPairs, pair)
		newData = append(newData, entry)
		beginContacts = append(beginContacts, ContactBeginTouchEvent{
			ShapeIDA:  entry.shapeIDA,
			ShapeIDB:  entry.shapeIDB,
			ContactID: entry.contactID,
		})
	}
	w.activeContactPairs = newPairs
	w.activeContactData = newData
	w.scratchMergedPairs = newPairs
	w.scratchMergedData = newData

	slices.SortFunc(beginContacts, func(a, b ContactBeginTouchEvent) int {
		return cmp.Compare(a.ContactID.Index1, b.ContactID.Index1)
	})
	slices.SortFunc(endContacts, func(a, b ContactEndTouchEvent) int {
		return cmp.Compare(a.ContactID.Index1, b.ContactID.Index1)
	})

	hitEvents := w.scratchHitEvents[:0]
	for ci, pair := range w.activeContactPairs {
		entry := w.activeContactData[ci]
		shapeA := w.shapes[pair.a]
		shapeB := w.shapes[pair.b]
		if !shapeA.enableHitEvents && !shapeB.enableHitEvents {
			continue
		}
		bodyA := w.bodies[shapeA.bodyIndex]
		bodyB := w.bodies[shapeB.bodyIndex]
		manifold := entry.manifold
		if manifold.PointCount == 0 {
			continue
		}
		approachSpeed := -Dot(Sub(bodyB.linearVelocity, bodyA.linearVelocity), manifold.Normal)
		if approachSpeed <= 0 {
			continue
		}
		if approachSpeed <= w.hitEventThreshold {
			continue
		}
		hitEvents = append(hitEvents, ContactHitEvent{
			ShapeIDA:      entry.shapeIDA,
			ShapeIDB:      entry.shapeIDB,
			ContactID:     entry.contactID,
			Point:         manifold.Point1.Point,
			Normal:        manifold.Normal,
			ApproachSpeed: approachSpeed,
		})
	}

	w.scratchBeginContacts = beginContacts
	w.scratchEndContacts = endContacts
	w.scratchHitEvents = hitEvents
	w.contactEvents = ContactEvents{
		BeginEvents: beginContacts,
		EndEvents:   endContacts,
		HitEvents:   hitEvents,
		BeginCount:  len(beginContacts),
		EndCount:    len(endContacts),
		HitCount:    len(hitEvents),
		Count:       len(w.activeContactPairs),
	}

	sensorPairs := w.collectTouchingSensorPairsFromCandidatesLocked(candidates)

	// Build sorted current sensor keys/data from touching sensor pairs
	currKeys := w.scratchCurrSensorKeys[:0]
	currData := w.scratchCurrSensorData[:0]
	for _, pair := range sensorPairs {
		shapeA := w.shapes[pair.a]
		shapeB := w.shapes[pair.b]
		if shapeA.isSensor {
			currKeys = append(currKeys, sensorTouchKey{sensor: pair.a, visitor: pair.b})
			currData = append(currData, activeSensorTouch{
				sensorShapeID:  w.shapeIDFromIndexLocked(pair.a),
				visitorShapeID: w.shapeIDFromIndexLocked(pair.b),
			})
		}
		if shapeB.isSensor {
			currKeys = append(currKeys, sensorTouchKey{sensor: pair.b, visitor: pair.a})
			currData = append(currData, activeSensorTouch{
				sensorShapeID:  w.shapeIDFromIndexLocked(pair.b),
				visitorShapeID: w.shapeIDFromIndexLocked(pair.a),
			})
		}
	}
	// Insertion sort: O(n) for nearly-sorted data (sensor pairs move slightly between frames)
	for i := 1; i < len(currKeys); i++ {
		kKey := currKeys[i]
		kData := currData[i]
		j := i - 1
		for j >= 0 && compareSensorTouchKeys(currKeys[j], kKey) > 0 {
			currKeys[j+1] = currKeys[j]
			currData[j+1] = currData[j]
			j--
		}
		currKeys[j+1] = kKey
		currData[j+1] = kData
	}
	w.scratchCurrSensorKeys = currKeys
	w.scratchCurrSensorData = currData

	// Sorted merge: diff previous (activeSensorKeys) vs current to find begin/end events
	prevKeys := w.activeSensorKeys
	prevData := w.activeSensorData
	beginSensors := w.scratchBeginSensors[:0]
	endSensors := w.scratchEndSensors[:0]

	pi, ci := 0, 0
	for pi < len(prevKeys) && ci < len(currKeys) {
		c := compareSensorTouchKeys(prevKeys[pi], currKeys[ci])
		if c < 0 {
			// In previous but not current: end event
			endSensors = append(endSensors, SensorEndTouchEvent{
				SensorShapeID:  prevData[pi].sensorShapeID,
				VisitorShapeID: prevData[pi].visitorShapeID,
			})
			pi++
		} else if c > 0 {
			// In current but not previous: begin event
			beginSensors = append(beginSensors, SensorBeginTouchEvent{
				SensorShapeID:  currData[ci].sensorShapeID,
				VisitorShapeID: currData[ci].visitorShapeID,
			})
			ci++
		} else {
			// In both: no event
			pi++
			ci++
		}
	}
	for ; pi < len(prevKeys); pi++ {
		endSensors = append(endSensors, SensorEndTouchEvent{
			SensorShapeID:  prevData[pi].sensorShapeID,
			VisitorShapeID: prevData[pi].visitorShapeID,
		})
	}
	for ; ci < len(currKeys); ci++ {
		beginSensors = append(beginSensors, SensorBeginTouchEvent{
			SensorShapeID:  currData[ci].sensorShapeID,
			VisitorShapeID: currData[ci].visitorShapeID,
		})
	}

	// Current becomes previous for next step (swap scratch into active)
	w.activeSensorKeys = currKeys
	w.activeSensorData = currData

	w.scratchBeginSensors = beginSensors
	w.scratchEndSensors = endSensors
	w.sensorEvents = SensorEvents{
		BeginEvents: beginSensors,
		EndEvents:   endSensors,
		BeginCount:  len(beginSensors),
		EndCount:    len(endSensors),
		Count:       len(w.activeSensorKeys),
	}
}

// refreshManifoldsAndEventsLocked updates manifolds for existing contacts
// and generates events, without re-running the broadphase.
func (w *World) refreshManifoldsAndEventsLocked() {
	// Update manifolds for all existing contacts
	for ci, pair := range w.activeContactPairs {
		shapeA := &w.shapes[pair.a]
		shapeB := &w.shapes[pair.b]
		if !shapeA.alive || !shapeB.alive {
			continue
		}
		bodyA := &w.bodies[shapeA.bodyIndex]
		bodyB := &w.bodies[shapeB.bodyIndex]
		entry := w.activeContactData[ci]
		manifold := assignManifoldPointMetadata(entry.manifold, contactManifoldFromPair(shapeA, bodyA, shapeB, bodyB), &entry.nextManifoldPointID)
		entry.manifold = manifold
		w.activeContactData[ci] = entry
	}

	// Generate hit events
	hitEvents := w.scratchHitEvents[:0]
	for ci, pair := range w.activeContactPairs {
		entry := w.activeContactData[ci]
		shapeA := w.shapes[pair.a]
		shapeB := w.shapes[pair.b]
		if !shapeA.enableHitEvents && !shapeB.enableHitEvents {
			continue
		}
		bodyA := w.bodies[shapeA.bodyIndex]
		bodyB := w.bodies[shapeB.bodyIndex]
		manifold := entry.manifold
		if manifold.PointCount == 0 {
			continue
		}
		approachSpeed := -Dot(Sub(bodyB.linearVelocity, bodyA.linearVelocity), manifold.Normal)
		if approachSpeed <= 0 || approachSpeed <= w.hitEventThreshold {
			continue
		}
		hitEvents = append(hitEvents, ContactHitEvent{
			ShapeIDA:      entry.shapeIDA,
			ShapeIDB:      entry.shapeIDB,
			ContactID:     entry.contactID,
			Point:         manifold.Point1.Point,
			Normal:        manifold.Normal,
			ApproachSpeed: approachSpeed,
		})
	}

	w.scratchHitEvents = hitEvents
	w.contactEvents = ContactEvents{
		BeginEvents: nil,
		EndEvents:   nil,
		HitEvents:   hitEvents,
		BeginCount:  0,
		EndCount:    0,
		HitCount:    len(hitEvents),
		Count:       len(w.activeContactPairs),
	}

	// No sensor broadphase needed — sensor state doesn't change during solving
	w.sensorEvents = SensorEvents{
		Count: len(w.activeSensorKeys),
	}
}

func (w *World) contactDataFromPairLocked(pair shapePair) ContactData {
	entry := w.activeContactFromPairLocked(pair)
	return ContactData{
		ContactID: entry.contactID,
		ShapeIDA:  entry.shapeIDA,
		ShapeIDB:  entry.shapeIDB,
		Manifold:  entry.manifold,
	}
}

func (w *World) sortedActiveContactPairsLocked() []shapePair {
	return w.activeContactPairs
}

// SetGravity sets the world gravity vector.
func (w *World) SetGravity(gravity Vec2) error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	w.mu.Lock()
	w.gravity = gravity
	w.mu.Unlock()
	return nil
}

// Gravity returns the current world gravity vector.
func (w *World) Gravity() (Vec2, error) {
	if !w.IsValid() {
		return Vec2{}, ErrWorldInvalid
	}
	w.mu.RLock()
	defer w.mu.RUnlock()
	return w.gravity, nil
}

func (w *World) GetGravity() (Vec2, error) {
	return w.Gravity()
}

// SetUserData stores user data on this world.
func (w *World) SetUserData(v any) error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	w.mu.Lock()
	w.userData = v
	w.mu.Unlock()
	return nil
}

// UserData returns user data attached to this world.
func (w *World) UserData() (any, error) {
	if !w.IsValid() {
		return nil, ErrWorldInvalid
	}
	w.mu.RLock()
	defer w.mu.RUnlock()
	return w.userData, nil
}

func (w *World) GetUserData() (any, error) {
	return w.UserData()
}

func (w *World) String() string {
	if w == nil {
		return "World<nil>"
	}
	id := w.ID()
	return fmt.Sprintf("World{id:%d gen:%d}", id.Index1, id.Generation)
}

type Counters struct {
	BodyCount  int
	ShapeCount int
	JointCount int
	ChainCount int
}

func (w *World) Counters() (Counters, error) {
	if !w.IsValid() {
		return Counters{}, ErrWorldInvalid
	}
	w.mu.RLock()
	defer w.mu.RUnlock()
	return Counters{
		BodyCount:  w.bodyCount,
		ShapeCount: w.shapeCount,
		JointCount: w.jointCount,
		ChainCount: w.chainCount,
	}, nil
}

func (w *World) GetCounters() (Counters, error) {
	return w.Counters()
}

func (w *World) GetBodyCount() (int, error) {
	counters, err := w.Counters()
	if err != nil {
		return 0, err
	}
	return counters.BodyCount, nil
}

func (w *World) GetShapeCount() (int, error) {
	counters, err := w.Counters()
	if err != nil {
		return 0, err
	}
	return counters.ShapeCount, nil
}

func (w *World) GetJointCount() (int, error) {
	counters, err := w.Counters()
	if err != nil {
		return 0, err
	}
	return counters.JointCount, nil
}

func (w *World) GetChainCount() (int, error) {
	counters, err := w.Counters()
	if err != nil {
		return 0, err
	}
	return counters.ChainCount, nil
}

func (w *World) GetContactCount() (int, error) {
	if !w.IsValid() {
		return 0, ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	if w.id.IsNull() {
		return 0, ErrWorldInvalid
	}
	w.refreshContactAndSensorStateLocked()
	return len(w.activeContactPairs), nil
}

func (w *World) GetSensorTouchCount() (int, error) {
	if !w.IsValid() {
		return 0, ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	if w.id.IsNull() {
		return 0, ErrWorldInvalid
	}
	w.refreshContactAndSensorStateLocked()
	return len(w.activeSensorKeys), nil
}

func (w *World) GetProxyCount() (int, error) {
	return w.GetShapeCount()
}

func (w *World) SetRestitutionThreshold(v float32) error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	w.restitutionThreshold = v
	return nil
}

func (w *World) GetRestitutionThreshold() (float32, error) {
	if !w.IsValid() {
		return 0, ErrWorldInvalid
	}
	w.mu.RLock()
	defer w.mu.RUnlock()
	return w.restitutionThreshold, nil
}

func (w *World) SetHitEventThreshold(v float32) error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	w.hitEventThreshold = v
	return nil
}

func (w *World) GetHitEventThreshold() (float32, error) {
	if !w.IsValid() {
		return 0, ErrWorldInvalid
	}
	w.mu.RLock()
	defer w.mu.RUnlock()
	return w.hitEventThreshold, nil
}

func (w *World) SetMaximumLinearSpeed(v float32) error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	w.maximumLinearSpeed = v
	return nil
}

func (w *World) GetMaximumLinearSpeed() (float32, error) {
	if !w.IsValid() {
		return 0, ErrWorldInvalid
	}
	w.mu.RLock()
	defer w.mu.RUnlock()
	return w.maximumLinearSpeed, nil
}

func (w *World) SetContactTuning(hertz, dampingRatio, pushSpeed float32) error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	w.contactHertz = hertz
	w.contactDampingRatio = dampingRatio
	w.contactSpeed = pushSpeed
	return nil
}

func (w *World) EnableSleeping(flag bool) error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	w.enableSleep = flag
	if !flag {
		for i := range w.bodies {
			if !w.bodies[i].alive || !w.bodies[i].isEnabled {
				continue
			}
			w.bodies[i].isAwake = true
			w.bodies[i].sleepTime = 0
		}
	}
	return nil
}

func (w *World) IsSleepingEnabled() (bool, error) {
	if !w.IsValid() {
		return false, ErrWorldInvalid
	}
	w.mu.RLock()
	defer w.mu.RUnlock()
	return w.enableSleep, nil
}

func (w *World) EnableContinuous(flag bool) error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	w.enableContinuous = flag
	return nil
}

func (w *World) IsContinuousEnabled() (bool, error) {
	if !w.IsValid() {
		return false, ErrWorldInvalid
	}
	w.mu.RLock()
	defer w.mu.RUnlock()
	return w.enableContinuous, nil
}

func (w *World) EnableWarmStarting(flag bool) error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	w.enableWarmStarting = flag
	return nil
}

func (w *World) IsWarmStartingEnabled() (bool, error) {
	if !w.IsValid() {
		return false, ErrWorldInvalid
	}
	w.mu.RLock()
	defer w.mu.RUnlock()
	return w.enableWarmStarting, nil
}

func (w *World) EnableSpeculative(flag bool) error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	w.enableSpeculative = flag
	return nil
}

func (w *World) IsSpeculativeEnabled() (bool, error) {
	if !w.IsValid() {
		return false, ErrWorldInvalid
	}
	w.mu.RLock()
	defer w.mu.RUnlock()
	return w.enableSpeculative, nil
}

func (w *World) GetAwakeBodyCount() (int, error) {
	if !w.IsValid() {
		return 0, ErrWorldInvalid
	}
	w.mu.RLock()
	defer w.mu.RUnlock()
	count := 0
	for i := range w.bodies {
		body := w.bodies[i]
		if body.alive && body.isEnabled && body.isAwake {
			count++
		}
	}
	return count, nil
}

func (w *World) OverlapAABB(query AABB) ([]ShapeID, error) {
	return w.OverlapAABBWithFilter(query, DefaultQueryFilter())
}

func (w *World) OverlapAABBWithFilter(query AABB, filter QueryFilter) ([]ShapeID, error) {
	if !w.IsValid() {
		return nil, ErrWorldInvalid
	}
	w.mu.RLock()
	defer w.mu.RUnlock()
	if w.id.IsNull() {
		return nil, ErrWorldInvalid
	}
	shapeIndices := w.collectShapeCandidatesForAABBLocked(query)
	slices.Sort(shapeIndices)
	ids := make([]ShapeID, 0, len(shapeIndices))
	for _, shapeIdx := range shapeIndices {
		shape := w.shapes[shapeIdx]
		if !shapePassesQueryFilter(shape, filter) {
			continue
		}
		ids = append(ids, ShapeID{
			Index1:     int32(shapeIdx + 1),
			World0:     w.id.Index1 - 1,
			Generation: shape.generation,
		})
	}
	return ids, nil
}

func (w *World) CastRay(input RayCastInput) ([]RayHit, error) {
	return w.CastRayWithFilter(input, DefaultQueryFilter())
}

func (w *World) CastRayWithFilter(input RayCastInput, filter QueryFilter) ([]RayHit, error) {
	if !w.IsValid() {
		return nil, ErrWorldInvalid
	}
	if !IsValidRay(input) {
		return []RayHit{}, nil
	}
	w.mu.RLock()
	defer w.mu.RUnlock()
	if w.id.IsNull() {
		return nil, ErrWorldInvalid
	}

	rayAABB := rayInputAABB(input)
	hits := make([]RayHit, 0)
	for _, shapeIdx := range w.collectShapeCandidatesForAABBLocked(rayAABB) {
		shape := w.shapes[shapeIdx]
		if !shapePassesQueryFilter(shape, filter) {
			continue
		}
		body := w.bodies[shape.bodyIndex]
		output := rayCastShapeWithBody(shape, body, input)
		if !output.Hit {
			continue
		}
		hits = append(hits, RayHit{
			ShapeID: ShapeID{
				Index1:     int32(shapeIdx + 1),
				World0:     w.id.Index1 - 1,
				Generation: shape.generation,
			},
			Output: output,
		})
	}

	slices.SortFunc(hits, func(a, b RayHit) int {
		if c := cmp.Compare(a.Output.Fraction, b.Output.Fraction); c != 0 {
			return c
		}
		return cmp.Compare(a.ShapeID.Index1, b.ShapeID.Index1)
	})

	return hits, nil
}

func (w *World) collectShapeCandidatesForAABBLocked(query AABB) []int {
	candidates := make([]broadPhaseShape, 0, len(w.shapes))
	for i := range w.shapes {
		shape := &w.shapes[i]
		if !shape.alive {
			continue
		}
		bodyIndex := shape.bodyIndex
		if bodyIndex < 0 || bodyIndex >= len(w.bodies) {
			continue
		}
		body := &w.bodies[bodyIndex]
		if !body.alive || !body.isEnabled {
			continue
		}
		aabb := shapeAABBWithBody(shape, body)
		candidates = append(candidates, broadPhaseShape{
			idx:       i,
			bodyIndex: bodyIndex,
			aabb:      aabb,
		})
	}

	slices.SortFunc(candidates, func(a, b broadPhaseShape) int {
		if c := cmp.Compare(a.aabb.LowerBound.X, b.aabb.LowerBound.X); c != 0 {
			return c
		}
		return cmp.Compare(a.idx, b.idx)
	})

	ids := make([]int, 0)
	for _, candidate := range candidates {
		if candidate.aabb.LowerBound.X > query.UpperBound.X {
			break
		}
		if candidate.aabb.UpperBound.X < query.LowerBound.X {
			continue
		}
		if !AABBOverlaps(query, candidate.aabb) {
			continue
		}
		ids = append(ids, candidate.idx)
	}
	return ids
}

func rayInputAABB(input RayCastInput) AABB {
	end := MulAdd(input.Origin, input.MaxFraction, input.Translation)
	return AABB{
		LowerBound: Min(input.Origin, end),
		UpperBound: Max(input.Origin, end),
	}
}

func (w *World) CastRayClosest(input RayCastInput) (RayHit, bool, error) {
	return w.CastRayClosestWithFilter(input, DefaultQueryFilter())
}

func (w *World) CastRayClosestWithFilter(input RayCastInput, filter QueryFilter) (RayHit, bool, error) {
	if !w.IsValid() {
		return RayHit{}, false, ErrWorldInvalid
	}
	if !IsValidRay(input) {
		return RayHit{}, false, nil
	}
	w.mu.RLock()
	defer w.mu.RUnlock()
	if w.id.IsNull() {
		return RayHit{}, false, ErrWorldInvalid
	}

	rayAABB := rayInputAABB(input)
	found := false
	best := RayHit{}
	for _, shapeIdx := range w.collectShapeCandidatesForAABBLocked(rayAABB) {
		shape := w.shapes[shapeIdx]
		if !shapePassesQueryFilter(shape, filter) {
			continue
		}
		body := w.bodies[shape.bodyIndex]
		output := rayCastShapeWithBody(shape, body, input)
		if !output.Hit {
			continue
		}
		hit := RayHit{
			ShapeID: ShapeID{
				Index1:     int32(shapeIdx + 1),
				World0:     w.id.Index1 - 1,
				Generation: shape.generation,
			},
			Output: output,
		}
		if hit.Output.Fraction == 0 {
			continue
		}
		if !found {
			best = hit
			found = true
			continue
		}
		if hit.Output.Fraction < best.Output.Fraction || (hit.Output.Fraction == best.Output.Fraction && hit.ShapeID.Index1 < best.ShapeID.Index1) {
			best = hit
		}
	}
	return best, found, nil
}

func shapePassesQueryFilter(shape shapeState, filter QueryFilter) bool {
	if shape.filter.CategoryBits&filter.MaskBits == 0 {
		return false
	}
	if shape.filter.MaskBits&filter.CategoryBits == 0 {
		return false
	}
	return true
}

func (w *World) OverlapShape() ([]ShapeID, error) {
	return w.OverlapShapeWithFilter(DefaultQueryFilter())
}

func (w *World) OverlapShapeWithFilter(filter QueryFilter) ([]ShapeID, error) {
	if !w.IsValid() {
		return nil, ErrWorldInvalid
	}
	w.mu.RLock()
	defer w.mu.RUnlock()
	if w.id.IsNull() {
		return nil, ErrWorldInvalid
	}
	pairs := w.collectTouchingContactPairsLocked()
	sensorPairs := w.collectTouchingSensorPairsLocked()
	if len(pairs) == 0 && len(sensorPairs) == 0 {
		return []ShapeID{}, nil
	}
	unique := make(map[int]struct{}, len(pairs)*2+len(sensorPairs)*2)
	for _, pair := range pairs {
		unique[pair.a] = struct{}{}
		unique[pair.b] = struct{}{}
	}
	for _, pair := range sensorPairs {
		unique[pair.a] = struct{}{}
		unique[pair.b] = struct{}{}
	}
	shapeIndices := make([]int, 0, len(unique))
	for idx := range unique {
		shapeIndices = append(shapeIndices, idx)
	}
	slices.Sort(shapeIndices)
	ids := make([]ShapeID, 0, len(shapeIndices))
	for _, idx := range shapeIndices {
		shape := w.shapes[idx]
		if !shape.alive {
			continue
		}
		if !shapePassesQueryFilter(shape, filter) {
			continue
		}
		ids = append(ids, w.shapeIDFromIndexLocked(idx))
	}
	return ids, nil
}

func (w *World) CastShape() ([]RayHit, error) {
	return w.CastShapeWithFilter(DefaultQueryFilter())
}

func (w *World) CastShapeWithFilter(filter QueryFilter) ([]RayHit, error) {
	if !w.IsValid() {
		return nil, ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	if w.id.IsNull() {
		return nil, ErrWorldInvalid
	}
	w.refreshContactAndSensorStateLocked()
	pairs := w.sortedActiveContactPairsLocked()
	hits := make([]RayHit, 0, len(pairs)*2)
	for ci := range pairs {
		entry := w.activeContactData[ci]
		if entry.manifold.PointCount == 0 {
			continue
		}
		shapeAIdx := int(entry.shapeIDA.Index1) - 1
		shapeBIdx := int(entry.shapeIDB.Index1) - 1
		aPass := shapeAIdx >= 0 && shapeAIdx < len(w.shapes) && shapePassesQueryFilter(w.shapes[shapeAIdx], filter)
		bPass := shapeBIdx >= 0 && shapeBIdx < len(w.shapes) && shapePassesQueryFilter(w.shapes[shapeBIdx], filter)
		if !aPass && !bPass {
			continue
		}
		if aPass {
			hits = append(hits, RayHit{
				ShapeID: entry.shapeIDA,
				Output: CastOutput{
					Normal:   entry.manifold.Normal,
					Point:    entry.manifold.Point1.Point,
					Fraction: 0,
					Hit:      true,
				},
			})
		}
		if bPass {
			hits = append(hits, RayHit{
				ShapeID: entry.shapeIDB,
				Output: CastOutput{
					Normal:   Neg(entry.manifold.Normal),
					Point:    entry.manifold.Point1.Point,
					Fraction: 0,
					Hit:      true,
				},
			})
		}
	}
	slices.SortFunc(hits, func(a, b RayHit) int {
		return cmp.Compare(a.ShapeID.Index1, b.ShapeID.Index1)
	})
	return hits, nil
}

func (w *World) CastMover() (CastOutput, error) {
	return w.CastMoverWithFilter(DefaultQueryFilter())
}

func (w *World) CastMoverWithFilter(filter QueryFilter) (CastOutput, error) {
	if !w.IsValid() {
		return CastOutput{}, ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	if w.id.IsNull() {
		return CastOutput{}, ErrWorldInvalid
	}
	w.refreshContactAndSensorStateLocked()
	pairs := w.sortedActiveContactPairsLocked()
	for ci := range pairs {
		entry := w.activeContactData[ci]
		if entry.manifold.PointCount == 0 {
			continue
		}
		shapeAIdx := int(entry.shapeIDA.Index1) - 1
		shapeBIdx := int(entry.shapeIDB.Index1) - 1
		aPass := shapeAIdx >= 0 && shapeAIdx < len(w.shapes) && shapePassesQueryFilter(w.shapes[shapeAIdx], filter)
		bPass := shapeBIdx >= 0 && shapeBIdx < len(w.shapes) && shapePassesQueryFilter(w.shapes[shapeBIdx], filter)
		if !aPass && !bPass {
			continue
		}
		normal := entry.manifold.Normal
		if aPass && bPass {
			if entry.shapeIDB.Index1 < entry.shapeIDA.Index1 {
				normal = Neg(normal)
			}
		} else if bPass {
			normal = Neg(normal)
		}
		return CastOutput{
			Normal:   normal,
			Point:    entry.manifold.Point1.Point,
			Fraction: 0,
			Hit:      true,
		}, nil
	}
	return CastOutput{}, nil
}

func (w *World) CollideMover() error {
	return w.CollideMoverWithFilter(DefaultQueryFilter())
}

func (w *World) CollideMoverWithFilter(filter QueryFilter) error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	if w.id.IsNull() {
		return ErrWorldInvalid
	}
	w.refreshContactAndSensorStateLocked()
	pairs := w.sortedActiveContactPairsLocked()
	matches := 0
	for ci := range pairs {
		entry := w.activeContactData[ci]
		if entry.manifold.PointCount == 0 {
			continue
		}
		shapeAIdx := int(entry.shapeIDA.Index1) - 1
		shapeBIdx := int(entry.shapeIDB.Index1) - 1
		aPass := shapeAIdx >= 0 && shapeAIdx < len(w.shapes) && shapePassesQueryFilter(w.shapes[shapeAIdx], filter)
		bPass := shapeBIdx >= 0 && shapeBIdx < len(w.shapes) && shapePassesQueryFilter(w.shapes[shapeBIdx], filter)
		if aPass || bPass {
			matches++
		}
	}
	w.lastMoverMatches = matches
	return nil
}

func explosionPerimeterForShape(shape shapeState) float32 {
	switch shape.shapeType {
	case CircleShape:
		return 2 * Pi * shape.circle.Radius
	case CapsuleShape:
		spine := Length(Sub(shape.capsule.Center2, shape.capsule.Center1))
		return 2*spine + 2*Pi*shape.capsule.Radius
	case PolygonShape:
		polygon := shape.polygon
		if polygon.Count < 2 {
			return 0
		}
		perimeter := float32(0)
		for i := range polygon.Count {
			j := i + 1
			if j == polygon.Count {
				j = 0
			}
			perimeter += Distance(polygon.Vertices[i], polygon.Vertices[j])
		}
		return perimeter + 2*Pi*polygon.Radius
	case SegmentShape, ChainSegmentShape:
		return 2 * Distance(shape.segment.Point1, shape.segment.Point2)
	default:
		return 0
	}
}

func shapeExplosionImpactPoint(shape shapeState, body bodyState, target Vec2) (Vec2, float32, bool) {
	transform := shapeWorldTransform(&body)
	localTarget := InvTransformPoint(transform, target)

	switch shape.shapeType {
	case CircleShape:
		delta := Sub(localTarget, shape.circle.Center)
		centerDistance := Length(delta)
		if centerDistance <= floatEpsilon {
			return Vec2Zero, 0, true
		}
		localPoint := MulAdd(shape.circle.Center, shape.circle.Radius/centerDistance, delta)
		distance := centerDistance - shape.circle.Radius
		if distance < 0 {
			distance = 0
		}
		return TransformPoint(transform, localPoint), distance, true
	case CapsuleShape:
		axisPoint := closestPointOnSegment(localTarget, shape.capsule.Center1, shape.capsule.Center2)
		delta := Sub(localTarget, axisPoint)
		centerDistance := Length(delta)
		if centerDistance <= floatEpsilon {
			return Vec2Zero, 0, true
		}
		localPoint := MulAdd(axisPoint, shape.capsule.Radius/centerDistance, delta)
		distance := centerDistance - shape.capsule.Radius
		if distance < 0 {
			distance = 0
		}
		return TransformPoint(transform, localPoint), distance, true
	case SegmentShape, ChainSegmentShape:
		localPoint := closestPointOnSegment(localTarget, shape.segment.Point1, shape.segment.Point2)
		distance := Distance(localTarget, localPoint)
		if distance <= floatEpsilon {
			return TransformPoint(transform, localPoint), 0, true
		}
		return TransformPoint(transform, localPoint), distance, true
	case PolygonShape:
		localPoint := closestPointOnPolygon(shape.polygon, localTarget)
		if PointInPolygon(&shape.polygon, localTarget) || DistanceSquared(localTarget, localPoint) <= floatEpsilon*floatEpsilon {
			return TransformPoint(transform, localPoint), 0, true
		}
		return TransformPoint(transform, localPoint), Distance(localTarget, localPoint), true
	default:
		return Vec2Zero, 0, false
	}
}

func shapeExplosionProjectedPerimeter(shape shapeState, localLine Vec2) float32 {
	switch shape.shapeType {
	case CircleShape:
		return 2 * shape.circle.Radius
	case CapsuleShape:
		axis := Sub(shape.capsule.Center2, shape.capsule.Center1)
		return AbsFloat(Dot(axis, localLine)) + 2*shape.capsule.Radius
	case PolygonShape:
		if shape.polygon.Count <= 0 {
			return 0
		}
		lower := Dot(shape.polygon.Vertices[0], localLine)
		upper := lower
		for i := 1; i < shape.polygon.Count; i++ {
			value := Dot(shape.polygon.Vertices[i], localLine)
			if value < lower {
				lower = value
			}
			if value > upper {
				upper = value
			}
		}
		return (upper - lower) + 2*shape.polygon.Radius
	case SegmentShape, ChainSegmentShape:
		return AbsFloat(Dot(Sub(shape.segment.Point2, shape.segment.Point1), localLine))
	default:
		return 0
	}
}

func (w *World) ExplodeWithDef(def ExplosionDef) error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	if !def.isInitialized() {
		return ErrInvalidExplosionDef
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	if w.id.IsNull() {
		return ErrWorldInvalid
	}

	radius := def.Radius
	falloff := def.Falloff
	reach := radius
	if falloff > 0 {
		reach += falloff
	}
	if reach <= 0 || def.ImpulsePerLength == 0 {
		return nil
	}

	for i := range w.bodies {
		body := &w.bodies[i]
		if !body.alive || !body.isEnabled || body.bodyType != DynamicBody || body.invMass == 0 {
			continue
		}
		totalLinearImpulse := Vec2Zero
		totalAngularImpulse := float32(0)
		matched := false
		center := Add(body.position, RotateVector(body.rotation, body.localCenter))
		for _, shapeIdx := range body.shapeIndices {
			if shapeIdx < 0 || shapeIdx >= len(w.shapes) {
				continue
			}
			shape := w.shapes[shapeIdx]
			if !shape.alive || shape.filter.CategoryBits&def.MaskBits == 0 {
				continue
			}
			closestPoint, distance, ok := shapeExplosionImpactPoint(shape, *body, def.Position)
			if !ok {
				continue
			}
			if distance > reach {
				continue
			}
			scale := float32(1)
			if distance > radius {
				if falloff <= 0 {
					continue
				}
				scale = 1.0 - (distance-radius)/falloff
				if scale <= 0 {
					continue
				}
			}
			if distance <= floatEpsilon {
				closestPoint = shapeRepresentativePoint(&shape, body)
			}
			delta := Sub(closestPoint, def.Position)
			dir := Vec2{X: 1, Y: 0}
			if LengthSquared(delta) > floatEpsilon*floatEpsilon {
				dir = Normalize(delta)
			}
			localLine := InvRotateVector(body.rotation, LeftPerp(dir))
			perimeter := shapeExplosionProjectedPerimeter(shape, localLine)
			if perimeter <= 0 {
				continue
			}
			impulse := MulSV(def.ImpulsePerLength*perimeter*scale, dir)
			totalLinearImpulse = Add(totalLinearImpulse, impulse)
			totalAngularImpulse += Cross(Sub(closestPoint, center), impulse)
			matched = true
		}
		if !matched {
			continue
		}
		body.linearVelocity = MulAdd(body.linearVelocity, body.invMass, totalLinearImpulse)
		body.angularVelocity += body.invInertia * totalAngularImpulse
		body.isAwake = true
		body.sleepTime = 0
	}

	return nil
}

func (w *World) Explode() error {
	def := DefaultExplosionDef()
	def.Position = Vec2Zero
	def.Radius = 10
	def.Falloff = 0
	def.ImpulsePerLength = 20
	return w.ExplodeWithDef(def)
}

func (w *World) Draw() error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	if w.id.IsNull() {
		return ErrWorldInvalid
	}
	w.refreshContactAndSensorStateLocked()
	w.profile.DrawCount++
	return nil
}

func (w *World) DumpMemoryStats() (string, error) {
	if !w.IsValid() {
		return "", ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	if w.id.IsNull() {
		return "", ErrWorldInvalid
	}
	w.refreshContactAndSensorStateLocked()

	stats := map[string]any{
		"bodies": map[string]int{
			"count":    w.bodyCount,
			"capacity": len(w.bodies),
			"free":     len(w.freeBodyIndices),
		},
		"shapes": map[string]int{
			"count":    w.shapeCount,
			"capacity": len(w.shapes),
			"free":     len(w.freeShapeIndices),
		},
		"joints": map[string]int{
			"count":    w.jointCount,
			"capacity": len(w.joints),
			"free":     len(w.freeJointIndices),
		},
		"chains": map[string]int{
			"count":    w.chainCount,
			"capacity": len(w.chains),
			"free":     len(w.freeChainIndices),
		},
		"contacts": map[string]int{
			"active": len(w.activeContactPairs),
		},
		"sensors": map[string]int{
			"activeTouches": len(w.activeSensorKeys),
		},
		"broadPhase": map[string]int{
			"candidatePairs": w.broadPhaseCandidates,
			"rebuilds":       w.staticTreeRebuilds,
		},
		"mover": map[string]int{
			"matches": w.lastMoverMatches,
		},
	}
	encoded, err := json.Marshal(stats)
	if err != nil {
		return "", err
	}
	return string(encoded), nil
}

func (w *World) RebuildStaticTree() error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	if w.id.IsNull() {
		return ErrWorldInvalid
	}
	w.broadPhaseCandidates = len(w.collectBroadPhaseCandidatesLocked())
	w.staticTreeRebuilds++
	return nil
}

func (w *World) SetCustomFilterCallback(cb CustomFilterCallback) error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	w.customFilterCallback = cb
	return nil
}

func (w *World) SetFrictionCallback(cb FrictionCallback) error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	w.frictionCallback = cb
	return nil
}

func (w *World) SetRestitutionCallback(cb RestitutionCallback) error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	w.restitutionCallback = cb
	return nil
}

func (w *World) SetPreSolveCallback(cb PreSolveCallback) error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	w.preSolveCallback = cb
	return nil
}

func (w *World) GetBodyEvents() (BodyEvents, error) {
	if !w.IsValid() {
		return BodyEvents{}, ErrWorldInvalid
	}
	w.mu.RLock()
	defer w.mu.RUnlock()
	if w.id.IsNull() {
		return BodyEvents{}, ErrWorldInvalid
	}
	events := w.bodyEvents
	events.MoveEvents = append([]BodyMoveEvent(nil), events.MoveEvents...)
	return events, nil
}

func (w *World) GetContactEvents() (ContactEvents, error) {
	if !w.IsValid() {
		return ContactEvents{}, ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	if w.id.IsNull() {
		return ContactEvents{}, ErrWorldInvalid
	}
	w.refreshContactAndSensorStateLocked()
	events := w.contactEvents
	events.BeginEvents = append([]ContactBeginTouchEvent(nil), events.BeginEvents...)
	events.EndEvents = append([]ContactEndTouchEvent(nil), events.EndEvents...)
	events.HitEvents = append([]ContactHitEvent(nil), events.HitEvents...)
	return events, nil
}

func (w *World) GetSensorEvents() (SensorEvents, error) {
	if !w.IsValid() {
		return SensorEvents{}, ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	if w.id.IsNull() {
		return SensorEvents{}, ErrWorldInvalid
	}
	w.refreshContactAndSensorStateLocked()
	events := w.sensorEvents
	events.BeginEvents = append([]SensorBeginTouchEvent(nil), events.BeginEvents...)
	events.EndEvents = append([]SensorEndTouchEvent(nil), events.EndEvents...)
	return events, nil
}

func (w *World) GetJointEvents() (JointEvents, error) {
	if !w.IsValid() {
		return JointEvents{}, ErrWorldInvalid
	}
	w.mu.RLock()
	defer w.mu.RUnlock()
	if w.id.IsNull() {
		return JointEvents{}, ErrWorldInvalid
	}
	events := w.jointEvents
	events.Events = append([]JointEvent(nil), events.Events...)
	return events, nil
}

func (w *World) GetProfile() (Profile, error) {
	if !w.IsValid() {
		return Profile{}, ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	if w.id.IsNull() {
		return Profile{}, ErrWorldInvalid
	}
	w.refreshContactAndSensorStateLocked()
	profile := w.profile
	profile.BodyCount = w.bodyCount
	profile.ShapeCount = w.shapeCount
	profile.ContactCount = len(w.activeContactPairs)
	profile.SensorCount = len(w.activeSensorKeys)
	profile.JointCount = w.jointCount
	awakeCount := 0
	for i := range w.bodies {
		body := w.bodies[i]
		if body.alive && body.isEnabled && body.isAwake {
			awakeCount++
		}
	}
	profile.AwakeBodyCount = awakeCount
	return profile, nil
}
