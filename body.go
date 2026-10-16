package box2d

import "errors"

var (
	ErrInvalidBodyDef  = errors.New("box2d: invalid body definition (use DefaultBodyDef)")
	ErrBodyInvalid     = errors.New("box2d: body is invalid")
	ErrInvalidMassData = errors.New("box2d: invalid mass data")
)

type bodyState struct {
	generation        uint16
	alive             bool
	bodyType          BodyType
	position          Vec2
	rotation          Rot
	linearVelocity    Vec2
	angularVelocity   float32
	force             Vec2
	torque            float32
	localCenter       Vec2
	mass              float32
	invMass           float32
	inertia           float32
	invInertia        float32
	linearDamping     float32
	angularDamping    float32
	gravityScale      float32
	sleepThreshold    float32
	name              string
	userData          any
	motionLocks       MotionLocks
	lockFlags         uint8
	enableSleep       bool
	isAwake           bool
	isBullet          bool
	isEnabled         bool
	allowFastRotation bool
	sleepTime         float32
	shapeIndices      []int
}

// Body is an OOP wrapper around a body id + world association.
type Body struct {
	world *World
	id    BodyID
}

// IsBodyIDValid reports whether an ID refers to a currently alive body.
func IsBodyIDValid(id BodyID) bool {
	if id.IsNull() {
		return false
	}
	world := worldFromIndex(id.World0 + 1)
	if world == nil {
		return false
	}

	world.mu.RLock()
	defer world.mu.RUnlock()
	return world.isBodyIDValidLocked(id)
}

func computeShapeMassData(shape shapeState) MassData {
	switch shape.shapeType {
	case CircleShape:
		return ComputeCircleMass(&shape.circle, shape.density)
	case CapsuleShape:
		return ComputeCapsuleMass(&shape.capsule, shape.density)
	case PolygonShape:
		return ComputePolygonMass(&shape.polygon, shape.density)
	case SegmentShape, ChainSegmentShape:
		return MassData{}
	default:
		return MassData{}
	}
}

func (w *World) updateBodyMassLocked(bodyIdx int) {
	if bodyIdx < 0 || bodyIdx >= len(w.bodies) {
		return
	}
	body := &w.bodies[bodyIdx]
	if !body.alive {
		return
	}

	if body.bodyType != DynamicBody {
		body.localCenter = Vec2Zero
		body.mass = 0
		body.invMass = 0
		body.inertia = 0
		body.invInertia = 0
		return
	}

	body.localCenter = Vec2Zero
	body.mass = 0
	body.invMass = 0
	body.inertia = 0
	body.invInertia = 0

	localCenter := Vec2Zero
	for _, shapeIdx := range body.shapeIndices {
		if shapeIdx < 0 || shapeIdx >= len(w.shapes) {
			continue
		}
		shape := w.shapes[shapeIdx]
		if !shape.alive || shape.density <= 0 {
			continue
		}
		massData := computeShapeMassData(shape)
		body.mass += massData.Mass
		localCenter = MulAdd(localCenter, massData.Mass, massData.Center)
	}

	if body.mass <= 0 {
		// Match upstream semantics: dynamic bodies without shape mass are effectively massless
		// until shapes are added or explicit mass data is set.
		body.mass = 0
		body.invMass = 0
		body.inertia = 0
		body.invInertia = 0
		return
	}

	body.invMass = 1.0 / body.mass
	body.localCenter = MulSV(body.invMass, localCenter)

	for _, shapeIdx := range body.shapeIndices {
		if shapeIdx < 0 || shapeIdx >= len(w.shapes) {
			continue
		}
		shape := w.shapes[shapeIdx]
		if !shape.alive || shape.density <= 0 {
			continue
		}
		massData := computeShapeMassData(shape)
		if massData.Mass <= 0 {
			continue
		}
		offset := Sub(body.localCenter, massData.Center)
		body.inertia += massData.RotationalInertia + massData.Mass*Dot(offset, offset)
	}

	if body.inertia > 0 {
		body.invInertia = 1.0 / body.inertia
	} else {
		body.inertia = 1.0
		body.invInertia = 1.0
	}
}

// CreateBody creates a body in the world.
func (w *World) CreateBody(def BodyDef) (*Body, error) {
	if !w.IsValid() {
		return nil, ErrWorldInvalid
	}
	if !def.isInitialized() {
		return nil, ErrInvalidBodyDef
	}

	w.mu.Lock()
	defer w.mu.Unlock()

	if w.id.IsNull() {
		return nil, ErrWorldInvalid
	}

	var idx int
	if n := len(w.freeBodyIndices); n > 0 {
		idx = w.freeBodyIndices[n-1]
		w.freeBodyIndices = w.freeBodyIndices[:n-1]
	} else {
		idx = len(w.bodies)
		w.bodies = append(w.bodies, bodyState{})
	}

	slot := &w.bodies[idx]
	slot.generation++
	if slot.generation == 0 {
		slot.generation = 1
	}
	slot.alive = true
	slot.bodyType = def.Type
	slot.position = def.Position
	slot.rotation = def.Rotation
	slot.linearVelocity = def.LinearVelocity
	slot.angularVelocity = def.AngularVelocity
	slot.force = Vec2Zero
	slot.torque = 0
	slot.linearDamping = def.LinearDamping
	slot.angularDamping = def.AngularDamping
	slot.gravityScale = def.GravityScale
	slot.sleepThreshold = def.SleepThreshold
	slot.name = def.Name
	slot.userData = def.UserData
	slot.motionLocks = def.MotionLocks
	slot.lockFlags = motionLockFlags(def.MotionLocks)
	slot.enableSleep = def.EnableSleep
	slot.isAwake = def.IsAwake
	slot.isBullet = def.IsBullet
	slot.isEnabled = def.IsEnabled
	slot.allowFastRotation = def.AllowFastRotation
	slot.sleepTime = 0
	slot.shapeIndices = slot.shapeIndices[:0]
	w.updateBodyMassLocked(idx)

	id := BodyID{
		Index1:     int32(idx + 1),
		World0:     w.id.Index1 - 1,
		Generation: slot.generation,
	}
	w.bodyCount++
	return &Body{world: w, id: id}, nil
}

func (w *World) isBodyIDValidLocked(id BodyID) bool {
	idx := int(id.Index1) - 1
	if idx < 0 || idx >= len(w.bodies) {
		return false
	}
	slot := w.bodies[idx]
	return slot.alive && slot.generation == id.Generation
}

func (w *World) destroyBodyIDLocked(id BodyID) error {
	if !w.isBodyIDValidLocked(id) {
		return ErrBodyInvalid
	}

	idx := int(id.Index1) - 1
	slot := &w.bodies[idx]

	shapeIndices := append([]int(nil), slot.shapeIndices...)
	for _, shapeIdx := range shapeIndices {
		if shapeIdx < 0 || shapeIdx >= len(w.shapes) {
			continue
		}
		shapeSlot := w.shapes[shapeIdx]
		if !shapeSlot.alive {
			continue
		}
		shapeID := ShapeID{
			Index1:     int32(shapeIdx + 1),
			World0:     w.id.Index1 - 1,
			Generation: shapeSlot.generation,
		}
		_ = w.destroyShapeIDLocked(shapeID, false)
	}
	slot.shapeIndices = slot.shapeIndices[:0]

	for chainIdx := range w.chains {
		chain := w.chains[chainIdx]
		if !chain.alive {
			continue
		}
		if chain.bodyIndex == idx {
			w.destroyChainIndexLocked(chainIdx, false)
		}
	}

	for jointIdx := range w.joints {
		joint := w.joints[jointIdx]
		if !joint.alive {
			continue
		}
		if joint.bodyAIndex == idx || joint.bodyBIndex == idx {
			w.destroyJointIndexLocked(jointIdx)
		}
	}

	slot.alive = false
	slot.generation++
	if slot.generation == 0 {
		slot.generation = 1
	}
	slot.userData = nil
	slot.name = ""
	w.freeBodyIndices = append(w.freeBodyIndices, idx)
	w.bodyCount--
	return nil
}

// DestroyBody destroys a body by id.
func (w *World) DestroyBody(id BodyID) error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	return w.destroyBodyIDLocked(id)
}

func worldFromIndex(index1 uint16) *World {
	if index1 == 0 {
		return nil
	}
	index := int(index1) - 1
	if index < 0 || index >= MaxWorlds {
		return nil
	}

	worldRegistryMu.Lock()
	defer worldRegistryMu.Unlock()
	return worldRegistry[index].world
}

// ID returns the body handle.
func (b *Body) ID() BodyID {
	if b == nil {
		return NullBodyID
	}
	return b.id
}

// IsValid reports whether this body handle still points to a live body.
func (b *Body) IsValid() bool {
	if b == nil || b.world == nil || b.id.IsNull() {
		return false
	}
	return IsBodyIDValid(b.id)
}

// Destroy destroys this body.
func (b *Body) Destroy() error {
	if b == nil || b.world == nil || b.id.IsNull() {
		return ErrBodyInvalid
	}
	err := b.world.DestroyBody(b.id)
	if err != nil {
		return err
	}
	b.id = NullBodyID
	return nil
}

func (b *Body) GetType() (BodyType, error) {
	if !b.IsValid() {
		return StaticBody, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return StaticBody, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return slot.bodyType, nil
}

func (b *Body) SetType(bodyType BodyType) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	slot.bodyType = bodyType
	b.world.updateBodyMassLocked(int(b.id.Index1) - 1)
	return nil
}

func (b *Body) GetPosition() (Vec2, error) {
	if !b.IsValid() {
		return Vec2{}, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return Vec2{}, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return slot.position, nil
}

func (b *Body) GetRotation() (Rot, error) {
	if !b.IsValid() {
		return RotIdentity, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return RotIdentity, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return slot.rotation, nil
}

func (b *Body) GetTransform() (Transform, error) {
	if !b.IsValid() {
		return TransformIdentity, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return TransformIdentity, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return Transform{P: slot.position, Q: slot.rotation}, nil
}

func (b *Body) GetLocalPoint(worldPoint Vec2) (Vec2, error) {
	if !b.IsValid() {
		return Vec2Zero, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return Vec2Zero, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	transform := Transform{P: slot.position, Q: slot.rotation}
	return InvTransformPoint(transform, worldPoint), nil
}

func (b *Body) GetWorldPoint(localPoint Vec2) (Vec2, error) {
	if !b.IsValid() {
		return Vec2Zero, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return Vec2Zero, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	transform := Transform{P: slot.position, Q: slot.rotation}
	return TransformPoint(transform, localPoint), nil
}

func (b *Body) GetLocalVector(worldVector Vec2) (Vec2, error) {
	if !b.IsValid() {
		return Vec2Zero, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return Vec2Zero, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return InvRotateVector(slot.rotation, worldVector), nil
}

func (b *Body) GetWorldVector(localVector Vec2) (Vec2, error) {
	if !b.IsValid() {
		return Vec2Zero, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return Vec2Zero, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return RotateVector(slot.rotation, localVector), nil
}

func (b *Body) SetTransform(position Vec2, rotation Rot) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	slot.position = position
	slot.rotation = rotation
	return nil
}

func (b *Body) SetLinearVelocity(v Vec2) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	slot.linearVelocity = v
	return nil
}

func (b *Body) GetLinearVelocity() (Vec2, error) {
	if !b.IsValid() {
		return Vec2{}, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return Vec2{}, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return slot.linearVelocity, nil
}

func (b *Body) SetAngularVelocity(w float32) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	slot.angularVelocity = w
	return nil
}

func (b *Body) SetTargetTransform(target Transform, timeStep float32, wake bool) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]

	if !slot.isEnabled {
		return nil
	}
	if slot.bodyType == StaticBody || timeStep <= 0 {
		return nil
	}
	if !slot.isAwake && !wake {
		return nil
	}

	center1 := Add(slot.position, RotateVector(slot.rotation, slot.localCenter))
	center2 := TransformPoint(target, slot.localCenter)
	invTimeStep := float32(1.0) / timeStep
	linearVelocity := MulSV(invTimeStep, Sub(center2, center1))
	angularVelocity := invTimeStep * RelativeAngle(slot.rotation, target.Q)

	if !slot.isAwake {
		bodyExtent := b.world.computeBodyExtentLocked(int(b.id.Index1)-1, slot.localCenter)
		maxVelocity := Length(linearVelocity) + bodyExtent*AbsFloat(angularVelocity)
		if maxVelocity < slot.sleepThreshold {
			return nil
		}
		slot.isAwake = true
	}

	slot.linearVelocity = linearVelocity
	slot.angularVelocity = angularVelocity
	return nil
}

func (w *World) computeBodyExtentLocked(bodyIdx int, localCenter Vec2) float32 {
	if bodyIdx < 0 || bodyIdx >= len(w.bodies) {
		return 0
	}
	body := w.bodies[bodyIdx]
	maxExtent := float32(0)
	for _, shapeIdx := range body.shapeIndices {
		if shapeIdx < 0 || shapeIdx >= len(w.shapes) {
			continue
		}
		shape := w.shapes[shapeIdx]
		if !shape.alive {
			continue
		}
		shapeExtent := float32(0)
		switch shape.shapeType {
		case CircleShape:
			shapeExtent = Distance(shape.circle.Center, localCenter) + shape.circle.Radius
		case CapsuleShape:
			dist1 := Distance(shape.capsule.Center1, localCenter)
			dist2 := Distance(shape.capsule.Center2, localCenter)
			shapeExtent = MaxFloat(dist1, dist2) + shape.capsule.Radius
		case SegmentShape, ChainSegmentShape:
			dist1 := Distance(shape.segment.Point1, localCenter)
			dist2 := Distance(shape.segment.Point2, localCenter)
			shapeExtent = MaxFloat(dist1, dist2)
		case PolygonShape:
			for i := 0; i < shape.polygon.Count; i++ {
				vertexExtent := Distance(shape.polygon.Vertices[i], localCenter)
				if vertexExtent > shapeExtent {
					shapeExtent = vertexExtent
				}
			}
			shapeExtent += shape.polygon.Radius
		default:
			continue
		}
		if shapeExtent > maxExtent {
			maxExtent = shapeExtent
		}
	}
	return maxExtent
}

func (b *Body) GetAngularVelocity() (float32, error) {
	if !b.IsValid() {
		return 0, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return 0, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return slot.angularVelocity, nil
}

func (b *Body) SetLinearDamping(linearDamping float32) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	slot.linearDamping = linearDamping
	return nil
}

func (b *Body) GetLinearDamping() (float32, error) {
	if !b.IsValid() {
		return 0, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return 0, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return slot.linearDamping, nil
}

func (b *Body) SetAngularDamping(angularDamping float32) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	slot.angularDamping = angularDamping
	return nil
}

func (b *Body) GetAngularDamping() (float32, error) {
	if !b.IsValid() {
		return 0, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return 0, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return slot.angularDamping, nil
}

func (b *Body) SetGravityScale(gravityScale float32) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	slot.gravityScale = gravityScale
	return nil
}

func (b *Body) GetGravityScale() (float32, error) {
	if !b.IsValid() {
		return 0, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return 0, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return slot.gravityScale, nil
}

func (b *Body) IsAwake() (bool, error) {
	if !b.IsValid() {
		return false, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return false, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return slot.isAwake, nil
}

func (b *Body) SetAwake(awake bool) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	slot.isAwake = awake
	if awake {
		slot.sleepTime = 0
	}
	return nil
}

func (b *Body) WakeTouching() error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	b.world.refreshContactAndSensorStateLocked()
	bodyIdx := int(b.id.Index1) - 1
	for _, pair := range b.world.sortedActiveContactPairsLocked() {
		shapeA := b.world.shapes[pair.a]
		shapeB := b.world.shapes[pair.b]
		var otherBodyIdx int = -1
		switch {
		case shapeA.bodyIndex == bodyIdx:
			otherBodyIdx = shapeB.bodyIndex
		case shapeB.bodyIndex == bodyIdx:
			otherBodyIdx = shapeA.bodyIndex
		default:
			continue
		}
		if otherBodyIdx < 0 || otherBodyIdx >= len(b.world.bodies) {
			continue
		}
		otherBody := &b.world.bodies[otherBodyIdx]
		if !otherBody.alive || !otherBody.isEnabled {
			continue
		}
		otherBody.isAwake = true
		otherBody.sleepTime = 0
	}
	return nil
}

func (b *Body) EnableSleep(enableSleep bool) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	slot.enableSleep = enableSleep
	if !enableSleep {
		slot.isAwake = true
		slot.sleepTime = 0
	}
	return nil
}

func (b *Body) IsSleepEnabled() (bool, error) {
	if !b.IsValid() {
		return false, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return false, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return slot.enableSleep, nil
}

func (b *Body) SetSleepThreshold(sleepThreshold float32) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	slot.sleepThreshold = sleepThreshold
	return nil
}

func (b *Body) GetSleepThreshold() (float32, error) {
	if !b.IsValid() {
		return 0, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return 0, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return slot.sleepThreshold, nil
}

func (b *Body) IsEnabled() (bool, error) {
	if !b.IsValid() {
		return false, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return false, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return slot.isEnabled, nil
}

func (b *Body) Disable() error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	slot.isEnabled = false
	slot.isAwake = false
	slot.force = Vec2Zero
	slot.torque = 0
	return nil
}

func (b *Body) Enable() error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	slot.isEnabled = true
	slot.isAwake = true
	return nil
}

func (b *Body) SetMotionLocks(locks MotionLocks) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	slot.motionLocks = locks
	slot.lockFlags = motionLockFlags(locks)
	return nil
}

func (b *Body) GetMotionLocks() (MotionLocks, error) {
	if !b.IsValid() {
		return MotionLocks{}, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return MotionLocks{}, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return slot.motionLocks, nil
}

func (b *Body) SetBullet(flag bool) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	slot.isBullet = flag
	return nil
}

func (b *Body) IsBullet() (bool, error) {
	if !b.IsValid() {
		return false, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return false, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return slot.isBullet, nil
}

func (b *Body) GetMass() (float32, error) {
	if !b.IsValid() {
		return 0, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return 0, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return slot.mass, nil
}

func (b *Body) GetRotationalInertia() (float32, error) {
	if !b.IsValid() {
		return 0, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return 0, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return slot.inertia, nil
}

func (b *Body) SetMassData(massData MassData) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	if !IsValidFloat(massData.Mass) || massData.Mass < 0 {
		return ErrInvalidMassData
	}
	if !IsValidFloat(massData.RotationalInertia) || massData.RotationalInertia < 0 {
		return ErrInvalidMassData
	}
	if !IsValidVec2(massData.Center) {
		return ErrInvalidMassData
	}

	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}

	slot := &b.world.bodies[int(b.id.Index1)-1]
	slot.mass = massData.Mass
	slot.inertia = massData.RotationalInertia
	slot.localCenter = massData.Center

	if slot.bodyType != DynamicBody {
		slot.invMass = 0
		slot.invInertia = 0
		return nil
	}

	if slot.mass > 0 {
		slot.invMass = 1.0 / slot.mass
	} else {
		slot.invMass = 0
	}
	if slot.inertia > 0 {
		slot.invInertia = 1.0 / slot.inertia
	} else {
		slot.invInertia = 0
	}
	return nil
}

func (b *Body) GetMassData() (MassData, error) {
	if !b.IsValid() {
		return MassData{}, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return MassData{}, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return MassData{
		Mass:              slot.mass,
		Center:            slot.localCenter,
		RotationalInertia: slot.inertia,
	}, nil
}

func (b *Body) GetLocalCenterOfMass() (Vec2, error) {
	if !b.IsValid() {
		return Vec2Zero, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return Vec2Zero, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return slot.localCenter, nil
}

func (b *Body) GetWorldCenterOfMass() (Vec2, error) {
	if !b.IsValid() {
		return Vec2Zero, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return Vec2Zero, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return Add(slot.position, RotateVector(slot.rotation, slot.localCenter)), nil
}

func (b *Body) GetLocalPointVelocity(localPoint Vec2) (Vec2, error) {
	if !b.IsValid() {
		return Vec2Zero, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return Vec2Zero, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	r := RotateVector(slot.rotation, Sub(localPoint, slot.localCenter))
	return Add(slot.linearVelocity, CrossSV(slot.angularVelocity, r)), nil
}

func (b *Body) GetWorldPointVelocity(worldPoint Vec2) (Vec2, error) {
	if !b.IsValid() {
		return Vec2Zero, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return Vec2Zero, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	center := Add(slot.position, RotateVector(slot.rotation, slot.localCenter))
	r := Sub(worldPoint, center)
	return Add(slot.linearVelocity, CrossSV(slot.angularVelocity, r)), nil
}

func (b *Body) ApplyMassFromShapes() error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	b.world.updateBodyMassLocked(int(b.id.Index1) - 1)
	return nil
}

func (b *Body) ApplyLinearImpulse(impulse Vec2, point Vec2, wake bool) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	if slot.bodyType != DynamicBody || !slot.isEnabled {
		return nil
	}
	if wake {
		slot.isAwake = true
	}
	if !slot.isAwake {
		return nil
	}
	slot.linearVelocity = MulAdd(slot.linearVelocity, slot.invMass, impulse)
	center := Add(slot.position, RotateVector(slot.rotation, slot.localCenter))
	slot.angularVelocity += slot.invInertia * Cross(Sub(point, center), impulse)
	return nil
}

func (b *Body) ApplyLinearImpulseToCenter(impulse Vec2, wake bool) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	if slot.bodyType != DynamicBody || !slot.isEnabled {
		return nil
	}
	if wake {
		slot.isAwake = true
	}
	if !slot.isAwake {
		return nil
	}
	slot.linearVelocity = MulAdd(slot.linearVelocity, slot.invMass, impulse)
	return nil
}

func (b *Body) ApplyAngularImpulse(impulse float32, wake bool) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	if slot.bodyType != DynamicBody || !slot.isEnabled {
		return nil
	}
	if wake {
		slot.isAwake = true
	}
	if !slot.isAwake {
		return nil
	}
	slot.angularVelocity += slot.invInertia * impulse
	return nil
}

func (b *Body) ApplyForce(force Vec2, point Vec2, wake bool) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	if slot.bodyType != DynamicBody || !slot.isEnabled {
		return nil
	}
	if wake {
		slot.isAwake = true
	}
	if !slot.isAwake {
		return nil
	}
	slot.force = Add(slot.force, force)
	center := Add(slot.position, RotateVector(slot.rotation, slot.localCenter))
	r := Sub(point, center)
	slot.torque += Cross(r, force)
	return nil
}

func (b *Body) ApplyForceToCenter(force Vec2, wake bool) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	if slot.bodyType != DynamicBody || !slot.isEnabled {
		return nil
	}
	if wake {
		slot.isAwake = true
	}
	if !slot.isAwake {
		return nil
	}
	slot.force = Add(slot.force, force)
	return nil
}

func (b *Body) ApplyTorque(torque float32, wake bool) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	if slot.bodyType != DynamicBody || !slot.isEnabled {
		return nil
	}
	if wake {
		slot.isAwake = true
	}
	if !slot.isAwake {
		return nil
	}
	slot.torque += torque
	return nil
}

func (b *Body) ClearForces() error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	slot.force = Vec2Zero
	slot.torque = 0
	return nil
}

func (b *Body) SetUserData(v any) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	slot.userData = v
	return nil
}

func (b *Body) UserData() (any, error) {
	if !b.IsValid() {
		return nil, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return nil, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return slot.userData, nil
}

func (b *Body) GetUserData() (any, error) {
	return b.UserData()
}

func (b *Body) SetName(name string) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	slot.name = name
	return nil
}

func (b *Body) Name() (string, error) {
	if !b.IsValid() {
		return "", ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return "", ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return slot.name, nil
}

func (b *Body) GetName() (string, error) {
	return b.Name()
}

func (b *Body) ShapeCount() (int, error) {
	if !b.IsValid() {
		return 0, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return 0, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	return len(slot.shapeIndices), nil
}

func (b *Body) GetShapeCount() (int, error) {
	return b.ShapeCount()
}

func (b *Body) GetShapes() ([]ShapeID, error) {
	if !b.IsValid() {
		return nil, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return nil, ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	shapeIDs := make([]ShapeID, 0, len(slot.shapeIndices))
	for _, shapeIdx := range slot.shapeIndices {
		if shapeIdx < 0 || shapeIdx >= len(b.world.shapes) {
			continue
		}
		shapeSlot := b.world.shapes[shapeIdx]
		if !shapeSlot.alive {
			continue
		}
		shapeIDs = append(shapeIDs, ShapeID{
			Index1:     int32(shapeIdx + 1),
			World0:     b.id.World0,
			Generation: shapeSlot.generation,
		})
	}
	return shapeIDs, nil
}

func (b *Body) ComputeAABB() (AABB, error) {
	if !b.IsValid() {
		return AABB{}, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return AABB{}, ErrBodyInvalid
	}

	slot := &b.world.bodies[int(b.id.Index1)-1]
	if len(slot.shapeIndices) == 0 {
		return AABB{LowerBound: slot.position, UpperBound: slot.position}, nil
	}

	transform := Transform{P: slot.position, Q: slot.rotation}
	found := false
	var aabb AABB
	for _, shapeIdx := range slot.shapeIndices {
		if shapeIdx < 0 || shapeIdx >= len(b.world.shapes) {
			continue
		}
		shapeSlot := b.world.shapes[shapeIdx]
		if !shapeSlot.alive {
			continue
		}

		var shapeAABB AABB
		switch shapeSlot.shapeType {
		case CircleShape:
			shapeAABB = ComputeCircleAABB(&shapeSlot.circle, transform)
		case CapsuleShape:
			shapeAABB = ComputeCapsuleAABB(&shapeSlot.capsule, transform)
		case PolygonShape:
			shapeAABB = ComputePolygonAABB(&shapeSlot.polygon, transform)
		case SegmentShape, ChainSegmentShape:
			shapeAABB = ComputeSegmentAABB(&shapeSlot.segment, transform)
		default:
			continue
		}

		if !found {
			aabb = shapeAABB
			found = true
			continue
		}
		aabb.LowerBound = Min(aabb.LowerBound, shapeAABB.LowerBound)
		aabb.UpperBound = Max(aabb.UpperBound, shapeAABB.UpperBound)
	}

	if !found {
		return AABB{LowerBound: slot.position, UpperBound: slot.position}, nil
	}
	return aabb, nil
}

func (b *Body) GetJointCount() (int, error) {
	if !b.IsValid() {
		return 0, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return 0, ErrBodyInvalid
	}
	bodyIdx := int(b.id.Index1) - 1
	count := 0
	for _, joint := range b.world.joints {
		if !joint.alive {
			continue
		}
		if joint.bodyAIndex == bodyIdx || joint.bodyBIndex == bodyIdx {
			count++
		}
	}
	return count, nil
}

func (b *Body) GetJoints() ([]JointID, error) {
	if !b.IsValid() {
		return nil, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return nil, ErrBodyInvalid
	}
	bodyIdx := int(b.id.Index1) - 1
	joints := make([]JointID, 0)
	for jointIdx, joint := range b.world.joints {
		if !joint.alive {
			continue
		}
		if joint.bodyAIndex != bodyIdx && joint.bodyBIndex != bodyIdx {
			continue
		}
		joints = append(joints, JointID{
			Index1:     int32(jointIdx + 1),
			World0:     b.id.World0,
			Generation: joint.generation,
		})
	}
	return joints, nil
}

func (b *Body) GetContactCapacity() (int, error) {
	if !b.IsValid() {
		return 0, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return 0, ErrBodyInvalid
	}
	b.world.refreshContactAndSensorStateLocked()
	bodyIdx := int(b.id.Index1) - 1
	count := 0
	for _, pair := range b.world.sortedActiveContactPairsLocked() {
		shapeA := b.world.shapes[pair.a]
		shapeB := b.world.shapes[pair.b]
		if shapeA.bodyIndex == bodyIdx || shapeB.bodyIndex == bodyIdx {
			count++
		}
	}
	return count, nil
}

func (b *Body) GetContactData(capacity int) ([]ContactData, error) {
	if !b.IsValid() {
		return nil, ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return nil, ErrBodyInvalid
	}
	b.world.refreshContactAndSensorStateLocked()
	if capacity <= 0 {
		return []ContactData{}, nil
	}
	bodyIdx := int(b.id.Index1) - 1
	limit := capacity
	if len(b.world.activeContactPairs) < limit {
		limit = len(b.world.activeContactPairs)
	}
	data := make([]ContactData, 0, limit)
	for _, pair := range b.world.sortedActiveContactPairsLocked() {
		shapeA := b.world.shapes[pair.a]
		shapeB := b.world.shapes[pair.b]
		if shapeA.bodyIndex != bodyIdx && shapeB.bodyIndex != bodyIdx {
			continue
		}
		data = append(data, b.world.contactDataFromPairLocked(pair))
		if len(data) == capacity {
			break
		}
	}
	return data, nil
}

func (b *Body) GetWorld() (WorldID, error) {
	if !b.IsValid() {
		return NullWorldID, ErrBodyInvalid
	}
	return b.world.ID(), nil
}

func (b *Body) EnableContactEvents(flag bool) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	for _, shapeIdx := range slot.shapeIndices {
		if shapeIdx < 0 || shapeIdx >= len(b.world.shapes) {
			continue
		}
		shapeSlot := &b.world.shapes[shapeIdx]
		if !shapeSlot.alive {
			continue
		}
		shapeSlot.enableContactEvents = flag
	}
	return nil
}

func (b *Body) CreateChain(def ChainDef) (*Chain, error) {
	if !b.IsValid() {
		return nil, ErrBodyInvalid
	}
	return b.world.CreateChainObject(b.id, def)
}

func (b *Body) EnableHitEvents(flag bool) error {
	if !b.IsValid() {
		return ErrBodyInvalid
	}
	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return ErrBodyInvalid
	}
	slot := &b.world.bodies[int(b.id.Index1)-1]
	for _, shapeIdx := range slot.shapeIndices {
		if shapeIdx < 0 || shapeIdx >= len(b.world.shapes) {
			continue
		}
		shapeSlot := &b.world.shapes[shapeIdx]
		if !shapeSlot.alive {
			continue
		}
		shapeSlot.enableHitEvents = flag
	}
	return nil
}
