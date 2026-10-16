package box2d

import (
	"errors"
)

var (
	ErrInvalidShapeDef = errors.New("box2d: invalid shape definition (use DefaultShapeDef)")
	ErrShapeInvalid    = errors.New("box2d: shape is invalid")
)

type shapeState struct {
	generation            uint16
	alive                 bool
	shapeType             ShapeType
	bodyIndex             int
	parentChainIndex      int
	userData              any
	material              SurfaceMaterial
	density               float32
	filter                Filter
	enableCustomFiltering bool
	isSensor              bool
	enableSensorEvents    bool
	enableContactEvents   bool
	enableHitEvents       bool
	enablePreSolveEvents  bool
	invokeContactCreation bool
	updateBodyMass        bool
	circle                Circle
	capsule               Capsule
	segment               Segment
	chainGhost1           Vec2
	chainGhost2           Vec2
	polygon               Polygon
}

type Shape struct {
	world *World
	id    ShapeID
}

func IsShapeIDValid(id ShapeID) bool {
	if id.IsNull() {
		return false
	}
	world := worldFromIndex(id.World0 + 1)
	if world == nil {
		return false
	}
	world.mu.RLock()
	defer world.mu.RUnlock()
	return world.isShapeIDValidLocked(id)
}

func (w *World) isShapeIDValidLocked(id ShapeID) bool {
	idx := int(id.Index1) - 1
	if idx < 0 || idx >= len(w.shapes) {
		return false
	}
	slot := w.shapes[idx]
	return slot.alive && slot.generation == id.Generation
}

func (w *World) allocShapeSlotLocked() int {
	if n := len(w.freeShapeIndices); n > 0 {
		idx := w.freeShapeIndices[n-1]
		w.freeShapeIndices = w.freeShapeIndices[:n-1]
		return idx
	}
	idx := len(w.shapes)
	w.shapes = append(w.shapes, shapeState{})
	return idx
}

func (w *World) createShapeLocked(bodyID BodyID, def ShapeDef, shapeType ShapeType) (int, error) {
	if !w.isBodyIDValidLocked(bodyID) {
		return -1, ErrBodyInvalid
	}

	shapeIdx := w.allocShapeSlotLocked()
	slot := &w.shapes[shapeIdx]
	slot.generation++
	if slot.generation == 0 {
		slot.generation = 1
	}
	slot.alive = true
	slot.shapeType = shapeType
	slot.bodyIndex = int(bodyID.Index1) - 1
	slot.parentChainIndex = -1
	slot.userData = def.UserData
	slot.material = def.Material
	slot.density = def.Density
	slot.filter = def.Filter
	slot.enableCustomFiltering = def.EnableCustomFiltering
	slot.isSensor = def.IsSensor
	slot.enableSensorEvents = def.EnableSensorEvents
	slot.enableContactEvents = def.EnableContactEvents
	slot.enableHitEvents = def.EnableHitEvents
	slot.enablePreSolveEvents = def.EnablePreSolveEvents
	slot.invokeContactCreation = def.InvokeContactCreation
	slot.updateBodyMass = def.UpdateBodyMass
	slot.chainGhost1 = Vec2Zero
	slot.chainGhost2 = Vec2Zero

	bodySlot := &w.bodies[int(bodyID.Index1)-1]
	bodySlot.shapeIndices = append(bodySlot.shapeIndices, shapeIdx)
	w.shapeCount++
	return shapeIdx, nil
}

func (b *Body) CreateCircleShape(def ShapeDef, circle Circle) (*Shape, error) {
	if !b.IsValid() {
		return nil, ErrBodyInvalid
	}
	if !def.isInitialized() {
		return nil, ErrInvalidShapeDef
	}

	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return nil, ErrBodyInvalid
	}

	idx, err := b.world.createShapeLocked(b.id, def, CircleShape)
	if err != nil {
		return nil, err
	}
	b.world.shapes[idx].circle = circle
	if def.UpdateBodyMass {
		b.world.updateBodyMassLocked(int(b.id.Index1) - 1)
	}

	id := ShapeID{
		Index1:     int32(idx + 1),
		World0:     b.world.id.Index1 - 1,
		Generation: b.world.shapes[idx].generation,
	}
	return &Shape{world: b.world, id: id}, nil
}

func (b *Body) CreateCapsuleShape(def ShapeDef, capsule Capsule) (*Shape, error) {
	if !b.IsValid() {
		return nil, ErrBodyInvalid
	}
	if !def.isInitialized() {
		return nil, ErrInvalidShapeDef
	}

	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return nil, ErrBodyInvalid
	}

	idx, err := b.world.createShapeLocked(b.id, def, CapsuleShape)
	if err != nil {
		return nil, err
	}
	b.world.shapes[idx].capsule = capsule
	if def.UpdateBodyMass {
		b.world.updateBodyMassLocked(int(b.id.Index1) - 1)
	}

	id := ShapeID{
		Index1:     int32(idx + 1),
		World0:     b.world.id.Index1 - 1,
		Generation: b.world.shapes[idx].generation,
	}
	return &Shape{world: b.world, id: id}, nil
}

func (b *Body) CreateSegmentShape(def ShapeDef, segment Segment) (*Shape, error) {
	if !b.IsValid() {
		return nil, ErrBodyInvalid
	}
	if !def.isInitialized() {
		return nil, ErrInvalidShapeDef
	}

	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return nil, ErrBodyInvalid
	}

	idx, err := b.world.createShapeLocked(b.id, def, SegmentShape)
	if err != nil {
		return nil, err
	}
	b.world.shapes[idx].segment = segment
	if def.UpdateBodyMass {
		b.world.updateBodyMassLocked(int(b.id.Index1) - 1)
	}

	id := ShapeID{
		Index1:     int32(idx + 1),
		World0:     b.world.id.Index1 - 1,
		Generation: b.world.shapes[idx].generation,
	}
	return &Shape{world: b.world, id: id}, nil
}

func (b *Body) CreatePolygonShape(def ShapeDef, polygon Polygon) (*Shape, error) {
	if !b.IsValid() {
		return nil, ErrBodyInvalid
	}
	if !def.isInitialized() {
		return nil, ErrInvalidShapeDef
	}

	b.world.mu.Lock()
	defer b.world.mu.Unlock()
	if !b.world.isBodyIDValidLocked(b.id) {
		return nil, ErrBodyInvalid
	}

	idx, err := b.world.createShapeLocked(b.id, def, PolygonShape)
	if err != nil {
		return nil, err
	}
	b.world.shapes[idx].polygon = polygon
	if def.UpdateBodyMass {
		b.world.updateBodyMassLocked(int(b.id.Index1) - 1)
	}

	id := ShapeID{
		Index1:     int32(idx + 1),
		World0:     b.world.id.Index1 - 1,
		Generation: b.world.shapes[idx].generation,
	}
	return &Shape{world: b.world, id: id}, nil
}

func (w *World) destroyShapeIDLocked(id ShapeID, updateBodyMass bool) error {
	if !w.isShapeIDValidLocked(id) {
		return ErrShapeInvalid
	}

	shapeIdx := int(id.Index1) - 1
	slot := &w.shapes[shapeIdx]
	if slot.parentChainIndex >= 0 && slot.parentChainIndex < len(w.chains) {
		chain := &w.chains[slot.parentChainIndex]
		for i, si := range chain.shapeIndices {
			if si == shapeIdx {
				chain.shapeIndices[i] = -1
				break
			}
		}
	}
	bodyIdx := slot.bodyIndex
	if bodyIdx >= 0 && bodyIdx < len(w.bodies) {
		bodySlot := &w.bodies[bodyIdx]
		for i, si := range bodySlot.shapeIndices {
			if si == shapeIdx {
				bodySlot.shapeIndices = append(bodySlot.shapeIndices[:i], bodySlot.shapeIndices[i+1:]...)
				break
			}
		}
	}

	slot.alive = false
	slot.generation++
	if slot.generation == 0 {
		slot.generation = 1
	}
	slot.userData = nil
	slot.parentChainIndex = -1
	w.freeShapeIndices = append(w.freeShapeIndices, shapeIdx)
	w.shapeCount--
	if updateBodyMass {
		w.updateBodyMassLocked(bodyIdx)
	}
	return nil
}

func (w *World) DestroyShape(id ShapeID, updateBodyMass bool) error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	return w.destroyShapeIDLocked(id, updateBodyMass)
}

func (s *Shape) ID() ShapeID {
	if s == nil {
		return NullShapeID
	}
	return s.id
}

func (s *Shape) IsValid() bool {
	if s == nil || s.world == nil || s.id.IsNull() {
		return false
	}
	return IsShapeIDValid(s.id)
}

func (s *Shape) Destroy(updateBodyMass bool) error {
	if !s.IsValid() {
		return ErrShapeInvalid
	}
	err := s.world.DestroyShape(s.id, updateBodyMass)
	if err != nil {
		return err
	}
	s.id = NullShapeID
	return nil
}

func (s *Shape) withSlot(mut func(*shapeState) error) error {
	if !s.IsValid() {
		return ErrShapeInvalid
	}
	s.world.mu.Lock()
	defer s.world.mu.Unlock()
	if !s.world.isShapeIDValidLocked(s.id) {
		return ErrShapeInvalid
	}
	slot := &s.world.shapes[int(s.id.Index1)-1]
	return mut(slot)
}

func (s *Shape) withSlotRead(read func(shapeState) error) error {
	if !s.IsValid() {
		return ErrShapeInvalid
	}
	s.world.mu.Lock()
	defer s.world.mu.Unlock()
	if !s.world.isShapeIDValidLocked(s.id) {
		return ErrShapeInvalid
	}
	slot := s.world.shapes[int(s.id.Index1)-1]
	return read(slot)
}

func (s *Shape) GetType() (ShapeType, error) {
	var shapeType ShapeType
	err := s.withSlotRead(func(slot shapeState) error {
		shapeType = slot.shapeType
		return nil
	})
	return shapeType, err
}

func (s *Shape) IsSensor() (bool, error) {
	var isSensor bool
	err := s.withSlotRead(func(slot shapeState) error {
		isSensor = slot.isSensor
		return nil
	})
	return isSensor, err
}

func (s *Shape) GetBody() (BodyID, error) {
	var bodyID BodyID
	err := s.withSlotRead(func(slot shapeState) error {
		bodyIdx := slot.bodyIndex
		bodySlot := s.world.bodies[bodyIdx]
		bodyID = BodyID{
			Index1:     int32(bodyIdx + 1),
			World0:     s.id.World0,
			Generation: bodySlot.generation,
		}
		return nil
	})
	return bodyID, err
}

func (s *Shape) GetWorld() (WorldID, error) {
	if !s.IsValid() {
		return NullWorldID, ErrShapeInvalid
	}
	return s.world.ID(), nil
}

func (s *Shape) SetUserData(v any) error {
	return s.withSlot(func(slot *shapeState) error {
		slot.userData = v
		return nil
	})
}

func (s *Shape) UserData() (any, error) {
	var v any
	err := s.withSlotRead(func(slot shapeState) error {
		v = slot.userData
		return nil
	})
	return v, err
}

func (s *Shape) SetDensity(density float32, updateBodyMass bool) error {
	if !s.IsValid() {
		return ErrShapeInvalid
	}
	s.world.mu.Lock()
	defer s.world.mu.Unlock()
	if !s.world.isShapeIDValidLocked(s.id) {
		return ErrShapeInvalid
	}
	slot := &s.world.shapes[int(s.id.Index1)-1]
	slot.density = density
	if updateBodyMass {
		s.world.updateBodyMassLocked(slot.bodyIndex)
	}
	return nil
}

func (s *Shape) Density() (float32, error) {
	var density float32
	err := s.withSlotRead(func(slot shapeState) error {
		density = slot.density
		return nil
	})
	return density, err
}

func (s *Shape) SetFriction(friction float32) error {
	return s.withSlot(func(slot *shapeState) error {
		slot.material.Friction = friction
		return nil
	})
}

func (s *Shape) Friction() (float32, error) {
	var friction float32
	err := s.withSlotRead(func(slot shapeState) error {
		friction = slot.material.Friction
		return nil
	})
	return friction, err
}

func (s *Shape) SetRestitution(restitution float32) error {
	return s.withSlot(func(slot *shapeState) error {
		slot.material.Restitution = restitution
		return nil
	})
}

func (s *Shape) Restitution() (float32, error) {
	var restitution float32
	err := s.withSlotRead(func(slot shapeState) error {
		restitution = slot.material.Restitution
		return nil
	})
	return restitution, err
}

func (s *Shape) SetUserMaterial(material uint64) error {
	return s.withSlot(func(slot *shapeState) error {
		slot.material.UserMaterialID = material
		return nil
	})
}

func (s *Shape) UserMaterial() (uint64, error) {
	var material uint64
	err := s.withSlotRead(func(slot shapeState) error {
		material = slot.material.UserMaterialID
		return nil
	})
	return material, err
}

func (s *Shape) SetSurfaceMaterial(material SurfaceMaterial) error {
	return s.withSlot(func(slot *shapeState) error {
		slot.material = material
		return nil
	})
}

func (s *Shape) SurfaceMaterial() (SurfaceMaterial, error) {
	var material SurfaceMaterial
	err := s.withSlotRead(func(slot shapeState) error {
		material = slot.material
		return nil
	})
	return material, err
}

func (s *Shape) Filter() (Filter, error) {
	var filter Filter
	err := s.withSlotRead(func(slot shapeState) error {
		filter = slot.filter
		return nil
	})
	return filter, err
}

func (s *Shape) SetFilter(filter Filter) error {
	return s.withSlot(func(slot *shapeState) error {
		slot.filter = filter
		return nil
	})
}

func (s *Shape) EnableSensorEvents(flag bool) error {
	return s.withSlot(func(slot *shapeState) error {
		slot.enableSensorEvents = flag
		return nil
	})
}

func (s *Shape) SensorEventsEnabled() (bool, error) {
	var v bool
	err := s.withSlotRead(func(slot shapeState) error {
		v = slot.enableSensorEvents
		return nil
	})
	return v, err
}

func (s *Shape) EnableContactEvents(flag bool) error {
	return s.withSlot(func(slot *shapeState) error {
		slot.enableContactEvents = flag
		return nil
	})
}

func (s *Shape) ContactEventsEnabled() (bool, error) {
	var v bool
	err := s.withSlotRead(func(slot shapeState) error {
		v = slot.enableContactEvents
		return nil
	})
	return v, err
}

func (s *Shape) EnablePreSolveEvents(flag bool) error {
	return s.withSlot(func(slot *shapeState) error {
		slot.enablePreSolveEvents = flag
		return nil
	})
}

func (s *Shape) PreSolveEventsEnabled() (bool, error) {
	var v bool
	err := s.withSlotRead(func(slot shapeState) error {
		v = slot.enablePreSolveEvents
		return nil
	})
	return v, err
}

func (s *Shape) EnableHitEvents(flag bool) error {
	return s.withSlot(func(slot *shapeState) error {
		slot.enableHitEvents = flag
		return nil
	})
}

func (s *Shape) HitEventsEnabled() (bool, error) {
	var v bool
	err := s.withSlotRead(func(slot shapeState) error {
		v = slot.enableHitEvents
		return nil
	})
	return v, err
}

func (s *Shape) GetCircle() (Circle, error) {
	var c Circle
	err := s.withSlotRead(func(slot shapeState) error {
		c = slot.circle
		return nil
	})
	return c, err
}

func (s *Shape) GetCapsule() (Capsule, error) {
	var c Capsule
	err := s.withSlotRead(func(slot shapeState) error {
		c = slot.capsule
		return nil
	})
	return c, err
}

func (s *Shape) GetSegment() (Segment, error) {
	var c Segment
	err := s.withSlotRead(func(slot shapeState) error {
		c = slot.segment
		return nil
	})
	return c, err
}

func (s *Shape) GetPolygon() (Polygon, error) {
	var p Polygon
	err := s.withSlotRead(func(slot shapeState) error {
		p = slot.polygon
		return nil
	})
	return p, err
}

func (s *Shape) GetChainSegment() (Segment, error) {
	var seg Segment
	err := s.withSlotRead(func(slot shapeState) error {
		if slot.shapeType != ChainSegmentShape {
			return ErrShapeInvalid
		}
		seg = slot.segment
		return nil
	})
	return seg, err
}

func (s *Shape) GetParentChain() (ChainID, error) {
	if !s.IsValid() {
		return NullChainID, ErrShapeInvalid
	}
	var chainID ChainID
	err := s.withSlotRead(func(slot shapeState) error {
		if slot.shapeType != ChainSegmentShape || slot.parentChainIndex < 0 || slot.parentChainIndex >= len(s.world.chains) {
			chainID = NullChainID
			return nil
		}
		chain := s.world.chains[slot.parentChainIndex]
		if !chain.alive {
			chainID = NullChainID
			return nil
		}
		chainID = ChainID{
			Index1:     int32(slot.parentChainIndex + 1),
			World0:     s.id.World0,
			Generation: chain.generation,
		}
		return nil
	})
	return chainID, err
}

func (s *Shape) GetUserData() (any, error) {
	return s.UserData()
}

func (s *Shape) GetDensity() (float32, error) {
	return s.Density()
}

func (s *Shape) GetFriction() (float32, error) {
	return s.Friction()
}

func (s *Shape) GetRestitution() (float32, error) {
	return s.Restitution()
}

func (s *Shape) GetUserMaterial() (uint64, error) {
	return s.UserMaterial()
}

func (s *Shape) GetSurfaceMaterial() (SurfaceMaterial, error) {
	return s.SurfaceMaterial()
}

func (s *Shape) GetFilter() (Filter, error) {
	return s.Filter()
}

func (s *Shape) AreSensorEventsEnabled() (bool, error) {
	return s.SensorEventsEnabled()
}

func (s *Shape) AreContactEventsEnabled() (bool, error) {
	return s.ContactEventsEnabled()
}

func (s *Shape) ArePreSolveEventsEnabled() (bool, error) {
	return s.PreSolveEventsEnabled()
}

func (s *Shape) AreHitEventsEnabled() (bool, error) {
	return s.HitEventsEnabled()
}

func (s *Shape) SetCircle(circle Circle) error {
	return s.withSlot(func(slot *shapeState) error {
		if slot.shapeType != CircleShape {
			return ErrShapeInvalid
		}
		slot.circle = circle
		s.world.updateBodyMassLocked(slot.bodyIndex)
		return nil
	})
}

func (s *Shape) SetCapsule(capsule Capsule) error {
	return s.withSlot(func(slot *shapeState) error {
		if slot.shapeType != CapsuleShape {
			return ErrShapeInvalid
		}
		slot.capsule = capsule
		s.world.updateBodyMassLocked(slot.bodyIndex)
		return nil
	})
}

func (s *Shape) SetSegment(segment Segment) error {
	return s.withSlot(func(slot *shapeState) error {
		if slot.shapeType != SegmentShape && slot.shapeType != ChainSegmentShape {
			return ErrShapeInvalid
		}
		slot.segment = segment
		s.world.updateBodyMassLocked(slot.bodyIndex)
		return nil
	})
}

func (s *Shape) SetPolygon(polygon Polygon) error {
	return s.withSlot(func(slot *shapeState) error {
		if slot.shapeType != PolygonShape {
			return ErrShapeInvalid
		}
		slot.polygon = polygon
		s.world.updateBodyMassLocked(slot.bodyIndex)
		return nil
	})
}

func (s *Shape) GetAABB() (AABB, error) {
	var aabb AABB
	err := s.withSlotRead(func(slot shapeState) error {
		if slot.bodyIndex < 0 || slot.bodyIndex >= len(s.world.bodies) {
			return ErrShapeInvalid
		}
		bodySlot := s.world.bodies[slot.bodyIndex]
		transform := Transform{P: bodySlot.position, Q: bodySlot.rotation}
		switch slot.shapeType {
		case CircleShape:
			aabb = ComputeCircleAABB(&slot.circle, transform)
		case CapsuleShape:
			aabb = ComputeCapsuleAABB(&slot.capsule, transform)
		case PolygonShape:
			aabb = ComputePolygonAABB(&slot.polygon, transform)
		case SegmentShape, ChainSegmentShape:
			aabb = ComputeSegmentAABB(&slot.segment, transform)
		default:
			aabb = AABB{}
		}
		return nil
	})
	return aabb, err
}

func (s *Shape) ComputeMassData() (MassData, error) {
	var md MassData
	err := s.withSlotRead(func(slot shapeState) error {
		md = computeShapeMassData(slot)
		return nil
	})
	return md, err
}

func closestPointOnSegment(point, a, b Vec2) Vec2 {
	ab := Sub(b, a)
	den := Dot(ab, ab)
	if den <= 0 {
		return a
	}
	t := Dot(Sub(point, a), ab) / den
	t = ClampFloat(t, 0.0, 1.0)
	return MulAdd(a, t, ab)
}

func closestPointOnPolygon(shape Polygon, point Vec2) Vec2 {
	if PointInPolygon(&shape, point) {
		return point
	}
	best := shape.Vertices[0]
	bestDist2 := DistanceSquared(point, best)
	for i := 0; i < shape.Count; i++ {
		j := i + 1
		if j == shape.Count {
			j = 0
		}
		candidate := closestPointOnSegment(point, shape.Vertices[i], shape.Vertices[j])
		d2 := DistanceSquared(point, candidate)
		if d2 < bestDist2 {
			bestDist2 = d2
			best = candidate
		}
	}
	return best
}

func (s *Shape) GetClosestPoint(target Vec2) (Vec2, error) {
	var worldPoint Vec2
	err := s.withSlotRead(func(slot shapeState) error {
		if slot.bodyIndex < 0 || slot.bodyIndex >= len(s.world.bodies) {
			return ErrShapeInvalid
		}
		bodySlot := s.world.bodies[slot.bodyIndex]
		transform := Transform{P: bodySlot.position, Q: bodySlot.rotation}
		localTarget := InvTransformPoint(transform, target)

		var localPoint Vec2
		switch slot.shapeType {
		case CircleShape:
			d := Sub(localTarget, slot.circle.Center)
			dist := Length(d)
			if dist <= slot.circle.Radius || dist <= floatEpsilon {
				localPoint = localTarget
			} else {
				localPoint = MulAdd(slot.circle.Center, slot.circle.Radius/dist, d)
			}
		case CapsuleShape:
			axisPoint := closestPointOnSegment(localTarget, slot.capsule.Center1, slot.capsule.Center2)
			d := Sub(localTarget, axisPoint)
			dist := Length(d)
			if dist <= slot.capsule.Radius || dist <= floatEpsilon {
				localPoint = localTarget
			} else {
				localPoint = MulAdd(axisPoint, slot.capsule.Radius/dist, d)
			}
		case SegmentShape, ChainSegmentShape:
			localPoint = closestPointOnSegment(localTarget, slot.segment.Point1, slot.segment.Point2)
		case PolygonShape:
			localPoint = closestPointOnPolygon(slot.polygon, localTarget)
		default:
			localPoint = localTarget
		}

		worldPoint = TransformPoint(transform, localPoint)
		return nil
	})
	return worldPoint, err
}

func (s *Shape) TestPoint(point Vec2) (bool, error) {
	var inside bool
	err := s.withSlotRead(func(slot shapeState) error {
		if slot.bodyIndex < 0 || slot.bodyIndex >= len(s.world.bodies) {
			return ErrShapeInvalid
		}
		bodySlot := s.world.bodies[slot.bodyIndex]
		transform := Transform{P: bodySlot.position, Q: bodySlot.rotation}
		localPoint := InvTransformPoint(transform, point)
		switch slot.shapeType {
		case CircleShape:
			inside = PointInCircle(&slot.circle, localPoint)
		case PolygonShape:
			inside = PointInPolygon(&slot.polygon, localPoint)
		case CapsuleShape:
			axisPoint := closestPointOnSegment(localPoint, slot.capsule.Center1, slot.capsule.Center2)
			inside = DistanceSquared(localPoint, axisPoint) <= slot.capsule.Radius*slot.capsule.Radius
		default:
			inside = false
		}
		return nil
	})
	return inside, err
}

func (s *Shape) RayCast(input RayCastInput) (CastOutput, error) {
	var output CastOutput
	err := s.withSlotRead(func(slot shapeState) error {
		if slot.bodyIndex < 0 || slot.bodyIndex >= len(s.world.bodies) {
			return ErrShapeInvalid
		}
		bodySlot := s.world.bodies[slot.bodyIndex]
		transform := Transform{P: bodySlot.position, Q: bodySlot.rotation}
		localInput := RayCastInput{
			Origin:      InvTransformPoint(transform, input.Origin),
			Translation: InvRotateVector(transform.Q, input.Translation),
			MaxFraction: input.MaxFraction,
		}

		switch slot.shapeType {
		case CircleShape:
			output = RayCastCircle(&slot.circle, &localInput)
		case PolygonShape:
			output = RayCastPolygon(&slot.polygon, &localInput)
		case SegmentShape:
			output = RayCastSegment(&slot.segment, &localInput, false)
		case ChainSegmentShape:
			output = RayCastSegment(&slot.segment, &localInput, true)
		case CapsuleShape:
			output = RayCastCapsule(&slot.capsule, &localInput)
		default:
			output = CastOutput{}
		}
		if output.Hit {
			output.Point = TransformPoint(transform, output.Point)
			output.Normal = RotateVector(transform.Q, output.Normal)
		}
		return nil
	})
	return output, err
}

func (s *Shape) GetContactCapacity() (int, error) {
	if !s.IsValid() {
		return 0, ErrShapeInvalid
	}
	s.world.mu.Lock()
	defer s.world.mu.Unlock()
	if !s.world.isShapeIDValidLocked(s.id) {
		return 0, ErrShapeInvalid
	}
	s.world.refreshContactAndSensorStateLocked()
	shapeIdx := int(s.id.Index1) - 1
	count := 0
	for _, pair := range s.world.sortedActiveContactPairsLocked() {
		if pair.a == shapeIdx || pair.b == shapeIdx {
			count++
		}
	}
	return count, nil
}

func (s *Shape) GetContactData(capacity int) ([]ContactData, error) {
	if !s.IsValid() {
		return nil, ErrShapeInvalid
	}
	s.world.mu.Lock()
	defer s.world.mu.Unlock()
	if !s.world.isShapeIDValidLocked(s.id) {
		return nil, ErrShapeInvalid
	}
	if capacity <= 0 {
		return []ContactData{}, nil
	}
	s.world.refreshContactAndSensorStateLocked()
	shapeIdx := int(s.id.Index1) - 1
	limit := capacity
	if len(s.world.activeContactPairs) < limit {
		limit = len(s.world.activeContactPairs)
	}
	data := make([]ContactData, 0, limit)
	for _, pair := range s.world.sortedActiveContactPairsLocked() {
		if pair.a != shapeIdx && pair.b != shapeIdx {
			continue
		}
		data = append(data, s.world.contactDataFromPairLocked(pair))
		if len(data) == capacity {
			break
		}
	}
	return data, nil
}

func (s *Shape) GetSensorCapacity() (int, error) {
	if !s.IsValid() {
		return 0, ErrShapeInvalid
	}
	s.world.mu.Lock()
	defer s.world.mu.Unlock()
	if !s.world.isShapeIDValidLocked(s.id) {
		return 0, ErrShapeInvalid
	}
	shapeIdx := int(s.id.Index1) - 1
	slot := s.world.shapes[shapeIdx]
	if !slot.isSensor {
		return 0, nil
	}
	s.world.refreshContactAndSensorStateLocked()
	count := 0
	for _, key := range s.world.activeSensorKeys {
		if key.sensor == shapeIdx {
			count++
		}
	}
	return count, nil
}

func (s *Shape) GetSensorData(capacity int) ([]ShapeID, error) {
	if !s.IsValid() {
		return nil, ErrShapeInvalid
	}
	s.world.mu.Lock()
	defer s.world.mu.Unlock()
	if !s.world.isShapeIDValidLocked(s.id) {
		return nil, ErrShapeInvalid
	}
	if capacity <= 0 {
		return []ShapeID{}, nil
	}
	shapeIdx := int(s.id.Index1) - 1
	slot := s.world.shapes[shapeIdx]
	if !slot.isSensor {
		return []ShapeID{}, nil
	}
	s.world.refreshContactAndSensorStateLocked()
	visitorIDs := make([]ShapeID, 0, capacity)
	// activeSensorKeys is already sorted by (sensor, visitor), so results for
	// a given sensor index are contiguous and visitor-ordered.
	for i, key := range s.world.activeSensorKeys {
		if key.sensor == shapeIdx {
			visitorIDs = append(visitorIDs, s.world.activeSensorData[i].visitorShapeID)
			if len(visitorIDs) == capacity {
				break
			}
		}
	}
	return visitorIDs, nil
}

func (s *Shape) GetSensorOverlaps(capacity int) ([]ShapeID, error) {
	return s.GetSensorData(capacity)
}

func (s *Shape) ApplyWind(wind Vec2, drag, lift float32, wake bool) error {
	if !s.IsValid() {
		return ErrShapeInvalid
	}

	s.world.mu.Lock()
	defer s.world.mu.Unlock()
	if !s.world.isShapeIDValidLocked(s.id) {
		return ErrShapeInvalid
	}

	shapeIdx := int(s.id.Index1) - 1
	slot := s.world.shapes[shapeIdx]
	if slot.bodyIndex < 0 || slot.bodyIndex >= len(s.world.bodies) {
		return ErrShapeInvalid
	}
	body := &s.world.bodies[slot.bodyIndex]
	if !body.alive || !body.isEnabled || body.bodyType != DynamicBody {
		return nil
	}
	if wake {
		body.isAwake = true
	}
	if !body.isAwake {
		return nil
	}

	transform := Transform{P: body.position, Q: body.rotation}
	localCenter := shapeLocalCenter(slot)
	worldPoint := TransformPoint(transform, localCenter)

	bodyCenter := Add(body.position, RotateVector(body.rotation, body.localCenter))
	pointVelocity := Add(body.linearVelocity, CrossSV(body.angularVelocity, Sub(worldPoint, bodyCenter)))
	relativeVelocity := Sub(wind, pointVelocity)
	speed := Length(relativeVelocity)
	if speed <= 0 {
		return nil
	}

	direction := MulSV(1.0/speed, relativeVelocity)
	perp := LeftPerp(direction)
	span := shapeProjectedSpanWorld(slot, transform, perp)
	if span <= 0 {
		return nil
	}

	dragForce := MulSV(drag*span*speed*speed, direction)
	liftForce := MulSV(lift*span*speed*speed, perp)
	force := Add(dragForce, liftForce)
	body.force = Add(body.force, force)

	r := Sub(worldPoint, bodyCenter)
	body.torque += Cross(r, force)
	return nil
}

func shapeLocalCenter(shape shapeState) Vec2 {
	switch shape.shapeType {
	case CircleShape:
		return shape.circle.Center
	case CapsuleShape:
		return MulSV(0.5, Add(shape.capsule.Center1, shape.capsule.Center2))
	case SegmentShape, ChainSegmentShape:
		return MulSV(0.5, Add(shape.segment.Point1, shape.segment.Point2))
	case PolygonShape:
		return shape.polygon.Centroid
	default:
		return Vec2Zero
	}
}

func shapeProjectedSpanWorld(shape shapeState, transform Transform, normal Vec2) float32 {
	switch shape.shapeType {
	case CircleShape:
		return 2.0 * shape.circle.Radius
	case CapsuleShape:
		p1 := TransformPoint(transform, shape.capsule.Center1)
		p2 := TransformPoint(transform, shape.capsule.Center2)
		axisSpan := AbsFloat(Dot(Sub(p2, p1), normal))
		return axisSpan + 2.0*shape.capsule.Radius
	case SegmentShape, ChainSegmentShape:
		p1 := TransformPoint(transform, shape.segment.Point1)
		p2 := TransformPoint(transform, shape.segment.Point2)
		return AbsFloat(Dot(Sub(p2, p1), normal))
	case PolygonShape:
		if shape.polygon.Count == 0 {
			return 0
		}
		minProj := Dot(normal, TransformPoint(transform, shape.polygon.Vertices[0]))
		maxProj := minProj
		for i := 1; i < shape.polygon.Count; i++ {
			proj := Dot(normal, TransformPoint(transform, shape.polygon.Vertices[i]))
			if proj < minProj {
				minProj = proj
			}
			if proj > maxProj {
				maxProj = proj
			}
		}
		return (maxProj - minProj) + 2.0*shape.polygon.Radius
	default:
		return 0
	}
}
