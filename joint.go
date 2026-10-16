package box2d

import (
	"errors"
	"math"
)

var (
	ErrJointInvalid             = errors.New("box2d: joint is invalid")
	ErrJointTypeMismatch        = errors.New("box2d: joint type mismatch")
	ErrInvalidDistanceJointDef  = errors.New("box2d: invalid distance joint definition (use DefaultDistanceJointDef)")
	ErrInvalidMotorJointDef     = errors.New("box2d: invalid motor joint definition (use DefaultMotorJointDef)")
	ErrInvalidFilterJointDef    = errors.New("box2d: invalid filter joint definition (use DefaultFilterJointDef)")
	ErrInvalidPrismaticJointDef = errors.New("box2d: invalid prismatic joint definition (use DefaultPrismaticJointDef)")
	ErrInvalidRevoluteJointDef  = errors.New("box2d: invalid revolute joint definition (use DefaultRevoluteJointDef)")
	ErrInvalidWeldJointDef      = errors.New("box2d: invalid weld joint definition (use DefaultWeldJointDef)")
	ErrInvalidWheelJointDef     = errors.New("box2d: invalid wheel joint definition (use DefaultWheelJointDef)")
)

type jointState struct {
	generation               uint16
	alive                    bool
	jointType                JointType
	bodyAIndex               int
	bodyBIndex               int
	userData                 any
	localFrameA              Transform
	localFrameB              Transform
	collideConnected         bool
	constraintHertz          float32
	dampingRatio             float32
	forceThreshold           float32
	torqueThreshold          float32
	distanceLength           float32
	distanceSpringEnabled    bool
	distanceLowerSpringForce float32
	distanceUpperSpringForce float32
	distanceSpringHertz      float32
	distanceSpringDamping    float32
	distanceLimitEnabled     bool
	distanceMinLength        float32
	distanceMaxLength        float32
	distanceMotorEnabled     bool
	distanceMotorSpeed       float32
	distanceMaxMotorForce    float32
	distanceMotorForce       float32
	motorLinearVelocity      Vec2
	motorAngularVelocity     float32
	motorMaxVelocityForce    float32
	motorMaxVelocityTorque   float32
	motorLinearHertz         float32
	motorLinearDampingRatio  float32
	motorMaxSpringForce      float32
	motorAngularHertz        float32
	motorAngularDampingRatio float32
	motorMaxSpringTorque     float32
	prismaticSpringEnabled   bool
	prismaticHertz           float32
	prismaticDampingRatio    float32
	prismaticTarget          float32
	prismaticLimitEnabled    bool
	prismaticLowerLimit      float32
	prismaticUpperLimit      float32
	prismaticMotorEnabled    bool
	prismaticMotorSpeed      float32
	prismaticMaxMotorForce   float32
	prismaticMotorForce      float32
	revoluteTargetAngle      float32
	revoluteSpringEnabled    bool
	revoluteHertz            float32
	revoluteDampingRatio     float32
	revoluteLimitEnabled     bool
	revoluteLowerAngle       float32
	revoluteUpperAngle       float32
	revoluteMotorEnabled     bool
	revoluteMotorSpeed       float32
	revoluteMaxMotorTorque   float32
	revoluteMotorTorque      float32
	weldLinearHertz          float32
	weldLinearDampingRatio   float32
	weldAngularHertz         float32
	weldAngularDampingRatio  float32
	wheelSpringEnabled       bool
	wheelHertz               float32
	wheelDampingRatio        float32
	wheelLimitEnabled        bool
	wheelLowerTranslation    float32
	wheelUpperTranslation    float32
	wheelMotorEnabled        bool
	wheelMotorSpeed          float32
	wheelMaxMotorTorque      float32
	wheelMotorTorque         float32
}

type Joint struct {
	world *World
	id    JointID
}

func IsJointIDValid(id JointID) bool {
	if id.IsNull() {
		return false
	}
	world := worldFromIndex(id.World0 + 1)
	if world == nil {
		return false
	}
	world.mu.RLock()
	defer world.mu.RUnlock()
	return world.isJointIDValidLocked(id)
}

func (w *World) isJointIDValidLocked(id JointID) bool {
	idx := int(id.Index1) - 1
	if idx < 0 || idx >= len(w.joints) {
		return false
	}
	slot := w.joints[idx]
	return slot.alive && slot.generation == id.Generation
}

func (w *World) allocJointSlotLocked() int {
	if n := len(w.freeJointIndices); n > 0 {
		idx := w.freeJointIndices[n-1]
		w.freeJointIndices = w.freeJointIndices[:n-1]
		return idx
	}
	idx := len(w.joints)
	w.joints = append(w.joints, jointState{})
	return idx
}

func (w *World) destroyJointIndexLocked(idx int) {
	if idx < 0 || idx >= len(w.joints) {
		return
	}
	slot := &w.joints[idx]
	if !slot.alive {
		return
	}
	slot.alive = false
	slot.generation++
	if slot.generation == 0 {
		slot.generation = 1
	}
	slot.userData = nil
	w.freeJointIndices = append(w.freeJointIndices, idx)
	w.jointCount--
}

func (w *World) destroyJointIDLocked(id JointID) error {
	if !w.isJointIDValidLocked(id) {
		return ErrJointInvalid
	}
	w.destroyJointIndexLocked(int(id.Index1) - 1)
	return nil
}

func initDistanceJointDefaults(slot *jointState) {
	slot.distanceLength = 1.0
	slot.distanceSpringEnabled = false
	slot.distanceLowerSpringForce = -float32(math.MaxFloat32)
	slot.distanceUpperSpringForce = float32(math.MaxFloat32)
	slot.distanceSpringHertz = 0
	slot.distanceSpringDamping = 0
	slot.distanceLimitEnabled = false
	slot.distanceMinLength = linearSlop()
	slot.distanceMaxLength = float32(math.MaxFloat32)
	slot.distanceMotorEnabled = false
	slot.distanceMotorSpeed = 0
	slot.distanceMaxMotorForce = 0
	slot.distanceMotorForce = 0
}

func initMotorJointDefaults(slot *jointState) {
	slot.motorLinearVelocity = Vec2Zero
	slot.motorAngularVelocity = 0
	slot.motorMaxVelocityForce = 0
	slot.motorMaxVelocityTorque = 0
	slot.motorLinearHertz = 0
	slot.motorLinearDampingRatio = 0
	slot.motorMaxSpringForce = 0
	slot.motorAngularHertz = 0
	slot.motorAngularDampingRatio = 0
	slot.motorMaxSpringTorque = 0
}

func initPrismaticJointDefaults(slot *jointState) {
	slot.prismaticSpringEnabled = false
	slot.prismaticHertz = 0
	slot.prismaticDampingRatio = 0
	slot.prismaticTarget = 0
	slot.prismaticLimitEnabled = false
	slot.prismaticLowerLimit = 0
	slot.prismaticUpperLimit = 0
	slot.prismaticMotorEnabled = false
	slot.prismaticMotorSpeed = 0
	slot.prismaticMaxMotorForce = 0
	slot.prismaticMotorForce = 0
}

func initRevoluteJointDefaults(slot *jointState) {
	slot.revoluteTargetAngle = 0
	slot.revoluteSpringEnabled = false
	slot.revoluteHertz = 0
	slot.revoluteDampingRatio = 0
	slot.revoluteLimitEnabled = false
	slot.revoluteLowerAngle = 0
	slot.revoluteUpperAngle = 0
	slot.revoluteMotorEnabled = false
	slot.revoluteMotorSpeed = 0
	slot.revoluteMaxMotorTorque = 0
	slot.revoluteMotorTorque = 0
}

func initWeldJointDefaults(slot *jointState) {
	slot.weldLinearHertz = 0
	slot.weldLinearDampingRatio = 0
	slot.weldAngularHertz = 0
	slot.weldAngularDampingRatio = 0
}

func initWheelJointDefaults(slot *jointState) {
	slot.wheelSpringEnabled = true
	slot.wheelHertz = 1.0
	slot.wheelDampingRatio = 0.7
	slot.wheelLimitEnabled = false
	slot.wheelLowerTranslation = 0
	slot.wheelUpperTranslation = 0
	slot.wheelMotorEnabled = false
	slot.wheelMotorSpeed = 0
	slot.wheelMaxMotorTorque = 0
	slot.wheelMotorTorque = 0
}

func (w *World) CreateJoint(bodyA, bodyB BodyID, jointType JointType) (*Joint, error) {
	if !w.IsValid() {
		return nil, ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	if !w.isBodyIDValidLocked(bodyA) || !w.isBodyIDValidLocked(bodyB) {
		return nil, ErrBodyInvalid
	}
	if BodyIDEqual(bodyA, bodyB) {
		return nil, ErrBodyInvalid
	}

	jointIdx := w.allocJointSlotLocked()
	slot := &w.joints[jointIdx]
	slot.generation++
	if slot.generation == 0 {
		slot.generation = 1
	}
	slot.alive = true
	slot.jointType = jointType
	slot.bodyAIndex = int(bodyA.Index1) - 1
	slot.bodyBIndex = int(bodyB.Index1) - 1
	slot.userData = nil
	slot.localFrameA = TransformIdentity
	slot.localFrameB = TransformIdentity
	slot.collideConnected = false
	slot.constraintHertz = 60.0
	slot.dampingRatio = 2.0
	slot.forceThreshold = float32(math.MaxFloat32)
	slot.torqueThreshold = float32(math.MaxFloat32)
	if jointType == DistanceJoint {
		initDistanceJointDefaults(slot)
	} else if jointType == MotorJoint {
		initMotorJointDefaults(slot)
	} else if jointType == PrismaticJoint {
		initPrismaticJointDefaults(slot)
	} else if jointType == RevoluteJoint {
		initRevoluteJointDefaults(slot)
	} else if jointType == WeldJoint {
		initWeldJointDefaults(slot)
	} else if jointType == WheelJoint {
		initWheelJointDefaults(slot)
	}
	w.jointCount++

	id := JointID{
		Index1:     int32(jointIdx + 1),
		World0:     w.id.Index1 - 1,
		Generation: slot.generation,
	}
	return &Joint{world: w, id: id}, nil
}

func (w *World) CreateDistanceJoint(bodyA, bodyB BodyID) (*Joint, error) {
	return w.CreateJoint(bodyA, bodyB, DistanceJoint)
}

func (w *World) CreateDistanceJointWithDef(def DistanceJointDef) (*Joint, error) {
	if !def.isInitialized() {
		return nil, ErrInvalidDistanceJointDef
	}
	joint, err := w.CreateJoint(def.Base.BodyIDA, def.Base.BodyIDB, DistanceJoint)
	if err != nil {
		return nil, err
	}

	var setErr error
	set := func(err error) {
		if setErr == nil && err != nil {
			setErr = err
		}
	}
	set(joint.SetUserData(def.Base.UserData))
	set(joint.SetLocalFrameA(def.Base.LocalFrameA))
	set(joint.SetLocalFrameB(def.Base.LocalFrameB))
	set(joint.SetCollideConnected(def.Base.CollideConnected))
	set(joint.SetConstraintTuning(def.Base.ConstraintHertz, def.Base.ConstraintDampingRatio))
	set(joint.SetForceThreshold(def.Base.ForceThreshold))
	set(joint.SetTorqueThreshold(def.Base.TorqueThreshold))
	set(joint.SetLength(def.Length))
	set(joint.EnableSpring(def.EnableSpring))
	set(joint.SetSpringForceRange(def.LowerSpringForce, def.UpperSpringForce))
	set(joint.SetSpringHertz(def.Hertz))
	set(joint.SetSpringDampingRatio(def.DampingRatio))
	set(joint.EnableLimit(def.EnableLimit))
	set(joint.SetLengthRange(def.MinLength, def.MaxLength))
	set(joint.EnableMotor(def.EnableMotor))
	set(joint.SetMotorSpeed(def.MotorSpeed))
	set(joint.SetMaxMotorForce(def.MaxMotorForce))

	if setErr != nil {
		_ = joint.Destroy()
		return nil, setErr
	}
	return joint, nil
}

func (w *World) CreateDistanceJointFromDef(def DistanceJointDef) (*Joint, error) {
	return w.CreateDistanceJointWithDef(def)
}

func (w *World) CreateMotorJoint(bodyA, bodyB BodyID) (*Joint, error) {
	return w.CreateJoint(bodyA, bodyB, MotorJoint)
}

func (w *World) CreateMotorJointWithDef(def MotorJointDef) (*Joint, error) {
	if !def.isInitialized() {
		return nil, ErrInvalidMotorJointDef
	}
	joint, err := w.CreateJoint(def.Base.BodyIDA, def.Base.BodyIDB, MotorJoint)
	if err != nil {
		return nil, err
	}

	var setErr error
	set := func(err error) {
		if setErr == nil && err != nil {
			setErr = err
		}
	}
	set(joint.SetUserData(def.Base.UserData))
	set(joint.SetLocalFrameA(def.Base.LocalFrameA))
	set(joint.SetLocalFrameB(def.Base.LocalFrameB))
	set(joint.SetCollideConnected(def.Base.CollideConnected))
	set(joint.SetConstraintTuning(def.Base.ConstraintHertz, def.Base.ConstraintDampingRatio))
	set(joint.SetForceThreshold(def.Base.ForceThreshold))
	set(joint.SetTorqueThreshold(def.Base.TorqueThreshold))
	set(joint.SetLinearVelocity(def.LinearVelocity))
	set(joint.SetAngularVelocity(def.AngularVelocity))
	set(joint.SetMaxVelocityForce(def.MaxVelocityForce))
	set(joint.SetMaxVelocityTorque(def.MaxVelocityTorque))
	set(joint.SetLinearHertz(def.LinearHertz))
	set(joint.SetLinearDampingRatio(def.LinearDampingRatio))
	set(joint.SetMaxSpringForce(def.MaxSpringForce))
	set(joint.SetAngularHertz(def.AngularHertz))
	set(joint.SetAngularDampingRatio(def.AngularDampingRatio))
	set(joint.SetMaxSpringTorque(def.MaxSpringTorque))

	if setErr != nil {
		_ = joint.Destroy()
		return nil, setErr
	}
	return joint, nil
}

func (w *World) CreateMotorJointFromDef(def MotorJointDef) (*Joint, error) {
	return w.CreateMotorJointWithDef(def)
}

func (w *World) CreateFilterJoint(bodyA, bodyB BodyID) (*Joint, error) {
	return w.CreateJoint(bodyA, bodyB, FilterJoint)
}

func (w *World) CreateFilterJointWithDef(def FilterJointDef) (*Joint, error) {
	if !def.isInitialized() {
		return nil, ErrInvalidFilterJointDef
	}
	joint, err := w.CreateJoint(def.Base.BodyIDA, def.Base.BodyIDB, FilterJoint)
	if err != nil {
		return nil, err
	}

	var setErr error
	set := func(err error) {
		if setErr == nil && err != nil {
			setErr = err
		}
	}
	set(joint.SetUserData(def.Base.UserData))
	set(joint.SetLocalFrameA(def.Base.LocalFrameA))
	set(joint.SetLocalFrameB(def.Base.LocalFrameB))
	set(joint.SetCollideConnected(def.Base.CollideConnected))
	set(joint.SetConstraintTuning(def.Base.ConstraintHertz, def.Base.ConstraintDampingRatio))
	set(joint.SetForceThreshold(def.Base.ForceThreshold))
	set(joint.SetTorqueThreshold(def.Base.TorqueThreshold))

	if setErr != nil {
		_ = joint.Destroy()
		return nil, setErr
	}
	return joint, nil
}

func (w *World) CreateFilterJointFromDef(def FilterJointDef) (*Joint, error) {
	return w.CreateFilterJointWithDef(def)
}

func (w *World) CreatePrismaticJoint(bodyA, bodyB BodyID) (*Joint, error) {
	return w.CreateJoint(bodyA, bodyB, PrismaticJoint)
}

func (w *World) CreatePrismaticJointWithDef(def PrismaticJointDef) (*Joint, error) {
	if !def.isInitialized() {
		return nil, ErrInvalidPrismaticJointDef
	}
	joint, err := w.CreateJoint(def.Base.BodyIDA, def.Base.BodyIDB, PrismaticJoint)
	if err != nil {
		return nil, err
	}

	var setErr error
	set := func(err error) {
		if setErr == nil && err != nil {
			setErr = err
		}
	}
	set(joint.SetUserData(def.Base.UserData))
	set(joint.SetLocalFrameA(def.Base.LocalFrameA))
	set(joint.SetLocalFrameB(def.Base.LocalFrameB))
	set(joint.SetCollideConnected(def.Base.CollideConnected))
	set(joint.SetConstraintTuning(def.Base.ConstraintHertz, def.Base.ConstraintDampingRatio))
	set(joint.SetForceThreshold(def.Base.ForceThreshold))
	set(joint.SetTorqueThreshold(def.Base.TorqueThreshold))
	set(joint.EnableSpring(def.EnableSpring))
	set(joint.SetSpringHertz(def.Hertz))
	set(joint.SetSpringDampingRatio(def.DampingRatio))
	set(joint.SetTargetTranslation(def.TargetTranslation))
	set(joint.EnableLimit(def.EnableLimit))
	set(joint.SetLimits(def.LowerTranslation, def.UpperTranslation))
	set(joint.EnableMotor(def.EnableMotor))
	set(joint.SetMotorSpeed(def.MotorSpeed))
	set(joint.SetMaxMotorForce(def.MaxMotorForce))

	if setErr != nil {
		_ = joint.Destroy()
		return nil, setErr
	}
	return joint, nil
}

func (w *World) CreatePrismaticJointFromDef(def PrismaticJointDef) (*Joint, error) {
	return w.CreatePrismaticJointWithDef(def)
}

func (w *World) CreateRevoluteJoint(bodyA, bodyB BodyID) (*Joint, error) {
	return w.CreateJoint(bodyA, bodyB, RevoluteJoint)
}

func (w *World) CreateRevoluteJointWithDef(def RevoluteJointDef) (*Joint, error) {
	if !def.isInitialized() {
		return nil, ErrInvalidRevoluteJointDef
	}
	joint, err := w.CreateJoint(def.Base.BodyIDA, def.Base.BodyIDB, RevoluteJoint)
	if err != nil {
		return nil, err
	}

	var setErr error
	set := func(err error) {
		if setErr == nil && err != nil {
			setErr = err
		}
	}
	set(joint.SetUserData(def.Base.UserData))
	set(joint.SetLocalFrameA(def.Base.LocalFrameA))
	set(joint.SetLocalFrameB(def.Base.LocalFrameB))
	set(joint.SetCollideConnected(def.Base.CollideConnected))
	set(joint.SetConstraintTuning(def.Base.ConstraintHertz, def.Base.ConstraintDampingRatio))
	set(joint.SetForceThreshold(def.Base.ForceThreshold))
	set(joint.SetTorqueThreshold(def.Base.TorqueThreshold))
	set(joint.SetTargetAngle(def.TargetAngle))
	set(joint.EnableSpring(def.EnableSpring))
	set(joint.SetSpringHertz(def.Hertz))
	set(joint.SetSpringDampingRatio(def.DampingRatio))
	set(joint.EnableLimit(def.EnableLimit))
	set(joint.SetLimits(def.LowerAngle, def.UpperAngle))
	set(joint.EnableMotor(def.EnableMotor))
	set(joint.SetMotorSpeed(def.MotorSpeed))
	set(joint.SetMaxMotorTorque(def.MaxMotorTorque))

	if setErr != nil {
		_ = joint.Destroy()
		return nil, setErr
	}
	return joint, nil
}

func (w *World) CreateRevoluteJointFromDef(def RevoluteJointDef) (*Joint, error) {
	return w.CreateRevoluteJointWithDef(def)
}

func (w *World) CreateWeldJoint(bodyA, bodyB BodyID) (*Joint, error) {
	return w.CreateJoint(bodyA, bodyB, WeldJoint)
}

func (w *World) CreateWeldJointWithDef(def WeldJointDef) (*Joint, error) {
	if !def.isInitialized() {
		return nil, ErrInvalidWeldJointDef
	}
	joint, err := w.CreateJoint(def.Base.BodyIDA, def.Base.BodyIDB, WeldJoint)
	if err != nil {
		return nil, err
	}

	var setErr error
	set := func(err error) {
		if setErr == nil && err != nil {
			setErr = err
		}
	}
	set(joint.SetUserData(def.Base.UserData))
	set(joint.SetLocalFrameA(def.Base.LocalFrameA))
	set(joint.SetLocalFrameB(def.Base.LocalFrameB))
	set(joint.SetCollideConnected(def.Base.CollideConnected))
	set(joint.SetConstraintTuning(def.Base.ConstraintHertz, def.Base.ConstraintDampingRatio))
	set(joint.SetForceThreshold(def.Base.ForceThreshold))
	set(joint.SetTorqueThreshold(def.Base.TorqueThreshold))
	set(joint.SetLinearHertz(def.LinearHertz))
	set(joint.SetLinearDampingRatio(def.LinearDampingRatio))
	set(joint.SetAngularHertz(def.AngularHertz))
	set(joint.SetAngularDampingRatio(def.AngularDampingRatio))

	if setErr != nil {
		_ = joint.Destroy()
		return nil, setErr
	}
	return joint, nil
}

func (w *World) CreateWeldJointFromDef(def WeldJointDef) (*Joint, error) {
	return w.CreateWeldJointWithDef(def)
}

func (w *World) CreateWheelJoint(bodyA, bodyB BodyID) (*Joint, error) {
	return w.CreateJoint(bodyA, bodyB, WheelJoint)
}

func (w *World) CreateWheelJointWithDef(def WheelJointDef) (*Joint, error) {
	if !def.isInitialized() {
		return nil, ErrInvalidWheelJointDef
	}
	joint, err := w.CreateJoint(def.Base.BodyIDA, def.Base.BodyIDB, WheelJoint)
	if err != nil {
		return nil, err
	}

	var setErr error
	set := func(err error) {
		if setErr == nil && err != nil {
			setErr = err
		}
	}
	set(joint.SetUserData(def.Base.UserData))
	set(joint.SetLocalFrameA(def.Base.LocalFrameA))
	set(joint.SetLocalFrameB(def.Base.LocalFrameB))
	set(joint.SetCollideConnected(def.Base.CollideConnected))
	set(joint.SetConstraintTuning(def.Base.ConstraintHertz, def.Base.ConstraintDampingRatio))
	set(joint.SetForceThreshold(def.Base.ForceThreshold))
	set(joint.SetTorqueThreshold(def.Base.TorqueThreshold))
	set(joint.EnableSpring(def.EnableSpring))
	set(joint.SetSpringHertz(def.Hertz))
	set(joint.SetSpringDampingRatio(def.DampingRatio))
	set(joint.EnableLimit(def.EnableLimit))
	set(joint.SetLimits(def.LowerTranslation, def.UpperTranslation))
	set(joint.EnableMotor(def.EnableMotor))
	set(joint.SetMotorSpeed(def.MotorSpeed))
	set(joint.SetMaxMotorTorque(def.MaxMotorTorque))

	if setErr != nil {
		_ = joint.Destroy()
		return nil, setErr
	}
	return joint, nil
}

func (w *World) CreateWheelJointFromDef(def WheelJointDef) (*Joint, error) {
	return w.CreateWheelJointWithDef(def)
}

func (w *World) DestroyJoint(id JointID) error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	return w.destroyJointIDLocked(id)
}

func (j *Joint) ID() JointID {
	if j == nil {
		return NullJointID
	}
	return j.id
}

func (j *Joint) IsValid() bool {
	if j == nil || j.world == nil || j.id.IsNull() {
		return false
	}
	return IsJointIDValid(j.id)
}

func (j *Joint) Destroy() error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	err := j.world.DestroyJoint(j.id)
	if err != nil {
		return err
	}
	j.id = NullJointID
	return nil
}

func (j *Joint) GetType() (JointType, error) {
	if !j.IsValid() {
		return UnknownJoint, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return UnknownJoint, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	return slot.jointType, nil
}

func (j *Joint) GetBodyA() (BodyID, error) {
	if !j.IsValid() {
		return NullBodyID, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return NullBodyID, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	body := j.world.bodies[slot.bodyAIndex]
	return BodyID{
		Index1:     int32(slot.bodyAIndex + 1),
		World0:     j.id.World0,
		Generation: body.generation,
	}, nil
}

func (j *Joint) GetBodyB() (BodyID, error) {
	if !j.IsValid() {
		return NullBodyID, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return NullBodyID, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	body := j.world.bodies[slot.bodyBIndex]
	return BodyID{
		Index1:     int32(slot.bodyBIndex + 1),
		World0:     j.id.World0,
		Generation: body.generation,
	}, nil
}

func (j *Joint) GetWorld() (WorldID, error) {
	if !j.IsValid() {
		return NullWorldID, ErrJointInvalid
	}
	return j.world.ID(), nil
}

func (j *Joint) distanceJointSlotLocked() (*jointState, error) {
	if !j.world.isJointIDValidLocked(j.id) {
		return nil, ErrJointInvalid
	}
	slot := &j.world.joints[int(j.id.Index1)-1]
	if slot.jointType != DistanceJoint {
		return nil, ErrJointTypeMismatch
	}
	return slot, nil
}

func (j *Joint) motorJointSlotLocked() (*jointState, error) {
	if !j.world.isJointIDValidLocked(j.id) {
		return nil, ErrJointInvalid
	}
	slot := &j.world.joints[int(j.id.Index1)-1]
	if slot.jointType != MotorJoint {
		return nil, ErrJointTypeMismatch
	}
	return slot, nil
}

func (j *Joint) prismaticJointSlotLocked() (*jointState, error) {
	if !j.world.isJointIDValidLocked(j.id) {
		return nil, ErrJointInvalid
	}
	slot := &j.world.joints[int(j.id.Index1)-1]
	if slot.jointType != PrismaticJoint {
		return nil, ErrJointTypeMismatch
	}
	return slot, nil
}

func (j *Joint) revoluteJointSlotLocked() (*jointState, error) {
	if !j.world.isJointIDValidLocked(j.id) {
		return nil, ErrJointInvalid
	}
	slot := &j.world.joints[int(j.id.Index1)-1]
	if slot.jointType != RevoluteJoint {
		return nil, ErrJointTypeMismatch
	}
	return slot, nil
}

func (j *Joint) SetUserData(v any) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return ErrJointInvalid
	}
	slot := &j.world.joints[int(j.id.Index1)-1]
	slot.userData = v
	return nil
}

func (j *Joint) UserData() (any, error) {
	if !j.IsValid() {
		return nil, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return nil, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	return slot.userData, nil
}

func (j *Joint) GetUserData() (any, error) {
	return j.UserData()
}

func (j *Joint) SetLocalFrameA(localFrame Transform) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return ErrJointInvalid
	}
	slot := &j.world.joints[int(j.id.Index1)-1]
	slot.localFrameA = localFrame
	return nil
}

func (j *Joint) GetLocalFrameA() (Transform, error) {
	if !j.IsValid() {
		return TransformIdentity, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return TransformIdentity, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	return slot.localFrameA, nil
}

func (j *Joint) SetLocalFrameB(localFrame Transform) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return ErrJointInvalid
	}
	slot := &j.world.joints[int(j.id.Index1)-1]
	slot.localFrameB = localFrame
	return nil
}

func (j *Joint) GetLocalFrameB() (Transform, error) {
	if !j.IsValid() {
		return TransformIdentity, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return TransformIdentity, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	return slot.localFrameB, nil
}

func (j *Joint) SetCollideConnected(shouldCollide bool) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return ErrJointInvalid
	}
	slot := &j.world.joints[int(j.id.Index1)-1]
	slot.collideConnected = shouldCollide
	return nil
}

func (j *Joint) GetCollideConnected() (bool, error) {
	if !j.IsValid() {
		return false, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return false, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	return slot.collideConnected, nil
}

func (j *Joint) WakeBodies() error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	if slot.bodyAIndex >= 0 && slot.bodyAIndex < len(j.world.bodies) {
		bodyA := &j.world.bodies[slot.bodyAIndex]
		if bodyA.alive && bodyA.isEnabled {
			bodyA.isAwake = true
			bodyA.sleepTime = 0
		}
	}
	if slot.bodyBIndex >= 0 && slot.bodyBIndex < len(j.world.bodies) {
		bodyB := &j.world.bodies[slot.bodyBIndex]
		if bodyB.alive && bodyB.isEnabled {
			bodyB.isAwake = true
			bodyB.sleepTime = 0
		}
	}
	return nil
}

func (j *Joint) GetConstraintForce() (Vec2, error) {
	if !j.IsValid() {
		return Vec2Zero, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return Vec2Zero, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	bodyA := j.world.bodies[slot.bodyAIndex]
	bodyB := j.world.bodies[slot.bodyBIndex]
	xfA := Transform{P: bodyA.position, Q: bodyA.rotation}
	xfB := Transform{P: bodyB.position, Q: bodyB.rotation}
	worldAnchorA := TransformPoint(xfA, slot.localFrameA.P)
	worldAnchorB := TransformPoint(xfB, slot.localFrameB.P)
	delta := Sub(worldAnchorB, worldAnchorA)
	force, _ := estimateJointForceTorque(slot, bodyA, bodyB)
	length, axis := GetLengthAndNormalize(delta)
	if length <= floatEpsilon {
		if slot.jointType == MotorJoint {
			errorVelocity := Sub(Sub(bodyB.linearVelocity, bodyA.linearVelocity), slot.motorLinearVelocity)
			if velLength, velAxis := GetLengthAndNormalize(errorVelocity); velLength > floatEpsilon {
				return MulSV(force, velAxis), nil
			}
		}
		return Vec2Zero, nil
	}
	return MulSV(force, axis), nil
}

func (j *Joint) GetConstraintTorque() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return 0, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	bodyA := j.world.bodies[slot.bodyAIndex]
	bodyB := j.world.bodies[slot.bodyBIndex]
	_, torque := estimateJointForceTorque(slot, bodyA, bodyB)
	return torque, nil
}

func (j *Joint) GetLinearSeparation() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return 0, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	bodyA := j.world.bodies[slot.bodyAIndex]
	bodyB := j.world.bodies[slot.bodyBIndex]
	transformA := Transform{P: bodyA.position, Q: bodyA.rotation}
	transformB := Transform{P: bodyB.position, Q: bodyB.rotation}
	worldAnchorA := TransformPoint(transformA, slot.localFrameA.P)
	worldAnchorB := TransformPoint(transformB, slot.localFrameB.P)
	return Length(Sub(worldAnchorB, worldAnchorA)), nil
}

func (j *Joint) GetAngularSeparation() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return 0, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	bodyA := j.world.bodies[slot.bodyAIndex]
	bodyB := j.world.bodies[slot.bodyBIndex]
	qa := MulRot(bodyA.rotation, slot.localFrameA.Q)
	qb := MulRot(bodyB.rotation, slot.localFrameB.Q)
	return RelativeAngle(qa, qb), nil
}

func (j *Joint) SetConstraintTuning(hertz, dampingRatio float32) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return ErrJointInvalid
	}
	slot := &j.world.joints[int(j.id.Index1)-1]
	slot.constraintHertz = hertz
	slot.dampingRatio = dampingRatio
	return nil
}

func (j *Joint) GetConstraintTuning() (float32, float32, error) {
	if !j.IsValid() {
		return 0, 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return 0, 0, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	return slot.constraintHertz, slot.dampingRatio, nil
}

func (j *Joint) SetForceThreshold(threshold float32) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return ErrJointInvalid
	}
	slot := &j.world.joints[int(j.id.Index1)-1]
	slot.forceThreshold = threshold
	return nil
}

func (j *Joint) GetForceThreshold() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return 0, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	return slot.forceThreshold, nil
}

func (j *Joint) SetTorqueThreshold(threshold float32) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return ErrJointInvalid
	}
	slot := &j.world.joints[int(j.id.Index1)-1]
	slot.torqueThreshold = threshold
	return nil
}

func (j *Joint) GetTorqueThreshold() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return 0, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	return slot.torqueThreshold, nil
}

func (j *Joint) SetLength(length float32) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	slot, err := j.distanceJointSlotLocked()
	if err != nil {
		return err
	}
	slot.distanceLength = ClampFloat(length, linearSlop(), float32(math.MaxFloat32))
	return nil
}

func (j *Joint) GetLength() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	slot, err := j.distanceJointSlotLocked()
	if err != nil {
		return 0, err
	}
	return slot.distanceLength, nil
}

func (j *Joint) EnableSpring(enable bool) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return ErrJointInvalid
	}
	slot := &j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case DistanceJoint:
		slot.distanceSpringEnabled = enable
	case PrismaticJoint:
		slot.prismaticSpringEnabled = enable
	case RevoluteJoint:
		slot.revoluteSpringEnabled = enable
	case WheelJoint:
		slot.wheelSpringEnabled = enable
	default:
		return ErrJointTypeMismatch
	}
	return nil
}

func (j *Joint) IsSpringEnabled() (bool, error) {
	if !j.IsValid() {
		return false, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return false, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case DistanceJoint:
		return slot.distanceSpringEnabled, nil
	case PrismaticJoint:
		return slot.prismaticSpringEnabled, nil
	case RevoluteJoint:
		return slot.revoluteSpringEnabled, nil
	case WheelJoint:
		return slot.wheelSpringEnabled, nil
	default:
		return false, ErrJointTypeMismatch
	}
}

func (j *Joint) SetSpringForceRange(lowerForce, upperForce float32) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	slot, err := j.distanceJointSlotLocked()
	if err != nil {
		return err
	}
	slot.distanceLowerSpringForce = MinFloat(lowerForce, upperForce)
	slot.distanceUpperSpringForce = MaxFloat(lowerForce, upperForce)
	return nil
}

func (j *Joint) GetSpringForceRange() (float32, float32, error) {
	if !j.IsValid() {
		return 0, 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	slot, err := j.distanceJointSlotLocked()
	if err != nil {
		return 0, 0, err
	}
	return slot.distanceLowerSpringForce, slot.distanceUpperSpringForce, nil
}

func (j *Joint) SetSpringHertz(hertz float32) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return ErrJointInvalid
	}
	hertz = MaxFloat(0, hertz)
	slot := &j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case DistanceJoint:
		slot.distanceSpringHertz = hertz
	case PrismaticJoint:
		slot.prismaticHertz = hertz
	case RevoluteJoint:
		slot.revoluteHertz = hertz
	case WheelJoint:
		slot.wheelHertz = hertz
	default:
		return ErrJointTypeMismatch
	}
	return nil
}

func (j *Joint) GetSpringHertz() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return 0, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case DistanceJoint:
		return slot.distanceSpringHertz, nil
	case PrismaticJoint:
		return slot.prismaticHertz, nil
	case RevoluteJoint:
		return slot.revoluteHertz, nil
	case WheelJoint:
		return slot.wheelHertz, nil
	default:
		return 0, ErrJointTypeMismatch
	}
}

func (j *Joint) SetSpringDampingRatio(dampingRatio float32) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return ErrJointInvalid
	}
	dampingRatio = MaxFloat(0, dampingRatio)
	slot := &j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case DistanceJoint:
		slot.distanceSpringDamping = dampingRatio
	case PrismaticJoint:
		slot.prismaticDampingRatio = dampingRatio
	case RevoluteJoint:
		slot.revoluteDampingRatio = dampingRatio
	case WheelJoint:
		slot.wheelDampingRatio = dampingRatio
	default:
		return ErrJointTypeMismatch
	}
	return nil
}

func (j *Joint) GetSpringDampingRatio() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return 0, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case DistanceJoint:
		return slot.distanceSpringDamping, nil
	case PrismaticJoint:
		return slot.prismaticDampingRatio, nil
	case RevoluteJoint:
		return slot.revoluteDampingRatio, nil
	case WheelJoint:
		return slot.wheelDampingRatio, nil
	default:
		return 0, ErrJointTypeMismatch
	}
}

func (j *Joint) EnableLimit(enable bool) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return ErrJointInvalid
	}
	slot := &j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case DistanceJoint:
		slot.distanceLimitEnabled = enable
	case PrismaticJoint:
		slot.prismaticLimitEnabled = enable
	case RevoluteJoint:
		slot.revoluteLimitEnabled = enable
	case WheelJoint:
		slot.wheelLimitEnabled = enable
	default:
		return ErrJointTypeMismatch
	}
	return nil
}

func (j *Joint) IsLimitEnabled() (bool, error) {
	if !j.IsValid() {
		return false, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return false, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case DistanceJoint:
		return slot.distanceLimitEnabled, nil
	case PrismaticJoint:
		return slot.prismaticLimitEnabled, nil
	case RevoluteJoint:
		return slot.revoluteLimitEnabled, nil
	case WheelJoint:
		return slot.wheelLimitEnabled, nil
	default:
		return false, ErrJointTypeMismatch
	}
}

func (j *Joint) SetLengthRange(minLength, maxLength float32) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	slot, err := j.distanceJointSlotLocked()
	if err != nil {
		return err
	}
	minLength = ClampFloat(minLength, linearSlop(), float32(math.MaxFloat32))
	maxLength = ClampFloat(maxLength, linearSlop(), float32(math.MaxFloat32))
	slot.distanceMinLength = MinFloat(minLength, maxLength)
	slot.distanceMaxLength = MaxFloat(minLength, maxLength)
	return nil
}

func (j *Joint) GetMinLength() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	slot, err := j.distanceJointSlotLocked()
	if err != nil {
		return 0, err
	}
	return slot.distanceMinLength, nil
}

func (j *Joint) GetMaxLength() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	slot, err := j.distanceJointSlotLocked()
	if err != nil {
		return 0, err
	}
	return slot.distanceMaxLength, nil
}

func (j *Joint) GetCurrentLength() (float32, error) {
	return j.GetLinearSeparation()
}

func (j *Joint) EnableMotor(enable bool) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return ErrJointInvalid
	}
	slot := &j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case DistanceJoint:
		slot.distanceMotorEnabled = enable
		if !enable {
			slot.distanceMotorForce = 0
		}
	case PrismaticJoint:
		slot.prismaticMotorEnabled = enable
		if !enable {
			slot.prismaticMotorForce = 0
		}
	case RevoluteJoint:
		slot.revoluteMotorEnabled = enable
		if !enable {
			slot.revoluteMotorTorque = 0
		}
	case WheelJoint:
		slot.wheelMotorEnabled = enable
		if !enable {
			slot.wheelMotorTorque = 0
		}
	default:
		return ErrJointTypeMismatch
	}
	return nil
}

func (j *Joint) IsMotorEnabled() (bool, error) {
	if !j.IsValid() {
		return false, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return false, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case DistanceJoint:
		return slot.distanceMotorEnabled, nil
	case PrismaticJoint:
		return slot.prismaticMotorEnabled, nil
	case RevoluteJoint:
		return slot.revoluteMotorEnabled, nil
	case WheelJoint:
		return slot.wheelMotorEnabled, nil
	default:
		return false, ErrJointTypeMismatch
	}
}

func (j *Joint) SetMotorSpeed(motorSpeed float32) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return ErrJointInvalid
	}
	slot := &j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case DistanceJoint:
		slot.distanceMotorSpeed = motorSpeed
	case PrismaticJoint:
		slot.prismaticMotorSpeed = motorSpeed
	case RevoluteJoint:
		slot.revoluteMotorSpeed = motorSpeed
	case WheelJoint:
		slot.wheelMotorSpeed = motorSpeed
	default:
		return ErrJointTypeMismatch
	}
	return nil
}

func (j *Joint) GetMotorSpeed() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return 0, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case DistanceJoint:
		return slot.distanceMotorSpeed, nil
	case PrismaticJoint:
		return slot.prismaticMotorSpeed, nil
	case RevoluteJoint:
		return slot.revoluteMotorSpeed, nil
	case WheelJoint:
		return slot.wheelMotorSpeed, nil
	default:
		return 0, ErrJointTypeMismatch
	}
}

func (j *Joint) SetMaxMotorForce(force float32) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return ErrJointInvalid
	}
	slot := &j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case DistanceJoint:
		slot.distanceMaxMotorForce = MaxFloat(0, force)
	case PrismaticJoint:
		slot.prismaticMaxMotorForce = MaxFloat(0, force)
	default:
		return ErrJointTypeMismatch
	}
	return nil
}

func (j *Joint) GetMaxMotorForce() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return 0, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case DistanceJoint:
		return slot.distanceMaxMotorForce, nil
	case PrismaticJoint:
		return slot.prismaticMaxMotorForce, nil
	default:
		return 0, ErrJointTypeMismatch
	}
}

func (j *Joint) GetMotorForce() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return 0, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case DistanceJoint:
		return slot.distanceMotorForce, nil
	case PrismaticJoint:
		return slot.prismaticMotorForce, nil
	default:
		return 0, ErrJointTypeMismatch
	}
}

func (j *Joint) SetLinearVelocity(velocity Vec2) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	slot, err := j.motorJointSlotLocked()
	if err != nil {
		return err
	}
	slot.motorLinearVelocity = velocity
	return nil
}

func (j *Joint) GetLinearVelocity() (Vec2, error) {
	if !j.IsValid() {
		return Vec2Zero, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	slot, err := j.motorJointSlotLocked()
	if err != nil {
		return Vec2Zero, err
	}
	return slot.motorLinearVelocity, nil
}

func (j *Joint) SetAngularVelocity(velocity float32) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	slot, err := j.motorJointSlotLocked()
	if err != nil {
		return err
	}
	slot.motorAngularVelocity = velocity
	return nil
}

func (j *Joint) GetAngularVelocity() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	slot, err := j.motorJointSlotLocked()
	if err != nil {
		return 0, err
	}
	return slot.motorAngularVelocity, nil
}

func (j *Joint) SetMaxVelocityForce(maxForce float32) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	slot, err := j.motorJointSlotLocked()
	if err != nil {
		return err
	}
	slot.motorMaxVelocityForce = MaxFloat(0, maxForce)
	return nil
}

func (j *Joint) GetMaxVelocityForce() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	slot, err := j.motorJointSlotLocked()
	if err != nil {
		return 0, err
	}
	return slot.motorMaxVelocityForce, nil
}

func (j *Joint) SetMaxVelocityTorque(maxTorque float32) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	slot, err := j.motorJointSlotLocked()
	if err != nil {
		return err
	}
	slot.motorMaxVelocityTorque = MaxFloat(0, maxTorque)
	return nil
}

func (j *Joint) GetMaxVelocityTorque() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	slot, err := j.motorJointSlotLocked()
	if err != nil {
		return 0, err
	}
	return slot.motorMaxVelocityTorque, nil
}

func (j *Joint) SetLinearHertz(hertz float32) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return ErrJointInvalid
	}
	hertz = MaxFloat(0, hertz)
	slot := &j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case MotorJoint:
		slot.motorLinearHertz = hertz
	case WeldJoint:
		slot.weldLinearHertz = hertz
	default:
		return ErrJointTypeMismatch
	}
	return nil
}

func (j *Joint) GetLinearHertz() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return 0, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case MotorJoint:
		return slot.motorLinearHertz, nil
	case WeldJoint:
		return slot.weldLinearHertz, nil
	default:
		return 0, ErrJointTypeMismatch
	}
}

func (j *Joint) SetLinearDampingRatio(damping float32) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return ErrJointInvalid
	}
	damping = MaxFloat(0, damping)
	slot := &j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case MotorJoint:
		slot.motorLinearDampingRatio = damping
	case WeldJoint:
		slot.weldLinearDampingRatio = damping
	default:
		return ErrJointTypeMismatch
	}
	return nil
}

func (j *Joint) GetLinearDampingRatio() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return 0, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case MotorJoint:
		return slot.motorLinearDampingRatio, nil
	case WeldJoint:
		return slot.weldLinearDampingRatio, nil
	default:
		return 0, ErrJointTypeMismatch
	}
}

func (j *Joint) SetAngularHertz(hertz float32) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return ErrJointInvalid
	}
	hertz = MaxFloat(0, hertz)
	slot := &j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case MotorJoint:
		slot.motorAngularHertz = hertz
	case WeldJoint:
		slot.weldAngularHertz = hertz
	default:
		return ErrJointTypeMismatch
	}
	return nil
}

func (j *Joint) GetAngularHertz() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return 0, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case MotorJoint:
		return slot.motorAngularHertz, nil
	case WeldJoint:
		return slot.weldAngularHertz, nil
	default:
		return 0, ErrJointTypeMismatch
	}
}

func (j *Joint) SetAngularDampingRatio(damping float32) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return ErrJointInvalid
	}
	damping = MaxFloat(0, damping)
	slot := &j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case MotorJoint:
		slot.motorAngularDampingRatio = damping
	case WeldJoint:
		slot.weldAngularDampingRatio = damping
	default:
		return ErrJointTypeMismatch
	}
	return nil
}

func (j *Joint) GetAngularDampingRatio() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return 0, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case MotorJoint:
		return slot.motorAngularDampingRatio, nil
	case WeldJoint:
		return slot.weldAngularDampingRatio, nil
	default:
		return 0, ErrJointTypeMismatch
	}
}

func (j *Joint) SetMaxSpringForce(maxForce float32) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	slot, err := j.motorJointSlotLocked()
	if err != nil {
		return err
	}
	slot.motorMaxSpringForce = MaxFloat(0, maxForce)
	return nil
}

func (j *Joint) GetMaxSpringForce() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	slot, err := j.motorJointSlotLocked()
	if err != nil {
		return 0, err
	}
	return slot.motorMaxSpringForce, nil
}

func (j *Joint) SetMaxSpringTorque(maxTorque float32) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	slot, err := j.motorJointSlotLocked()
	if err != nil {
		return err
	}
	slot.motorMaxSpringTorque = MaxFloat(0, maxTorque)
	return nil
}

func (j *Joint) GetMaxSpringTorque() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	slot, err := j.motorJointSlotLocked()
	if err != nil {
		return 0, err
	}
	return slot.motorMaxSpringTorque, nil
}

func (j *Joint) SetTargetTranslation(translation float32) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	slot, err := j.prismaticJointSlotLocked()
	if err != nil {
		return err
	}
	slot.prismaticTarget = translation
	return nil
}

func (j *Joint) GetTargetTranslation() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	slot, err := j.prismaticJointSlotLocked()
	if err != nil {
		return 0, err
	}
	return slot.prismaticTarget, nil
}

func (j *Joint) GetLowerLimit() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return 0, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case PrismaticJoint:
		return slot.prismaticLowerLimit, nil
	case RevoluteJoint:
		return slot.revoluteLowerAngle, nil
	case WheelJoint:
		return slot.wheelLowerTranslation, nil
	default:
		return 0, ErrJointTypeMismatch
	}
}

func (j *Joint) GetUpperLimit() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return 0, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case PrismaticJoint:
		return slot.prismaticUpperLimit, nil
	case RevoluteJoint:
		return slot.revoluteUpperAngle, nil
	case WheelJoint:
		return slot.wheelUpperTranslation, nil
	default:
		return 0, ErrJointTypeMismatch
	}
}

func (j *Joint) SetLimits(lower, upper float32) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return ErrJointInvalid
	}
	slot := &j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case PrismaticJoint:
		slot.prismaticLowerLimit = MinFloat(lower, upper)
		slot.prismaticUpperLimit = MaxFloat(lower, upper)
	case RevoluteJoint:
		limit := float32(0.99 * math.Pi)
		lower = ClampFloat(lower, -limit, limit)
		upper = ClampFloat(upper, -limit, limit)
		slot.revoluteLowerAngle = MinFloat(lower, upper)
		slot.revoluteUpperAngle = MaxFloat(lower, upper)
	case WheelJoint:
		slot.wheelLowerTranslation = MinFloat(lower, upper)
		slot.wheelUpperTranslation = MaxFloat(lower, upper)
	default:
		return ErrJointTypeMismatch
	}
	return nil
}

func (j *Joint) GetTranslation() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	slot, err := j.prismaticJointSlotLocked()
	if err != nil {
		return 0, err
	}
	bodyA := j.world.bodies[slot.bodyAIndex]
	bodyB := j.world.bodies[slot.bodyBIndex]

	transformA := Transform{P: bodyA.position, Q: bodyA.rotation}
	transformB := Transform{P: bodyB.position, Q: bodyB.rotation}

	localAxisA := RotateVector(slot.localFrameA.Q, Vec2{X: 1, Y: 0})
	axisA := RotateVector(transformA.Q, localAxisA)
	pA := TransformPoint(transformA, slot.localFrameA.P)
	pB := TransformPoint(transformB, slot.localFrameB.P)
	d := Sub(pB, pA)
	return Dot(d, axisA), nil
}

func (j *Joint) GetSpeed() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	slot, err := j.prismaticJointSlotLocked()
	if err != nil {
		return 0, err
	}
	bodyA := j.world.bodies[slot.bodyAIndex]
	bodyB := j.world.bodies[slot.bodyBIndex]

	localAxisA := RotateVector(slot.localFrameA.Q, Vec2{X: 1, Y: 0})
	axisA := RotateVector(bodyA.rotation, localAxisA)

	centerA := Add(bodyA.position, RotateVector(bodyA.rotation, bodyA.localCenter))
	centerB := Add(bodyB.position, RotateVector(bodyB.rotation, bodyB.localCenter))
	rA := RotateVector(bodyA.rotation, Sub(slot.localFrameA.P, bodyA.localCenter))
	rB := RotateVector(bodyB.rotation, Sub(slot.localFrameB.P, bodyB.localCenter))
	d := Add(Sub(centerB, centerA), Sub(rB, rA))

	vA := bodyA.linearVelocity
	vB := bodyB.linearVelocity
	wA := bodyA.angularVelocity
	wB := bodyB.angularVelocity
	vRel := Sub(Add(vB, CrossSV(wB, rB)), Add(vA, CrossSV(wA, rA)))
	return Dot(d, CrossSV(wA, axisA)) + Dot(axisA, vRel), nil
}

func (j *Joint) SetTargetAngle(angle float32) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	slot, err := j.revoluteJointSlotLocked()
	if err != nil {
		return err
	}
	slot.revoluteTargetAngle = angle
	return nil
}

func (j *Joint) GetTargetAngle() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	slot, err := j.revoluteJointSlotLocked()
	if err != nil {
		return 0, err
	}
	return slot.revoluteTargetAngle, nil
}

func (j *Joint) GetAngle() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	slot, err := j.revoluteJointSlotLocked()
	if err != nil {
		return 0, err
	}
	bodyA := j.world.bodies[slot.bodyAIndex]
	bodyB := j.world.bodies[slot.bodyBIndex]
	qA := MulRot(bodyA.rotation, slot.localFrameA.Q)
	qB := MulRot(bodyB.rotation, slot.localFrameB.Q)
	return RelativeAngle(qA, qB), nil
}

func (j *Joint) SetMaxMotorTorque(torque float32) error {
	if !j.IsValid() {
		return ErrJointInvalid
	}
	j.world.mu.Lock()
	defer j.world.mu.Unlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return ErrJointInvalid
	}
	slot := &j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case RevoluteJoint:
		slot.revoluteMaxMotorTorque = MaxFloat(0, torque)
	case WheelJoint:
		slot.wheelMaxMotorTorque = MaxFloat(0, torque)
	default:
		return ErrJointTypeMismatch
	}
	return nil
}

func (j *Joint) GetMaxMotorTorque() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return 0, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case RevoluteJoint:
		return slot.revoluteMaxMotorTorque, nil
	case WheelJoint:
		return slot.wheelMaxMotorTorque, nil
	default:
		return 0, ErrJointTypeMismatch
	}
}

func (j *Joint) GetMotorTorque() (float32, error) {
	if !j.IsValid() {
		return 0, ErrJointInvalid
	}
	j.world.mu.RLock()
	defer j.world.mu.RUnlock()
	if !j.world.isJointIDValidLocked(j.id) {
		return 0, ErrJointInvalid
	}
	slot := j.world.joints[int(j.id.Index1)-1]
	switch slot.jointType {
	case RevoluteJoint:
		return slot.revoluteMotorTorque, nil
	case WheelJoint:
		return slot.wheelMotorTorque, nil
	default:
		return 0, ErrJointTypeMismatch
	}
}
