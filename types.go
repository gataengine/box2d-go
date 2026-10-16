package box2d

import "math"

const (
	defaultCategoryBits = 1
	defaultMaskBits     = math.MaxUint64
	secretCookie        = 1152023
)

// WorldDef defines world creation parameters.
type WorldDef struct {
	Gravity                 Vec2
	RestitutionThreshold    float32
	HitEventThreshold       float32
	ContactHertz            float32
	ContactDampingRatio     float32
	ContactSpeed            float32
	MaximumLinearSpeed      float32
	EnableSleep             bool
	EnableContinuous        bool
	EnableContactSoftening  bool
	WorkerCount             int
	UserData                any
	internalValidationValue int
}

// DefaultWorldDef returns the Box2D default world definition.
func DefaultWorldDef() WorldDef {
	units := GetLengthUnitsPerMeter()
	return WorldDef{
		Gravity:                 Vec2{X: 0.0, Y: -10.0},
		RestitutionThreshold:    1.0 * units,
		HitEventThreshold:       1.0 * units,
		ContactHertz:            30.0,
		ContactDampingRatio:     10.0,
		ContactSpeed:            3.0 * units,
		MaximumLinearSpeed:      400.0 * units,
		EnableSleep:             true,
		EnableContinuous:        true,
		EnableContactSoftening:  false,
		WorkerCount:             0,
		UserData:                nil,
		internalValidationValue: secretCookie,
	}
}

// BodyType is the rigid body simulation type.
type BodyType int

const (
	StaticBody BodyType = iota
	KinematicBody
	DynamicBody
)

// MotionLocks restrict body movement axes.
type MotionLocks struct {
	LinearX  bool
	LinearY  bool
	AngularZ bool
}

// Motion lock bitmask constants for efficient solver checks.
const (
	lockFlagLinearX  uint8 = 1 << iota // 0x01
	lockFlagLinearY                    // 0x02
	lockFlagAngularZ                   // 0x04
)

// motionLockFlags converts MotionLocks to a bitmask.
func motionLockFlags(m MotionLocks) uint8 {
	var f uint8
	if m.LinearX {
		f |= lockFlagLinearX
	}
	if m.LinearY {
		f |= lockFlagLinearY
	}
	if m.AngularZ {
		f |= lockFlagAngularZ
	}
	return f
}

// BodyDef defines body creation parameters.
type BodyDef struct {
	Type                    BodyType
	Position                Vec2
	Rotation                Rot
	LinearVelocity          Vec2
	AngularVelocity         float32
	LinearDamping           float32
	AngularDamping          float32
	GravityScale            float32
	SleepThreshold          float32
	Name                    string
	UserData                any
	MotionLocks             MotionLocks
	EnableSleep             bool
	IsAwake                 bool
	IsBullet                bool
	IsEnabled               bool
	AllowFastRotation       bool
	internalValidationValue int
}

// DefaultBodyDef returns the Box2D default body definition.
func DefaultBodyDef() BodyDef {
	return BodyDef{
		Type:                    StaticBody,
		Position:                Vec2{},
		Rotation:                RotIdentity,
		LinearVelocity:          Vec2{},
		AngularVelocity:         0,
		LinearDamping:           0,
		AngularDamping:          0,
		GravityScale:            1.0,
		SleepThreshold:          0.05 * GetLengthUnitsPerMeter(),
		Name:                    "",
		UserData:                nil,
		MotionLocks:             MotionLocks{},
		EnableSleep:             true,
		IsAwake:                 true,
		IsBullet:                false,
		IsEnabled:               true,
		AllowFastRotation:       false,
		internalValidationValue: secretCookie,
	}
}

// Filter controls shape collision filtering.
type Filter struct {
	CategoryBits uint64
	MaskBits     uint64
	GroupIndex   int
}

// DefaultFilter returns the Box2D default filter.
func DefaultFilter() Filter {
	return Filter{
		CategoryBits: defaultCategoryBits,
		MaskBits:     defaultMaskBits,
		GroupIndex:   0,
	}
}

// QueryFilter controls query-vs-shape filtering.
type QueryFilter struct {
	CategoryBits uint64
	MaskBits     uint64
}

// DefaultQueryFilter returns the Box2D default query filter.
func DefaultQueryFilter() QueryFilter {
	return QueryFilter{
		CategoryBits: defaultCategoryBits,
		MaskBits:     defaultMaskBits,
	}
}

type ExplosionDef struct {
	MaskBits                uint64
	Position                Vec2
	Radius                  float32
	Falloff                 float32
	ImpulsePerLength        float32
	internalValidationValue int
}

func DefaultExplosionDef() ExplosionDef {
	return ExplosionDef{
		MaskBits:                defaultMaskBits,
		Position:                Vec2Zero,
		Radius:                  0,
		Falloff:                 0,
		ImpulsePerLength:        0,
		internalValidationValue: secretCookie,
	}
}

type ShapeType int

const (
	CircleShape ShapeType = iota
	CapsuleShape
	SegmentShape
	PolygonShape
	ChainSegmentShape
)

type JointType int

const (
	UnknownJoint JointType = iota
	DistanceJoint
	MotorJoint
	FilterJoint
	PrismaticJoint
	RevoluteJoint
	WeldJoint
	WheelJoint
)

type JointDef struct {
	UserData                any
	BodyIDA                 BodyID
	BodyIDB                 BodyID
	LocalFrameA             Transform
	LocalFrameB             Transform
	ForceThreshold          float32
	TorqueThreshold         float32
	ConstraintHertz         float32
	ConstraintDampingRatio  float32
	DrawScale               float32
	CollideConnected        bool
	internalValidationValue int
}

func DefaultJointDef() JointDef {
	return JointDef{
		UserData:                nil,
		BodyIDA:                 NullBodyID,
		BodyIDB:                 NullBodyID,
		LocalFrameA:             TransformIdentity,
		LocalFrameB:             TransformIdentity,
		ForceThreshold:          float32(math.MaxFloat32),
		TorqueThreshold:         float32(math.MaxFloat32),
		ConstraintHertz:         60.0,
		ConstraintDampingRatio:  2.0,
		DrawScale:               GetLengthUnitsPerMeter(),
		CollideConnected:        false,
		internalValidationValue: secretCookie,
	}
}

type DistanceJointDef struct {
	Base                    JointDef
	Length                  float32
	EnableSpring            bool
	LowerSpringForce        float32
	UpperSpringForce        float32
	Hertz                   float32
	DampingRatio            float32
	EnableLimit             bool
	MinLength               float32
	MaxLength               float32
	EnableMotor             bool
	MaxMotorForce           float32
	MotorSpeed              float32
	internalValidationValue int
}

func DefaultDistanceJointDef() DistanceJointDef {
	return DistanceJointDef{
		Base:                    DefaultJointDef(),
		Length:                  1.0,
		EnableSpring:            false,
		LowerSpringForce:        -float32(math.MaxFloat32),
		UpperSpringForce:        float32(math.MaxFloat32),
		Hertz:                   0,
		DampingRatio:            0,
		EnableLimit:             false,
		MinLength:               0,
		MaxLength:               float32(math.MaxFloat32),
		EnableMotor:             false,
		MaxMotorForce:           0,
		MotorSpeed:              0,
		internalValidationValue: secretCookie,
	}
}

type MotorJointDef struct {
	Base                    JointDef
	LinearVelocity          Vec2
	MaxVelocityForce        float32
	AngularVelocity         float32
	MaxVelocityTorque       float32
	LinearHertz             float32
	LinearDampingRatio      float32
	MaxSpringForce          float32
	AngularHertz            float32
	AngularDampingRatio     float32
	MaxSpringTorque         float32
	internalValidationValue int
}

func DefaultMotorJointDef() MotorJointDef {
	return MotorJointDef{
		Base:                    DefaultJointDef(),
		LinearVelocity:          Vec2Zero,
		MaxVelocityForce:        0,
		AngularVelocity:         0,
		MaxVelocityTorque:       0,
		LinearHertz:             0,
		LinearDampingRatio:      0,
		MaxSpringForce:          0,
		AngularHertz:            0,
		AngularDampingRatio:     0,
		MaxSpringTorque:         0,
		internalValidationValue: secretCookie,
	}
}

type FilterJointDef struct {
	Base                    JointDef
	internalValidationValue int
}

func DefaultFilterJointDef() FilterJointDef {
	return FilterJointDef{
		Base:                    DefaultJointDef(),
		internalValidationValue: secretCookie,
	}
}

type PrismaticJointDef struct {
	Base                    JointDef
	EnableSpring            bool
	Hertz                   float32
	DampingRatio            float32
	TargetTranslation       float32
	EnableLimit             bool
	LowerTranslation        float32
	UpperTranslation        float32
	EnableMotor             bool
	MaxMotorForce           float32
	MotorSpeed              float32
	internalValidationValue int
}

func DefaultPrismaticJointDef() PrismaticJointDef {
	return PrismaticJointDef{
		Base:                    DefaultJointDef(),
		EnableSpring:            false,
		Hertz:                   0,
		DampingRatio:            0,
		TargetTranslation:       0,
		EnableLimit:             false,
		LowerTranslation:        0,
		UpperTranslation:        0,
		EnableMotor:             false,
		MaxMotorForce:           0,
		MotorSpeed:              0,
		internalValidationValue: secretCookie,
	}
}

type RevoluteJointDef struct {
	Base                    JointDef
	TargetAngle             float32
	EnableSpring            bool
	Hertz                   float32
	DampingRatio            float32
	EnableLimit             bool
	LowerAngle              float32
	UpperAngle              float32
	EnableMotor             bool
	MaxMotorTorque          float32
	MotorSpeed              float32
	internalValidationValue int
}

func DefaultRevoluteJointDef() RevoluteJointDef {
	return RevoluteJointDef{
		Base:                    DefaultJointDef(),
		TargetAngle:             0,
		EnableSpring:            false,
		Hertz:                   0,
		DampingRatio:            0,
		EnableLimit:             false,
		LowerAngle:              0,
		UpperAngle:              0,
		EnableMotor:             false,
		MaxMotorTorque:          0,
		MotorSpeed:              0,
		internalValidationValue: secretCookie,
	}
}

type WeldJointDef struct {
	Base                    JointDef
	LinearHertz             float32
	AngularHertz            float32
	LinearDampingRatio      float32
	AngularDampingRatio     float32
	internalValidationValue int
}

func DefaultWeldJointDef() WeldJointDef {
	return WeldJointDef{
		Base:                    DefaultJointDef(),
		LinearHertz:             0,
		AngularHertz:            0,
		LinearDampingRatio:      0,
		AngularDampingRatio:     0,
		internalValidationValue: secretCookie,
	}
}

type WheelJointDef struct {
	Base                    JointDef
	EnableSpring            bool
	Hertz                   float32
	DampingRatio            float32
	EnableLimit             bool
	LowerTranslation        float32
	UpperTranslation        float32
	EnableMotor             bool
	MaxMotorTorque          float32
	MotorSpeed              float32
	internalValidationValue int
}

func DefaultWheelJointDef() WheelJointDef {
	return WheelJointDef{
		Base:                    DefaultJointDef(),
		EnableSpring:            true,
		Hertz:                   1.0,
		DampingRatio:            0.7,
		EnableLimit:             false,
		LowerTranslation:        0,
		UpperTranslation:        0,
		EnableMotor:             false,
		MaxMotorTorque:          0,
		MotorSpeed:              0,
		internalValidationValue: secretCookie,
	}
}

type SurfaceMaterial struct {
	Friction          float32
	Restitution       float32
	RollingResistance float32
	TangentSpeed      float32
	UserMaterialID    uint64
	CustomColor       uint32
}

func DefaultSurfaceMaterial() SurfaceMaterial {
	return SurfaceMaterial{
		Friction:          0.6,
		Restitution:       0,
		RollingResistance: 0,
		TangentSpeed:      0,
		UserMaterialID:    0,
		CustomColor:       0,
	}
}

type ShapeDef struct {
	UserData                any
	Material                SurfaceMaterial
	Density                 float32
	Filter                  Filter
	EnableCustomFiltering   bool
	IsSensor                bool
	EnableSensorEvents      bool
	EnableContactEvents     bool
	EnableHitEvents         bool
	EnablePreSolveEvents    bool
	InvokeContactCreation   bool
	UpdateBodyMass          bool
	internalValidationValue int
}

func DefaultShapeDef() ShapeDef {
	return ShapeDef{
		UserData:                nil,
		Material:                DefaultSurfaceMaterial(),
		Density:                 1.0,
		Filter:                  DefaultFilter(),
		EnableCustomFiltering:   false,
		IsSensor:                false,
		EnableSensorEvents:      false,
		EnableContactEvents:     false,
		EnableHitEvents:         false,
		EnablePreSolveEvents:    false,
		InvokeContactCreation:   true,
		UpdateBodyMass:          true,
		internalValidationValue: secretCookie,
	}
}

type ChainDef struct {
	UserData                any
	Points                  []Vec2
	Count                   int
	Materials               []SurfaceMaterial
	MaterialCount           int
	Filter                  Filter
	IsLoop                  bool
	EnableSensorEvents      bool
	internalValidationValue int
}

func DefaultChainDef() ChainDef {
	defaultMaterial := DefaultSurfaceMaterial()
	return ChainDef{
		UserData:                nil,
		Points:                  nil,
		Count:                   0,
		Materials:               []SurfaceMaterial{defaultMaterial},
		MaterialCount:           1,
		Filter:                  DefaultFilter(),
		IsLoop:                  false,
		EnableSensorEvents:      false,
		internalValidationValue: secretCookie,
	}
}

func (def WorldDef) isInitialized() bool {
	return def.internalValidationValue == secretCookie
}

func (def BodyDef) isInitialized() bool {
	return def.internalValidationValue == secretCookie
}

func (def ShapeDef) isInitialized() bool {
	return def.internalValidationValue == secretCookie
}

func (def ChainDef) isInitialized() bool {
	return def.internalValidationValue == secretCookie
}

func (def ExplosionDef) isInitialized() bool {
	return def.internalValidationValue == secretCookie
}

func (def JointDef) isInitialized() bool {
	return def.internalValidationValue == secretCookie
}

func (def DistanceJointDef) isInitialized() bool {
	return def.internalValidationValue == secretCookie && def.Base.isInitialized()
}

func (def MotorJointDef) isInitialized() bool {
	return def.internalValidationValue == secretCookie && def.Base.isInitialized()
}

func (def FilterJointDef) isInitialized() bool {
	return def.internalValidationValue == secretCookie && def.Base.isInitialized()
}

func (def PrismaticJointDef) isInitialized() bool {
	return def.internalValidationValue == secretCookie && def.Base.isInitialized()
}

func (def RevoluteJointDef) isInitialized() bool {
	return def.internalValidationValue == secretCookie && def.Base.isInitialized()
}

func (def WeldJointDef) isInitialized() bool {
	return def.internalValidationValue == secretCookie && def.Base.isInitialized()
}

func (def WheelJointDef) isInitialized() bool {
	return def.internalValidationValue == secretCookie && def.Base.isInitialized()
}
