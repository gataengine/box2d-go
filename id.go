package box2d

// WorldID references a world instance. Treat this as an opaque handle.
type WorldID struct {
	Index1     uint16
	Generation uint16
}

// BodyID references a body instance. Treat this as an opaque handle.
type BodyID struct {
	Index1     int32
	World0     uint16
	Generation uint16
}

// ShapeID references a shape instance. Treat this as an opaque handle.
type ShapeID struct {
	Index1     int32
	World0     uint16
	Generation uint16
}

// ChainID references a chain instance. Treat this as an opaque handle.
type ChainID struct {
	Index1     int32
	World0     uint16
	Generation uint16
}

// JointID references a joint instance. Treat this as an opaque handle.
type JointID struct {
	Index1     int32
	World0     uint16
	Generation uint16
}

// ContactID references a contact instance. Treat this as an opaque handle.
type ContactID struct {
	Index1     int32
	World0     uint16
	Padding    int16
	Generation uint32
}

var (
	NullWorldID   = WorldID{}
	NullBodyID    = BodyID{}
	NullShapeID   = ShapeID{}
	NullChainID   = ChainID{}
	NullJointID   = JointID{}
	NullContactID = ContactID{}
)

func (id WorldID) IsNull() bool   { return id.Index1 == 0 }
func (id BodyID) IsNull() bool    { return id.Index1 == 0 }
func (id ShapeID) IsNull() bool   { return id.Index1 == 0 }
func (id ChainID) IsNull() bool   { return id.Index1 == 0 }
func (id JointID) IsNull() bool   { return id.Index1 == 0 }
func (id ContactID) IsNull() bool { return id.Index1 == 0 }

func (id WorldID) IsNonNull() bool   { return !id.IsNull() }
func (id BodyID) IsNonNull() bool    { return !id.IsNull() }
func (id ShapeID) IsNonNull() bool   { return !id.IsNull() }
func (id ChainID) IsNonNull() bool   { return !id.IsNull() }
func (id JointID) IsNonNull() bool   { return !id.IsNull() }
func (id ContactID) IsNonNull() bool { return !id.IsNull() }

func BodyIDEqual(a, b BodyID) bool {
	return a.Index1 == b.Index1 && a.World0 == b.World0 && a.Generation == b.Generation
}

func ShapeIDEqual(a, b ShapeID) bool {
	return a.Index1 == b.Index1 && a.World0 == b.World0 && a.Generation == b.Generation
}

func ChainIDEqual(a, b ChainID) bool {
	return a.Index1 == b.Index1 && a.World0 == b.World0 && a.Generation == b.Generation
}

func JointIDEqual(a, b JointID) bool {
	return a.Index1 == b.Index1 && a.World0 == b.World0 && a.Generation == b.Generation
}

// StoreWorldID packs a WorldID into a uint32.
func StoreWorldID(id WorldID) uint32 {
	return uint32(id.Index1)<<16 | uint32(id.Generation)
}

// LoadWorldID unpacks a uint32 into a WorldID.
func LoadWorldID(x uint32) WorldID {
	return WorldID{
		Index1:     uint16(x >> 16),
		Generation: uint16(x),
	}
}

// StoreBodyID packs a BodyID into a uint64.
func StoreBodyID(id BodyID) uint64 {
	return uint64(id.Index1)<<32 | uint64(id.World0)<<16 | uint64(id.Generation)
}

// LoadBodyID unpacks a uint64 into a BodyID.
func LoadBodyID(x uint64) BodyID {
	return BodyID{
		Index1:     int32(x >> 32),
		World0:     uint16(x >> 16),
		Generation: uint16(x),
	}
}

// StoreShapeID packs a ShapeID into a uint64.
func StoreShapeID(id ShapeID) uint64 {
	return uint64(id.Index1)<<32 | uint64(id.World0)<<16 | uint64(id.Generation)
}

// LoadShapeID unpacks a uint64 into a ShapeID.
func LoadShapeID(x uint64) ShapeID {
	return ShapeID{
		Index1:     int32(x >> 32),
		World0:     uint16(x >> 16),
		Generation: uint16(x),
	}
}

// StoreChainID packs a ChainID into a uint64.
func StoreChainID(id ChainID) uint64 {
	return uint64(id.Index1)<<32 | uint64(id.World0)<<16 | uint64(id.Generation)
}

// LoadChainID unpacks a uint64 into a ChainID.
func LoadChainID(x uint64) ChainID {
	return ChainID{
		Index1:     int32(x >> 32),
		World0:     uint16(x >> 16),
		Generation: uint16(x),
	}
}

// StoreJointID packs a JointID into a uint64.
func StoreJointID(id JointID) uint64 {
	return uint64(id.Index1)<<32 | uint64(id.World0)<<16 | uint64(id.Generation)
}

// LoadJointID unpacks a uint64 into a JointID.
func LoadJointID(x uint64) JointID {
	return JointID{
		Index1:     int32(x >> 32),
		World0:     uint16(x >> 16),
		Generation: uint16(x),
	}
}

// StoreContactID stores a ContactID in three uint32 values.
func StoreContactID(id ContactID) [3]uint32 {
	return [3]uint32{
		uint32(id.Index1),
		uint32(id.World0),
		id.Generation,
	}
}

// LoadContactID loads a ContactID from three uint32 values.
func LoadContactID(values [3]uint32) ContactID {
	return ContactID{
		Index1:     int32(values[0]),
		World0:     uint16(values[1]),
		Padding:    0,
		Generation: values[2],
	}
}
