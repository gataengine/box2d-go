package box2d

import "math/bits"

// BitSet provides fast operations on large arrays of bits.
type BitSet struct {
	bits          []uint64
	blockCapacity uint32
	blockCount    uint32
}

func blockSizeForBits(bitCapacity uint32) uint32 {
	return (bitCapacity + 64 - 1) / 64
}

// CreateBitSet creates a bitset with the requested bit capacity.
func CreateBitSet(bitCapacity uint32) BitSet {
	blockCapacity := blockSizeForBits(bitCapacity)
	return BitSet{
		bits:          make([]uint64, blockCapacity),
		blockCapacity: blockCapacity,
		blockCount:    0,
	}
}

// Destroy releases the bitset storage.
func (b *BitSet) Destroy() {
	b.bits = nil
	b.blockCapacity = 0
	b.blockCount = 0
}

// SetBitCountAndClear configures bit count and clears active blocks.
func (b *BitSet) SetBitCountAndClear(bitCount uint32) {
	blockCount := blockSizeForBits(bitCount)
	if b.blockCapacity < blockCount {
		newBitCapacity := bitCount + (bitCount >> 1)
		*b = CreateBitSet(newBitCapacity)
	}

	b.blockCount = blockCount
	for i := uint32(0); i < b.blockCount; i++ {
		b.bits[i] = 0
	}
}

// Grow grows active block count, growing capacity if needed.
func (b *BitSet) Grow(blockCount uint32) {
	if blockCount <= b.blockCount {
		return
	}
	if blockCount > b.blockCapacity {
		newCapacity := blockCount + blockCount/2
		newBits := make([]uint64, newCapacity)
		copy(newBits, b.bits)
		b.bits = newBits
		b.blockCapacity = newCapacity
	}
	b.blockCount = blockCount
}

// InPlaceUnion ORs this set with other. Requires matching blockCount.
func (b *BitSet) InPlaceUnion(other *BitSet) bool {
	if b.blockCount != other.blockCount {
		return false
	}
	for i := uint32(0); i < b.blockCount; i++ {
		b.bits[i] |= other.bits[i]
	}
	return true
}

// SetBit sets a bit if it is in range.
func (b *BitSet) SetBit(bitIndex uint32) bool {
	blockIndex := bitIndex / 64
	if blockIndex >= b.blockCount {
		return false
	}
	b.bits[blockIndex] |= uint64(1) << (bitIndex % 64)
	return true
}

// SetBitGrow sets a bit and grows if needed.
func (b *BitSet) SetBitGrow(bitIndex uint32) {
	blockIndex := bitIndex / 64
	if blockIndex >= b.blockCount {
		b.Grow(blockIndex + 1)
	}
	b.bits[blockIndex] |= uint64(1) << (bitIndex % 64)
}

// ClearBit clears a bit if it is in range.
func (b *BitSet) ClearBit(bitIndex uint32) {
	blockIndex := bitIndex / 64
	if blockIndex >= b.blockCount {
		return
	}
	b.bits[blockIndex] &^= uint64(1) << (bitIndex % 64)
}

// GetBit gets a bit, returning false when out of range.
func (b *BitSet) GetBit(bitIndex uint32) bool {
	blockIndex := bitIndex / 64
	if blockIndex >= b.blockCount {
		return false
	}
	return (b.bits[blockIndex] & (uint64(1) << (bitIndex % 64))) != 0
}

// CountSetBits counts set bits across active blocks.
func (b *BitSet) CountSetBits() int {
	total := 0
	for i := uint32(0); i < b.blockCount; i++ {
		total += bits.OnesCount64(b.bits[i])
	}
	return total
}

// Bytes returns allocated backing-store size in bytes.
func (b *BitSet) Bytes() int {
	return int(b.blockCapacity * 8)
}
