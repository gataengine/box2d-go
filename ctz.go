package box2d

import "math/bits"

func CTZ32(block uint32) uint32 {
	return uint32(bits.TrailingZeros32(block))
}

func CLZ32(value uint32) uint32 {
	return uint32(bits.LeadingZeros32(value))
}

func CTZ64(block uint64) uint32 {
	return uint32(bits.TrailingZeros64(block))
}

func PopCount64(block uint64) int {
	return bits.OnesCount64(block)
}

func IsPowerOf2(x int) bool {
	return (x & (x - 1)) == 0
}

func BoundingPowerOf2(x int) int {
	if x <= 1 {
		return 1
	}
	return 32 - int(CLZ32(uint32(x)-1))
}

func RoundUpPowerOf2(x int) int {
	if x <= 1 {
		return 1
	}
	return 1 << (32 - int(CLZ32(uint32(x)-1)))
}
