package box2d_test

import (
	"github.com/gataengine/box2d-go"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

var _ = Describe("BitSet", func() {
	It("matches upstream test_bitset.c behavior", func() {
		const count = 169

		bitSet := box2d.CreateBitSet(count)
		bitSet.SetBitCountAndClear(count)

		values := make([]bool, count)

		i1 := uint32(0)
		i2 := uint32(1)
		Expect(bitSet.SetBit(i1)).To(BeTrue())
		values[i1] = true

		for i2 < count {
			Expect(bitSet.SetBit(i2)).To(BeTrue())
			values[i2] = true
			next := i1 + i2
			i1 = i2
			i2 = next
		}

		for i := uint32(0); i < count; i++ {
			value := bitSet.GetBit(i)
			Expect(value).To(Equal(values[i]))
		}

		bitSet.Destroy()
		Expect(bitSet.Bytes()).To(Equal(0))
	})
})
