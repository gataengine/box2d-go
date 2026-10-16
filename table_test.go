package box2d_test

import (
	"github.com/gataengine/box2d-go"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

const (
	setSpan   = 317
	itemCount = (setSpan*setSpan - setSpan) / 2
	testSize  = 1000
)

var _ = Describe("Table/HashSet", func() {
	It("matches helper power-of-two behavior", func() {
		power := box2d.BoundingPowerOf2(3008)
		Expect(power).To(Equal(12))

		nextPowerOf2 := box2d.RoundUpPowerOf2(3008)
		Expect(nextPowerOf2).To(Equal(1 << power))
	})

	It("passes basic create/destroy", func() {
		set := box2d.CreateSet(16)
		Expect(set.Count()).To(Equal(0))
		Expect(set.Capacity()).To(Equal(16))

		set.Destroy()
		Expect(set.Count()).To(Equal(0))
		Expect(set.Capacity()).To(Equal(0))
		Expect(set.Bytes()).To(Equal(0))
	})

	It("handles capacity rounding rules", func() {
		set := box2d.CreateSet(1)
		Expect(set.Capacity()).To(Equal(16))
		set.Destroy()

		set = box2d.CreateSet(15)
		Expect(set.Capacity()).To(Equal(16))
		set.Destroy()

		set = box2d.CreateSet(32)
		Expect(set.Capacity()).To(Equal(32))
		set.Destroy()

		set = box2d.CreateSet(33)
		Expect(set.Capacity()).To(Equal(64))
		set.Destroy()
	})

	It("handles add/remove/contains", func() {
		set := box2d.CreateSet(16)

		found := set.AddKey(42)
		Expect(found).To(BeFalse())
		Expect(set.Count()).To(Equal(1))

		found = set.AddKey(123)
		Expect(found).To(BeFalse())
		Expect(set.Count()).To(Equal(2))

		found = set.AddKey(42)
		Expect(found).To(BeTrue())
		Expect(set.Count()).To(Equal(2))

		Expect(set.ContainsKey(42)).To(BeTrue())
		Expect(set.ContainsKey(123)).To(BeTrue())
		Expect(set.ContainsKey(999)).To(BeFalse())

		removed := set.RemoveKey(42)
		Expect(removed).To(BeTrue())
		Expect(set.Count()).To(Equal(1))
		Expect(set.ContainsKey(42)).To(BeFalse())
		Expect(set.ContainsKey(123)).To(BeTrue())

		removed = set.RemoveKey(999)
		Expect(removed).To(BeFalse())
		Expect(set.Count()).To(Equal(1))

		removed = set.RemoveKey(42)
		Expect(removed).To(BeFalse())
		Expect(set.Count()).To(Equal(1))

		set.Destroy()
	})

	It("supports clear and reuse", func() {
		set := box2d.CreateSet(16)
		set.AddKey(10)
		set.AddKey(20)
		set.AddKey(30)
		Expect(set.Count()).To(Equal(3))

		set.Clear()
		Expect(set.Count()).To(Equal(0))
		Expect(set.ContainsKey(10)).To(BeFalse())
		Expect(set.ContainsKey(20)).To(BeFalse())
		Expect(set.ContainsKey(30)).To(BeFalse())

		set.AddKey(40)
		Expect(set.Count()).To(Equal(1))
		Expect(set.ContainsKey(40)).To(BeTrue())

		set.Destroy()
	})

	It("grows and retains keys", func() {
		set := box2d.CreateSet(16)
		initialCapacity := set.Capacity()

		for i := uint64(0); i < 8; i++ {
			set.AddKey(i + 1)
		}

		Expect(set.Capacity()).To(BeNumerically(">=", initialCapacity))
		Expect(set.Count()).To(Equal(8))
		for i := uint64(1); i <= 8; i++ {
			Expect(set.ContainsKey(i)).To(BeTrue())
		}
		set.Destroy()
	})

	It("handles edge key patterns", func() {
		set := box2d.CreateSet(16)

		largeKey := ^uint64(0) - 1
		set.AddKey(largeKey)
		Expect(set.ContainsKey(largeKey)).To(BeTrue())
		Expect(set.Count()).To(Equal(1))

		key1 := uint64(0x123456789ABCDEF)
		key2 := uint64(0x987654321FEDCBA)
		set.AddKey(key1)
		set.AddKey(key2)
		Expect(set.ContainsKey(key1)).To(BeTrue())
		Expect(set.ContainsKey(key2)).To(BeTrue())

		for i := uint64(0x1000); i < 0x1010; i++ {
			set.AddKey(i)
		}
		for i := uint64(0x1000); i < 0x1010; i++ {
			Expect(set.ContainsKey(i)).To(BeTrue())
		}

		set.Destroy()
	})

	It("reorganizes correctly after removals", func() {
		set := box2d.CreateSet(16)
		keys := []uint64{100, 116, 132, 148, 164}

		for _, k := range keys {
			set.AddKey(k)
		}
		for _, k := range keys {
			Expect(set.ContainsKey(k)).To(BeTrue())
		}

		set.RemoveKey(keys[2])
		Expect(set.ContainsKey(keys[2])).To(BeFalse())
		for i, k := range keys {
			if i == 2 {
				continue
			}
			Expect(set.ContainsKey(k)).To(BeTrue())
		}
		set.Destroy()
	})

	It("passes stress add/remove workflow", func() {
		set := box2d.CreateSet(32)
		keys := make([]uint64, testSize)
		for i := range testSize {
			keys[i] = uint64(i*7 + 13)
		}

		for i := range testSize {
			found := set.AddKey(keys[i])
			Expect(found).To(BeFalse())
		}
		Expect(set.Count()).To(Equal(testSize))

		for i := range testSize {
			Expect(set.ContainsKey(keys[i])).To(BeTrue())
		}

		removedCount := 0
		for i := 0; i < testSize; i += 2 {
			removed := set.RemoveKey(keys[i])
			Expect(removed).To(BeTrue())
			removedCount++
		}

		Expect(set.Count()).To(Equal(testSize - removedCount))
		for i := range testSize {
			shouldBePresent := i%2 == 1
			Expect(set.ContainsKey(keys[i])).To(Equal(shouldBePresent))
		}

		set.Destroy()
	})

	It("uses shape pair key canonicalization", func() {
		set := box2d.CreateSet(16)

		key1 := box2d.ShapePairKey(5, 10)
		key2 := box2d.ShapePairKey(10, 5)
		Expect(key1).To(Equal(key2))

		set.AddKey(key1)
		Expect(set.ContainsKey(key1)).To(BeTrue())
		Expect(set.ContainsKey(key2)).To(BeTrue())

		key3 := box2d.ShapePairKey(1, 2)
		key4 := box2d.ShapePairKey(2, 3)
		Expect(key3).NotTo(Equal(key4))

		set.AddKey(key3)
		set.AddKey(key4)
		Expect(set.Count()).To(Equal(3))

		set.Destroy()
	})

	It("reports bytes correctly", func() {
		set := box2d.CreateSet(32)
		expectedBytes := 32 * 8
		Expect(set.Bytes()).To(Equal(expectedBytes))

		set.AddKey(100)
		set.AddKey(200)
		Expect(set.Bytes()).To(Equal(expectedBytes))

		set.Destroy()
	})

	It("passes large pair-set workflow", func() {
		n := setSpan
		removed := make([]bool, itemCount)

		set := box2d.CreateSet(16)

		for i := range n {
			for j := i + 1; j < n; j++ {
				key := box2d.ShapePairKey(uint32(i), uint32(j))
				found := set.AddKey(key)
				Expect(found).To(BeFalse())
			}
		}
		Expect(set.Count()).To(Equal(itemCount))

		k := 0
		removeCount := 0
		for i := range n {
			for j := i + 1; j < n; j++ {
				if j == i+1 {
					key := box2d.ShapePairKey(uint32(i), uint32(j))
					size1 := set.Count()
					found := set.RemoveKey(key)
					Expect(found).To(BeTrue())
					size2 := set.Count()
					Expect(size2).To(Equal(size1 - 1))
					removed[k] = true
					removeCount++
				} else {
					removed[k] = false
				}
				k++
			}
		}
		Expect(set.Count()).To(Equal(itemCount - removeCount))

		k = 0
		for i := range n {
			for j := i + 1; j < n; j++ {
				key := box2d.ShapePairKey(uint32(j), uint32(i))
				found := set.ContainsKey(key)
				Expect(found || removed[k]).To(BeTrue())
				k++
			}
		}

		for i := range n {
			for j := i + 1; j < n; j++ {
				key := box2d.ShapePairKey(uint32(i), uint32(j))
				set.RemoveKey(key)
			}
		}

		Expect(set.Count()).To(Equal(0))
		set.Destroy()
	})
})
