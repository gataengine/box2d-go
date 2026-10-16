package box2d_test

import (
	"github.com/gataengine/box2d-go"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

var _ = Describe("IdPool", func() {
	It("allocates sequential IDs and reuses freed IDs", func() {
		pool := box2d.CreateIdPool()

		id0 := pool.AllocID()
		id1 := pool.AllocID()
		id2 := pool.AllocID()
		Expect(id0).To(Equal(0))
		Expect(id1).To(Equal(1))
		Expect(id2).To(Equal(2))
		Expect(pool.IDCount()).To(Equal(3))
		Expect(pool.IDCapacity()).To(Equal(3))

		Expect(pool.FreeID(id1)).To(BeTrue())
		Expect(pool.ValidateFreeID(id1)).To(BeTrue())
		Expect(pool.ValidateUsedID(id1)).To(BeFalse())
		Expect(pool.IDCount()).To(Equal(2))

		reused := pool.AllocID()
		Expect(reused).To(Equal(id1))
		Expect(pool.ValidateUsedID(id1)).To(BeTrue())
		Expect(pool.IDCount()).To(Equal(3))

		pool.Destroy()
		Expect(pool.IDCount()).To(Equal(0))
		Expect(pool.IDCapacity()).To(Equal(0))
	})

	It("rejects freeing invalid ids", func() {
		pool := box2d.CreateIdPool()
		Expect(pool.FreeID(0)).To(BeFalse())

		_ = pool.AllocID()
		Expect(pool.FreeID(-1)).To(BeFalse())
		Expect(pool.FreeID(5)).To(BeFalse())
	})
})
