package box2d_test

import (
	"github.com/gataengine/box2d-go"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

var _ = Describe("ID Encoding", func() {
	It("matches upstream test_id.c pack/unpack behavior", func() {
		a := uint32(0x01234567)

		worldID := box2d.LoadWorldID(a)
		b := box2d.StoreWorldID(worldID)
		Expect(b).To(Equal(a))

		x := uint64(0x0123456789ABCDEF)

		bodyID := box2d.LoadBodyID(x)
		y := box2d.StoreBodyID(bodyID)
		Expect(y).To(Equal(x))

		shapeID := box2d.LoadShapeID(x)
		y = box2d.StoreShapeID(shapeID)
		Expect(y).To(Equal(x))

		chainID := box2d.LoadChainID(x)
		y = box2d.StoreChainID(chainID)
		Expect(y).To(Equal(x))

		jointID := box2d.LoadJointID(x)
		y = box2d.StoreJointID(jointID)
		Expect(y).To(Equal(x))
	})
})
