package box2d_test

import (
	"github.com/gataengine/box2d-go"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

var _ = Describe("IntArray", func() {
	It("supports push/pop/remove/clear behavior", func() {
		a := box2d.IntArrayCreate(0)
		Expect(a.Count()).To(Equal(0))
		Expect(a.Capacity()).To(Equal(0))

		a.Push(10)
		a.Push(20)
		a.Push(30)
		Expect(a.Count()).To(Equal(3))
		Expect(a.Get(0)).To(Equal(10))
		Expect(a.Get(1)).To(Equal(20))
		Expect(a.Get(2)).To(Equal(30))

		moved := a.RemoveSwap(1)
		Expect(moved).To(Equal(2))
		Expect(a.Count()).To(Equal(2))
		Expect(a.Get(1)).To(Equal(30))

		v := a.Pop()
		Expect(v).To(Equal(30))
		Expect(a.Count()).To(Equal(1))

		a.Clear()
		Expect(a.Count()).To(Equal(0))
	})
})
