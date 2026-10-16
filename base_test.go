package box2d_test

import (
	"time"

	"github.com/gataengine/box2d-go"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

var _ = Describe("Base APIs", func() {
	It("returns upstream version numbers", func() {
		v := box2d.GetVersion()
		Expect(v.Major).To(Equal(3))
		Expect(v.Minor).To(Equal(2))
		Expect(v.Revision).To(Equal(0))
	})

	It("computes djb2 hash deterministically", func() {
		data := []byte("box2d")
		h := box2d.Hash(box2d.HashInit, data, len(data))
		Expect(h).To(Equal(uint32(254493924)))
	})

	It("provides timing helpers", func() {
		ticks := box2d.GetTicks()
		time.Sleep(2 * time.Millisecond)
		ms := box2d.GetMilliseconds(ticks)
		Expect(ms).To(BeNumerically(">", float32(0)))

		ms2 := box2d.GetMillisecondsAndReset(&ticks)
		Expect(ms2).To(BeNumerically(">=", float32(0)))
	})

	It("invokes configured assert and log callbacks", func() {
		calledAssert := false
		box2d.SetAssertFcn(func(condition string, fileName string, lineNumber int) int {
			calledAssert = true
			Expect(condition).To(Equal("x"))
			Expect(fileName).To(Equal("f.go"))
			Expect(lineNumber).To(Equal(42))
			return 0
		})
		ret := box2d.InternalAssertFcn("x", "f.go", 42)
		Expect(ret).To(Equal(0))
		Expect(calledAssert).To(BeTrue())

		calledLog := false
		box2d.SetLogFcn(func(message string) {
			calledLog = true
			Expect(message).To(Equal("warn"))
		})
		box2d.Log("warn")
		Expect(calledLog).To(BeTrue())
	})
})
