package box2d_test

import (
	"github.com/gataengine/box2d-go"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

var _ = Describe("Default Types", func() {
	It("matches upstream default world definition values", func() {
		box2d.SetLengthUnitsPerMeter(1.0)
		def := box2d.DefaultWorldDef()

		Expect(def.Gravity).To(Equal(box2d.Vec2{X: 0.0, Y: -10.0}))
		Expect(def.HitEventThreshold).To(BeNumerically("==", float32(1.0)))
		Expect(def.RestitutionThreshold).To(BeNumerically("==", float32(1.0)))
		Expect(def.ContactSpeed).To(BeNumerically("==", float32(3.0)))
		Expect(def.ContactHertz).To(BeNumerically("==", float32(30.0)))
		Expect(def.ContactDampingRatio).To(BeNumerically("==", float32(10.0)))
		Expect(def.MaximumLinearSpeed).To(BeNumerically("==", float32(400.0)))
		Expect(def.EnableSleep).To(BeTrue())
		Expect(def.EnableContinuous).To(BeTrue())
	})

	It("matches upstream default body definition values", func() {
		box2d.SetLengthUnitsPerMeter(1.0)
		def := box2d.DefaultBodyDef()

		Expect(def.Type).To(Equal(box2d.StaticBody))
		Expect(def.Rotation).To(Equal(box2d.RotIdentity))
		Expect(def.SleepThreshold).To(BeNumerically("==", float32(0.05)))
		Expect(def.GravityScale).To(BeNumerically("==", float32(1.0)))
		Expect(def.EnableSleep).To(BeTrue())
		Expect(def.IsAwake).To(BeTrue())
		Expect(def.IsEnabled).To(BeTrue())
	})

	It("matches upstream default filter values", func() {
		filter := box2d.DefaultFilter()
		Expect(filter.CategoryBits).To(Equal(uint64(1)))
		Expect(filter.MaskBits).To(Equal(^uint64(0)))
		Expect(filter.GroupIndex).To(Equal(0))

		queryFilter := box2d.DefaultQueryFilter()
		Expect(queryFilter.CategoryBits).To(Equal(uint64(1)))
		Expect(queryFilter.MaskBits).To(Equal(^uint64(0)))
	})
})
