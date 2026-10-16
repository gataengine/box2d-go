package box2d_test

import (
	"github.com/gataengine/box2d-go"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

var _ = Describe("Chain APIs", func() {
	It("creates and destroys loop chains with valid segment ids", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())
		defer func() { _ = world.Destroy() }()

		body, err := world.CreateBody(box2d.DefaultBodyDef())
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultChainDef()
		def.IsLoop = true
		def.Points = []box2d.Vec2{
			{X: -1, Y: -1},
			{X: 1, Y: -1},
			{X: 1, Y: 1},
			{X: -1, Y: 1},
		}
		def.Count = len(def.Points)

		chainID, err := world.CreateChain(body.ID(), def)
		Expect(err).NotTo(HaveOccurred())
		Expect(box2d.IsChainIDValid(chainID)).To(BeTrue())
		counters, err := world.Counters()
		Expect(err).NotTo(HaveOccurred())
		Expect(counters.ChainCount).To(Equal(1))
		chainCount, err := world.GetChainCount()
		Expect(err).NotTo(HaveOccurred())
		Expect(chainCount).To(Equal(1))

		segmentCount, err := world.GetChainSegmentCount(chainID)
		Expect(err).NotTo(HaveOccurred())
		Expect(segmentCount).To(Equal(4))

		segments, err := world.GetChainSegments(chainID, 8)
		Expect(err).NotTo(HaveOccurred())
		Expect(segments).To(HaveLen(4))
		for _, seg := range segments {
			Expect(box2d.IsShapeIDValid(seg)).To(BeTrue())
		}

		bodyShapeCount, err := body.GetShapeCount()
		Expect(err).NotTo(HaveOccurred())
		Expect(bodyShapeCount).To(Equal(4))

		Expect(world.DestroyChain(chainID)).To(Succeed())
		Expect(box2d.IsChainIDValid(chainID)).To(BeFalse())
		counters, err = world.Counters()
		Expect(err).NotTo(HaveOccurred())
		Expect(counters.ChainCount).To(Equal(0))
		chainCount, err = world.GetChainCount()
		Expect(err).NotTo(HaveOccurred())
		Expect(chainCount).To(Equal(0))
		Expect(world.DestroyChain(chainID)).To(MatchError(box2d.ErrChainInvalid))
		for _, seg := range segments {
			Expect(box2d.IsShapeIDValid(seg)).To(BeFalse())
		}

		bodyShapeCount, err = body.GetShapeCount()
		Expect(err).NotTo(HaveOccurred())
		Expect(bodyShapeCount).To(Equal(0))
	})

	It("invalidates attached chains when destroying a body", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())
		defer func() { _ = world.Destroy() }()

		body, err := world.CreateBody(box2d.DefaultBodyDef())
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultChainDef()
		def.IsLoop = false
		def.Points = []box2d.Vec2{
			{X: -2, Y: 0},
			{X: -1, Y: 0},
			{X: 0, Y: 0},
			{X: 1, Y: 0},
			{X: 2, Y: 0},
			{X: 3, Y: 0},
		}
		def.Count = len(def.Points)
		def.MaterialCount = def.Count
		def.Materials = make([]box2d.SurfaceMaterial, def.MaterialCount)
		for i := range def.Materials {
			m := box2d.DefaultSurfaceMaterial()
			m.Friction = 0.2 + float32(i)*0.1
			def.Materials[i] = m
		}

		chainID, err := world.CreateChain(body.ID(), def)
		Expect(err).NotTo(HaveOccurred())
		counters, err := world.Counters()
		Expect(err).NotTo(HaveOccurred())
		Expect(counters.ChainCount).To(Equal(1))

		materialCount, err := world.GetChainSurfaceMaterialCount(chainID)
		Expect(err).NotTo(HaveOccurred())
		Expect(materialCount).To(Equal(def.Count))

		material, err := world.GetChainSurfaceMaterial(chainID, 3)
		Expect(err).NotTo(HaveOccurred())
		Expect(material.Friction).To(BeNumerically("~", 0.5, 1e-6))

		updated := box2d.DefaultSurfaceMaterial()
		updated.Friction = 1.75
		Expect(world.SetChainSurfaceMaterial(chainID, updated, 3)).To(Succeed())
		material, err = world.GetChainSurfaceMaterial(chainID, 3)
		Expect(err).NotTo(HaveOccurred())
		Expect(material.Friction).To(BeNumerically("~", 1.75, 1e-6))
		Expect(world.SetChainSurfaceMaterial(chainID, updated, -1)).To(MatchError(box2d.ErrChainInvalid))

		segments, err := world.GetChainSegments(chainID, 16)
		Expect(err).NotTo(HaveOccurred())
		Expect(segments).To(HaveLen(3))

		Expect(world.DestroyBody(body.ID())).To(Succeed())
		Expect(box2d.IsChainIDValid(chainID)).To(BeFalse())
		counters, err = world.Counters()
		Expect(err).NotTo(HaveOccurred())
		Expect(counters.ChainCount).To(Equal(0))
		_, err = world.GetChainSegmentCount(chainID)
		Expect(err).To(MatchError(box2d.ErrChainInvalid))
		_, err = world.GetChainWorld(chainID)
		Expect(err).To(MatchError(box2d.ErrChainInvalid))
		for _, seg := range segments {
			Expect(box2d.IsShapeIDValid(seg)).To(BeFalse())
		}
	})

	It("supports OOP chain handles", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())
		defer func() { _ = world.Destroy() }()

		body, err := world.CreateBody(box2d.DefaultBodyDef())
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultChainDef()
		def.IsLoop = true
		def.Points = []box2d.Vec2{
			{X: -1, Y: -1},
			{X: 1, Y: -1},
			{X: 1, Y: 1},
			{X: -1, Y: 1},
		}
		def.Count = len(def.Points)

		chainID, err := world.CreateChain(body.ID(), def)
		Expect(err).NotTo(HaveOccurred())

		chain, err := world.Chain(chainID)
		Expect(err).NotTo(HaveOccurred())
		Expect(chain.IsValid()).To(BeTrue())
		Expect(box2d.ChainIDEqual(chain.ID(), chainID)).To(BeTrue())

		worldID, err := chain.GetWorld()
		Expect(err).NotTo(HaveOccurred())
		Expect(worldID).To(Equal(world.ID()))

		segmentCount, err := chain.GetSegmentCount()
		Expect(err).NotTo(HaveOccurred())
		Expect(segmentCount).To(Equal(4))

		segments, err := chain.GetSegments(8)
		Expect(err).NotTo(HaveOccurred())
		Expect(segments).To(HaveLen(4))

		materialCount, err := chain.GetSurfaceMaterialCount()
		Expect(err).NotTo(HaveOccurred())
		Expect(materialCount).To(Equal(1))

		material, err := chain.GetSurfaceMaterial(0)
		Expect(err).NotTo(HaveOccurred())
		material.Friction = 0.9
		Expect(chain.SetSurfaceMaterial(material, 0)).To(Succeed())

		Expect(chain.Destroy()).To(Succeed())
		Expect(chain.IsValid()).To(BeFalse())
		Expect(chain.Destroy()).To(MatchError(box2d.ErrChainInvalid))
	})

	It("validates chain handles against invalid ids and destroyed worlds", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())

		_, err = world.Chain(box2d.NullChainID)
		Expect(err).To(MatchError(box2d.ErrChainInvalid))

		Expect(world.Destroy()).To(Succeed())
		_, err = world.Chain(box2d.NullChainID)
		Expect(err).To(MatchError(box2d.ErrWorldInvalid))
	})

	It("creates chain handles directly from world/body helper methods", func() {
		world, err := box2d.NewWorld(box2d.DefaultWorldDef())
		Expect(err).NotTo(HaveOccurred())
		defer func() { _ = world.Destroy() }()

		body, err := world.CreateBody(box2d.DefaultBodyDef())
		Expect(err).NotTo(HaveOccurred())

		def := box2d.DefaultChainDef()
		def.IsLoop = true
		def.Points = []box2d.Vec2{
			{X: -1, Y: -1},
			{X: 1, Y: -1},
			{X: 1, Y: 1},
			{X: -1, Y: 1},
		}
		def.Count = len(def.Points)

		chainFromWorld, err := world.CreateChainObject(body.ID(), def)
		Expect(err).NotTo(HaveOccurred())
		Expect(chainFromWorld.IsValid()).To(BeTrue())
		Expect(chainFromWorld.Destroy()).To(Succeed())

		chainFromBody, err := body.CreateChain(def)
		Expect(err).NotTo(HaveOccurred())
		Expect(chainFromBody.IsValid()).To(BeTrue())
		Expect(chainFromBody.Destroy()).To(Succeed())
	})
})
