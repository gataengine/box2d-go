package box2d_test

import (
	"github.com/gataengine/box2d-go"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

func makePartialRebuildTree() (box2d.DynamicTree, []int) {
	tree := box2d.Create()
	ids := make([]int, 0, 8)
	for i, x := range []float32{0, 1, 10, 11, 20, 21, 30, 31} {
		ids = append(ids, tree.CreateProxy(box2d.AABB{
			LowerBound: box2d.Vec2{X: x, Y: -1},
			UpperBound: box2d.Vec2{X: x + 0.5, Y: 1},
		}, 0x1, uint64(i)))
	}
	return tree, ids
}

var _ = Describe("Compatibility Advanced APIs", func() {
	It("validates AABB and ray-cast behavior", func() {
		a := box2d.AABB{
			LowerBound: box2d.Vec2{X: -1, Y: -1},
			UpperBound: box2d.Vec2{X: 1, Y: 1},
		}
		Expect(box2d.IsValidAABB(a)).To(BeTrue())
		Expect(box2d.IsValidAABB(box2d.AABB{
			LowerBound: box2d.Vec2{X: 1, Y: 1},
			UpperBound: box2d.Vec2{X: -1, Y: -1},
		})).To(BeFalse())

		hit := box2d.AABBRayCast(a, box2d.Vec2{X: -3, Y: 0}, box2d.Vec2{X: 3, Y: 0})
		Expect(hit.Hit).To(BeTrue())
		Expect(hit.Fraction).To(BeNumerically("~", float32(1.0/3.0), 1e-6))

		miss := box2d.AABBRayCast(a, box2d.Vec2{X: 0, Y: 0}, box2d.Vec2{X: 2, Y: 0})
		Expect(miss.Hit).To(BeFalse())
	})

	It("matches distance/shape-cast/toi baseline cases", func() {
		proxyA := box2d.MakeProxy([]box2d.Vec2{
			{X: -1, Y: -1}, {X: 1, Y: -1}, {X: 1, Y: 1}, {X: -1, Y: 1},
		}, 4, 0)
		proxyB := box2d.MakeProxy([]box2d.Vec2{
			{X: 2, Y: -1}, {X: 2, Y: 1},
		}, 2, 0)

		dist := box2d.ShapeDistance(&box2d.DistanceInput{
			ProxyA:     proxyA,
			ProxyB:     proxyB,
			TransformA: box2d.TransformIdentity,
			TransformB: box2d.TransformIdentity,
			UseRadii:   false,
		}, nil, nil, 0)
		Expect(dist.Distance).To(BeNumerically("~", float32(1.0), 1e-6))

		cast := box2d.ShapeCast(&box2d.ShapeCastPairInput{
			ProxyA:       proxyA,
			ProxyB:       proxyB,
			TransformA:   box2d.TransformIdentity,
			TransformB:   box2d.TransformIdentity,
			TranslationB: box2d.Vec2{X: -2, Y: 0},
			MaxFraction:  1,
		})
		Expect(cast.Hit).To(BeTrue())
		Expect(cast.Fraction).To(BeNumerically("~", float32(0.5), 0.01))

		toi := box2d.TimeOfImpact(&box2d.TOIInput{
			ProxyA: proxyA,
			ProxyB: proxyB,
			SweepA: box2d.Sweep{
				LocalCenter: box2d.Vec2Zero,
				C1:          box2d.Vec2Zero,
				C2:          box2d.Vec2Zero,
				Q1:          box2d.RotIdentity,
				Q2:          box2d.RotIdentity,
			},
			SweepB: box2d.Sweep{
				LocalCenter: box2d.Vec2Zero,
				C1:          box2d.Vec2Zero,
				C2:          box2d.Vec2{X: -2, Y: 0},
				Q1:          box2d.RotIdentity,
				Q2:          box2d.RotIdentity,
			},
			MaxFraction: 1,
		})
		Expect(toi.State).To(Equal(box2d.TOIStateHit))
		Expect(toi.Fraction).To(BeNumerically("~", float32(0.5), 0.05))
	})

	It("supports dynamic-tree query/raycast/shapecast baseline behavior", func() {
		tree := box2d.Create()
		a1 := box2d.AABB{LowerBound: box2d.Vec2{X: -1, Y: -1}, UpperBound: box2d.Vec2{X: 1, Y: 1}}
		proxy := tree.CreateProxy(a1, 0x1, 11)

		hitID := -1
		stats := tree.RayCast(&box2d.RayCastInput{
			Origin:      box2d.Vec2{X: -3, Y: 0},
			Translation: box2d.Vec2{X: 6, Y: 0},
			MaxFraction: 1,
		}, 0x1, func(input *box2d.RayCastInput, proxyID int, userData uint64, context any) float32 {
			hitID = proxyID
			return 0
		}, nil)
		Expect(stats.LeafVisits).To(BeNumerically(">", 0))
		Expect(hitID).To(Equal(proxy))

		queryCount := 0
		qstats := tree.Query(box2d.AABB{
			LowerBound: box2d.Vec2{X: -2, Y: -2},
			UpperBound: box2d.Vec2{X: 2, Y: 2},
		}, 0x1, func(proxyID int, userData uint64, context any) bool {
			queryCount++
			return true
		}, nil)
		Expect(qstats.LeafVisits).To(BeNumerically(">", 0))
		Expect(queryCount).To(Equal(1))

		near := tree.CreateProxy(box2d.AABB{
			LowerBound: box2d.Vec2{X: 4.5, Y: -1},
			UpperBound: box2d.Vec2{X: 5.5, Y: 1},
		}, 0x1, 22)
		tree.CreateProxy(box2d.AABB{
			LowerBound: box2d.Vec2{X: 9.5, Y: -1},
			UpperBound: box2d.Vec2{X: 10.5, Y: 1},
		}, 0x1, 33)

		bestID := -1
		bestFraction := float32(1e9)
		scStats := tree.ShapeCast(&box2d.ShapeCastInput{
			Proxy: box2d.ShapeProxy{
				Points: [box2d.MaxPolygonVertices]box2d.Vec2{{X: 0, Y: 0}},
				Count:  1,
			},
			Translation: box2d.Vec2{X: 20, Y: 0},
			MaxFraction: 1,
		}, 0x1, func(input *box2d.ShapeCastInput, proxyID int, userData uint64, context any) float32 {
			aabb := tree.GetAABB(proxyID)
			p1 := input.Proxy.Points[0]
			p2 := box2d.MulAdd(p1, input.MaxFraction, input.Translation)
			out := box2d.AABBRayCast(aabb, p1, p2)
			if !out.Hit {
				return input.MaxFraction
			}
			f := out.Fraction * input.MaxFraction
			if f < bestFraction {
				bestFraction = f
				bestID = proxyID
			}
			return f
		}, nil)
		Expect(scStats.LeafVisits).To(BeNumerically(">", 0))
		Expect(bestID).To(Equal(near))
		Expect(bestFraction).To(BeNumerically("~", float32(4.5/20.0), 0.02))
	})

	It("reuses proxy ids and retains tree storage after destroy/create cycles", func() {
		tree := box2d.Create()
		first := tree.CreateProxy(box2d.AABB{
			LowerBound: box2d.Vec2{X: -1, Y: -1},
			UpperBound: box2d.Vec2{X: 1, Y: 1},
		}, 0x1, 101)
		second := tree.CreateProxy(box2d.AABB{
			LowerBound: box2d.Vec2{X: 3, Y: -1},
			UpperBound: box2d.Vec2{X: 5, Y: 1},
		}, 0x1, 202)
		Expect(tree.GetProxyCount()).To(Equal(2))

		baseBytes := tree.GetByteCount()
		tree.DestroyProxy(first)

		reused := tree.CreateProxy(box2d.AABB{
			LowerBound: box2d.Vec2{X: -5, Y: -1},
			UpperBound: box2d.Vec2{X: -3, Y: 1},
		}, 0x2, 303)
		Expect(reused).To(Equal(first))
		Expect(tree.GetUserData(reused)).To(Equal(uint64(303)))
		Expect(tree.GetCategoryBits(reused)).To(Equal(uint64(0x2)))
		Expect(tree.GetProxyCount()).To(Equal(2))
		Expect(tree.GetByteCount()).To(Equal(baseBytes))

		tree.DestroyProxy(second)
		Expect(tree.GetByteCount()).To(Equal(baseBytes))
		Expect(tree.GetProxyCount()).To(Equal(1))
	})

	It("matches parity edge cases for radii, overlap, filters, and tree height", func() {
		dist := box2d.ShapeDistance(&box2d.DistanceInput{
			ProxyA:     box2d.MakeProxy([]box2d.Vec2{{X: 0, Y: 0}}, 1, 1),
			ProxyB:     box2d.MakeProxy([]box2d.Vec2{{X: 3, Y: 0}}, 1, 1),
			TransformA: box2d.TransformIdentity,
			TransformB: box2d.TransformIdentity,
			UseRadii:   true,
		}, nil, nil, 0)
		Expect(dist.Distance).To(BeNumerically("~", float32(1), 1e-6))
		Expect(dist.Normal.X).To(BeNumerically("~", float32(1), 1e-6))

		toi := box2d.TimeOfImpact(&box2d.TOIInput{
			ProxyA: box2d.MakeProxy([]box2d.Vec2{{X: 0, Y: 0}}, 1, 1),
			ProxyB: box2d.MakeProxy([]box2d.Vec2{{X: 0, Y: 0}}, 1, 1),
			SweepA: box2d.Sweep{
				LocalCenter: box2d.Vec2Zero,
				C1:          box2d.Vec2Zero,
				C2:          box2d.Vec2Zero,
				Q1:          box2d.RotIdentity,
				Q2:          box2d.RotIdentity,
			},
			SweepB: box2d.Sweep{
				LocalCenter: box2d.Vec2Zero,
				C1:          box2d.Vec2Zero,
				C2:          box2d.Vec2Zero,
				Q1:          box2d.RotIdentity,
				Q2:          box2d.RotIdentity,
			},
			MaxFraction: 1,
		})
		Expect(toi.State).To(Equal(box2d.TOIStateOverlapped))
		Expect(toi.Fraction).To(BeNumerically("~", float32(0), 1e-6))

		tree := box2d.Create()
		leaf := tree.CreateProxy(box2d.AABB{
			LowerBound: box2d.Vec2{X: -1, Y: -1},
			UpperBound: box2d.Vec2{X: 1, Y: 1},
		}, 0x1, 7)
		Expect(tree.GetHeight()).To(Equal(0))
		Expect(tree.GetAreaRatio()).To(BeNumerically("~", float32(0), 1e-6))

		maskHits := 0
		stats := tree.Query(box2d.AABB{
			LowerBound: box2d.Vec2{X: -2, Y: -2},
			UpperBound: box2d.Vec2{X: 2, Y: 2},
		}, 0, func(proxyID int, userData uint64, context any) bool {
			maskHits++
			return true
		}, nil)
		Expect(maskHits).To(Equal(0))
		Expect(stats.LeafVisits).To(Equal(0))
		Expect(stats.NodeVisits).To(BeNumerically(">", 0))

		rayTree := box2d.Create()
		near := rayTree.CreateProxy(box2d.AABB{
			LowerBound: box2d.Vec2{X: 4.5, Y: -1},
			UpperBound: box2d.Vec2{X: 5.5, Y: 1},
		}, 0x1, 22)
		rayTree.CreateProxy(box2d.AABB{
			LowerBound: box2d.Vec2{X: 9.5, Y: -1},
			UpperBound: box2d.Vec2{X: 10.5, Y: 1},
		}, 0x1, 33)

		hitID := -1
		rayStats := rayTree.RayCast(&box2d.RayCastInput{
			Origin:      box2d.Vec2{X: -3, Y: 0},
			Translation: box2d.Vec2{X: 20, Y: 0},
			MaxFraction: 1,
		}, 0x1, func(input *box2d.RayCastInput, proxyID int, userData uint64, context any) float32 {
			hitID = proxyID
			return 0
		}, nil)
		Expect(hitID).To(Equal(near))
		Expect(rayStats.LeafVisits).To(Equal(1))
		Expect(rayStats.NodeVisits).To(BeNumerically(">=", 1))
		Expect(rayTree.Rebuild(true)).To(Equal(2))
		Expect(rayTree.GetHeight()).To(Equal(1))
		Expect(leaf).To(BeNumerically(">=", 0))
	})

	It("keeps unchanged subtrees on partial rebuild and partitions fewer boxes than a full rebuild", func() {
		partialTree, partialIDs := makePartialRebuildTree()
		fullTree, fullIDs := makePartialRebuildTree()

		partialTree.EnlargeProxy(partialIDs[0], box2d.AABB{
			LowerBound: box2d.Vec2{X: -4, Y: -2},
			UpperBound: box2d.Vec2{X: 4, Y: 2},
		})
		fullTree.EnlargeProxy(fullIDs[0], box2d.AABB{
			LowerBound: box2d.Vec2{X: -4, Y: -2},
			UpperBound: box2d.Vec2{X: 4, Y: 2},
		})

		Expect(func() {
			partialTree.Validate()
		}).NotTo(Panic())
		Expect(func() {
			partialTree.ValidateNoEnlarged()
		}).To(Panic())

		partialCount := partialTree.Rebuild(false)
		Expect(partialCount).To(BeNumerically(">", 1))
		Expect(partialCount).To(BeNumerically("<", len(partialIDs)))
		Expect(func() {
			partialTree.Validate()
		}).NotTo(Panic())
		Expect(func() {
			partialTree.ValidateNoEnlarged()
		}).NotTo(Panic())

		stableCount := partialTree.Rebuild(false)
		Expect(stableCount).To(Equal(1))
		Expect(func() {
			partialTree.Validate()
		}).NotTo(Panic())

		fullCount := fullTree.Rebuild(true)
		Expect(fullCount).To(Equal(len(fullIDs)))
		Expect(fullCount).To(BeNumerically(">", partialCount))
		Expect(func() {
			fullTree.Validate()
		}).NotTo(Panic())
		Expect(func() {
			fullTree.ValidateNoEnlarged()
		}).NotTo(Panic())
	})
})
