package box2d

import (
	"math"
	"testing"
)

func BenchmarkBitSet(b *testing.B) {
	b.Run("SetBit", func(b *testing.B) {
		bs := CreateBitSet(1024)
		for i := 0; i < b.N; i++ {
			bs.SetBit(uint32(i & 1023))
		}
	})
	b.Run("GetBit", func(b *testing.B) {
		bs := CreateBitSet(1024)
		for i := uint32(0); i < 1024; i++ {
			if i%3 == 0 {
				bs.SetBit(i)
			}
		}
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			sinkBool = bs.GetBit(uint32(i & 1023))
		}
	})
	b.Run("ClearBit", func(b *testing.B) {
		bs := CreateBitSet(1024)
		for i := uint32(0); i < 1024; i++ {
			bs.SetBit(i)
		}
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			bs.ClearBit(uint32(i & 1023))
		}
	})
	b.Run("InPlaceUnion", func(b *testing.B) {
		bs1 := CreateBitSet(1024)
		bs2 := CreateBitSet(1024)
		for i := uint32(0); i < 1024; i++ {
			if i%2 == 0 {
				bs1.SetBit(i)
			}
			if i%3 == 0 {
				bs2.SetBit(i)
			}
		}
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			bs1.InPlaceUnion(&bs2)
		}
	})
	b.Run("CountSetBits", func(b *testing.B) {
		bs := CreateBitSet(1024)
		for i := uint32(0); i < 1024; i++ {
			if i%2 == 0 {
				bs.SetBit(i)
			}
		}
		b.ResetTimer()
		var n int
		for i := 0; i < b.N; i++ {
			n = bs.CountSetBits()
		}
		_ = n
	})
}

func BenchmarkHashSet(b *testing.B) {
	b.Run("AddKey", func(b *testing.B) {
		s := CreateSet(256)
		for i := 0; i < b.N; i++ {
			s.AddKey(uint64(i))
		}
	})
	b.Run("ContainsKey/Hit", func(b *testing.B) {
		s := CreateSet(256)
		for i := 0; i < 1000; i++ {
			s.AddKey(uint64(i))
		}
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			sinkBool = s.ContainsKey(uint64(i % 1000))
		}
	})
	b.Run("ContainsKey/Miss", func(b *testing.B) {
		s := CreateSet(256)
		for i := 0; i < 1000; i++ {
			s.AddKey(uint64(i))
		}
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			sinkBool = s.ContainsKey(uint64(i + 100000))
		}
	})
	b.Run("RemoveKey", func(b *testing.B) {
		s := CreateSet(256)
		for i := 0; i < b.N; i++ {
			s.AddKey(uint64(i))
		}
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			s.RemoveKey(uint64(i))
		}
	})
}

func BenchmarkDynamicTree(b *testing.B) {
	const proxyCount = 1000

	buildTree := func() (DynamicTree, []int) {
		tree := Create()
		ids := make([]int, proxyCount)
		for i := 0; i < proxyCount; i++ {
			x := float32(i%32) * 2.0
			y := float32(i/32) * 2.0
			aabb := AABB{
				LowerBound: Vec2{X: x, Y: y},
				UpperBound: Vec2{X: x + 1, Y: y + 1},
			}
			ids[i] = tree.CreateProxy(aabb, 0xFFFFFFFFFFFFFFFF, uint64(i))
		}
		return tree, ids
	}

	b.Run("CreateProxy", func(b *testing.B) {
		tree := Create()
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			x := float32(i%32) * 2.0
			y := float32(i/32) * 2.0
			aabb := AABB{
				LowerBound: Vec2{X: x, Y: y},
				UpperBound: Vec2{X: x + 1, Y: y + 1},
			}
			tree.CreateProxy(aabb, 0xFFFFFFFFFFFFFFFF, uint64(i))
		}
	})
	b.Run("DestroyProxy", func(b *testing.B) {
		b.StopTimer()
		tree := Create()
		ids := make([]int, b.N)
		for i := 0; i < b.N; i++ {
			x := float32(i%32) * 2.0
			y := float32(i/32) * 2.0
			aabb := AABB{
				LowerBound: Vec2{X: x, Y: y},
				UpperBound: Vec2{X: x + 1, Y: y + 1},
			}
			ids[i] = tree.CreateProxy(aabb, 0xFFFFFFFFFFFFFFFF, uint64(i))
		}
		b.StartTimer()
		for i := 0; i < b.N; i++ {
			tree.DestroyProxy(ids[i])
		}
	})
	b.Run("MoveProxy", func(b *testing.B) {
		tree, ids := buildTree()
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			idx := i % proxyCount
			x := float32(idx%32)*2.0 + float32(math.Sin(float64(i))*0.5)
			y := float32(idx/32)*2.0 + float32(math.Cos(float64(i))*0.5)
			aabb := AABB{
				LowerBound: Vec2{X: x, Y: y},
				UpperBound: Vec2{X: x + 1, Y: y + 1},
			}
			tree.MoveProxy(ids[idx], aabb)
		}
	})
	b.Run("Query", func(b *testing.B) {
		tree, _ := buildTree()
		queryAABB := AABB{LowerBound: Vec2{X: 10, Y: 10}, UpperBound: Vec2{X: 20, Y: 20}}
		cb := func(proxyID int, userData uint64, context any) bool { return true }
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			tree.QueryAll(queryAABB, cb, nil)
		}
	})
	b.Run("RayCast", func(b *testing.B) {
		tree, _ := buildTree()
		input := RayCastInput{
			Origin:      Vec2{X: -5, Y: 15},
			Translation: Vec2{X: 80, Y: 0},
			MaxFraction: 1.0,
		}
		cb := func(inp *RayCastInput, proxyID int, userData uint64, context any) float32 {
			return inp.MaxFraction
		}
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			tree.RayCast(&input, 0xFFFFFFFFFFFFFFFF, cb, nil)
		}
	})
	b.Run("Rebuild", func(b *testing.B) {
		tree, _ := buildTree()
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			tree.Rebuild(true)
		}
	})
}
