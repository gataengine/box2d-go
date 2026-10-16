package box2d

import (
	"fmt"
	"math"
	"unsafe"
)

type TreeStats struct {
	NodeVisits int
	LeafVisits int
}

type TreeQueryCallbackFunc func(proxyID int, userData uint64, context any) bool
type TreeRayCastCallbackFunc func(input *RayCastInput, proxyID int, userData uint64, context any) float32
type TreeShapeCastCallbackFunc func(input *ShapeCastInput, proxyID int, userData uint64, context any) float32

type treeNode struct {
	AABB         AABB
	CategoryBits uint64
	UserData     uint64
	Child1       int
	Child2       int
	Parent       int
	Next         int
	Height       int
	Flags        uint8
}

const (
	treeNodeAllocated uint8 = 1 << iota
	treeNodeLeaf
	treeNodeEnlarged
)

type DynamicTree struct {
	root            int
	nodeCapacity    int
	nodeCount       int
	freeList        int
	proxyCount      int
	nodes           []treeNode
	leafIndices     []int
	leafCenters     []Vec2
	rebuildCapacity int
}

type rebuildEntry struct {
	id     int
	center Vec2
}

func (n *treeNode) isLeaf() bool {
	return n.Flags&treeNodeLeaf != 0
}

func (n *treeNode) isAllocated() bool {
	return n.Flags&treeNodeAllocated != 0
}

func Create() DynamicTree {
	t := DynamicTree{}
	t.initTree(16)
	return t
}

func (t *DynamicTree) initTree(capacity int) {
	if capacity < 16 {
		capacity = 16
	}
	t.root = -1
	t.nodeCapacity = capacity
	t.nodeCount = 0
	t.freeList = 0
	t.proxyCount = 0
	t.nodes = make([]treeNode, capacity)
	for i := 0; i < capacity-1; i++ {
		t.nodes[i].Next = i + 1
	}
	t.nodes[capacity-1].Next = -1
	t.leafIndices = nil
	t.leafCenters = nil
	t.rebuildCapacity = 0
}

func (t *DynamicTree) ensureInitialized() {
	if t.nodeCapacity == 0 || len(t.nodes) == 0 {
		t.initTree(16)
	}
}

func (t *DynamicTree) Destroy() {
	*t = DynamicTree{}
}

func (t *DynamicTree) allocateNode() int {
	t.ensureInitialized()

	if t.freeList == -1 {
		oldCapacity := t.nodeCapacity
		if oldCapacity == 0 {
			oldCapacity = 16
		}
		newCapacity := oldCapacity + oldCapacity/2
		if newCapacity <= oldCapacity {
			newCapacity = oldCapacity + 1
		}

		newNodes := make([]treeNode, newCapacity)
		copy(newNodes, t.nodes)
		t.nodes = newNodes
		for i := t.nodeCount; i < newCapacity-1; i++ {
			t.nodes[i].Next = i + 1
		}
		t.nodes[newCapacity-1].Next = -1
		t.freeList = t.nodeCount
		t.nodeCapacity = newCapacity
	}

	id := t.freeList
	node := &t.nodes[id]
	t.freeList = node.Next
	*node = treeNode{
		Child1: -1,
		Child2: -1,
		Parent: -1,
		Next:   -1,
		Height: 0,
		Flags:  treeNodeAllocated,
	}
	t.nodeCount++
	return id
}

func (t *DynamicTree) freeNode(nodeID int) {
	if nodeID < 0 || nodeID >= len(t.nodes) {
		return
	}
	node := &t.nodes[nodeID]
	if node.Flags&treeNodeAllocated == 0 {
		return
	}
	node.Next = t.freeList
	node.Flags = 0
	node.Child1 = -1
	node.Child2 = -1
	node.Parent = -1
	node.Height = 0
	t.freeList = nodeID
	t.nodeCount--
}

func (t *DynamicTree) CreateProxy(aabb AABB, categoryBits uint64, userData uint64) int {
	if !IsValidAABB(aabb) {
		return -1
	}

	leaf := t.allocateNode()
	node := &t.nodes[leaf]
	node.AABB = aabb
	node.CategoryBits = categoryBits
	node.UserData = userData
	node.Height = 0
	node.Flags = treeNodeAllocated | treeNodeLeaf
	node.Parent = -1
	node.Child1 = -1
	node.Child2 = -1

	t.insertLeaf(leaf, true)
	t.proxyCount++
	return leaf
}

func (t *DynamicTree) DestroyProxy(proxyID int) {
	if proxyID < 0 || proxyID >= len(t.nodes) {
		return
	}
	node := &t.nodes[proxyID]
	if !node.isAllocated() || !node.isLeaf() {
		return
	}
	t.removeLeaf(proxyID)
	t.freeNode(proxyID)
	if t.proxyCount > 0 {
		t.proxyCount--
	}
}

func (t *DynamicTree) MoveProxy(proxyID int, aabb AABB) {
	if proxyID < 0 || proxyID >= len(t.nodes) || !IsValidAABB(aabb) {
		return
	}
	node := &t.nodes[proxyID]
	if !node.isAllocated() || !node.isLeaf() {
		return
	}
	t.removeLeaf(proxyID)
	node.AABB = aabb
	t.insertLeaf(proxyID, false)
}

func (t *DynamicTree) EnlargeProxy(proxyID int, aabb AABB) {
	if proxyID < 0 || proxyID >= len(t.nodes) || !IsValidAABB(aabb) {
		return
	}
	node := &t.nodes[proxyID]
	if !node.isAllocated() || !node.isLeaf() {
		return
	}

	if !AABBContains(node.AABB, aabb) {
		node.AABB = aabbUnion(node.AABB, aabb)
	}

	parentIndex := node.Parent
	for parentIndex != -1 {
		parent := &t.nodes[parentIndex]
		changed := false
		if !AABBContains(parent.AABB, node.AABB) {
			parent.AABB = aabbUnion(parent.AABB, node.AABB)
			changed = true
		}
		parent.Flags |= treeNodeEnlarged
		parentIndex = parent.Parent
		if !changed {
			break
		}
	}

	for parentIndex != -1 {
		parent := &t.nodes[parentIndex]
		if parent.Flags&treeNodeEnlarged != 0 {
			break
		}
		parent.Flags |= treeNodeEnlarged
		parentIndex = parent.Parent
	}
}

func (t *DynamicTree) SetCategoryBits(proxyID int, categoryBits uint64) {
	if proxyID < 0 || proxyID >= len(t.nodes) {
		return
	}
	node := &t.nodes[proxyID]
	if !node.isAllocated() || !node.isLeaf() {
		return
	}
	node.CategoryBits = categoryBits
	for parentIndex := node.Parent; parentIndex != -1; parentIndex = t.nodes[parentIndex].Parent {
		parent := &t.nodes[parentIndex]
		child1 := &t.nodes[parent.Child1]
		child2 := &t.nodes[parent.Child2]
		parent.CategoryBits = child1.CategoryBits | child2.CategoryBits
	}
}

func (t *DynamicTree) GetCategoryBits(proxyID int) uint64 {
	if proxyID < 0 || proxyID >= len(t.nodes) {
		return 0
	}
	node := &t.nodes[proxyID]
	if !node.isAllocated() {
		return 0
	}
	return node.CategoryBits
}

func (t *DynamicTree) Query(aabb AABB, maskBits uint64, callback TreeQueryCallbackFunc, context any) TreeStats {
	stats := TreeStats{}
	if t.proxyCount == 0 || t.root == -1 {
		return stats
	}

	stack := []int{t.root}
	for len(stack) > 0 {
		nodeID := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		node := &t.nodes[nodeID]
		stats.NodeVisits++

		if !AABBOverlaps(node.AABB, aabb) || (node.CategoryBits&maskBits) == 0 {
			continue
		}

		if node.isLeaf() {
			proceed := true
			if callback != nil {
				proceed = callback(nodeID, node.UserData, context)
			}
			stats.LeafVisits++
			if !proceed {
				return stats
			}
			continue
		}

		if node.Child1 != -1 {
			stack = append(stack, node.Child1)
		}
		if node.Child2 != -1 {
			stack = append(stack, node.Child2)
		}
	}

	return stats
}

func (t *DynamicTree) QueryAll(aabb AABB, callback TreeQueryCallbackFunc, context any) TreeStats {
	stats := TreeStats{}
	if t.proxyCount == 0 || t.root == -1 {
		return stats
	}

	stack := []int{t.root}
	for len(stack) > 0 {
		nodeID := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		node := &t.nodes[nodeID]
		stats.NodeVisits++

		if !AABBOverlaps(node.AABB, aabb) {
			continue
		}

		if node.isLeaf() {
			proceed := true
			if callback != nil {
				proceed = callback(nodeID, node.UserData, context)
			}
			stats.LeafVisits++
			if !proceed {
				return stats
			}
			continue
		}

		if node.Child1 != -1 {
			stack = append(stack, node.Child1)
		}
		if node.Child2 != -1 {
			stack = append(stack, node.Child2)
		}
	}

	return stats
}

func (t *DynamicTree) RayCast(input *RayCastInput, maskBits uint64, callback TreeRayCastCallbackFunc, context any) TreeStats {
	stats := TreeStats{}
	if t.proxyCount == 0 || t.root == -1 || input == nil {
		return stats
	}

	p1 := input.Origin
	d := input.Translation
	r := Normalize(d)
	v := CrossSV(1.0, r)
	absV := absVec2(v)

	maxFraction := input.MaxFraction
	p2 := MulAdd(p1, maxFraction, d)
	segmentAABB := aabbFromPoints(p1, p2)

	stack := []int{t.root}
	for len(stack) > 0 {
		nodeID := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		node := &t.nodes[nodeID]
		stats.NodeVisits++

		if (node.CategoryBits&maskBits) == 0 || !AABBOverlaps(node.AABB, segmentAABB) {
			continue
		}

		c := aabbCenter(node.AABB)
		h := aabbExtents(node.AABB)
		if Dot(absV, h) < AbsFloat(Dot(v, Sub(p1, c))) {
			continue
		}

		if node.isLeaf() {
			stats.LeafVisits++
			if callback == nil {
				continue
			}
			subInput := *input
			subInput.MaxFraction = maxFraction
			value := callback(&subInput, nodeID, node.UserData, context)
			if value == 0 {
				return stats
			}
			if value > 0 && value <= maxFraction {
				maxFraction = value
				p2 = MulAdd(p1, maxFraction, d)
				segmentAABB = aabbFromPoints(p1, p2)
			}
			continue
		}

		left := node.Child1
		right := node.Child2
		if left < 0 || right < 0 {
			continue
		}
		t1 := distanceSquaredToPoint(aabbCenter(t.nodes[left].AABB), p1)
		t2 := distanceSquaredToPoint(aabbCenter(t.nodes[right].AABB), p1)
		if t1 < t2 {
			stack = append(stack, right, left)
		} else {
			stack = append(stack, left, right)
		}
	}

	return stats
}

func (t *DynamicTree) ShapeCast(input *ShapeCastInput, maskBits uint64, callback TreeShapeCastCallbackFunc, context any) TreeStats {
	stats := TreeStats{}
	if t.proxyCount == 0 || t.root == -1 || input == nil || input.Proxy.Count == 0 {
		return stats
	}

	originAABB := shapeProxyAABB(input.Proxy)
	p1 := aabbCenter(originAABB)
	extension := aabbExtents(originAABB)
	r := input.Translation
	v := CrossSV(1.0, r)
	absV := absVec2(v)

	maxFraction := input.MaxFraction
	shift := MulSV(maxFraction, input.Translation)
	totalAABB := aabbUnion(originAABB, aabbOffset(originAABB, shift))

	stack := []int{t.root}
	for len(stack) > 0 {
		nodeID := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		node := &t.nodes[nodeID]
		stats.NodeVisits++

		if (node.CategoryBits&maskBits) == 0 || !AABBOverlaps(node.AABB, totalAABB) {
			continue
		}

		c := aabbCenter(node.AABB)
		h := Add(aabbExtents(node.AABB), extension)
		if Dot(absV, h) < AbsFloat(Dot(v, Sub(p1, c))) {
			continue
		}

		if node.isLeaf() {
			stats.LeafVisits++
			if callback == nil {
				continue
			}
			subInput := *input
			subInput.MaxFraction = maxFraction
			value := callback(&subInput, nodeID, node.UserData, context)
			if value == 0 {
				return stats
			}
			if value > 0 && value < maxFraction {
				maxFraction = value
				shift = MulSV(maxFraction, input.Translation)
				totalAABB = aabbUnion(originAABB, aabbOffset(originAABB, shift))
			}
			continue
		}

		left := node.Child1
		right := node.Child2
		if left < 0 || right < 0 {
			continue
		}
		t1 := distanceSquaredToPoint(aabbCenter(t.nodes[left].AABB), p1)
		t2 := distanceSquaredToPoint(aabbCenter(t.nodes[right].AABB), p1)
		if t1 < t2 {
			stack = append(stack, right, left)
		} else {
			stack = append(stack, left, right)
		}
	}

	return stats
}

func (t *DynamicTree) GetHeight() int {
	if t.root == -1 || t.root >= len(t.nodes) {
		return 0
	}
	return t.nodes[t.root].Height
}

func (t *DynamicTree) GetAreaRatio() float32 {
	if t.root == -1 || t.root >= len(t.nodes) {
		return 0
	}
	root := &t.nodes[t.root]
	rootArea := aabbPerimeter(root.AABB)
	if rootArea <= 0 {
		return 0
	}
	totalArea := float32(0)
	for i := 0; i < len(t.nodes); i++ {
		node := &t.nodes[i]
		if !node.isAllocated() || node.isLeaf() || i == t.root {
			continue
		}
		totalArea += aabbPerimeter(node.AABB)
	}
	return totalArea / rootArea
}

func (t *DynamicTree) GetRootBounds() AABB {
	if t.root == -1 || t.root >= len(t.nodes) {
		return AABB{}
	}
	return t.nodes[t.root].AABB
}

func (t *DynamicTree) GetProxyCount() int {
	return t.proxyCount
}

func (t *DynamicTree) Rebuild(fullBuild bool) int {
	if t.proxyCount == 0 || t.root == -1 {
		return 0
	}

	t.ensureRebuildCapacity(t.proxyCount)
	entries := make([]rebuildEntry, 0, t.rebuildCapacity)
	stack := []int{t.root}

	// fullBuild rebuilds every proxy leaf. Partial rebuild keeps stable
	// subtrees intact and only flattens the branches that were enlarged.
	for len(stack) > 0 {
		nodeID := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		if nodeID < 0 || nodeID >= len(t.nodes) {
			continue
		}

		node := &t.nodes[nodeID]
		if !node.isAllocated() {
			continue
		}

		if node.isLeaf() || (!fullBuild && node.Flags&treeNodeEnlarged == 0) {
			entries = append(entries, rebuildEntry{
				id:     nodeID,
				center: aabbCenter(node.AABB),
			})
			node.Parent = -1
			if node.isLeaf() {
				node.Child1 = -1
				node.Child2 = -1
				node.Height = 0
				node.Flags = treeNodeAllocated | treeNodeLeaf
			} else {
				node.Flags &^= treeNodeEnlarged
			}
			continue
		}

		left := node.Child1
		right := node.Child2
		t.freeNode(nodeID)
		if left != -1 {
			stack = append(stack, left)
		}
		if right != -1 {
			stack = append(stack, right)
		}
	}

	if len(entries) == 0 {
		t.root = -1
		return 0
	}

	t.root = t.buildTreeEntries(entries)
	return len(entries)
}

func (t *DynamicTree) ensureRebuildCapacity(needed int) {
	if needed <= t.rebuildCapacity {
		return
	}
	newCapacity := needed + needed/2
	if newCapacity <= needed {
		newCapacity = needed + 1
	}
	t.leafIndices = make([]int, newCapacity)
	t.leafCenters = make([]Vec2, newCapacity)
	t.rebuildCapacity = newCapacity
}

func (t *DynamicTree) GetByteCount() int {
	size := unsafe.Sizeof(*t)
	size += uintptr(cap(t.nodes)) * unsafe.Sizeof(treeNode{})
	size += uintptr(cap(t.leafIndices)) * unsafe.Sizeof(int(0))
	size += uintptr(cap(t.leafCenters)) * unsafe.Sizeof(Vec2{})
	return int(size)
}

func (t *DynamicTree) GetUserData(proxyID int) uint64 {
	if proxyID < 0 || proxyID >= len(t.nodes) {
		return 0
	}
	node := &t.nodes[proxyID]
	if !node.isAllocated() {
		return 0
	}
	return node.UserData
}

func (t *DynamicTree) GetAABB(proxyID int) AABB {
	if proxyID < 0 || proxyID >= len(t.nodes) {
		return AABB{}
	}
	node := &t.nodes[proxyID]
	if !node.isAllocated() {
		return AABB{}
	}
	return node.AABB
}

func (t *DynamicTree) Validate() {
	t.validateTree(false)
}

func (t *DynamicTree) ValidateNoEnlarged() {
	t.validateTree(true)
}

func (t *DynamicTree) validateTree(noEnlarged bool) {
	if t.root == -1 {
		if t.nodeCount != 0 || t.proxyCount != 0 {
			panic(fmt.Sprintf("dynamic tree: empty root with nodeCount=%d proxyCount=%d", t.nodeCount, t.proxyCount))
		}
		return
	}

	if t.root < 0 || t.root >= len(t.nodes) {
		panic(fmt.Sprintf("dynamic tree: root index %d out of range", t.root))
	}

	visited := make([]bool, len(t.nodes))
	freeVisited := make([]bool, len(t.nodes))
	freeCount := 0
	for index := t.freeList; index != -1; index = t.nodes[index].Next {
		if index < 0 || index >= len(t.nodes) {
			panic(fmt.Sprintf("dynamic tree: free list index %d out of range", index))
		}
		if freeVisited[index] {
			panic(fmt.Sprintf("dynamic tree: free list cycle at node %d", index))
		}
		freeVisited[index] = true
		node := &t.nodes[index]
		if node.isAllocated() {
			panic(fmt.Sprintf("dynamic tree: free node %d is marked allocated", index))
		}
		freeCount++
	}

	leafCount := 0
	allocatedCount := 0
	computedHeight := t.validateNode(t.root, -1, noEnlarged, visited, &leafCount, &allocatedCount)
	if allocatedCount != t.nodeCount {
		panic(fmt.Sprintf("dynamic tree: visited %d allocated nodes, nodeCount=%d", allocatedCount, t.nodeCount))
	}
	if leafCount != t.proxyCount {
		panic(fmt.Sprintf("dynamic tree: visited %d leaves, proxyCount=%d", leafCount, t.proxyCount))
	}
	if computedHeight != t.nodes[t.root].Height {
		panic(fmt.Sprintf("dynamic tree: root height=%d expected=%d", t.nodes[t.root].Height, computedHeight))
	}
	if allocatedCount+freeCount != t.nodeCapacity {
		panic(fmt.Sprintf("dynamic tree: allocated=%d free=%d nodeCapacity=%d", allocatedCount, freeCount, t.nodeCapacity))
	}

	for i := 0; i < len(t.nodes); i++ {
		node := &t.nodes[i]
		if node.isAllocated() && !visited[i] {
			panic(fmt.Sprintf("dynamic tree: allocated node %d is unreachable from root", i))
		}
	}
}

func (t *DynamicTree) validateNode(index int, parent int, noEnlarged bool, visited []bool, leafCount *int, allocatedCount *int) int {
	if index < 0 || index >= len(t.nodes) {
		panic(fmt.Sprintf("dynamic tree: node index %d out of range", index))
	}
	if visited[index] {
		panic(fmt.Sprintf("dynamic tree: node %d visited more than once", index))
	}
	visited[index] = true
	*allocatedCount = *allocatedCount + 1

	node := &t.nodes[index]
	if !node.isAllocated() {
		panic(fmt.Sprintf("dynamic tree: node %d is not allocated", index))
	}
	if noEnlarged && node.Flags&treeNodeEnlarged != 0 {
		panic(fmt.Sprintf("dynamic tree: node %d is enlarged", index))
	}
	if node.Parent != parent {
		panic(fmt.Sprintf("dynamic tree: node %d parent=%d expected=%d", index, node.Parent, parent))
	}

	if node.isLeaf() {
		if node.Child1 != -1 || node.Child2 != -1 {
			panic(fmt.Sprintf("dynamic tree: leaf node %d has children %d/%d", index, node.Child1, node.Child2))
		}
		if node.Height != 0 {
			panic(fmt.Sprintf("dynamic tree: leaf node %d height=%d", index, node.Height))
		}
		*leafCount = *leafCount + 1
		return 0
	}

	if node.Child1 < 0 || node.Child1 >= len(t.nodes) {
		panic(fmt.Sprintf("dynamic tree: node %d child1=%d out of range", index, node.Child1))
	}
	if node.Child2 < 0 || node.Child2 >= len(t.nodes) {
		panic(fmt.Sprintf("dynamic tree: node %d child2=%d out of range", index, node.Child2))
	}

	child1 := &t.nodes[node.Child1]
	child2 := &t.nodes[node.Child2]
	if child1.Parent != index || child2.Parent != index {
		panic(fmt.Sprintf("dynamic tree: node %d child parents do not point back", index))
	}

	height1 := t.validateNode(node.Child1, index, noEnlarged, visited, leafCount, allocatedCount)
	height2 := t.validateNode(node.Child2, index, noEnlarged, visited, leafCount, allocatedCount)
	if node.Height != 1+maxInt(height1, height2) {
		panic(fmt.Sprintf("dynamic tree: node %d height=%d expected=%d", index, node.Height, 1+maxInt(height1, height2)))
	}
	if !AABBContains(node.AABB, child1.AABB) {
		panic(fmt.Sprintf("dynamic tree: node %d bounds do not contain child1", index))
	}
	if !AABBContains(node.AABB, child2.AABB) {
		panic(fmt.Sprintf("dynamic tree: node %d bounds do not contain child2", index))
	}
	if node.CategoryBits != (child1.CategoryBits | child2.CategoryBits) {
		panic(fmt.Sprintf("dynamic tree: node %d category bits mismatch", index))
	}
	if (child1.Flags|child2.Flags)&treeNodeEnlarged != 0 && node.Flags&treeNodeEnlarged == 0 {
		panic(fmt.Sprintf("dynamic tree: node %d missing enlarged flag from children", index))
	}

	return 1 + maxInt(height1, height2)
}

func (t *DynamicTree) buildTreeEntries(entries []rebuildEntry) int {
	if len(entries) == 0 {
		return -1
	}
	if len(entries) == 1 {
		leaf := entries[0].id
		node := &t.nodes[leaf]
		node.Parent = -1
		return leaf
	}

	mid := partitionRebuildEntries(entries)
	left := t.buildTreeEntries(entries[:mid])
	right := t.buildTreeEntries(entries[mid:])

	index := t.allocateNode()
	node := &t.nodes[index]
	node.Flags = treeNodeAllocated
	node.Parent = -1
	node.Child1 = left
	node.Child2 = right
	if left != -1 {
		t.nodes[left].Parent = index
		node.AABB = t.nodes[left].AABB
		node.CategoryBits = t.nodes[left].CategoryBits
		node.Height = t.nodes[left].Height + 1
	}
	if right != -1 {
		t.nodes[right].Parent = index
		if left == -1 {
			node.AABB = t.nodes[right].AABB
		} else {
			node.AABB = aabbUnion(node.AABB, t.nodes[right].AABB)
		}
		node.CategoryBits |= t.nodes[right].CategoryBits
		rightHeight := t.nodes[right].Height + 1
		if rightHeight > node.Height {
			node.Height = rightHeight
		}
	}
	if left != -1 && right != -1 {
		node.Height = 1 + maxInt(t.nodes[left].Height, t.nodes[right].Height)
	}
	return index
}

func partitionRebuildEntries(entries []rebuildEntry) int {
	if len(entries) <= 2 {
		return len(entries) / 2
	}

	lower := entries[0].center
	upper := entries[0].center
	for i := 1; i < len(entries); i++ {
		lower = Min(lower, entries[i].center)
		upper = Max(upper, entries[i].center)
	}

	spread := Sub(upper, lower)
	axis := 0
	pivot := float32(0.5) * (lower.X + upper.X)
	if spread.Y > spread.X {
		axis = 1
		pivot = float32(0.5) * (lower.Y + upper.Y)
	}

	i1, i2 := 0, len(entries)
	for i1 < i2 {
		for i1 < i2 && entryAxisValue(entries[i1], axis) < pivot {
			i1++
		}
		for i1 < i2 && entryAxisValue(entries[i2-1], axis) >= pivot {
			i2--
		}
		if i1 < i2 {
			entries[i1], entries[i2-1] = entries[i2-1], entries[i1]
			i1++
			i2--
		}
	}

	if i1 > 0 && i1 < len(entries) {
		return i1
	}
	return len(entries) / 2
}

func entryAxisValue(entry rebuildEntry, axis int) float32 {
	if axis == 0 {
		return entry.center.X
	}
	return entry.center.Y
}

func (t *DynamicTree) insertLeaf(leaf int, shouldRotate bool) {
	if t.root == -1 {
		t.root = leaf
		t.nodes[leaf].Parent = -1
		return
	}

	leafAABB := t.nodes[leaf].AABB
	sibling := t.findBestSibling(leafAABB)
	oldParent := t.nodes[sibling].Parent
	newParent := t.allocateNode()

	nodes := t.nodes
	nodes[newParent].Parent = oldParent
	nodes[newParent].UserData = math.MaxUint64
	nodes[newParent].AABB = aabbUnion(leafAABB, nodes[sibling].AABB)
	nodes[newParent].CategoryBits = nodes[leaf].CategoryBits | nodes[sibling].CategoryBits
	nodes[newParent].Height = nodes[sibling].Height + 1
	nodes[newParent].Flags = treeNodeAllocated
	nodes[newParent].Child1 = sibling
	nodes[newParent].Child2 = leaf
	nodes[sibling].Parent = newParent
	nodes[leaf].Parent = newParent

	if oldParent != -1 {
		if nodes[oldParent].Child1 == sibling {
			nodes[oldParent].Child1 = newParent
		} else {
			nodes[oldParent].Child2 = newParent
		}
	} else {
		t.root = newParent
	}

	index := nodes[leaf].Parent
	for index != -1 {
		child1 := nodes[index].Child1
		child2 := nodes[index].Child2
		nodes[index].AABB = aabbUnion(nodes[child1].AABB, nodes[child2].AABB)
		nodes[index].CategoryBits = nodes[child1].CategoryBits | nodes[child2].CategoryBits
		nodes[index].Height = 1 + maxInt(nodes[child1].Height, nodes[child2].Height)
		nodes[index].Flags |= (nodes[child1].Flags | nodes[child2].Flags) & treeNodeEnlarged
		if shouldRotate {
			t.rotateNodes(index)
		}
		index = nodes[index].Parent
	}
}

func (t *DynamicTree) removeLeaf(leaf int) {
	if leaf == t.root {
		t.root = -1
		return
	}

	nodes := t.nodes
	parent := nodes[leaf].Parent
	grandParent := nodes[parent].Parent
	var sibling int
	if nodes[parent].Child1 == leaf {
		sibling = nodes[parent].Child2
	} else {
		sibling = nodes[parent].Child1
	}

	if grandParent != -1 {
		if nodes[grandParent].Child1 == parent {
			nodes[grandParent].Child1 = sibling
		} else {
			nodes[grandParent].Child2 = sibling
		}
		nodes[sibling].Parent = grandParent
		t.freeNode(parent)

		index := grandParent
		for index != -1 {
			node := &nodes[index]
			child1 := &nodes[node.Child1]
			child2 := &nodes[node.Child2]
			node.AABB = aabbUnion(child1.AABB, child2.AABB)
			node.CategoryBits = child1.CategoryBits | child2.CategoryBits
			node.Height = 1 + maxInt(child1.Height, child2.Height)
			index = node.Parent
		}
	} else {
		t.root = sibling
		nodes[sibling].Parent = -1
		t.freeNode(parent)
	}
}

func (t *DynamicTree) findBestSibling(boxD AABB) int {
	centerD := aabbCenter(boxD)
	areaD := aabbPerimeter(boxD)

	nodes := t.nodes
	rootIndex := t.root
	rootBox := nodes[rootIndex].AABB
	areaBase := aabbPerimeter(rootBox)
	directCost := aabbPerimeter(aabbUnion(rootBox, boxD))
	inheritedCost := float32(0)

	bestSibling := rootIndex
	bestCost := directCost
	index := rootIndex

	for nodes[index].Height > 0 {
		child1 := nodes[index].Child1
		child2 := nodes[index].Child2

		cost := directCost + inheritedCost
		if cost < bestCost {
			bestSibling = index
			bestCost = cost
		}

		inheritedCost += directCost - areaBase

		leaf1 := nodes[child1].Height == 0
		leaf2 := nodes[child2].Height == 0

		lowerCost1 := float32(math.MaxFloat32)
		box1 := nodes[child1].AABB
		directCost1 := aabbPerimeter(aabbUnion(box1, boxD))
		area1 := float32(0)
		if leaf1 {
			cost1 := directCost1 + inheritedCost
			if cost1 < bestCost {
				bestSibling = child1
				bestCost = cost1
			}
		} else {
			area1 = aabbPerimeter(box1)
			lowerCost1 = inheritedCost + directCost1 + minFloat(areaD-area1, 0)
		}

		lowerCost2 := float32(math.MaxFloat32)
		box2 := nodes[child2].AABB
		directCost2 := aabbPerimeter(aabbUnion(box2, boxD))
		area2 := float32(0)
		if leaf2 {
			cost2 := directCost2 + inheritedCost
			if cost2 < bestCost {
				bestSibling = child2
				bestCost = cost2
			}
		} else {
			area2 = aabbPerimeter(box2)
			lowerCost2 = inheritedCost + directCost2 + minFloat(areaD-area2, 0)
		}

		if leaf1 && leaf2 {
			break
		}

		if bestCost <= lowerCost1 && bestCost <= lowerCost2 {
			break
		}

		if lowerCost1 == lowerCost2 && !leaf1 {
			d1 := Sub(aabbCenter(box1), centerD)
			d2 := Sub(aabbCenter(box2), centerD)
			lowerCost1 = LengthSquared(d1)
			lowerCost2 = LengthSquared(d2)
		}

		if lowerCost1 < lowerCost2 && !leaf1 {
			index = child1
			areaBase = area1
			directCost = directCost1
		} else {
			index = child2
			areaBase = area2
			directCost = directCost2
		}
	}

	return bestSibling
}

func (t *DynamicTree) rotateNodes(iA int) {
	if iA < 0 || iA >= len(t.nodes) {
		return
	}
	nodes := t.nodes
	A := &nodes[iA]
	if A.Height < 2 {
		return
	}

	iB := A.Child1
	iC := A.Child2
	B := &nodes[iB]
	C := &nodes[iC]

	if B.Height == 0 {
		if C.Height == 0 {
			return
		}
		iF := C.Child1
		iG := C.Child2
		F := &nodes[iF]
		G := &nodes[iG]

		costBase := aabbPerimeter(C.AABB)
		costBF := aabbPerimeter(aabbUnion(B.AABB, G.AABB))
		costBG := aabbPerimeter(aabbUnion(B.AABB, F.AABB))
		if costBase < costBF && costBase < costBG {
			return
		}

		if costBF < costBG {
			A.Child1 = iF
			C.Child1 = iB
			B.Parent = iC
			F.Parent = iA
			C.AABB = aabbUnion(B.AABB, G.AABB)
			C.Height = 1 + maxInt(B.Height, G.Height)
			A.Height = 1 + maxInt(C.Height, F.Height)
			C.CategoryBits = B.CategoryBits | G.CategoryBits
			A.CategoryBits = C.CategoryBits | F.CategoryBits
			C.Flags |= (B.Flags | G.Flags) & treeNodeEnlarged
			A.Flags |= (C.Flags | F.Flags) & treeNodeEnlarged
		} else {
			A.Child1 = iG
			C.Child2 = iB
			B.Parent = iC
			G.Parent = iA
			C.AABB = aabbUnion(B.AABB, F.AABB)
			C.Height = 1 + maxInt(B.Height, F.Height)
			A.Height = 1 + maxInt(C.Height, G.Height)
			C.CategoryBits = B.CategoryBits | F.CategoryBits
			A.CategoryBits = C.CategoryBits | G.CategoryBits
			C.Flags |= (B.Flags | F.Flags) & treeNodeEnlarged
			A.Flags |= (C.Flags | G.Flags) & treeNodeEnlarged
		}
		return
	}

	if C.Height == 0 {
		iD := B.Child1
		iE := B.Child2
		D := &nodes[iD]
		E := &nodes[iE]

		costBase := aabbPerimeter(B.AABB)
		costCD := aabbPerimeter(aabbUnion(C.AABB, E.AABB))
		costCE := aabbPerimeter(aabbUnion(C.AABB, D.AABB))
		if costBase < costCD && costBase < costCE {
			return
		}

		if costCD < costCE {
			A.Child2 = iD
			B.Child1 = iC
			C.Parent = iB
			D.Parent = iA
			B.AABB = aabbUnion(C.AABB, E.AABB)
			B.Height = 1 + maxInt(C.Height, E.Height)
			A.Height = 1 + maxInt(B.Height, D.Height)
			B.CategoryBits = C.CategoryBits | E.CategoryBits
			A.CategoryBits = B.CategoryBits | D.CategoryBits
			B.Flags |= (C.Flags | E.Flags) & treeNodeEnlarged
			A.Flags |= (B.Flags | D.Flags) & treeNodeEnlarged
		} else {
			A.Child2 = iE
			B.Child2 = iC
			C.Parent = iB
			E.Parent = iA
			B.AABB = aabbUnion(C.AABB, D.AABB)
			B.Height = 1 + maxInt(C.Height, D.Height)
			A.Height = 1 + maxInt(B.Height, E.Height)
			B.CategoryBits = C.CategoryBits | D.CategoryBits
			A.CategoryBits = B.CategoryBits | E.CategoryBits
			B.Flags |= (C.Flags | D.Flags) & treeNodeEnlarged
			A.Flags |= (B.Flags | E.Flags) & treeNodeEnlarged
		}
		return
	}

	iD := B.Child1
	iE := B.Child2
	iF := C.Child1
	iG := C.Child2
	D := &nodes[iD]
	E := &nodes[iE]
	F := &nodes[iF]
	G := &nodes[iG]

	areaB := aabbPerimeter(B.AABB)
	areaC := aabbPerimeter(C.AABB)
	costBase := areaB + areaC
	bestRotation := 0
	bestCost := costBase

	costBF := areaB + aabbPerimeter(aabbUnion(B.AABB, G.AABB))
	if costBF < bestCost {
		bestRotation = 1
		bestCost = costBF
	}

	costBG := areaB + aabbPerimeter(aabbUnion(B.AABB, F.AABB))
	if costBG < bestCost {
		bestRotation = 2
		bestCost = costBG
	}

	costCD := areaC + aabbPerimeter(aabbUnion(C.AABB, E.AABB))
	if costCD < bestCost {
		bestRotation = 3
		bestCost = costCD
	}

	costCE := areaC + aabbPerimeter(aabbUnion(C.AABB, D.AABB))
	if costCE < bestCost {
		bestRotation = 4
	}

	switch bestRotation {
	case 0:
		return
	case 1:
		A.Child1 = iF
		C.Child1 = iB
		B.Parent = iC
		F.Parent = iA
		C.AABB = aabbUnion(B.AABB, G.AABB)
		C.Height = 1 + maxInt(B.Height, G.Height)
		A.Height = 1 + maxInt(C.Height, F.Height)
		C.CategoryBits = B.CategoryBits | G.CategoryBits
		A.CategoryBits = C.CategoryBits | F.CategoryBits
		C.Flags |= (B.Flags | G.Flags) & treeNodeEnlarged
		A.Flags |= (C.Flags | F.Flags) & treeNodeEnlarged
	case 2:
		A.Child1 = iG
		C.Child2 = iB
		B.Parent = iC
		G.Parent = iA
		C.AABB = aabbUnion(B.AABB, F.AABB)
		C.Height = 1 + maxInt(B.Height, F.Height)
		A.Height = 1 + maxInt(C.Height, G.Height)
		C.CategoryBits = B.CategoryBits | F.CategoryBits
		A.CategoryBits = C.CategoryBits | G.CategoryBits
		C.Flags |= (B.Flags | F.Flags) & treeNodeEnlarged
		A.Flags |= (C.Flags | G.Flags) & treeNodeEnlarged
	case 3:
		A.Child2 = iD
		B.Child1 = iC
		C.Parent = iB
		D.Parent = iA
		B.AABB = aabbUnion(C.AABB, E.AABB)
		B.Height = 1 + maxInt(C.Height, E.Height)
		A.Height = 1 + maxInt(B.Height, D.Height)
		B.CategoryBits = C.CategoryBits | E.CategoryBits
		A.CategoryBits = B.CategoryBits | D.CategoryBits
		B.Flags |= (C.Flags | E.Flags) & treeNodeEnlarged
		A.Flags |= (B.Flags | D.Flags) & treeNodeEnlarged
	case 4:
		A.Child2 = iE
		B.Child2 = iC
		C.Parent = iB
		E.Parent = iA
		B.AABB = aabbUnion(C.AABB, D.AABB)
		B.Height = 1 + maxInt(C.Height, D.Height)
		A.Height = 1 + maxInt(B.Height, E.Height)
		B.CategoryBits = C.CategoryBits | D.CategoryBits
		A.CategoryBits = B.CategoryBits | E.CategoryBits
		B.Flags |= (C.Flags | D.Flags) & treeNodeEnlarged
		A.Flags |= (B.Flags | E.Flags) & treeNodeEnlarged
	}
}

func minFloat(a, b float32) float32 {
	if a < b {
		return a
	}
	return b
}

func maxInt(a, b int) int {
	if a > b {
		return a
	}
	return b
}

func aabbCenter(a AABB) Vec2 {
	return MulSV(0.5, Add(a.LowerBound, a.UpperBound))
}

func aabbExtents(a AABB) Vec2 {
	return MulSV(0.5, Sub(a.UpperBound, a.LowerBound))
}

func aabbPerimeter(a AABB) float32 {
	w := a.UpperBound.X - a.LowerBound.X
	h := a.UpperBound.Y - a.LowerBound.Y
	return 2 * (w + h)
}

func aabbUnion(a, b AABB) AABB {
	return AABB{
		LowerBound: Min(a.LowerBound, b.LowerBound),
		UpperBound: Max(a.UpperBound, b.UpperBound),
	}
}

func aabbOffset(a AABB, delta Vec2) AABB {
	return AABB{
		LowerBound: Add(a.LowerBound, delta),
		UpperBound: Add(a.UpperBound, delta),
	}
}

func aabbFromPoints(a, b Vec2) AABB {
	return AABB{
		LowerBound: Min(a, b),
		UpperBound: Max(a, b),
	}
}

func absVec2(v Vec2) Vec2 {
	return Vec2{X: AbsFloat(v.X), Y: AbsFloat(v.Y)}
}

func distanceSquaredToPoint(a, b Vec2) float32 {
	return DistanceSquared(a, b)
}

func shapeProxyAABB(proxy ShapeProxy) AABB {
	if proxy.Count == 0 {
		return AABB{}
	}
	lower := proxy.Points[0]
	upper := proxy.Points[0]
	for i := 1; i < proxy.Count; i++ {
		lower = Min(lower, proxy.Points[i])
		upper = Max(upper, proxy.Points[i])
	}
	radius := Vec2{X: proxy.Radius, Y: proxy.Radius}
	return AABB{
		LowerBound: Sub(lower, radius),
		UpperBound: Add(upper, radius),
	}
}

func rayAABBMayHit(input *RayCastInput, aabb AABB) bool {
	if input == nil {
		return false
	}
	return rayCastAABB(*input, aabb).Hit
}

func shapeCastAABBMayHit(input *ShapeCastInput, aabb AABB) bool {
	if input == nil || input.Proxy.Count == 0 {
		return false
	}

	originAABB := shapeProxyAABB(input.Proxy)
	p1 := aabbCenter(originAABB)
	extension := aabbExtents(originAABB)
	v := CrossSV(1.0, input.Translation)
	absV := absVec2(v)
	t := MulSV(input.MaxFraction, input.Translation)
	totalAABB := aabbUnion(originAABB, aabbOffset(originAABB, t))

	if !AABBOverlaps(totalAABB, aabb) {
		return false
	}

	c := aabbCenter(aabb)
	h := Add(aabbExtents(aabb), extension)
	return Dot(absV, h) >= AbsFloat(Dot(v, Sub(p1, c)))
}
