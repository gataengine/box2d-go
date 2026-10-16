package box2d

import "errors"

var (
	ErrInvalidChainDef = errors.New("box2d: invalid chain definition (use DefaultChainDef)")
	ErrChainInvalid    = errors.New("box2d: chain is invalid")
)

type chainState struct {
	generation   uint16
	alive        bool
	bodyIndex    int
	shapeIndices []int
	materials    []SurfaceMaterial
	isLoop       bool
}

type Chain struct {
	world *World
	id    ChainID
}

func (c *Chain) ID() ChainID {
	if c == nil {
		return NullChainID
	}
	return c.id
}

func (c *Chain) IsValid() bool {
	if c == nil || c.world == nil || c.id.IsNull() {
		return false
	}
	return IsChainIDValid(c.id)
}

func (c *Chain) Destroy() error {
	if !c.IsValid() {
		return ErrChainInvalid
	}
	if err := c.world.DestroyChain(c.id); err != nil {
		return err
	}
	c.id = NullChainID
	return nil
}

func (c *Chain) GetWorld() (WorldID, error) {
	if !c.IsValid() {
		return NullWorldID, ErrChainInvalid
	}
	return c.world.GetChainWorld(c.id)
}

func (c *Chain) GetSegmentCount() (int, error) {
	if !c.IsValid() {
		return 0, ErrChainInvalid
	}
	return c.world.GetChainSegmentCount(c.id)
}

func (c *Chain) GetSegments(capacity int) ([]ShapeID, error) {
	if !c.IsValid() {
		return nil, ErrChainInvalid
	}
	return c.world.GetChainSegments(c.id, capacity)
}

func (c *Chain) GetSurfaceMaterialCount() (int, error) {
	if !c.IsValid() {
		return 0, ErrChainInvalid
	}
	return c.world.GetChainSurfaceMaterialCount(c.id)
}

func (c *Chain) SetSurfaceMaterial(material SurfaceMaterial, materialIndex int) error {
	if !c.IsValid() {
		return ErrChainInvalid
	}
	return c.world.SetChainSurfaceMaterial(c.id, material, materialIndex)
}

func (c *Chain) GetSurfaceMaterial(materialIndex int) (SurfaceMaterial, error) {
	if !c.IsValid() {
		return SurfaceMaterial{}, ErrChainInvalid
	}
	return c.world.GetChainSurfaceMaterial(c.id, materialIndex)
}

func (w *World) Chain(id ChainID) (*Chain, error) {
	if !w.IsValid() {
		return nil, ErrWorldInvalid
	}
	w.mu.RLock()
	defer w.mu.RUnlock()
	if !w.isChainIDValidLocked(id) {
		return nil, ErrChainInvalid
	}
	return &Chain{world: w, id: id}, nil
}

func (w *World) CreateChainObject(bodyID BodyID, def ChainDef) (*Chain, error) {
	id, err := w.CreateChain(bodyID, def)
	if err != nil {
		return nil, err
	}
	return w.Chain(id)
}

func IsChainIDValid(id ChainID) bool {
	if id.IsNull() {
		return false
	}
	world := worldFromIndex(id.World0 + 1)
	if world == nil {
		return false
	}
	world.mu.RLock()
	defer world.mu.RUnlock()
	return world.isChainIDValidLocked(id)
}

func (w *World) isChainIDValidLocked(id ChainID) bool {
	idx := int(id.Index1) - 1
	if idx < 0 || idx >= len(w.chains) {
		return false
	}
	slot := w.chains[idx]
	return slot.alive && slot.generation == id.Generation
}

func (w *World) allocChainSlotLocked() int {
	if n := len(w.freeChainIndices); n > 0 {
		idx := w.freeChainIndices[n-1]
		w.freeChainIndices = w.freeChainIndices[:n-1]
		return idx
	}
	idx := len(w.chains)
	w.chains = append(w.chains, chainState{})
	return idx
}

func (w *World) chainIDFromIndexLocked(chainIdx int) ChainID {
	return ChainID{
		Index1:     int32(chainIdx + 1),
		World0:     w.id.Index1 - 1,
		Generation: w.chains[chainIdx].generation,
	}
}

func (w *World) CreateChain(bodyID BodyID, def ChainDef) (ChainID, error) {
	if !w.IsValid() {
		return NullChainID, ErrWorldInvalid
	}
	if !def.isInitialized() {
		return NullChainID, ErrInvalidChainDef
	}
	if def.Count < 4 || len(def.Points) < def.Count {
		return NullChainID, ErrInvalidChainDef
	}
	if def.MaterialCount != 1 && def.MaterialCount != def.Count {
		return NullChainID, ErrInvalidChainDef
	}
	if def.MaterialCount <= 0 || len(def.Materials) < def.MaterialCount {
		return NullChainID, ErrInvalidChainDef
	}

	w.mu.Lock()
	defer w.mu.Unlock()
	if !w.isBodyIDValidLocked(bodyID) {
		return NullChainID, ErrBodyInvalid
	}

	chainIdx := w.allocChainSlotLocked()
	chain := &w.chains[chainIdx]
	chain.generation++
	if chain.generation == 0 {
		chain.generation = 1
	}
	chain.alive = true
	chain.bodyIndex = int(bodyID.Index1) - 1
	chain.shapeIndices = chain.shapeIndices[:0]
	chain.materials = append(chain.materials[:0], def.Materials[:def.MaterialCount]...)
	chain.isLoop = def.IsLoop
	w.chainCount++

	shapeDef := DefaultShapeDef()
	shapeDef.UserData = def.UserData
	shapeDef.Material = chain.materials[0]
	shapeDef.Density = 0
	shapeDef.Filter = def.Filter
	shapeDef.EnableCustomFiltering = false
	shapeDef.IsSensor = false
	shapeDef.EnableSensorEvents = def.EnableSensorEvents
	shapeDef.EnableContactEvents = false
	shapeDef.EnableHitEvents = false
	shapeDef.EnablePreSolveEvents = false
	shapeDef.InvokeContactCreation = true
	shapeDef.UpdateBodyMass = false

	segmentMaterialIndex := func(i int) int {
		if def.MaterialCount == 1 {
			return 0
		}
		if def.IsLoop {
			return i
		}
		return i + 1
	}

	if def.IsLoop {
		for i := 0; i < def.Count; i++ {
			shapeDef.Material = chain.materials[segmentMaterialIndex(i)]
			shapeIdx, err := w.createShapeLocked(bodyID, shapeDef, ChainSegmentShape)
			if err != nil {
				return NullChainID, err
			}
			next := (i + 1) % def.Count
			w.shapes[shapeIdx].segment = Segment{
				Point1: def.Points[i],
				Point2: def.Points[next],
			}
			prev := (i - 1 + def.Count) % def.Count
			next2 := (i + 2) % def.Count
			w.shapes[shapeIdx].chainGhost1 = def.Points[prev]
			w.shapes[shapeIdx].chainGhost2 = def.Points[next2]
			w.shapes[shapeIdx].parentChainIndex = chainIdx
			chain.shapeIndices = append(chain.shapeIndices, shapeIdx)
		}
	} else {
		for i := 0; i < def.Count-3; i++ {
			shapeDef.Material = chain.materials[segmentMaterialIndex(i)]
			shapeIdx, err := w.createShapeLocked(bodyID, shapeDef, ChainSegmentShape)
			if err != nil {
				return NullChainID, err
			}
			w.shapes[shapeIdx].segment = Segment{
				Point1: def.Points[i+1],
				Point2: def.Points[i+2],
			}
			w.shapes[shapeIdx].chainGhost1 = def.Points[i]
			w.shapes[shapeIdx].chainGhost2 = def.Points[i+3]
			w.shapes[shapeIdx].parentChainIndex = chainIdx
			chain.shapeIndices = append(chain.shapeIndices, shapeIdx)
		}
	}

	return w.chainIDFromIndexLocked(chainIdx), nil
}

func (w *World) destroyChainIndexLocked(chainIdx int, destroySegments bool) {
	if chainIdx < 0 || chainIdx >= len(w.chains) {
		return
	}
	chain := &w.chains[chainIdx]
	if !chain.alive {
		return
	}

	if destroySegments {
		for _, shapeIdx := range chain.shapeIndices {
			if shapeIdx < 0 || shapeIdx >= len(w.shapes) {
				continue
			}
			shape := w.shapes[shapeIdx]
			if !shape.alive {
				continue
			}
			shapeID := ShapeID{
				Index1:     int32(shapeIdx + 1),
				World0:     w.id.Index1 - 1,
				Generation: shape.generation,
			}
			_ = w.destroyShapeIDLocked(shapeID, false)
		}
	}

	chain.alive = false
	chain.generation++
	if chain.generation == 0 {
		chain.generation = 1
	}
	chain.shapeIndices = nil
	chain.materials = nil
	chain.isLoop = false
	w.freeChainIndices = append(w.freeChainIndices, chainIdx)
	w.chainCount--
}

func (w *World) DestroyChain(id ChainID) error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	if !w.isChainIDValidLocked(id) {
		return ErrChainInvalid
	}
	w.destroyChainIndexLocked(int(id.Index1)-1, true)
	return nil
}

func (w *World) GetChainWorld(id ChainID) (WorldID, error) {
	if !w.IsValid() {
		return NullWorldID, ErrWorldInvalid
	}
	w.mu.RLock()
	defer w.mu.RUnlock()
	if !w.isChainIDValidLocked(id) {
		return NullWorldID, ErrChainInvalid
	}
	return w.id, nil
}

func (w *World) GetChainSegmentCount(id ChainID) (int, error) {
	if !w.IsValid() {
		return 0, ErrWorldInvalid
	}
	w.mu.RLock()
	defer w.mu.RUnlock()
	if !w.isChainIDValidLocked(id) {
		return 0, ErrChainInvalid
	}
	return len(w.chains[int(id.Index1)-1].shapeIndices), nil
}

func (w *World) GetChainSegments(id ChainID, capacity int) ([]ShapeID, error) {
	if !w.IsValid() {
		return nil, ErrWorldInvalid
	}
	w.mu.RLock()
	defer w.mu.RUnlock()
	if !w.isChainIDValidLocked(id) {
		return nil, ErrChainInvalid
	}
	if capacity <= 0 {
		return []ShapeID{}, nil
	}
	chain := w.chains[int(id.Index1)-1]
	segments := make([]ShapeID, 0, capacity)
	for _, shapeIdx := range chain.shapeIndices {
		if len(segments) == capacity {
			break
		}
		if shapeIdx < 0 || shapeIdx >= len(w.shapes) {
			continue
		}
		shape := w.shapes[shapeIdx]
		if !shape.alive {
			continue
		}
		segments = append(segments, ShapeID{
			Index1:     int32(shapeIdx + 1),
			World0:     w.id.Index1 - 1,
			Generation: shape.generation,
		})
	}
	return segments, nil
}

func (w *World) GetChainSurfaceMaterialCount(id ChainID) (int, error) {
	if !w.IsValid() {
		return 0, ErrWorldInvalid
	}
	w.mu.RLock()
	defer w.mu.RUnlock()
	if !w.isChainIDValidLocked(id) {
		return 0, ErrChainInvalid
	}
	return len(w.chains[int(id.Index1)-1].materials), nil
}

func (w *World) SetChainSurfaceMaterial(id ChainID, material SurfaceMaterial, materialIndex int) error {
	if !w.IsValid() {
		return ErrWorldInvalid
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	if !w.isChainIDValidLocked(id) {
		return ErrChainInvalid
	}
	chain := &w.chains[int(id.Index1)-1]
	if materialIndex < 0 || materialIndex >= len(chain.materials) {
		return ErrChainInvalid
	}
	chain.materials[materialIndex] = material

	if len(chain.materials) == 1 {
		for _, shapeIdx := range chain.shapeIndices {
			if shapeIdx < 0 || shapeIdx >= len(w.shapes) {
				continue
			}
			if !w.shapes[shapeIdx].alive {
				continue
			}
			w.shapes[shapeIdx].material = material
		}
	} else {
		shapeListIndex := materialIndex
		if !chain.isLoop {
			shapeListIndex = materialIndex - 1
		}
		if shapeListIndex < 0 || shapeListIndex >= len(chain.shapeIndices) {
			return nil
		}
		shapeIdx := chain.shapeIndices[shapeListIndex]
		if shapeIdx >= 0 && shapeIdx < len(w.shapes) && w.shapes[shapeIdx].alive {
			w.shapes[shapeIdx].material = material
		}
	}
	return nil
}

func (w *World) GetChainSurfaceMaterial(id ChainID, materialIndex int) (SurfaceMaterial, error) {
	if !w.IsValid() {
		return SurfaceMaterial{}, ErrWorldInvalid
	}
	w.mu.RLock()
	defer w.mu.RUnlock()
	if !w.isChainIDValidLocked(id) {
		return SurfaceMaterial{}, ErrChainInvalid
	}
	chain := w.chains[int(id.Index1)-1]
	if materialIndex < 0 || materialIndex >= len(chain.materials) {
		return SurfaceMaterial{}, ErrChainInvalid
	}
	return chain.materials[materialIndex], nil
}
