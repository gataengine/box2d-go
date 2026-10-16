package box2d

// IdPool allocates reusable integer IDs.
type IdPool struct {
	freeArray IntArray
	nextIndex int
}

func CreateIdPool() IdPool {
	return IdPool{
		freeArray: IntArrayCreate(32),
		nextIndex: 0,
	}
}

func (p *IdPool) Destroy() {
	p.freeArray.Destroy()
	p.nextIndex = 0
}

func (p *IdPool) AllocID() int {
	if p.freeArray.count > 0 {
		return p.freeArray.Pop()
	}
	id := p.nextIndex
	p.nextIndex++
	return id
}

func (p *IdPool) FreeID(id int) bool {
	if p.nextIndex <= 0 || id < 0 || id >= p.nextIndex {
		return false
	}
	p.freeArray.Push(id)
	return true
}

func (p *IdPool) ValidateFreeID(id int) bool {
	for i := range p.freeArray.count {
		if p.freeArray.data[i] == id {
			return true
		}
	}
	return false
}

func (p *IdPool) ValidateUsedID(id int) bool {
	for i := range p.freeArray.count {
		if p.freeArray.data[i] == id {
			return false
		}
	}
	return true
}

func (p *IdPool) IDCount() int {
	return p.nextIndex - p.freeArray.count
}

func (p *IdPool) IDCapacity() int {
	return p.nextIndex
}

func (p *IdPool) IDBytes() int {
	return p.freeArray.ByteCount()
}
