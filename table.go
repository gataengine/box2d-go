package box2d

// ShapePairKey canonicalizes two 32-bit ids into one order-independent 64-bit key.
func ShapePairKey(k1, k2 uint32) uint64 {
	if k1 < k2 {
		return uint64(k1)<<32 | uint64(k2)
	}
	return uint64(k2)<<32 | uint64(k1)
}

type setItem struct {
	key uint64
}

// HashSet is a simple open-addressing hash set keyed by uint64.
type HashSet struct {
	items    []setItem
	capacity uint32
	count    uint32
}

// CreateSet creates a hash set with power-of-two capacity and minimum capacity 16.
func CreateSet(capacity int) HashSet {
	set := HashSet{}
	if capacity > 16 {
		set.capacity = uint32(RoundUpPowerOf2(capacity))
	} else {
		set.capacity = 16
	}
	set.items = make([]setItem, set.capacity)
	return set
}

// Destroy resets set storage.
func (s *HashSet) Destroy() {
	s.items = nil
	s.count = 0
	s.capacity = 0
}

// Clear empties the set while keeping capacity.
func (s *HashSet) Clear() {
	s.count = 0
	clear(s.items)
}

// Count returns number of keys in the set.
func (s *HashSet) Count() int {
	return int(s.count)
}

// Capacity returns table capacity.
func (s *HashSet) Capacity() int {
	return int(s.capacity)
}

// Bytes returns memory use for table items.
func (s *HashSet) Bytes() int {
	return int(s.capacity) * 8
}

func keyHash(key uint64) uint64 {
	// Murmur hash finalizer, mirrored from upstream.
	h := key
	h ^= h >> 33
	h *= 0xff51afd7ed558ccd
	h ^= h >> 33
	h *= 0xc4ceb9fe1a85ec53
	h ^= h >> 33
	return h
}

func (s *HashSet) findSlot(key, hash uint64) uint32 {
	capacity := s.capacity
	index := uint32(hash) & (capacity - 1)
	for s.items[index].key != 0 && s.items[index].key != key {
		index = (index + 1) & (capacity - 1)
	}
	return index
}

func (s *HashSet) addKeyHaveCapacity(key, hash uint64) {
	index := s.findSlot(key, hash)
	s.items[index].key = key
	s.count++
}

func (s *HashSet) growTable() {
	oldCount := s.count
	oldCapacity := s.capacity
	oldItems := s.items

	s.count = 0
	s.capacity = 2 * oldCapacity
	s.items = make([]setItem, s.capacity)

	for i := uint32(0); i < oldCapacity; i++ {
		item := oldItems[i]
		if item.key == 0 {
			continue
		}
		hash := keyHash(item.key)
		s.addKeyHaveCapacity(item.key, hash)
	}

	_ = oldCount
}

// ContainsKey reports whether key is in the set. Key=0 is always false (sentinel).
func (s *HashSet) ContainsKey(key uint64) bool {
	if key == 0 || s.capacity == 0 {
		return false
	}
	hash := keyHash(key)
	index := s.findSlot(key, hash)
	return s.items[index].key == key
}

// AddKey adds a key. Returns true when key already existed, false when newly inserted.
func (s *HashSet) AddKey(key uint64) bool {
	if key == 0 {
		return false
	}

	hash := keyHash(key)
	index := s.findSlot(key, hash)
	if s.items[index].key != 0 {
		return true
	}

	if 2*s.count >= s.capacity {
		s.growTable()
	}

	s.addKeyHaveCapacity(key, hash)
	return false
}

// RemoveKey removes a key. Returns true if removed.
func (s *HashSet) RemoveKey(key uint64) bool {
	if key == 0 || s.capacity == 0 {
		return false
	}

	hash := keyHash(key)
	i := int(s.findSlot(key, hash))
	if s.items[i].key == 0 {
		return false
	}

	s.items[i].key = 0
	s.count--

	j := i
	capacity := int(s.capacity)
	for {
		j = (j + 1) & (capacity - 1)
		if s.items[j].key == 0 {
			break
		}

		hashJ := keyHash(s.items[j].key)
		k := int(hashJ) & (capacity - 1)

		if i <= j {
			if i < k && k <= j {
				continue
			}
		} else {
			if i < k || k <= j {
				continue
			}
		}

		s.items[i] = s.items[j]
		s.items[j].key = 0
		i = j
	}

	return true
}
