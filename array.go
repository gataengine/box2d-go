package box2d

// IntArray is a dynamic array utility that mirrors b2IntArray behavior.
type IntArray struct {
	data     []int
	count    int
	capacity int
}

func IntArrayCreate(capacity int) IntArray {
	a := IntArray{}
	if capacity > 0 {
		a.data = make([]int, capacity)
		a.capacity = capacity
	}
	return a
}

func (a *IntArray) Reserve(newCapacity int) {
	if newCapacity <= a.capacity {
		return
	}
	newData := make([]int, newCapacity)
	copy(newData, a.data[:a.count])
	a.data = newData
	a.capacity = newCapacity
}

func (a *IntArray) Destroy() {
	a.data = nil
	a.count = 0
	a.capacity = 0
}

func (a *IntArray) Resize(count int) {
	a.Reserve(count)
	a.count = count
}

func (a *IntArray) Get(index int) int {
	return a.data[index]
}

func (a *IntArray) Add() *int {
	if a.count == a.capacity {
		newCapacity := 2
		if a.capacity >= 2 {
			newCapacity = a.capacity + (a.capacity >> 1)
		}
		a.Reserve(newCapacity)
	}
	a.count++
	return &a.data[a.count-1]
}

func (a *IntArray) Push(value int) {
	if a.count == a.capacity {
		newCapacity := 2
		if a.capacity >= 2 {
			newCapacity = a.capacity + (a.capacity >> 1)
		}
		a.Reserve(newCapacity)
	}
	a.data[a.count] = value
	a.count++
}

func (a *IntArray) Set(index int, value int) {
	a.data[index] = value
}

func (a *IntArray) RemoveSwap(index int) int {
	movedIndex := -1
	if index != a.count-1 {
		movedIndex = a.count - 1
		a.data[index] = a.data[movedIndex]
	}
	a.count--
	return movedIndex
}

func (a *IntArray) Pop() int {
	value := a.data[a.count-1]
	a.count--
	return value
}

func (a *IntArray) Clear() {
	a.count = 0
}

func (a *IntArray) ByteCount() int {
	return a.capacity * 8
}

func (a *IntArray) Count() int {
	return a.count
}

func (a *IntArray) Capacity() int {
	return a.capacity
}
