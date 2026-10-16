package box2d

import (
	"runtime"
	"sync/atomic"
	"time"
)

const HashInit uint32 = 5381

type AllocFunc func(size uint32, alignment int) any
type FreeFunc func(mem any, size uint32)
type AssertFunc func(condition string, fileName string, lineNumber int) int
type LogFunc func(message string)

type Version struct {
	Major    int
	Minor    int
	Revision int
}

var (
	allocHandler  atomic.Value
	freeHandler   atomic.Value
	assertHandler atomic.Value
	logHandler    atomic.Value
	byteCount     atomic.Int64
)

func init() {
	assertHandler.Store(AssertFunc(defaultAssert))
	logHandler.Store(LogFunc(defaultLog))
}

func defaultAssert(condition string, fileName string, lineNumber int) int {
	_ = condition
	_ = fileName
	_ = lineNumber
	return 1
}

func defaultLog(message string) {
	_ = message
}

func SetAllocator(allocFcn AllocFunc, freeFcn FreeFunc) {
	allocHandler.Store(allocFcn)
	freeHandler.Store(freeFcn)
}

func GetByteCount() int {
	return int(byteCount.Load())
}

func SetAssertFcn(assertFcn AssertFunc) {
	if assertFcn == nil {
		return
	}
	assertHandler.Store(assertFcn)
}

func SetLogFcn(logFcn LogFunc) {
	if logFcn == nil {
		return
	}
	logHandler.Store(logFcn)
}

func Log(message string) {
	h := logHandler.Load().(LogFunc)
	h(message)
}

func InternalAssertFcn(condition string, fileName string, lineNumber int) int {
	h := assertHandler.Load().(AssertFunc)
	return h(condition, fileName, lineNumber)
}

func GetVersion() Version {
	return Version{Major: 3, Minor: 2, Revision: 0}
}

func GetTicks() uint64 {
	return uint64(time.Now().UnixNano())
}

func GetMilliseconds(ticks uint64) float32 {
	now := GetTicks()
	if now <= ticks {
		return 0
	}
	return float32(float64(now-ticks) / 1_000_000.0)
}

func GetMillisecondsAndReset(ticks *uint64) float32 {
	if ticks == nil {
		return 0
	}
	now := GetTicks()
	start := *ticks
	*ticks = now
	if now <= start {
		return 0
	}
	return float32(float64(now-start) / 1_000_000.0)
}

func Yield() {
	runtime.Gosched()
}

func Hash(hash uint32, data []byte, count int) uint32 {
	if count < 0 {
		count = 0
	}
	if count > len(data) {
		count = len(data)
	}
	result := hash
	for i := 0; i < count; i++ {
		result = (result << 5) + result + uint32(data[i])
	}
	return result
}
