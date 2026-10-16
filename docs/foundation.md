# Foundation APIs

## Version and timing

```go
version := box2d.GetVersion()
start := box2d.GetTicks()
_ = box2d.GetMilliseconds(start)
_ = version
```

## Logging and assert hooks

```go
box2d.SetLogFcn(func(msg string) { fmt.Println(msg) })
box2d.SetAssertFcn(func(cond, file string, line int) int {
	fmt.Println("assert", cond, file, line)
	return 0
})
```

## Allocator hooks and memory stats

```go
box2d.SetAllocator(nil, nil)
bytes := box2d.GetByteCount()
_ = bytes
```
