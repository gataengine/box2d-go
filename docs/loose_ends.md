# Loose Ends

## User data

All major objects support user data setters/getters:

```go
_ = world.SetUserData(myWorldData)
_ = body.SetUserData(myBodyData)
_ = joint.SetUserData(myJointData)
```

## Length units

```go
box2d.SetLengthUnitsPerMeter(1.0)
units := box2d.GetLengthUnitsPerMeter()
_ = units
```

## Determinism

Use fixed time step and deterministic creation order.
