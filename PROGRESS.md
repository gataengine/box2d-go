# Box2D Go Port Progress

## Goal
Achieve **full feature parity** with upstream Box2D in pure Go, while preserving deterministic behavior and the OOP API shape (`world.Method(...)`, `body.Method(...)`, `shape.Method(...)`).

## Repository Paths
- Target repo: `libs/box2d-go` (gata monorepo)
- Upstream reference: [erincatto/box2d](https://github.com/erincatto/box2d) @ c05c48738fbe

## Locked Direction
- Port strategy: pure Go first (no cgo bridge-first phase)
- API shape: OOP (`world.Method(...)`, `body.Method(...)`, `shape.Method(...)`)
- Determinism: deterministic behavior within Go implementation
- Threading rollout: single-thread first, multithreading later
- Test migration: phased full migration to Ginkgo/Gomega
- Performance: correctness first, optimize after parity

## Current Status (Now)
### Implemented
- Foundation and math/types:
  - `id.go`, `math.go`, `types.go`
- Internal utilities/data structures:
  - `bitset.go`, `table.go`, `ctz.go`, `array.go`, `id_pool.go`
- Core world/body/shape lifecycle:
  - `world.go`, `body.go`, `shape.go`, `joint.go`
- Deterministic stepping subset:
  - `World.Step()` deterministic sub-step integration for gravity + velocity/position/rotation
  - damping + motion lock integration
- Body API coverage:
  - API name-level parity with upstream `b2Body_*` achieved
  - transform/point/vector conversion helpers
  - velocity/damping/gravity accessors
  - forces/torques/impulses + clear behavior
  - mass/inertia/COM APIs (including shape-derived mass and manual override)
  - shape enumeration and bounds:
    - `GetShapeCount`, `GetShapes`, `ComputeAABB`
  - awake/sleep/enabled controls (`WakeTouching` now wakes touching bodies via overlap contacts)
  - runtime flags: motion locks, bullet
  - target transform velocity solve subset (`SetTargetTransform`)
  - body-level shape event toggles (`EnableContactEvents`, `EnableHitEvents`)
- Shape API coverage:
  - API name-level parity with upstream `b2Shape_*` achieved
  - create/destroy primitives currently in repo
  - mutable material/filter/event flags
  - density updates with optional body-mass recompute
  - overlap-backed contact/sensor data queries:
    - `GetContactCapacity`, `GetContactData`
    - `GetSensorCapacity`, `GetSensorData`, `GetSensorOverlaps`
  - wind force application:
    - `Shape.ApplyWind(...)` now applies drag/lift-derived forces to awake dynamic bodies (with wake gating) instead of no-op placeholder
- World API coverage:
  - API name-level parity with upstream `b2World_*` achieved
- implemented world query subset:
    - `OverlapAABB`, `CastRay`, `CastRayClosest` (deterministic AABB-pruned shape queries)
    - query-filter variants:
      - `OverlapAABBWithFilter`
      - `CastRayWithFilter`
      - `CastRayClosestWithFilter`
      - `OverlapShapeWithFilter`
      - `CastShapeWithFilter`
      - `CastMoverWithFilter`
      - `CollideMoverWithFilter`
  - overlap-backed contact/sensor event counts:
    - `GetContactEvents`, `GetSensorEvents`
  - event stream improvements:
    - `BodyEvents` now includes move-event payloads (`BodyMoveEvent`) with:
      - `UserData`, `Transform`, `BodyID`, `FellAsleep`
    - `GetBodyEvents` now returns step-populated move events instead of a placeholder count
    - `JointEvents` now includes event payloads (`JointEvent`) with:
      - `JointID`, `UserData`
    - `GetJointEvents` now returns awake/threshold-gated joint events with deterministic ordering
  - custom filter callback is now applied when shape custom filtering is enabled
  - query parity refinements:
    - `CastRayClosest` now ignores initial-overlap (`fraction == 0`) hits
    - AABB/ray query candidate collection now excludes shapes on disabled bodies
    - query category bits are validated against shape mask bits in addition to mask/category matching
  - determinism coverage:
    - repeated identical world runs now have regression coverage for stable final transforms
  - implemented configuration/state toggles:
    - sleep/continuous/warm-start/speculative flags
    - speculative state getter: `IsSpeculativeEnabled`
    - restitution/hit thresholds, contact tuning, max linear speed
    - awake-body counting and counter aliases
    - explicit world count accessors:
      - `GetBodyCount`, `GetShapeCount`, `GetJointCount`, `GetChainCount`
    - additional world count aliases:
      - `GetProxyCount` (shape/proxy count alias)
      - `GetContactCount`
      - `GetSensorTouchCount`
  - memory/debug surface progress:
    - `DumpMemoryStats()` now returns serialized world allocation/state counters (bodies/shapes/joints/contacts/sensors) instead of an empty placeholder
    - chain counters included (`chains.count/capacity/free`)
  - chain-shape API surface slice:
    - new chain definition: `ChainDef`
    - defaults: `DefaultChainDef`
    - chain lifecycle:
      - `CreateChain`, `DestroyChain`, `IsChainIDValid`
      - `GetChainWorld`, `GetChainSegmentCount`, `GetChainSegments`
      - `GetChainSurfaceMaterialCount`, `SetChainSurfaceMaterial`, `GetChainSurfaceMaterial`
    - chain segments are now linked back to parent chains for `Shape.GetParentChain()`
    - body destruction now invalidates/destroys attached chains (matching upstream lifetime behavior)
    - OOP chain handle support:
      - `World.Chain(id)` handle lookup
      - object-creation helpers:
        - `World.CreateChainObject(bodyID, def)`
        - `Body.CreateChain(def)`
      - `Chain` methods: `Destroy`, `GetWorld`, `GetSegmentCount`, `GetSegments`, `GetSurfaceMaterialCount`, `SetSurfaceMaterial`, `GetSurfaceMaterial`
  - initial joint registry/lifecycle:
    - `CreateJoint`, `CreateDistanceJoint`, `DestroyJoint`
    - `IsJointIDValid`, `Joint` object lifecycle and accessors
    - body joint enumeration now reflects attached joints
  - distance-joint API surface slice:
    - new joint definitions: `JointDef`, `DistanceJointDef`
    - defaults: `DefaultJointDef`, `DefaultDistanceJointDef`
    - creation from definitions: `CreateDistanceJointWithDef` / `CreateDistanceJointFromDef`
    - distance joint runtime controls:
      - length: `Set/GetLength`, `SetLengthRange`, `GetMinLength`, `GetMaxLength`, `GetCurrentLength`
      - spring: `Enable/IsSpringEnabled`, `Set/GetSpringForceRange`, `Set/GetSpringHertz`, `Set/GetSpringDampingRatio`
      - limits: `Enable/IsLimitEnabled`
      - motor: `Enable/IsMotorEnabled`, `Set/GetMotorSpeed`, `Set/GetMaxMotorForce`, `GetMotorForce`
    - distance-joint methods now validate joint type (`ErrJointTypeMismatch`) and distance-joint defs validate initialization (`ErrInvalidDistanceJointDef`)
    - negative max motor force inputs now clamp to zero
  - motor-joint API surface slice:
    - new joint definition: `MotorJointDef`
    - defaults: `DefaultMotorJointDef`
    - creation from definitions: `CreateMotorJointWithDef` / `CreateMotorJointFromDef`
    - convenience creation: `CreateMotorJoint(bodyA, bodyB)`
    - motor joint runtime controls:
      - velocity targets: `Set/GetLinearVelocity`, `Set/GetAngularVelocity`
      - velocity limits: `Set/GetMaxVelocityForce`, `Set/GetMaxVelocityTorque`
      - spring tuning: `Set/GetLinearHertz`, `Set/GetLinearDampingRatio`, `Set/GetAngularHertz`, `Set/GetAngularDampingRatio`
    - spring limits: `Set/GetMaxSpringForce`, `Set/GetMaxSpringTorque`
    - motor-joint defs validate initialization (`ErrInvalidMotorJointDef`)
    - negative max velocity force/torque inputs now clamp to zero
  - filter-joint API surface slice:
    - new joint definition: `FilterJointDef`
    - defaults: `DefaultFilterJointDef`
    - creation from definitions: `CreateFilterJointWithDef` / `CreateFilterJointFromDef`
    - convenience creation: `CreateFilterJoint(bodyA, bodyB)`
    - filter-joint defs validate initialization (`ErrInvalidFilterJointDef`)
  - prismatic-joint API surface slice:
    - new joint definition: `PrismaticJointDef`
    - defaults: `DefaultPrismaticJointDef`
    - creation from definitions: `CreatePrismaticJointWithDef` / `CreatePrismaticJointFromDef`
    - convenience creation: `CreatePrismaticJoint(bodyA, bodyB)`
    - prismatic runtime controls:
      - spring: `Enable/IsSpringEnabled`, `Set/GetSpringHertz`, `Set/GetSpringDampingRatio`
      - target/limits: `Set/GetTargetTranslation`, `Enable/IsLimitEnabled`, `SetLimits`, `GetLowerLimit`, `GetUpperLimit`
      - motor: `Enable/IsMotorEnabled`, `Set/GetMotorSpeed`, `Set/GetMaxMotorForce`, `GetMotorForce`
      - kinematics: `GetTranslation`, `GetSpeed`
    - shared spring/limit/motor methods now support both distance and prismatic joints by joint type
    - prismatic-joint defs validate initialization (`ErrInvalidPrismaticJointDef`)
  - revolute-joint API surface slice:
    - new joint definition: `RevoluteJointDef`
    - defaults: `DefaultRevoluteJointDef`
    - creation from definitions: `CreateRevoluteJointWithDef` / `CreateRevoluteJointFromDef`
    - convenience creation: `CreateRevoluteJoint(bodyA, bodyB)`
    - revolute runtime controls:
      - spring: `Enable/IsSpringEnabled`, `Set/GetSpringHertz`, `Set/GetSpringDampingRatio`
      - target/limits: `Set/GetTargetAngle`, `GetAngle`, `Enable/IsLimitEnabled`, `SetLimits`, `GetLowerLimit`, `GetUpperLimit`
      - motor: `Enable/IsMotorEnabled`, `Set/GetMotorSpeed`, `Set/GetMaxMotorTorque`, `GetMotorTorque`
    - shared spring/limit/motor methods now support distance + prismatic + revolute by joint type where applicable
    - revolute-joint defs validate initialization (`ErrInvalidRevoluteJointDef`)
    - negative max motor torque inputs now clamp to zero
    - negative spring hertz/damping inputs now clamp to zero
  - weld-joint API surface slice:
    - new joint definition: `WeldJointDef`
    - defaults: `DefaultWeldJointDef`
    - creation from definitions: `CreateWeldJointWithDef` / `CreateWeldJointFromDef`
    - convenience creation: `CreateWeldJoint(bodyA, bodyB)`
    - weld runtime controls:
      - `Set/GetLinearHertz`, `Set/GetLinearDampingRatio`
      - `Set/GetAngularHertz`, `Set/GetAngularDampingRatio`
    - shared linear/angular hertz+damping methods now support motor + weld by joint type
    - weld-joint defs validate initialization (`ErrInvalidWeldJointDef`)
  - wheel-joint API surface slice:
    - new joint definition: `WheelJointDef`
    - defaults: `DefaultWheelJointDef`
    - creation from definitions: `CreateWheelJointWithDef` / `CreateWheelJointFromDef`
    - convenience creation: `CreateWheelJoint(bodyA, bodyB)`
    - wheel runtime controls:
      - spring: `Enable/IsSpringEnabled`, `Set/GetSpringHertz`, `Set/GetSpringDampingRatio`
      - limits: `Enable/IsLimitEnabled`, `SetLimits`, `GetLowerLimit`, `GetUpperLimit`
      - motor: `Enable/IsMotorEnabled`, `Set/GetMotorSpeed`, `Set/GetMaxMotorTorque`, `GetMotorTorque`
    - shared spring/limit/motor methods now route across distance + prismatic + revolute + wheel by joint type where applicable
    - wheel-joint defs validate initialization (`ErrInvalidWheelJointDef`)
    - negative max motor torque inputs now clamp to zero
    - negative spring hertz/damping inputs now clamp to zero
- Collision shape primitive math:
  - `collision.go` + parity-focused shape math tests
  - capsule ray cast support:
    - `RayCastCapsule` implemented and wired through both:
      - `Shape.RayCast(...)`
      - `World.CastRay(...)`

### Test Status
- `go test ./...` passing
- `go run github.com/onsi/ginkgo/v2/ginkgo -r` passing
- **154 specs passing, 0 failing**

## Remaining Work To Reach Full Feature Parity
### Major systems not yet ported
1. Contact pipeline and graph
- broad-phase acceleration structure:
  - contact and sensor pair collection now uses a deterministic sweep-and-prune candidate phase over shape AABBs
  - still lacks full upstream dynamic tree/static tree proxy management and persistent broad-phase data structures
- persistent contact ids/manifolds and full manifold generation
- island/contact graph parity and solver-set integration

2. Constraint solver parity
- velocity/position solver phases:
  - deterministic minimal contact solver is now in place:
    - normal impulse
    - tangential friction impulse
    - penetration correction
    - callback-driven friction/restitution mixing during solve
    - cached-contact warm-starting path wired through `enableWarmStarting`
  - deterministic minimal joint solve is now in place for:
    - distance joints (length correction + motor-force application)
    - motor joints (linear/angular relative velocity targets + spring-based anchor correction)
    - prismatic motor drive (axis speed target + motor force tracking)
    - revolute/wheel motor drive (angular speed target + motor torque tracking)
    - weld joint linear/angular correction
    - distance/revolute/prismatic/wheel limit correction subsets during step
  - still missing full upstream warm-started sequential-impulse feature set
- warm-starting and stabilization details
- full upstream-stable restitution/friction/contact response details across all edge cases

3. Joint subsystem
- core registry/lifecycle is now present
- distance, motor, filter, prismatic, revolute, weld, and wheel joint API surface controls are now present
- per-joint constraint solving and physically-correct limits/motors still missing
- joint event/query semantics still partial

4. World/query/event behavioral parity
- replace current brute-force queries with full broad-phase backed behavior
  - query progression:
    - `OverlapAABB` and ray-query candidate enumeration now use AABB candidate pruning instead of full-shape scans
    - `CastRayClosest` now uses direct closest-hit scanning (without full-hit allocation/sort pass)
- port event pipelines and data buffers (body/contact/sensor/joint events)
- port callback invocation semantics during actual contact solving
  - `SetFrictionCallback` and `SetRestitutionCallback` now affect runtime contact response in `Step()`
  - `collideConnected` is now honored during pair generation:
    - joint-connected bodies with `collideConnected=false` are filtered from contact/sensor pair creation
  - sleep progression now implemented:
    - dynamic bodies transition to asleep state after sustained low motion
    - per-body sleep disable and world-level sleep disable are respected
  - world profile payload now populated:
    - `GetProfile()` returns step/substep/time counters and current world/body/contact counts
  - note: body/joint event payload surfaces are now present, but force/torque event semantics remain solver-approximate until full solver parity
  - advanced world API progress:
    - `OverlapShape()` now returns touching non-sensor shape ids from active contact pairs
    - `CastShape()` now returns contact-derived hit payloads for active touching pairs
    - `CastMover()` now returns first contact-derived cast output when touching pairs exist
    - `CollideMover()` now refreshes mover/contact state instead of pure no-op
    - `CollideMoverWithFilter()` now evaluates filtered contact matches and records mover match counts in memory stats
    - explosion API progression:
      - `ExplosionDef` + `DefaultExplosionDef` added
      - `World.ExplodeWithDef(...)` applies configurable radial impulse with mask filtering
      - impulse scaling now accounts for matching-shape perimeter (circles/capsules/polygons)
      - explosion influence is evaluated per matching shape position (not only body COM)
      - `World.Explode()` now delegates to default `ExplosionDef` behavior for consistency
      - negative impulse values are supported (implosion behavior)
    - contact manifold payload progression:
      - `ContactData.Manifold` now carries populated payload fields (`Normal`, `Point1/Point2`, `PointCount`)
    - manifold generation is implemented for:
      - circle-circle
      - circle-capsule / capsule-circle
      - circle-segment / segment-circle (including chain segments)
      - circle-polygon / polygon-circle
      - capsule-capsule
      - capsule-segment / segment-capsule (including chain segments)
      - polygon-polygon
      - capsule-polygon / polygon-capsule
      - segment-polygon / polygon-segment (including chain segments)
      - segment-segment / chainSegment-segment / chainSegment-chainSegment
    - overlapping parallel segment-like pairs (for example capsule-capsule) now emit two manifold points when appropriate
    - supported pairs now use narrow-phase gating in contact/sensor pair collection to avoid AABB-only false positives
    - chain segment one-sided behavior:
      - chain segments now store ghost-adjacent vertices from `ChainDef` topology
      - contact generation rejects back-face contacts for `ChainSegmentShape` pairs
      - endpoint ghost-edge checks now suppress inside-corner ghost contacts at chain bends
    - joint event threshold behavior:
      - joint events now use estimated constraint force/torque magnitudes from current anchor separation and angular separation (instead of fixed zero), enabling meaningful positive threshold filtering
      - motor-joint velocity-target mismatch now contributes to estimated force/torque magnitudes for thresholded joint events
      - motor-joint per-axis hertz tuning now influences estimated linear/angular event magnitudes
    - joint constraint query behavior:
      - `Joint.GetConstraintForce()` and `Joint.GetConstraintTorque()` now return estimated non-placeholder values derived from current joint state (instead of always zero)
      - motor joints with coincident anchors now estimate force direction from velocity-target error instead of returning zero-vector force
    - contact event stream improvements:
      - `ContactHitEvent` generation now implemented for touching pairs when:
        - hit events are enabled on either shape
        - computed approach speed exceeds world hit-event threshold
      - hit events now include point/normal/approach speed payloads from generated manifolds
    - pre-solve callback semantics:
      - `SetPreSolveCallback` now actively gates contact pair generation (callback `false` suppresses contact)

### Known placeholder / partial behavior
- Contact queries/events now provide manifold payloads and hit events for supported shape pairs listed above, but manifold quality is still simplified vs upstream (no persistent manifold-point ids, no full clipping/warm-start data).
- `SetTargetTransform` now includes shape-extent-aware sleep gating, but CCD interactions still need additional tightening.
- chain one-sided behavior now includes back-face and endpoint ghost-corner suppression, but full upstream ghost/corner parity remains partial.
- Several world APIs remain intentionally partial:
  - explosion API now applies a deterministic baseline radial impulse (still partial compared to upstream configurable semantics)
  - debug draw integration is still a no-op surface

## Full-Parity Execution Plan
### Phase 1: API Surface Closure (no heavy solver changes)
- Close remaining low-dependency API gaps in `Body`, `Shape`, and `World`.
- Add/expand Ginkgo specs for each method and edge case.
- Exit criteria:
  - No trivial API gaps left in currently implemented subsystems.
  - All tests green after each slice.

Status: complete (surface parity reached for world/body/shape; behavior parity still pending).

### Phase 2: Contact Graph + Broad-Phase Foundation
- Introduce minimal broad-phase proxying and contact pair tracking.
- Build contact graph connectivity required for wake propagation and island building.
- Upgrade `WakeTouching` from placeholder to real behavior.
- Exit criteria:
  - deterministic contact pair lifecycle
  - `WakeTouching` parity behavior backed by tests

### Phase 3: Contact Manifolds + Solver Core
- Port narrow-phase contact manifold generation for supported shapes.
- Port contact solver loop (velocity + position) with deterministic ordering.
- Validate against upstream behavior subsets and regression tests.
- Exit criteria:
  - stable contact resolution in representative scenarios
  - no regressions in existing body/shape/world tests

### Phase 4: Joint System Port
- Port joint ids/defs and world integration.
- Implement core joint types and solver coupling.
- Add parity-focused joint tests incrementally.
- Exit criteria:
  - supported joint types behave within upstream tolerance for core scenarios

### Phase 5: World Queries + Events + Polish
- Port full query behavior over broad-phase/contact data and complete event streams.
- Tighten remaining semantic gaps and edge cases.
- Expand parity test matrix for long-running determinism and behavior consistency.
- Exit criteria:
  - full public feature parity target reached for chosen upstream baseline
  - deterministic repeatability checks pass

## Process To Keep (Every Session)
1. Implement in small vertical slices.
2. After each meaningful edit batch, run:
   - `go test ./...`
   - `go run github.com/onsi/ginkgo/v2/ginkgo -r`
3. Keep existing tests green while adding new ones.
4. Prefer upstream behavioral parity before refactors/optimization.
5. Record exactly what was ported and what remains.

## Definition of Done (Full Parity)
- All targeted upstream Box2D features are represented in Go with matching behavior (within agreed numeric tolerance).
- No placeholder methods remain for core public APIs.
- Full suite passes (`go test ./...` and recursive Ginkgo).
- Determinism expectations validated on repeated runs.
- `PROGRESS.md` reflects completed parity and residual known limitations (if any).
