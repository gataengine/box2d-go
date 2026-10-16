## ADDED Requirements

### Requirement: Contact manifold dispatch via function table
The `contactManifoldFromPair()` function SHALL use a 2D function pointer array indexed by `[shapeTypeA][shapeTypeB]` to dispatch to the correct manifold computation function, replacing the current if-chain.

#### Scenario: O(1) manifold dispatch
- **WHEN** a broadphase candidate pair requires manifold computation
- **THEN** the correct manifold function is selected via a single array index lookup (no branching)

#### Scenario: All shape type combinations covered
- **WHEN** any valid combination of shape types is encountered
- **THEN** the dispatch table returns the correct manifold function (matching current if-chain behavior exactly)

#### Scenario: Invalid shape type combinations
- **WHEN** a shape type combination has no manifold function (nil entry in table)
- **THEN** the pair is skipped (no manifold produced), matching current behavior for unhandled pairs

### Requirement: Joint solver uses switch dispatch
The joint solver dispatch in `solveJointsLocked()` and joint initialization SHALL use Go switch statements on `joint.typ` instead of if-chains.

#### Scenario: Switch dispatch for joint velocity solving
- **WHEN** the solver iterates over active joints to apply velocity constraints
- **THEN** joint type dispatch uses a switch statement for compiler jump table optimization

#### Scenario: Behavioral equivalence
- **WHEN** any joint type is solved via switch dispatch
- **THEN** the result is identical to the current if-chain dispatch
