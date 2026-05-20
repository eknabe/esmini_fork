# Road Objects Refactor Blueprint

## Purpose

Refactor OpenDRIVE object handling so parsing, visualization, and OSI reporting all consume one canonical object representation.

Priorities:
1. Continuous and discrete repeats for all object types, including mixed outline coordinate systems.
2. Object markings (e.g., parking space markings).

## Why Refactor Instead of Patch

Current implementation intermixes:
- parsing,
- semantic interpretation,
- geometry expansion,
- consumer-specific behavior.

This makes feature additions expensive and inconsistent across viewer and OSI.

## Current Coupling Hotspots

- Parser creates special continuous-repeat outlines during parse time instead of preserving source semantics.
  - `EnvironmentSimulator/Modules/RoadManager/RoadManager.cpp` (object parsing and `CreateContinuousRepeatOutline`)
- Viewer uses mostly one repeat (`GetRepeat`) despite repeat list availability.
  - `EnvironmentSimulator/Modules/ViewerBase/roadgeom.cpp`
- OSI assembles polygon points directly from outline corner containers with consumer-specific assumptions.
  - `EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/OSIReporter.cpp`

## Target Architecture

### Layer 1: Source-Faithful Model (Parse Output)

Create immutable source-faithful types. No tessellation, no consumer fallback behavior.

```cpp
struct RMObjectDefinition {
    uint64_t id;
    std::string name;
    RMObjectType type;

    // Pose at object anchor in road coordinates
    double s;
    double t;
    double zOffset;
    double hdg;
    double pitch;
    double roll;
    Orientation orientation;

    // Optional primitive extents (box/circle-like semantics)
    std::optional<double> length;
    std::optional<double> width;
    std::optional<double> height;
    std::optional<double> radius;

    std::vector<RMOutlineDefinition> outlines;
    std::vector<RMRepeatDefinition> repeats;
    std::vector<RMObjectMarkingDefinition> markings;

    std::optional<ParkingSpace> parkingSpace;
    std::vector<ValidityRecord> validity;
};

enum class RMCornerCoordSystem { Road, Local };

struct RMOutlineCornerDefinition {
    RMCornerCoordSystem coordSystem;

    // Road corner
    std::optional<double> s;
    std::optional<double> t;
    std::optional<double> dz;

    // Local corner
    std::optional<double> u;
    std::optional<double> v;
    std::optional<double> zLocal;

    // Common
    std::optional<double> height;
};

  // Coordinate semantics (must be preserved by parser and expansion):
  // - Road (cornerRoad): (s, t, dz) is attached to road reference geometry, so point location
  //   follows road curvature/superelevation at that s.
  // - Local (cornerLocal): (u, v, zLocal) is in object-local coordinates (fixed shape). The local
  //   frame origin is the object anchor at (object.s, object.t), then transformed to world by
  //   road pose at anchor + object heading/pitch/roll.

struct RMOutlineDefinition {
    uint64_t id;
    bool closed;
    OutlineFillType fillType;
    std::vector<RMOutlineCornerDefinition> corners;
};

struct RMRepeatDefinition {
    double s;
    double length;
    double distance; // 0 => continuous

    std::optional<double> tStart;
    std::optional<double> tEnd;
    std::optional<double> zOffsetStart;
    std::optional<double> zOffsetEnd;
    std::optional<double> widthStart;
    std::optional<double> widthEnd;
    std::optional<double> heightStart;
    std::optional<double> heightEnd;
    std::optional<double> lengthStart;
    std::optional<double> lengthEnd;
    std::optional<double> radiusStart;
    std::optional<double> radiusEnd;
};

struct RMObjectMarkingDefinition {
    // Keep broad to support OpenDRIVE 1.8 evolution and userData extras.
    uint64_t id;
    std::string type;
    std::optional<double> width;
    std::optional<double> length;
    std::optional<double> zOffset;
    std::optional<std::string> color;

  enum class Side {
    Front,
    Left,
    Rear,
    Right
  };

  // Marking can target one or multiple object edges.
  std::vector<Side> sides;

    // Marking geometry anchor policy (edge-relative, outline-relative, etc.)
    std::string placementMode;
};
```

### Layer 2: Geometry Expansion Engine (Single Truth)

Convert one `RMObjectDefinition` into one or many concrete geometry instances for consumers.

```cpp
struct RMExpandedObject {
    uint64_t sourceObjectId;
    uint64_t expandedId;

    RMPose3D pose;
    RMObjectType type;

    // Either primitive box, explicit polygon(s), or mesh ref
    std::optional<RMBox3D> primitiveBox;
    std::vector<RMPolygon3D> polygons;

    std::vector<RMExpandedMarking> markings;

    RMExpansionMetadata metadata;
};
```

Expansion rules:
- If no repeat: emit one instance.
- If repeat distance > 0: emit discrete instances with interpolated transform/size.
- If repeat distance == 0: emit continuous strip segments and edge-stitched polygons.
- For mixed `cornerRoad` + `cornerLocal` outlines: normalize to world-space polygon(s) before emission.
- For markings: resolve `sides` against the expanded object boundary in object-local order (front/left/rear/right),
  then emit one marking primitive per selected side.

Normalization details for mixed coordinate systems:
- `cornerRoad`: evaluate each corner at its own (s, t, dz) on the road. Curvature affects each corner independently.
- `cornerLocal`: build corner from local (u, v, zLocal) in the object frame, then apply object anchor road pose and object
  orientation once. Curvature does not deform the local shape itself.
- For repeated objects, re-evaluate both corner types per emitted repeat instance/segment so anchor and interpolation changes
  are reflected correctly.

### Layer 3: Consumer Adapters

- Viewer adapter: expanded objects -> OSG geometry / model transforms.
- OSI adapter: expanded objects -> OSI stationary objects and road markings.

Both adapters read `RMExpandedObject`, not parser internals.

## Continuous Repeat Geometry Rule (No Gap/Overlap)

For each step `i` along s:
1. Evaluate left/right edge points at section `i` from interpolated params.
2. Form provisional quad between section `i` and `i+1`.
3. At shared boundary between neighbor quads, compute edge-line intersection in local road-plane:
   - right edge of quad `i` with right edge of quad `i+1`,
   - left edge of quad `i` with left edge of quad `i+1`.
4. Clamp with fallback (midpoint) if lines are near-parallel or numerically unstable.
5. Replace boundary vertices with intersection points.

Result: connected 4-corner polygons with continuous boundaries and no overlap/gap artifacts from curvature.

## Parser Changes

In object parsing flow:
- Parse `outlines/outline/(cornerRoad|cornerLocal)` into source model only.
- Parse all repeats into `repeats` vector only.
- Parse `markings` subtree (new).
- Remove parse-time shape synthesis (`CreateContinuousRepeatOutline`) from primary object path.

Keep old code path behind temporary fallback flag while migrating.

## Viewer Changes

In road object generation flow:
- Replace direct `GetRepeat()` branch logic with adapter consuming expanded objects.
- Keep model file loading behavior, but apply it on expanded instances.
- Preserve tunnel implementation until object pipeline is stable, then optionally migrate tunnel to same expansion engine.

## OSI Changes

In ODR stationary object update flow:
- Build OSI shape from expanded polygon/box data.
- Keep source references to original ODR object id.
- Add separate reporting path for object markings using expanded markings.

## Data Ownership and Safety

Current classes use raw owning pointers in several containers. During refactor:
- Prefer value types for definition and expanded DTOs.
- Use `std::unique_ptr` only where polymorphism is required.
- Avoid exposing mutable internals from parser classes.

## Feature Flags

Use runtime toggles to derisk migration:
- `road_objects_refactor_parser`
- `road_objects_refactor_viewer`
- `road_objects_refactor_osi`
- `road_objects_refactor_markings`

Recommended switch strategy:
1. Parser flag ON in tests only.
2. Viewer flag ON for selected maps.
3. OSI flag ON after geometry parity checks.
4. Markings flag ON last.

## Milestones

### M1: Canonical Definition Model
- Add source-faithful structs and parser population.
- Keep existing RMObject population unchanged in parallel.
- Unit tests: parse snapshot equivalence for objects/repeats/outlines.

### M2: Expansion Engine for Repeat + Outlines
- Implement discrete repeat expansion.
- Implement continuous repeat expansion with edge stitching.
- Unit tests: geometric continuity assertions and corner count checks.

### M3: Viewer Adapter
- Render from expanded objects under feature flag.
- Visual parity checks with existing maps.

### M4: OSI Adapter
- Populate stationary objects from expanded objects.
- Add object marking emission.
- Regression checks against OSI golden outputs.

### M5: Remove Legacy Branches
- Remove parse-time continuous-outline generation from legacy parser path.
- Remove single-repeat assumptions (`GetRepeat`) from viewer.
- Keep tunnel path as-is unless consciously migrated.

## Current Status (May 25 2026)

This section captures implementation progress after the recent roadmap slice for repeated objects and markings.

Completed now:
- Expansion kinds are in active use in viewer flow:
  - single-anchor object,
  - discrete repeat instance,
  - continuous repeat segment,
  - outline object.
- Continuous-repeat segments are generated as stitched quads with world-corner continuity handling.
- Object heading propagation is applied from definition to expanded instances and used in viewer transforms.
- Marking rendering supports:
  - segment-aligned rectangular dashes,
  - conditional corner stitching only for full-edge coverage,
  - continuous-line semantics when `spaceLength == 0`.
- Continuous-repeat outline markings (including corner references) are handled through segment world-corner edges.
- `cornerRoad` outline corners for repeated instances are re-evaluated per instance in road coordinates, avoiding shape reuse artifacts on changing curvature.

Not yet completed:
- OSI adapter migration to consume expanded objects as primary shape source.
- Full retirement of legacy viewer/parser branches.
- Dedicated automated test coverage for all new marking and outline edge cases.

## Implemented Decision Notes

1. `cornerRoad` in repeated instances
- Decision: do not reuse one Euclidean shape across repeat instances.
- Rationale: road curvature and elevation vary with s, so corner road coordinates must be re-evaluated for each instance.

2. Dash generation at corners
- Decision: generate dashes segment-aligned from centerline + fixed segment normal.
- Rationale: prevents skewed dash quads when adjacent edge directions differ.

3. Corner stitching scope
- Decision: apply join stitching only when a marking segment covers an entire edge.
- Rationale: avoids asymmetric joins for partial edge coverage.

4. Continuous-line interpretation
- Decision: interpret `spaceLength == 0` as infinite line length for continuous marks.
- Rationale: provides consistent, explicit behavior for solid markings.

5. Continuous-repeat marking edge resolution
- Decision: treat continuous-repeat segments with world corners as outline-capable for reference resolution.
- Rationale: corner/edge reference markings otherwise disappear when object kind is segment-based.

## Testing Plan

1. Unit tests (RoadManager)
- Mixed `cornerRoad` and `cornerLocal` in same object.
- Shape-invariance check for `cornerLocal` on curved roads (pairwise edge lengths in local frame remain constant).
- Curvature-following check for `cornerRoad` on curved roads (corner world positions vary with road geometry as expected).
- Repeat list with distance 0 and >0.
- Interpolation endpoint behavior (`*Start/*End`).
- Degenerate dimensions and near-parallel edge cases.

2. Integration tests (Viewer)
- Curved-road continuous object strip: no visible gaps/overlaps.
- Parking-space objects with markings.

3. Integration tests (OSI)
- Stationary object dimensions/pose correctness.
- Base polygon consistency for expanded outlines.
- Marking presence/type/color mapping.

4. Performance checks
- Expansion time and memory for long roads with dense repeats.

## Recommended First Coding Slice (Priority-Aligned)

1. Add new source-faithful object DTOs and parser fill-in (no consumer changes).
2. Implement expansion engine for:
   - simple box object,
   - repeat distance > 0,
   - repeat distance == 0 continuous strip with edge stitching.
3. Add a debug dump utility to compare legacy vs expanded instance count and poses.
4. Wire viewer adapter for expanded boxes only under feature flag.

This provides immediate progress on your priority #1 with minimal risk and a clear validation path.
