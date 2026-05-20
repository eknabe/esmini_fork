# Road Objects Refactor Status - 2026-05

## Scope of This Milestone

This milestone focused on repeated object geometry and object markings in the viewer expansion path.

Primary outcomes:
- Resolved dash-shape artifacts (tilted/skewed segment quads).
- Added consistent heading propagation for expanded objects.
- Improved marking placement and stitching behavior.
- Fixed missing markings on continuous-repeat objects with outline references.
- Corrected repeated `cornerRoad` outline handling to re-evaluate corners per instance.

## Implemented Changes

1. Marking geometry generation
- Dashes are emitted as segment-aligned rectangles using segment centerline and normal.
- `spaceLength == 0` is normalized to continuous line behavior.
- Corner join stitching is applied only when a segment spans a full edge.

2. Pose and placement behavior
- Object heading (`hdg`) is propagated from object definition into expanded instances and used in viewer transforms.
- Marking Z placement is aligned at the intended surface/vertex-level basis for outline-driven geometry.

3. Continuous repeat support
- Continuous repeat objects expose stitched world corners per segment.
- Marking references (corner and edge) are resolved against these world-corner edges.
- Continuous-repeat objects with outline references now render markings correctly.

4. Repeated cornerRoad outline behavior
- Repeated instances no longer reuse a single Euclidean outline shape for `cornerRoad` corners.
- Per-instance corner evaluation now uses road-frame offsets (`ds`, `dt`) from definition anchor and re-samples road geometry at each instance position.
- This prevents curvature-dependent shape drift/gaps on roads with changing geometry.

## Validation Summary

Build and runtime validation performed in this milestone:
- `odrviewer` Release builds succeeded (exit code 0).
- Scenario runs for updated test maps loaded and executed successfully (exit code 0).
- Continuous parking-space marking scenario renders with expected corner-reference markings.
- Known build warning observed (MSB8065 in CommonMini custom build output generation), not introduced by this milestone.

## Current Known Risks and Gaps

- Automated regression tests for new marking edge-cases should be expanded.
- Legacy OSI fallback path for roads without object definitions remains as an intentional compatibility path.

## Recommended Next Steps

1. (Done, May 2026) Add targeted unit tests in RoadManager for:
- repeated `cornerRoad` on varying curvature,
- mixed `cornerRoad` + `cornerLocal` repeats,
- corner/edge reference resolution for continuous segments.

2. (Done, May 2026) Add viewer integration checks for:
- partial-edge markings (no unintended joins),
- full-edge markings (expected joins),
- `spaceLength == 0` continuity behavior.

3. (Done, May 2026) Complete M4 migration:
- OSI object and marking emission now uses expanded-object data for roads with canonical object definitions.

4. (Done, May 2026) Execute M5 parser/viewer cleanup slice:
- Removed parse-time continuous-repeat outline synthesis from the legacy parser path.
- Removed viewer single-repeat assumption by replacing `GetRepeat()` usage with iteration over all repeats (`GetRepeatByIdx`).

5. (Done, May 2026) Finalize M5 viewer branch retirement:
- Removed `road_objects_refactor_viewer` branch in viewer object rendering.
- Expanded-object rendering is now unconditional.
- Preserved tunnel node-prefix behavior in the expanded path.
- Removed obsolete `road_objects_refactor_viewer` CLI/config option wiring.
