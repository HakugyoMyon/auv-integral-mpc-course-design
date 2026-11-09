# Static local-planning component

Entry: `sta_local_plan(xhat, master, mission, map, opts)`; defaults from `sta_planner_config`. Only public navigation, original task geometry and `sta_map_query` are used. No world/truth argument is accepted.

Output `proposal.accepted` means a **candidate**, not MPC admission or mission success. `proposal.route` uses standard `d11_segment` XY geometry and adds:

- `depthTargets`: one constant target per segment for the new static-task wrapper. A vertical maneuver holds its escape target until a position-defined hold end, then commands the original nominal depth.
- `depthSchedule`: `[horizontal cumulative arc length, nominal depth]` used for geometry/tube checking, with .85m/s upper speed and .06m/s depth slew. This is a conservative nominal schedule, not actual physical motion.
- `resumeRoute` and `resumePoint.segment/progress`: original task continuation, also usable by `d16_path_preview`. Original target IDs/ledger are not recreated.
- `maneuverKind` (`nominal`, `horizontal`, `vertical`) and `maneuverSide` (`-1` left/up, `+1` right/down).

`proposal.candidates` records prefix checks, known-occupied collisions, unknown sample count, tube assumptions and scores. A whole candidate may have an unknown tail, but that tail is uncommitted. Only the declared short commit prefix may be used, provided the caller additionally applies fresh MPC/actuator admission and recertifies the prefix before its budget expires. Replanning every tick is neither needed nor recommended; retain the active route while its rolling prefix remains certified.

The vehicle box used by the baseline clearance model is covered by 7 longitudinal balls at body x∈[-1.30,.65]m. Each ball radius includes sqrt((sqrt(2)*.205)^2 + half-center-gap^2), followed once by current navigation, tracking-tube, separate residual-clearance, attitude and sample-resolution terms. The 0.5m tracking tube and 0.5m residual physical gap are distinct. The current navigation ball is hypot(horizontal radius95, 3*depth sigma); this is an enclosing uncertainty budget, not a joint-calibration claim. Sensor-frame pose/range/beam uncertainty belongs to the perception certificate and is not silently removed. These are conditional model assumptions, not a proof of calibrated navigation confidence or universal physical safety.

Synthetic checks: `sta_planning_contract(outputDirectory)` exercises map-state semantics, unknown-prefix rejection, uncommitted unknown tails, XY curvature/continuity, and horizontal/vertical geometric candidates. The parent agent runs MATLAB; this subagent does not start or use a MATLAB session.

`[pointer,audit]=sta_project_master(master,xhat,oldPointer)` performs estimated-pose reprojection onto a finite forward part of the original route. Defaults: 100m forward, at most 8 segments, at most 12m lateral distance and 70deg tangent/yaw discrepancy. It never searches backwards, copies all old pointer memory fields, and changes only segment/progress when an acceptable nearby projection exists. It does not mark work complete or certify a handoff connection. Optional pointer fields are `projectionWindow_m` (≤150), `projectionMaxSegments` (≤16), `projectionMaxDistance_m` (≤25) and `projectionMaxHeadingError_rad` (≤pi/2). The caller may use a tighter distance/heading gate for actual detour handoff than for bookkeeping during avoidance. `sta_projection_contract(outdir)` supplies seven synthetic monotonic/window/crossing/lap cases.

## Method grounding

The forward-only path family and bounded vertical-rate decomposition are informed by [Owen, Beard and McLain (2014), Implementing Dubins Airplane Paths on Fixed-wing UAVs](https://scholarsarchive.byu.edu/facpub/1900/). Aircraft coordinated-turn dynamics are **not** imported. The AUV retains its full six-DOF physical model, multivariable MPC and nonlinear admissibility checks, because a geometric Dubins route does not itself establish actuator/vehicle feasibility.

The finite library is deliberately incomplete: no accepted candidate is not a proof that every feasible route is impossible. Horizontal or vertical avoidance can be physically infeasible when sensor lookahead, depth slew, the original task and uncertainty budgets leave insufficient maneuvering room. Do not shrink the real vehicle or confidence budget to force a pass.

## Optional depth-acquisition circle

`enableHelical=false` preserves the regular catalog. Enabling it with a separately declared `maximumRouteLength_m=360` adds a15m constant-depth sensing lead-in, R25 full-circle depth acquisition, a50m straight pass, depth restoration, and a Dubins rejoin. `helicalDepthOffsets_m=6.5` remains subject to the original depth bounds, depth reference slew, chart limits, uncertainty and map checks. The nominal depth schedule includes the existing five-second acquisition reserve. `sta_helical_contract` is a synthetic geometry test only. An unobserved circle is never a guaranteed safe loiter; fresh nonlinear MPC and observed-free rolling prefix admission remain mandatory, and the integration must check the remaining task/return time separately.

The motion-primitive idea follows Owen, Beard and McLain (2014), [Implementing Dubins Airplane Paths on Fixed-wing UAVs](https://scholarsarchive.byu.edu/facpub/1900/). Aircraft results do not establish AUV control or obstacle-avoidance guarantees.

