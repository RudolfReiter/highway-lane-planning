# Reproduction scope

This is a new, simplified implementation of **A Long-Short-Term Mixed-Integer Formulation for Highway Lane Change Planning**, Reiter et al., IEEE TIV 2024, DOI [10.1109/TIV.2024.3398805](https://doi.org/10.1109/TIV.2024.3398805). It is **not the original repository**. The [author manuscript](https://publications.syscop.de/Reiter2024.pdf) defines the research architecture. All examples and reported numbers here are new synthetic results.

## Paper-to-code mapping

| Paper concept | Implementation | Difference |
| --- | --- | --- |
| Four-state STF model, Eq. 17 | Exact constant-acceleration point-mass dynamics in `dynamics_map` | Straight road and fixed acceleration bounds |
| Short-term collision-free corridor | Conservative source and target gap bounds, checked every 0.1 s | Fixed overlap phases replace the original disjunction geometry |
| First lane transition consistency, Eq. 33 | A lateral crossing equality at a selected short-term knot | First crossing time is enumerated, not continuous |
| LTF transition time and position | Continuous second transition `(tau, sigma)` | One future transition, three total lanes |
| Reachable set, Eq. 24 | A cone from the final short-term position with speed bounds 18–34 m/s | Simplified cone without the paper's shifted bounds |
| Gap centering, Section V-B | Weighted radius in time–position inequalities | Reference-speed transition segment and radius cap |
| Gap disjunctions, Eqs. 26–28 | Explicit enumeration of all ordered gaps | No branch-and-bound MIQP solver |
| Consistent joint formulation, Section VI | Short controls and future transition in the same convex QP | Reduced candidate set and simplified costs |

The formulation remains coupled within each candidate: changing short-term acceleration changes both its final position and first transition position, which directly affect future reachability and cost. The detailed first plan is not generated after a separately fixed high-level transition.

## Discrete choices and continuous variables

The horizon is **6 seconds**, with **12 intervals of 0.5 seconds**. States are `[s, n, v_s, v_n]`; inputs are longitudinal and lateral acceleration. The exact model in each coordinate is `p_next = p + dt*v + dt²*a/2`, `v_next = v + dt*a`.

Each short-term branch selects a first lane crossing at **2.5, 3.0 or 3.5 s** and a gap in lane 2. A gap is bounded by a preceding/following vehicle, with unbounded exterior gaps also included. The coupled planner additionally enumerates a gap in lane 3. Candidate enumeration is a transparent finite mixed-discrete search; it is not the original full MIQP implementation and does not scale well to many lanes or vehicles.

Each continuous problem contains 24 short-term accelerations and, for a coupled branch, the future transition time, position and centering radius. SciPy/HiGHS first checks linear feasibility. **HiGHS through CasADi 3.7.2** then solves the convex QP. It uses one solver thread and a 10 s limit per QP. Unexpected solver failures raise an error; infeasible branches are explicitly excluded.

Fixed baseline controls are eliminated before solving the smaller future-transition problem. Constant constraints are removed only after their feasibility is checked.

## Constraints and costs

Lanes are 3.6 m wide. Cars have a nominal axis-aligned footprint of 4.6 × 1.8 m. Longitudinal acceleration is limited to −3 to +2 m/s², lateral acceleration to ±1.2 m/s², longitudinal speed to 16–34 m/s, and lateral speed to 0–1.7 m/s. The first maneuver ends at the center of lane 2 with zero lateral speed.

Before `crossing - 1.5 s`, lateral position stays at or below 0.6 m. After `crossing + 1.5 s`, it stays at or above 3.0 m. Both lane gaps constrain the longitudinal trajectory during the overlap interval. The center-to-center longitudinal buffer is **11 m**. Collision constraints are imposed at 0.1 s intervals; exported trajectories are evaluated at 0.02 s intervals.

The second transition time is continuous in **[8,18] s**. Its longitudinal position satisfies

```
18 (tau - 6) <= sigma - s(6) <= 34 (tau - 6).
```

Both adjoining gaps must be free along a three-second reference-speed line centered on `(tau,sigma)`. The weighted centering normal is `sqrt(1 + (traffic_speed/29)²)`; radius is bounded by 0–8 m. This coarse representation does **not** guarantee that a complete dynamically feasible second lane change exists, including acceleration continuity with the first plan.

The reference speed is 29 m/s. The objective sums squared speed error, longitudinal/lateral control effort and control differences, lateral velocity, first crossing time, future transition time, a future reference-speed displacement penalty, and a linear reward for centering radius. Exact coefficients are visible in `branch()`. The future-time coefficient is 6; first-crossing coefficient is 4. The newly chosen weights are illustrative, not recovered from the paper's experiments.

## Baseline and metrics

The **coupled** method optimizes all variables together and selects the lowest-cost feasible discrete candidate. The **short-term-first** baseline selects its best short-only candidate, freezes its controls, and then optimizes its later transition over all goal-lane gaps. This baseline is an explanatory ablation, not the paper's MIP-DM or hybrid A* comparator.

Three deterministic traffic scenes are evaluated with both methods: **six runs**. Traffic follows constant-speed, fixed-lane predictions and does not respond to the ego car. There is no SUMO, CommonRoad, low-level NMPC, interactive braking or independent vehicle tracking simulation.

- **Future transition time** is a coarse planning output, not a realized goal-lane arrival.
- **Speed RMSE** compares the detailed six-second longitudinal speed against 29 m/s.
- **Combined objective** evaluates the same complete surrogate cost for both methods, including the baseline's subsequently selected transition.
- **Body separation** uses `max(|delta s| - 4.6, |delta n| - 1.8)` over all exported time samples and traffic cars. It is signed axis-aligned rectangular separation, not Euclidean distance or an exact rotated-body metric.
- **Solve time** includes enumeration, feasibility checks and QPs. First-call overhead and machine differences matter; no real-time claim is made.

In the compact goal-lane traffic scene, the coupled planner slows earlier and schedules the coarse transition at about **10.51 s**, compared with **12.94 s** for the baseline. Its short-term speed RMSE is worse (**2.28 vs 0.50 m/s**). Both outcomes are displayed: the objective explicitly values transition timing as well as reference-speed tracking. The other scenes show much smaller differences. No paper benchmark percentage has been recovered.

## Reproduction and validation

```bash
python -m pip install -r requirements-reproduce.txt -e .
OPENBLAS_NUM_THREADS=1 python -m highway_planning.experiment
MPLCONFIGDIR=/tmp/highway-planning-mpl python scripts/plot_results.py
OPENBLAS_NUM_THREADS=1 python -m pytest -q
node scripts/check_interactions.mjs
python scripts/build_site.py
python scripts/check_site.py
```

Tests include an independently known convex-QP solution, infeasibility handling, exact dynamics between grid knots, a fresh coupled-versus-baseline comparison, and independent checks of all exported dynamics, input limits, vehicle separation and future gap margins. Sampled validation is not a proof of continuous-time collision avoidance.

The scene uses the detailed plan for six seconds only. Future transition markers and dotted connectors in the time–position plot are explicitly labeled coarse approximations. Both cars are shown at the same elapsed time. Playback starts paused, stops at the end, and has a smoothly filtered follow camera.

The UI has application-state tests and scene exports from its actual renderer. These do not replace browser layout testing; browser screenshot verification was unavailable in the development sandbox.
