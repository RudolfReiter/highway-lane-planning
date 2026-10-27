"""Coupled short-horizon QP and continuous long-horizon gap transitions.

Discrete gap choices and first-crossing times are explicitly enumerated. Each
branch is a convex QP, solved by HiGHS through CasADi after a feasibility solve. This
avoids proprietary MIQP solvers, at the expense of combinatorial scalability.
The first crossing is quantized; the second transition time/position remain
continuous and are coupled to the short trajectory in the same optimization.
"""
from dataclasses import dataclass
from itertools import product
from time import perf_counter
import numpy as np
import casadi as ca
from scipy.optimize import linprog

DT, N, LANE_WIDTH = .5, 12, 3.6
HORIZON, LONG_HORIZON = DT * N, 18.0
BUFFER = 11.0  # Center-to-center minimum, including 4.6 m vehicle length.
VREF = 29.0
FIRST_CROSSINGS = (2.5, 3.0, 3.5)


@dataclass(frozen=True)
class Vehicle:
    name: str
    lane: int
    s: float
    speed: float


@dataclass(frozen=True)
class Scenario:
    id: str
    name: str
    description: str
    speed: float
    vehicles: tuple


SCENARIOS = (
    Scenario("open", "Staggered traffic", "Two lane changes are needed. The next lane is open, but the goal-lane gap is moving faster.", 27,
             (Vehicle("A", 0, 72, 24), Vehicle("B", 0, -70, 27), Vehicle("C", 1, -34, 25), Vehicle("D", 1, 55, 25),
              Vehicle("E", 2, -80, 31), Vehicle("F", 2, 24, 31), Vehicle("G", 2, 140, 31))),
    Scenario("slow", "A slower middle lane", "The middle-lane leader limits acceleration. A coupled plan chooses speed with the next transition in mind.", 26,
             (Vehicle("A", 0, 76, 23), Vehicle("B", 0, -75, 26), Vehicle("C", 1, -30, 23), Vehicle("D", 1, 43, 23),
              Vehicle("E", 2, -60, 28), Vehicle("F", 2, 20, 28), Vehicle("G", 2, 118, 28))),
    Scenario("dense", "A delayed goal-lane gap", "A compact, slower pack occupies the goal lane. Planning an earlier speed reduction can make its rear gap reachable sooner.", 27,
             (Vehicle("A", 0, 68, 25), Vehicle("B", 0, -62, 27), Vehicle("C", 1, -95, 26), Vehicle("D", 1, 53, 26),
              Vehicle("E", 2, 20, 23), Vehicle("F", 2, 45, 23), Vehicle("G", 2, 70, 23), Vehicle("H", 2, 95, 23))),
)


def gaps(scenario, lane):
    vehicles = sorted((v for v in scenario.vehicles if v.lane == lane), key=lambda v: v.s)
    return list(zip([None] + vehicles, vehicles + [None]))


def dynamics_map(times):
    """Exact constant-acceleration influence matrices, also between knots."""
    times = np.asarray(times)
    j = np.arange(N) * DT
    elapsed = np.clip(times[:, None] - j, 0, DT)
    velocity = elapsed
    position = elapsed * np.maximum(times[:, None] - j - .5 * elapsed, 0)
    return position, velocity


class Program:
    def __init__(self, size):
        self.size = size
        self.rows, self.lo, self.hi = [], [], []
        self.Q = np.zeros((size, size)); self.c = np.zeros(size); self.constant = 0.0

    def constrain(self, row, low=-np.inf, high=np.inf):
        self.rows.append(np.asarray(row)); self.lo.append(low); self.hi.append(high)

    def square(self, row, offset, weight):
        self.Q += 2 * weight * np.outer(row, row)
        self.c += 2 * weight * offset * row
        self.constant += weight * offset ** 2

    def solve(self, lower, upper):
        A, lo, hi = np.asarray(self.rows), np.array(self.lo), np.array(self.hi)
        # Eliminate fixed variables, including a frozen baseline.
        # Drop constant rows only after checking that they are feasible.
        fixed = abs(np.asarray(upper) - lower) < 1e-10
        free = ~fixed
        offset = np.zeros(self.size); offset[fixed] = lower[fixed]
        lo, hi = lo - A @ offset, hi - A @ offset
        A = A[:, free]
        zero = np.linalg.norm(A, axis=1) < 1e-12
        if np.any(lo[zero] > 2e-6) or np.any(hi[zero] < -2e-6): return None
        A, lo, hi = A[~zero], lo[~zero], hi[~zero]
        Q = self.Q[np.ix_(free, free)]
        c = (self.c + self.Q @ offset)[free]
        constant = self.constant + self.c @ offset + .5 * offset @ self.Q @ offset
        lower, upper = np.array(lower)[free], np.array(upper)[free]
        equal = np.isfinite(lo) & np.isfinite(hi) & (abs(lo - hi) < 1e-9)
        lower_rows, upper_rows = np.isfinite(lo) & ~equal, np.isfinite(hi) & ~equal
        Aineq = np.vstack((-A[lower_rows], A[upper_rows]))
        bineq = np.r_[-lo[lower_rows], hi[upper_rows]]
        feasible = linprog(np.zeros(sum(free)), A_ub=Aineq, b_ub=bineq, A_eq=A[equal], b_eq=lo[equal],
                           bounds=list(zip(lower, upper)), method="highs")
        if feasible.status == 2:
            return None
        if not feasible.success:
            raise RuntimeError(f"Feasibility solver failed: {feasible.message}")
        solver = ca.conic("branch", "highs", {"h": ca.DM(Q).sparsity(), "a": ca.DM(A).sparsity()},
                          {"error_on_fail": False, "highs": {"output_flag": False, "threads": 1, "time_limit": 10}})
        result = solver(h=Q, g=c, a=A, lba=lo, uba=hi, lbx=lower, ubx=upper, x0=feasible.x)
        x = np.asarray(result["x"]).ravel()
        residual = max(float(np.max(lo - A @ x)), float(np.max(A @ x - hi)), 0)
        if not solver.stats()["success"] or residual > 2e-5:
            raise RuntimeError(f"QP solve failed: {solver.stats()}; constraint violation={residual}")
        offset[free] = x
        return offset, float(.5 * x @ Q @ x + c @ x + constant), residual


def branch(scenario, crossing, gap1, gap2=None, fixed_controls=None):
    coupled = gap2 is not None
    size = 2 * N + (3 if coupled else 0)
    program = Program(size)
    eye = np.eye(size)
    t = np.linspace(0, HORIZON, 61)
    P, V = dynamics_map(t)
    s_rows, n_rows, vs_rows, vn_rows = (np.zeros((len(t), size)) for _ in range(4))
    s_rows[:, :N] = P; vs_rows[:, :N] = V
    n_rows[:, N:2 * N] = P; vn_rows[:, N:2 * N] = V
    base_s = scenario.speed * t
    source_gap = next(g for g in gaps(scenario, 0) if (g[0] is None or g[0].s < 0) and (g[1] is None or g[1].s > 0))
    enter, leave = crossing - 1.5, crossing + 1.5
    for k, ti in enumerate(t):
        program.constrain(vs_rows[k], 16 - scenario.speed, 34 - scenario.speed)
        program.constrain(n_rows[k], 0, LANE_WIDTH)
        program.constrain(vn_rows[k], 0, 1.7)
        if ti <= enter + 1e-8: program.constrain(n_rows[k], high=.6)
        if ti >= leave - 1e-8: program.constrain(n_rows[k], low=3.0)
        active_gaps = []
        if ti <= leave + 1e-8: active_gaps.append(source_gap)
        if ti >= enter - 1e-8: active_gaps.append(gap1)
        for rear, front in active_gaps:
            if rear: program.constrain(s_rows[k], low=rear.s + rear.speed * ti + BUFFER - base_s[k])
            if front: program.constrain(s_rows[k], high=front.s + front.speed * ti - BUFFER - base_s[k])
    cross_index = round(crossing * 10)
    program.constrain(n_rows[cross_index], LANE_WIDTH / 2, LANE_WIDTH / 2)
    program.constrain(n_rows[-1], LANE_WIDTH, LANE_WIDTH)
    program.constrain(vn_rows[-1], 0, 0)
    for k in range(0, len(t), 5):
        program.square(vs_rows[k], scenario.speed - VREF, .4)
        program.square(vn_rows[k], 0, .15)
    for j in range(N):
        program.square(eye[j], 0, .12)
        program.square(eye[N + j], 0, .15)
        if j:
            program.square(eye[j] - eye[j-1], 0, .08)
            program.square(eye[N+j] - eye[N+j-1], 0, .12)
    program.constant += 4 * crossing
    lower = np.r_[np.full(N, -3.), np.full(N, -1.2)]
    upper = np.r_[np.full(N, 2.), np.full(N, 1.2)]
    if fixed_controls is not None:
        lower[:2*N] = fixed_controls; upper[:2*N] = fixed_controls
    if coupled:
        tau, sigma, radius = eye[-3], eye[-2], eye[-1]
        lower = np.r_[lower, 8, 0, 0]; upper = np.r_[upper, LONG_HORIZON, 700, 8]
        # Continuous time-space reachability, coupled directly to the final
        # short-term position. This deliberately simpler cone omits Eq. 24's
        # shifted bounds; lane-change duration is enforced by tau >= 8 s.
        program.constrain(sigma - s_rows[-1] - 18 * tau, low=scenario.speed * HORIZON - 18 * HORIZON)
        program.constrain(sigma - s_rows[-1] - 34 * tau, high=scenario.speed * HORIZON - 34 * HORIZON)
        # Both adjoining gaps must remain free over a three-second transition.
        # A reference-speed line through (tau, sigma) approximates that segment.
        for rear, front in (gap1, gap2):
            for vehicle, sign in ((rear, 1), (front, -1)):
                if vehicle is None: continue
                normal = np.sqrt(1 + (vehicle.speed / VREF) ** 2)
                for offset in (-1.5, 1.5):
                    row = sign * (sigma - vehicle.speed * tau) - normal * radius
                    bound = BUFFER + sign * (vehicle.s + (vehicle.speed - VREF) * offset)
                    program.constrain(row, low=bound)
        program.c += 6 * tau - .3 * radius
        program.square(sigma - s_rows[cross_index] - VREF * tau,
                       -base_s[cross_index] + VREF * crossing, .014)
    solution = program.solve(lower, upper)
    if solution is None: return None
    z, cost, residual = solution
    return {"z": z, "cost": cost, "crossing": crossing, "gap1": gap1, "gap2": gap2, "residual": residual}


def gap_names(gap):
    if gap is None: return None
    return [v.name if v else None for v in gap]


def plan(scenario, mode="coupled"):
    start = perf_counter()
    feasible = []; attempts = 0
    if mode == "coupled":
        for crossing, gap1, gap2 in product(FIRST_CROSSINGS, gaps(scenario, 1), gaps(scenario, 2)):
            attempts += 1
            candidate = branch(scenario, crossing, gap1, gap2)
            if candidate is not None: feasible.append(candidate)
    elif mode == "short":
        short = []
        for crossing, gap1 in product(FIRST_CROSSINGS, gaps(scenario, 1)):
            attempts += 1
            candidate = branch(scenario, crossing, gap1)
            if candidate is not None: short.append(candidate)
        if not short: raise RuntimeError("No feasible short-term plan")
        first = min(short, key=lambda c: c["cost"])
        for gap2 in gaps(scenario, 2):
            attempts += 1
            candidate = branch(scenario, first["crossing"], first["gap1"], gap2, fixed_controls=first["z"])
            if candidate is not None: feasible.append(candidate)
        if not feasible:
            first["z"] = np.r_[first["z"], np.nan, np.nan, 0]
            feasible = [first]
    else:
        raise ValueError("mode must be coupled or short")
    if not feasible: raise RuntimeError("No feasible coupled plan")
    best = min(feasible, key=lambda c: c["cost"])
    traj = rollout(scenario.speed, best["z"][:2*N])
    collision_margin = []
    for vehicle in scenario.vehicles:
        dx = abs(traj["s"] - (vehicle.s + vehicle.speed * traj["time"])) - 4.6
        dy = abs(traj["n"] - LANE_WIDTH * vehicle.lane) - 1.8
        collision_margin.append(float(np.min(np.maximum(dx, dy))))
    valid_long = np.isfinite(best["z"][-3])
    metrics = {"speed_rmse_mps": float(np.sqrt(np.mean((traj["speed"] - VREF) ** 2))),
               "first_transition_s": best["crossing"], "goal_transition_s": float(best["z"][-3]) if valid_long else None,
               "objective": best["cost"], "centering_radius_m": float(best["z"][-1]),
               "min_body_separation_m": min(collision_margin), "collision_free": min(collision_margin) >= -1e-6,
               "max_constraint_violation": best["residual"], "solve_ms": (perf_counter()-start)*1000,
               "candidate_count": attempts, "feasible_candidates": len(feasible)}
    if not metrics["collision_free"]: raise RuntimeError("Short-term plan intersects traffic")
    return {"id": mode, "name": "Coupled long–short term" if mode == "coupled" else "Short-term first",
            "trajectory": traj, "controls": best["z"][:2*N].reshape(2, N).T,
            "transitions": [{"time": best["crossing"], "s": float(np.interp(best["crossing"], traj["time"], traj["s"])), "lane": .5},
                            {"time": float(best["z"][-3]), "s": float(best["z"][-2]), "lane": 1.5}] if valid_long else [],
            "gaps": [gap_names(best["gap1"]), gap_names(best["gap2"])], "metrics": metrics}


def rollout(speed, controls, dt=.02):
    time = np.linspace(0, HORIZON, round(HORIZON/dt) + 1)
    P, V = dynamics_map(time)
    ax, ay = controls[:N], controls[N:2*N]
    s, n = speed * time + P @ ax, P @ ay
    vs, vn = speed + V @ ax, V @ ay
    return {"time": time, "s": s, "n": n, "xy": np.column_stack((s, n)),
            "speed": vs, "lateral_speed": vn, "heading": np.arctan2(vn, vs)}
