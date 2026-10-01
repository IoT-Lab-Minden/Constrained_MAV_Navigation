import numpy as np
import scipy.sparse as sp
import math
import osqp

# ------------------------------------------------------------
# Basis functions
# ------------------------------------------------------------
def derivative_row(poly_order: int, deriv_order: int, t: float) -> np.ndarray:
    row = np.zeros(poly_order + 1)
    for i in range(deriv_order, poly_order + 1):
        row[i] = math.factorial(i) / math.factorial(i - deriv_order) * (t ** (i - deriv_order))
    return row


def compute_H_segment(T, n=7, k=4):
    """
    Builds H_j for
        integral_0^T (d^k p(t)/dt^k)^2 dt
    """
    H = np.zeros((n + 1, n + 1))
    for p in range(k, n + 1):
        for q in range(k, n + 1):
            H[p, q] = (
                math.factorial(p) / math.factorial(p - k)
                * math.factorial(q) / math.factorial(q - k)
                * T ** (p + q - 2 * k + 1)
                / (p + q - 2 * k + 1)
            )
    return H


def compute_H_all(segment_times, n=7, k=4):
    blocks = [compute_H_segment(T, n=n, k=k) for T in segment_times]
    return sp.block_diag(blocks, format="csc")


# ------------------------------------------------------------
# 1D equality constraints A c = b
# ------------------------------------------------------------
def build_A_b_from_waypoints(
    waypoints,
    segment_times,
    poly_order=7,
    continuity_order=3,
    start_vel=0.0,
    start_acc=0.0,
    end_vel=0.0,
    end_acc=0.0,
):
    """
    1D equality constraints for a piecewise polynomial trajectory.
    Local segment time tau in [0, T_j].
    """
    waypoints = np.asarray(waypoints, dtype=float)
    segment_times = np.asarray(segment_times, dtype=float)

    n_seg = len(segment_times)
    if len(waypoints) != n_seg + 1:
        raise ValueError("The number of waypoints must be one greater than the number of segments.")

    n_coeff = poly_order + 1
    n_vars = n_seg * n_coeff

    rows = []
    rhs = []

    def place_block(seg_idx: int, local_row: np.ndarray) -> np.ndarray:
        row = np.zeros(n_vars)
        start = seg_idx * n_coeff
        row[start:start + n_coeff] = local_row
        return row

    # Waypoints
    rows.append(place_block(0, derivative_row(poly_order, 0, 0.0)))
    rhs.append(waypoints[0])

    for j in range(1, n_seg):
        T_prev = segment_times[j - 1]

        rows.append(place_block(j - 1, derivative_row(poly_order, 0, T_prev)))
        rhs.append(waypoints[j])

        rows.append(place_block(j, derivative_row(poly_order, 0, 0.0)))
        rhs.append(waypoints[j])

    rows.append(place_block(n_seg - 1, derivative_row(poly_order, 0, segment_times[-1])))
    rhs.append(waypoints[-1])

    # Initial boundary conditions
    rows.append(place_block(0, derivative_row(poly_order, 1, 0.0)))
    rhs.append(start_vel)

    rows.append(place_block(0, derivative_row(poly_order, 2, 0.0)))
    rhs.append(start_acc)

    # Final boundary conditions
    rows.append(place_block(n_seg - 1, derivative_row(poly_order, 1, segment_times[-1])))
    rhs.append(end_vel)

    rows.append(place_block(n_seg - 1, derivative_row(poly_order, 2, segment_times[-1])))
    rhs.append(end_acc)

    # Continuity through continuity_order
    for j in range(n_seg - 1):
        Tj = segment_times[j]
        for r in range(continuity_order + 1):
            row = np.zeros(n_vars)
            n_coeff = poly_order + 1
            row[j * n_coeff:(j + 1) * n_coeff] = derivative_row(poly_order, r, Tj)
            row[(j + 1) * n_coeff:(j + 2) * n_coeff] = -derivative_row(poly_order, r, 0.0)
            rows.append(row)
            rhs.append(0.0)

    A = sp.csc_matrix(np.vstack(rows))
    b = np.asarray(rhs, dtype=float)
    return A, b


# ------------------------------------------------------------
# Stack the 3D equality constraints
# ------------------------------------------------------------
def stack_equalities_3d(A_x, b_x, A_y, b_y, A_z, b_z):
    A_eq = sp.block_diag((A_x, A_y, A_z), format="csc")
    b_eq = np.concatenate([b_x, b_y, b_z])
    return A_eq, b_eq


# ------------------------------------------------------------
# 3D corridor constraints
# ------------------------------------------------------------
def build_corridor_constraints_3d(
    waypoints_xyz,
    segment_times,
    poly_order=7,
    corridor_widths=None,
    n_corridor_samples=8,
):
    """
    Builds 3D corridor constraints for
        c_pos = [c_x; c_y; c_z]

    Returns:
        A_corr, l_corr, u_corr
    with
        l_corr <= A_corr @ c_pos <= u_corr
    """
    waypoints_xyz = np.asarray(waypoints_xyz, dtype=float)
    segment_times = np.asarray(segment_times, dtype=float)

    n_seg = len(segment_times)
    n_coeff = poly_order + 1
    n_vars_1d = n_seg * n_coeff
    n_vars = 3 * n_vars_1d

    if waypoints_xyz.shape != (n_seg + 1, 3):
        raise ValueError("waypoints_xyz must have shape (n_seg + 1, 3).")

    if corridor_widths is None:
        corridor_widths = np.full(n_seg, np.inf)
    corridor_widths = np.asarray(corridor_widths, dtype=float)

    rows = []
    lower = []
    upper = []

    for seg_idx in range(n_seg):
        delta = corridor_widths[seg_idx]
        if not np.isfinite(delta):
            continue

        r_i = waypoints_xyz[seg_idx]
        r_ip1 = waypoints_xyz[seg_idx + 1]
        diff = r_ip1 - r_i
        norm = np.linalg.norm(diff)

        if norm <= 1e-12:
            raise ValueError(f"Segment {seg_idx} has nearly identical waypoints.")

        # t_i from the paper
        t_hat = diff / norm

        # Projector onto the direction orthogonal to the segment line
        M = np.eye(3) - np.outer(t_hat, t_hat)

        T = segment_times[seg_idx]

        for j in range(1, n_corridor_samples + 1):
            tau = j / (n_corridor_samples + 1) * T
            phi = derivative_row(poly_order, 0, tau)

            for k in range(3):
                row = np.zeros(n_vars)

                # x block
                row[seg_idx * n_coeff:(seg_idx + 1) * n_coeff] = M[k, 0] * phi

                # y block
                offset_y = n_vars_1d
                row[offset_y + seg_idx * n_coeff:offset_y + (seg_idx + 1) * n_coeff] = M[k, 1] * phi

                # z block
                offset_z = 2 * n_vars_1d
                row[offset_z + seg_idx * n_coeff:offset_z + (seg_idx + 1) * n_coeff] = M[k, 2] * phi

                const = M[k, :] @ r_i

                # -delta <= row @ c_pos - const <= delta
                rows.append(row)
                lower.append(-delta + const)
                upper.append(delta + const)

    if len(rows) == 0:
        A_corr = sp.csc_matrix((0, n_vars))
        l_corr = np.zeros(0)
        u_corr = np.zeros(0)
    else:
        A_corr = sp.csc_matrix(np.vstack(rows))
        l_corr = np.asarray(lower, dtype=float)
        u_corr = np.asarray(upper, dtype=float)

    return A_corr, l_corr, u_corr


def build_velocity_constraints_3d_segmentwise(
    segment_times,
    poly_order=7,
    v_max_per_segment=None,
    n_velocity_samples=8,
):
    """
    Builds segment-wise velocity constraints:

        -v_i <= x_dot(t), y_dot(t), z_dot(t) <= v_i

    for segment i.

    Parameters
    ---------
    v_max_per_segment : array-like, length = n_seg
        Maximum speed per segment

    Returns
    --------
    A_vel, l_vel, u_vel
    """
    segment_times = np.asarray(segment_times, dtype=float)
    n_seg = len(segment_times)

    if v_max_per_segment is None:
        v_max_per_segment = np.full(n_seg, np.inf)

    v_max_per_segment = np.asarray(v_max_per_segment, dtype=float)
    if v_max_per_segment.shape != (n_seg,):
        raise ValueError("v_max_per_segment must have length n_seg.")

    n_coeff = poly_order + 1
    n_vars_1d = n_seg * n_coeff
    n_vars = 3 * n_vars_1d

    rows = []
    lower = []
    upper = []

    for seg_idx in range(n_seg):
        v_max = v_max_per_segment[seg_idx]

        if not np.isfinite(v_max):
            continue

        T = segment_times[seg_idx]

        for j in range(1, n_velocity_samples + 1):
            tau = j / (n_velocity_samples + 1) * T
            dphi = derivative_row(poly_order, deriv_order=1, t=tau)

            # x_dot
            row = np.zeros(n_vars)
            row[seg_idx * n_coeff:(seg_idx + 1) * n_coeff] = dphi
            rows.append(row)
            lower.append(-v_max)
            upper.append(v_max)

            # y_dot
            row = np.zeros(n_vars)
            offset_y = n_vars_1d
            row[offset_y + seg_idx * n_coeff:offset_y + (seg_idx + 1) * n_coeff] = dphi
            rows.append(row)
            lower.append(-v_max)
            upper.append(v_max)

            # z_dot
            row = np.zeros(n_vars)
            offset_z = 2 * n_vars_1d
            row[offset_z + seg_idx * n_coeff:offset_z + (seg_idx + 1) * n_coeff] = dphi
            rows.append(row)
            lower.append(-v_max)
            upper.append(v_max)

    if len(rows) == 0:
        A_vel = sp.csc_matrix((0, n_vars))
        l_vel = np.zeros(0)
        u_vel = np.zeros(0)
    else:
        A_vel = sp.csc_matrix(np.vstack(rows))
        l_vel = np.asarray(lower, dtype=float)
        u_vel = np.asarray(upper, dtype=float)

    return A_vel, l_vel, u_vel


# ------------------------------------------------------------
# QP for 3D position
# ------------------------------------------------------------
def build_qp_position_3d(
    waypoints_xyz,
    segment_times,
    corridor_widths=None,
    poly_order=7,
    continuity_order=3,
    start_vel_xyz=(0.0, 0.0, 0.0),
    start_acc_xyz=(0.0, 0.0, 0.0),
    end_vel_xyz=(0.0, 0.0, 0.0),
    end_acc_xyz=(0.0, 0.0, 0.0),
    n_corridor_samples=8,
    v_max_per_segment=None,
    n_velocity_samples=8,
):
    """
    Builds the 3D position QP:
        min 1/2 c_pos^T P_pos c_pos + q_pos^T c_pos
        s.t. l_pos <= A_pos c_pos <= u_pos

    with c_pos = [c_x; c_y; c_z].
    """
    waypoints_xyz = np.asarray(waypoints_xyz, dtype=float)
    if waypoints_xyz.ndim != 2 or waypoints_xyz.shape[1] != 3:
        raise ValueError("waypoints_xyz must have shape (n_waypoints, 3).")

    x_waypoints = waypoints_xyz[:, 0]
    y_waypoints = waypoints_xyz[:, 1]
    z_waypoints = waypoints_xyz[:, 2]

    # H for one axis, k = 4 (snap)
    H_1d = compute_H_all(segment_times, n=poly_order, k=4)

    # OSQP form: 1/2 x^T P x  -> P = 2H
    P_pos = sp.block_diag((2 * H_1d, 2 * H_1d, 2 * H_1d), format="csc")
    q_pos = np.zeros(P_pos.shape[0])

    A_x, b_x = build_A_b_from_waypoints(
        x_waypoints, segment_times, poly_order, continuity_order,
        start_vel_xyz[0], start_acc_xyz[0], end_vel_xyz[0], end_acc_xyz[0]
    )
    A_y, b_y = build_A_b_from_waypoints(
        y_waypoints, segment_times, poly_order, continuity_order,
        start_vel_xyz[1], start_acc_xyz[1], end_vel_xyz[1], end_acc_xyz[1]
    )
    A_z, b_z = build_A_b_from_waypoints(
        z_waypoints, segment_times, poly_order, continuity_order,
        start_vel_xyz[2], start_acc_xyz[2], end_vel_xyz[2], end_acc_xyz[2]
    )

    A_eq, b_eq = stack_equalities_3d(A_x, b_x, A_y, b_y, A_z, b_z)

    A_corr, l_corr, u_corr = build_corridor_constraints_3d(
        waypoints_xyz=waypoints_xyz,
        segment_times=segment_times,
        poly_order=poly_order,
        corridor_widths=corridor_widths,
        n_corridor_samples=n_corridor_samples,
    )

    A_vel, l_vel, u_vel = build_velocity_constraints_3d_segmentwise(
        segment_times=segment_times,
        poly_order=poly_order,
        v_max_per_segment=v_max_per_segment,
        n_velocity_samples=n_velocity_samples,
    )

    A_pos = sp.vstack([A_eq, A_corr, A_vel], format="csc")
    l_pos = np.concatenate([b_eq, l_corr, l_vel])
    u_pos = np.concatenate([b_eq, u_corr, u_vel])

    return P_pos, q_pos, A_pos, l_pos, u_pos


def build_yaw_constraints(
    yaws,
    segment_times,
    poly_order=7,
    n_settle_samples=1,
    settle_start_ratio=0.15,
    settle_end_ratio=0.20,
):
    """
    Builds equality constraints A_yaw c_yaw = b_yaw for a yaw trajectory
    that reaches the next yaw as early as possible after each waypoint.

    For each segment i:
      psi_i(0)       = yaws[i]
      psi_i(delta_i) = yaws[i+1]

    Optionally, also:
      psi_last(T_last) = yaws[-1]

    Parameters
    ---------
    yaws : array-like, length n_seg + 1
        Yaw values at the waypoints.
    segment_times : array-like, length n_seg
        Segment durations.
    poly_order : int
        Polynomial degree.
    n_settle_samples : int
        Number of intermediate equality constraints per segment set to yaws[i+1].
    settle_start_ratio, settle_end_ratio : float
        Range within the segment where these intermediate points are placed.
        Example: 0.10 to 0.20 means between 10% and 20% of the segment duration.

    Returns
    --------
    A_yaw : scipy.sparse.csc_matrix
    b_yaw : np.ndarray
    """
    yaws = np.asarray(yaws, dtype=float)
    segment_times = np.asarray(segment_times, dtype=float)

    n_seg = len(segment_times)
    if len(yaws) != n_seg + 1:
        raise ValueError("The number of yaw values must be one greater than the number of segments.")

    if n_settle_samples < 1:
        raise ValueError("n_settle_samples must be at least 1.")

    if not (0.0 <= settle_start_ratio <= 1.0 and 0.0 <= settle_end_ratio <= 1.0):
        raise ValueError("settle_start_ratio and settle_end_ratio must be between 0 and 1.")

    if settle_start_ratio > settle_end_ratio:
        raise ValueError("settle_start_ratio must not exceed settle_end_ratio.")

    n_coeff = poly_order + 1
    n_vars = n_seg * n_coeff

    rows = []
    rhs = []

    def place_block(seg_idx: int, local_row: np.ndarray) -> np.ndarray:
        row = np.zeros(n_vars)
        start = seg_idx * n_coeff
        row[start:start + n_coeff] = local_row
        return row
    
    early_ratio=0.2
    early_time_cap = 0.5  # Optional: maximum duration of the early transition (e.g., 0.5 s)

    for seg_idx in range(n_seg):
        T = float(segment_times[seg_idx])
        delta = early_ratio * T
        if early_time_cap is not None:
            delta = min(delta, float(early_time_cap))
        delta = max(0.0, min(delta, T))

        rows.append(place_block(seg_idx, derivative_row(poly_order, 0, 0.0)))
        rhs.append(yaws[seg_idx])

        rows.append(place_block(seg_idx, derivative_row(poly_order, 0, delta)))
        rhs.append(yaws[seg_idx + 1])

    A_yaw = sp.csc_matrix(np.vstack(rows))
    b_yaw = np.asarray(rhs, dtype=float)
    return A_yaw, b_yaw


def build_yaw_settle_cost(
    yaws,
    segment_times,
    poly_order=7,
    n_settle_samples=3,
    settle_start_ratio=0.10,
    settle_end_ratio=0.20,
    settle_weight=100.0,
    unwrap_yaw=True,
):
    """
    Builds an additional quadratic cost term for yaw:

        sum_{segments i} sum_j settle_weight * (psi_i(tau_ij) - yaws[i+1])^2

    Returns:
        P_settle, q_settle

    These terms contribute to the OSQP objective as
        1/2 c^T P_settle c + q_settle^T c.
    """
    yaws = np.asarray(yaws, dtype=float)
    if unwrap_yaw:
        yaws = np.unwrap(yaws)

    segment_times = np.asarray(segment_times, dtype=float)

    n_seg = len(segment_times)
    if len(yaws) != n_seg + 1:
        raise ValueError("The number of yaw values must be one greater than the number of segments.")

    if n_settle_samples < 1:
        raise ValueError("n_settle_samples must be at least 1.")

    if not (0.0 <= settle_start_ratio <= 1.0 and 0.0 <= settle_end_ratio <= 1.0):
        raise ValueError("settle_start_ratio and settle_end_ratio must be between 0 and 1.")

    if settle_start_ratio > settle_end_ratio:
        raise ValueError("settle_start_ratio must not exceed settle_end_ratio.")

    n_coeff = poly_order + 1
    n_vars = n_seg * n_coeff

    P = sp.lil_matrix((n_vars, n_vars))
    q = np.zeros(n_vars)

    for seg_idx in range(n_seg):
        T = float(segment_times[seg_idx])

        if n_settle_samples == 1:
            ratios = np.array([(settle_start_ratio + settle_end_ratio) * 0.5], dtype=float)
        else:
            ratios = np.linspace(settle_start_ratio, settle_end_ratio, n_settle_samples)

        psi_target = yaws[seg_idx + 1]
        offset = seg_idx * n_coeff

        for r in ratios:
            tau = r * T
            phi = derivative_row(poly_order, 0, tau)  # Evaluation of psi(tau)

            # Cost:
            # w * (phi^T c_seg - psi_target)^2
            # = w * (c^T phi phi^T c - 2 psi_target phi^T c + psi_target^2)
            #
            # In OSQP form:
            # 1/2 c^T P c + q^T c
            # => P += 2 w phi phi^T
            # => q += -2 w psi_target phi

            P_block = 2.0 * settle_weight * np.outer(phi, phi)
            q_block = -2.0 * settle_weight * psi_target * phi

            P[offset:offset + n_coeff, offset:offset + n_coeff] += P_block
            q[offset:offset + n_coeff] += q_block

    return P.tocsc(), q


# ------------------------------------------------------------
# Separate QP for yaw
# ------------------------------------------------------------
def build_qp_yaw(
    yaws,
    segment_times,
    poly_order=7,
    continuity_order=1,
    start_yaw_rate=0.0,
    start_yaw_acc=0.0,
    end_yaw_rate=0.0,
    end_yaw_acc=0.0,
):
    """
    Builds the 1D QP for yaw:
        min 1/2 c_psi^T P_psi c_psi + q_psi^T c_psi
        s.t. A_psi c_psi = b_psi

    Following the paper, the second derivative of yaw is minimized, so k_psi = 2.
    """
    # H for yaw, k_psi = 2
    H_yaw = compute_H_all(segment_times, n=poly_order, k=2)
    P_yaw = 2 * H_yaw
    q_yaw = np.zeros(P_yaw.shape[0])

    # Soft settling cost
    P_settle, q_settle = build_yaw_settle_cost(
        yaws=yaws,
        segment_times=segment_times,
        poly_order=poly_order,
        n_settle_samples=3,
        settle_start_ratio=0.1,
        settle_end_ratio=0.2,
        settle_weight=100,
        unwrap_yaw=True,
    )

    P_yaw = P_yaw + P_settle
    q_yaw = q_yaw + q_settle
    
    regularization=1e-8

    if regularization is not None and regularization > 0.0:
        P_yaw = P_yaw + regularization * sp.eye(P_yaw.shape[0], format="csc")

    # Continuity through the first derivative is usually sufficient for yaw.
    # build_A_b_from_waypoints also constrains the initial and final
    # second derivatives, if that is desired.
    A_yaw_points, b_yaw_points = build_A_b_from_waypoints(
        waypoints=yaws,
        segment_times=segment_times,
        poly_order=poly_order,
        continuity_order=continuity_order,
        start_vel=start_yaw_rate,
        start_acc=start_yaw_acc,
        end_vel=end_yaw_rate,
        end_acc=end_yaw_acc,
    )

    l_yaw_points = b_yaw_points.copy()
    u_yaw_points = b_yaw_points.copy()

    return P_yaw, q_yaw, A_yaw_points, l_yaw_points, u_yaw_points

# ------------------------------------------------------------
# Combined QP
# ------------------------------------------------------------
def build_qp(
    waypoints_xyz,
    yaws,
    segment_times,
    corridor_widths=None,
    v_max_per_segment=None,
    poly_order=7,
    n_corridor_samples=8,
    n_velocity_samples=8,
    pos_continuity_order=3,
    yaw_continuity_order=1,
    start_vel_xyz=(0.0, 0.0, 0.0),
    start_acc_xyz=(0.0, 0.0, 0.0),
    end_vel_xyz=(0.0, 0.0, 0.0),
    end_acc_xyz=(0.0, 0.0, 0.0),
    start_yaw_rate=0.0,
    start_yaw_acc=0.0,
    end_yaw_rate=0.0,
    end_yaw_acc=0.0,
):
    """
    Builds two QPs:
      - 3D position
      - yaw separately

    Returns:
      {
        "position": (P_pos, q_pos, A_pos, l_pos, u_pos),
        "yaw":      (P_yaw, q_yaw, A_yaw, l_yaw, u_yaw),
      }
    """
    P_pos, q_pos, A_pos, l_pos, u_pos = build_qp_position_3d(
        waypoints_xyz=waypoints_xyz,
        segment_times=segment_times,
        corridor_widths=corridor_widths,
        poly_order=poly_order,
        continuity_order=pos_continuity_order,
        start_vel_xyz=start_vel_xyz,
        start_acc_xyz=start_acc_xyz,
        end_vel_xyz=end_vel_xyz,
        end_acc_xyz=end_acc_xyz,
        n_corridor_samples=n_corridor_samples,
        v_max_per_segment=v_max_per_segment,
        n_velocity_samples=n_velocity_samples,
    )

    P_yaw, q_yaw, A_yaw, l_yaw, u_yaw = build_qp_yaw(
        yaws=yaws,
        segment_times=segment_times,
        poly_order=poly_order,
        continuity_order=yaw_continuity_order,
        start_yaw_rate=start_yaw_rate,
        start_yaw_acc=start_yaw_acc,
        end_yaw_rate=end_yaw_rate,
        end_yaw_acc=end_yaw_acc,
    )

    return {
        "position": (P_pos, q_pos, A_pos, l_pos, u_pos),
        "yaw": (P_yaw, q_yaw, A_yaw, l_yaw, u_yaw),
    }

def solve_osqp_qp(P, q, A, l, u, verbose=False, eps_abs=1e-6, eps_rel=1e-6, max_iter=100000):
    """
    Solves a QP of the form

        min  1/2 x^T P x + q^T x
        s.t. l <= A x <= u

    using OSQP.

    Parameters
    ---------
    P, A : scipy.sparse matrices
    q, l, u : np.ndarray

    Returns
    --------
    x : np.ndarray
        Optimal solution
    res : osqp.Result
        Complete OSQP result object
    """
    P = sp.csc_matrix(P)
    A = sp.csc_matrix(A)
    q = np.asarray(q, dtype=float)
    l = np.asarray(l, dtype=float)
    u = np.asarray(u, dtype=float)

    prob = osqp.OSQP()
    prob.setup(
        P=P,
        q=q,
        A=A,
        l=l,
        u=u,
        verbose=verbose,
        eps_abs=eps_abs,
        eps_rel=eps_rel,
        max_iter=max_iter,
    )

    res = prob.solve()

    if res.info.status not in ("solved", "solved inaccurate"):
        raise RuntimeError(f"OSQP could not solve the problem. Status: {res.info.status}")

    return res.x, res


def evaluate_polynomial_segment(coeffs, t, derivative_order=0):
    """
    Evaluates a polynomial segment or one of its derivatives at local time t.

    p(t) = c0 + c1 t + c2 t^2 + ...

    Parameters
    ---------
    coeffs : array-like
        Coefficients of a segment
    t : float
        Local segment time
    derivative_order : int
        0 -> polynomial
        1 -> first derivative
        2 -> second derivative
        ...

    Returns
    --------
    value : float
    """
    coeffs = np.asarray(coeffs, dtype=float)
    n = len(coeffs) - 1

    value = 0.0
    for i in range(derivative_order, n + 1):
        value += (
            math.factorial(i) / math.factorial(i - derivative_order)
            * coeffs[i]
            * (t ** (i - derivative_order))
        )
    return value


def evaluate_piecewise_polynomial_at_time(solution, segment_times, poly_order=7, t=0.0, derivative_order=0):
    """
    Evaluates a 1D piecewise polynomial trajectory at global time t.

    The 'solution' array contains the coefficients of all segments in sequence:
        [seg0_coeffs, seg1_coeffs, ..., segM_coeffs]

    Parameters
    ---------
    solution : np.ndarray
        OSQP solution for a 1D trajectory
    segment_times : list/array
        Segmentdauern
    poly_order : int
        Polynomial degree
    t : float
        Global time
    derivative_order : int
        0=position, 1=velocity, 2=acceleration, ...

    Returns
    --------
    value : float
    seg_idx : int
        Index of the active segment
    tau : float
        Local time within the segment
    """
    solution = np.asarray(solution, dtype=float)
    segment_times = np.asarray(segment_times, dtype=float)

    n_seg = len(segment_times)
    n_coeff = poly_order + 1

    total_time = np.sum(segment_times)

    # Clamp time to the valid range
    t = float(np.clip(t, 0.0, total_time))

    cumulative = np.cumsum(segment_times)

    seg_idx = 0
    while seg_idx < n_seg - 1 and t > cumulative[seg_idx]:
        seg_idx += 1

    t_start = 0.0 if seg_idx == 0 else cumulative[seg_idx - 1]
    tau = t - t_start

    coeffs = solution[seg_idx * n_coeff:(seg_idx + 1) * n_coeff]
    value = evaluate_polynomial_segment(coeffs, tau, derivative_order=derivative_order)

    return value, seg_idx, tau


def sample_piecewise_polynomial(solution, segment_times, poly_order=7, n_samples_per_segment=50, derivative_order=0):
    """
    Samples a 1D piecewise polynomial trajectory on a time grid.

    Returns
    --------
    t_grid : np.ndarray
        Global times
    values : np.ndarray
        Evaluated values
    segment_ids : np.ndarray
        Segment index for each sample
    """
    solution = np.asarray(solution, dtype=float)
    segment_times = np.asarray(segment_times, dtype=float)

    n_seg = len(segment_times)
    n_coeff = poly_order + 1

    times = []
    values = []
    segment_ids = []

    t_global_start = 0.0

    for seg_idx in range(n_seg):
        T = segment_times[seg_idx]
        coeffs = solution[seg_idx * n_coeff:(seg_idx + 1) * n_coeff]

        # Include the endpoint only in the last segment to avoid duplicates at boundaries
        if seg_idx < n_seg - 1:
            tau_grid = np.linspace(0.0, T, n_samples_per_segment, endpoint=False)
        else:
            tau_grid = np.linspace(0.0, T, n_samples_per_segment, endpoint=True)

        for tau in tau_grid:
            times.append(t_global_start + tau)
            values.append(evaluate_polynomial_segment(coeffs, tau, derivative_order=derivative_order))
            segment_ids.append(seg_idx)

        t_global_start += T

    return np.asarray(times), np.asarray(values), np.asarray(segment_ids)

def qp_objective_value(P, q, x):
    """
    Computes the OSQP objective value
        1/2 x^T P x + q^T x
    for a given solution x.
    """
    x = np.asarray(x, dtype=float)
    return 0.5 * x @ (P @ x) + q @ x

def trajectory_cost_for_segment_times(
    waypoints_xyz,
    yaws,
    segment_times,
    corridor_widths=None,
    poly_order=7,
    n_corridor_samples=8,
    pos_continuity_order=3,
    yaw_continuity_order=1,
    start_vel_xyz=(0.0, 0.0, 0.0),
    start_acc_xyz=(0.0, 0.0, 0.0),
    end_vel_xyz=(0.0, 0.0, 0.0),
    end_acc_xyz=(0.0, 0.0, 0.0),
    start_yaw_rate=0.0,
    start_yaw_acc=0.0,
    end_yaw_rate=0.0,
    end_yaw_acc=0.0,
    yaw_weight=1.0,
    solve_yaw=True,
    v_max_per_segment=None,
    n_velocity_samples=8,
    infeasible_cost=1e12,
    osqp_verbose=False,
):
    try:
        qp = build_qp(
            waypoints_xyz=waypoints_xyz,
            yaws=yaws,
            segment_times=segment_times,
            corridor_widths=corridor_widths,
            poly_order=poly_order,
            n_corridor_samples=n_corridor_samples,
            pos_continuity_order=pos_continuity_order,
            yaw_continuity_order=yaw_continuity_order,
            start_vel_xyz=start_vel_xyz,
            start_acc_xyz=start_acc_xyz,
            end_vel_xyz=end_vel_xyz,
            end_acc_xyz=end_acc_xyz,
            start_yaw_rate=start_yaw_rate,
            start_yaw_acc=start_yaw_acc,
            end_yaw_rate=end_yaw_rate,
            end_yaw_acc=end_yaw_acc,
            v_max_per_segment=v_max_per_segment,
            n_velocity_samples=n_velocity_samples,
        )

        P_pos, q_pos, A_pos, l_pos, u_pos = qp["position"]
        x_pos, res_pos = solve_osqp_qp(P_pos, q_pos, A_pos, l_pos, u_pos, verbose=osqp_verbose)
        J_pos = qp_objective_value(P_pos, q_pos, x_pos)

        total_cost = J_pos
        info = {
            "position_solution": x_pos,
            "position_result": res_pos,
            "position_cost": J_pos,
            "feasible": True,
        }

        if solve_yaw:
            P_yaw, q_yaw, A_yaw, l_yaw, u_yaw = qp["yaw"]
            x_yaw, res_yaw = solve_osqp_qp(P_yaw, q_yaw, A_yaw, l_yaw, u_yaw, verbose=osqp_verbose)
            J_yaw = qp_objective_value(P_yaw, q_yaw, x_yaw)
            total_cost += yaw_weight * J_yaw
            info.update({
                "yaw_solution": x_yaw,
                "yaw_result": res_yaw,
                "yaw_cost": J_yaw,
            })

        info["total_cost"] = total_cost
        return total_cost, info

    except Exception as e:
        return infeasible_cost, {
            "feasible": False,
            "total_cost": infeasible_cost,
            "error": str(e),
        }

def build_time_descent_directions(n_seg):
    """
    Builds direction vectors g_i as described in the paper:
    g_i[i] = 1
    all others = -1 / (n_seg - 1)

    This keeps the sum of segment times constant.
    """
    if n_seg < 2:
        raise ValueError("At least 2 segments are required.")

    directions = []
    for i in range(n_seg):
        g = np.full(n_seg, -1.0 / (n_seg - 1))
        g[i] = 1.0
        directions.append(g)
    return directions

def optimize_segment_times(
    waypoints_xyz,
    yaws,
    total_time,
    initial_segment_times=None,
    corridor_widths=None,
    v_max_per_segment=None,
    poly_order=7,
    n_corridor_samples=8,
    n_velocity_samples=8,
    pos_continuity_order=3,
    yaw_continuity_order=1,
    start_vel_xyz=(0.0, 0.0, 0.0),
    start_acc_xyz=(0.0, 0.0, 0.0),
    end_vel_xyz=(0.0, 0.0, 0.0),
    end_acc_xyz=(0.0, 0.0, 0.0),
    start_yaw_rate=0.0,
    start_yaw_acc=0.0,
    end_yaw_rate=0.0,
    end_yaw_acc=0.0,
    yaw_weight=1.0,
    solve_yaw=True,
    finite_diff_step=1e-3,
    min_segment_time=1e-3,
    max_iterations=10,
    backtracking_beta=0.5,
    armijo_c=1e-4,
    osqp_verbose=False,
):
    """
    Optimizes segment times T subject to:
        sum(T) = total_time
        T_i >= min_segment_time

    Following the approach in Section V.C. of the paper:
    - numerical directional derivatives
    - gradient descent
    - backtracking line search

    Returns:
        best_T, history
    """
    waypoints_xyz = np.asarray(waypoints_xyz, dtype=float)
    n_seg = len(waypoints_xyz) - 1

    if initial_segment_times is None:
        # Reasonable initialization:
        # proportional to Euclidean distance
        diffs = waypoints_xyz[1:] - waypoints_xyz[:-1]
        lengths = np.linalg.norm(diffs, axis=1)
        if np.all(lengths < 1e-12):
            lengths = np.ones(n_seg)
        initial_segment_times = total_time * lengths / np.sum(lengths)

    T = np.asarray(initial_segment_times, dtype=float).copy()

    # Normalize to total_time
    T = total_time * T / np.sum(T)

    directions = build_time_descent_directions(n_seg)
    history = []

    def feasible(Tcand):
        return np.all(Tcand >= min_segment_time) and abs(np.sum(Tcand) - total_time) < 1e-8

    # Initial cost
    f_current, info_current = trajectory_cost_for_segment_times(
        waypoints_xyz=waypoints_xyz,
        yaws=yaws,
        segment_times=T,
        corridor_widths=corridor_widths,
        v_max_per_segment=v_max_per_segment,
        poly_order=poly_order,
        n_corridor_samples=n_corridor_samples,
        n_velocity_samples=n_velocity_samples,
        pos_continuity_order=pos_continuity_order,
        yaw_continuity_order=yaw_continuity_order,
        start_vel_xyz=start_vel_xyz,
        start_acc_xyz=start_acc_xyz,
        end_vel_xyz=end_vel_xyz,
        end_acc_xyz=end_acc_xyz,
        start_yaw_rate=start_yaw_rate,
        start_yaw_acc=start_yaw_acc,
        end_yaw_rate=end_yaw_rate,
        end_yaw_acc=end_yaw_acc,
        yaw_weight=yaw_weight,
        solve_yaw=solve_yaw,
        osqp_verbose=osqp_verbose,
    )

    history.append({
        "iteration": 0,
        "segment_times": T.copy(),
        "cost": f_current,
    })

    for it in range(1, max_iterations + 1):
        directional_derivatives = np.zeros(n_seg)

        # Numerical directional derivatives along g_i
        for i, g in enumerate(directions):
            T_trial = T + finite_diff_step * g

            # If negative, try a smaller h
            h = finite_diff_step
            while np.any(T_trial < min_segment_time):
                h *= 0.5
                if h < 1e-8:
                    break
                T_trial = T + h * g

            if np.any(T_trial < min_segment_time):
                directional_derivatives[i] = 0.0
                continue

            # Rescale the sum to total_time for numerical stability
            T_trial *= total_time / np.sum(T_trial)

            f_trial, _ = trajectory_cost_for_segment_times(
                waypoints_xyz=waypoints_xyz,
                yaws=yaws,
                segment_times=T_trial,
                corridor_widths=corridor_widths,
                v_max_per_segment=v_max_per_segment,
                poly_order=poly_order,
                n_corridor_samples=n_corridor_samples,
                n_velocity_samples=n_velocity_samples,
                pos_continuity_order=pos_continuity_order,
                yaw_continuity_order=yaw_continuity_order,
                start_vel_xyz=start_vel_xyz,
                start_acc_xyz=start_acc_xyz,
                end_vel_xyz=end_vel_xyz,
                end_acc_xyz=end_acc_xyz,
                start_yaw_rate=start_yaw_rate,
                start_yaw_acc=start_yaw_acc,
                end_yaw_rate=end_yaw_rate,
                end_yaw_acc=end_yaw_acc,
                yaw_weight=yaw_weight,
                solve_yaw=solve_yaw,
                osqp_verbose=False,
            )

            directional_derivatives[i] = (f_trial - f_current) / h

        # Combined descent direction
        # Combine g_i with the directional derivatives
        descent_direction = np.zeros(n_seg)
        for i, g in enumerate(directions):
            descent_direction -= directional_derivatives[i] * g

        # Stop if the direction is nearly zero
        norm_dir = np.linalg.norm(descent_direction)
        if norm_dir < 1e-10:
            break

        # Backtracking line search
        alpha = 1.0
        accepted = False

        while alpha > 1e-8:
            T_new = T + alpha * descent_direction

            if np.any(T_new < min_segment_time):
                alpha *= backtracking_beta
                continue

            # Normalize to total_time again
            T_new *= total_time / np.sum(T_new)

            if np.any(T_new < min_segment_time):
                alpha *= backtracking_beta
                continue

            f_new, info_new = trajectory_cost_for_segment_times(
                waypoints_xyz=waypoints_xyz,
                yaws=yaws,
                segment_times=T_new,
                corridor_widths=corridor_widths,
                v_max_per_segment=v_max_per_segment,
                poly_order=poly_order,
                n_corridor_samples=n_corridor_samples,
                n_velocity_samples=n_velocity_samples,
                pos_continuity_order=pos_continuity_order,
                yaw_continuity_order=yaw_continuity_order,
                start_vel_xyz=start_vel_xyz,
                start_acc_xyz=start_acc_xyz,
                end_vel_xyz=end_vel_xyz,
                end_acc_xyz=end_acc_xyz,
                start_yaw_rate=start_yaw_rate,
                start_yaw_acc=start_yaw_acc,
                end_yaw_rate=end_yaw_rate,
                end_yaw_acc=end_yaw_acc,
                yaw_weight=yaw_weight,
                solve_yaw=solve_yaw,
                osqp_verbose=False,
            )

            # Simple acceptance condition
            if f_new < f_current - armijo_c * alpha * (norm_dir ** 2):
                T = T_new
                f_current = f_new
                info_current = info_new
                accepted = True
                break

            alpha *= backtracking_beta

        history.append({
            "iteration": it,
            "segment_times": T.copy(),
            "cost": f_current,
            "step_size": alpha,
            "accepted": accepted,
            "directional_derivatives": directional_derivatives.copy(),
        })

        if not accepted:
            break

    return T, history, info_current

def find_feasible_total_time(
    waypoints_xyz,
    yaws,
    initial_total_time,
    corridor_widths=None,
    v_max_per_segment=None,
    max_trials=10,
    growth_factor=1.25,
    **kwargs,
):
    total_time = initial_total_time

    for _ in range(max_trials):
        n_seg = len(waypoints_xyz) - 1
        initial_segment_times = [total_time / n_seg] * n_seg

        best_T, history, info = optimize_segment_times(
            waypoints_xyz=waypoints_xyz,
            yaws=yaws,
            total_time=total_time,
            initial_segment_times=initial_segment_times,
            corridor_widths=corridor_widths,
            v_max_per_segment=v_max_per_segment,
            **kwargs,
        )

        if info.get("feasible", False):
            return total_time, best_T, history, info

        total_time *= growth_factor

    raise RuntimeError("No feasible total duration found.")

def split_solution_3d(solution_pos, segment_times, poly_order=7):
    """
    Splits the 3D position solution c_pos = [c_x; c_y; c_z]
    into the three 1D solutions.
    """
    solution_pos = np.asarray(solution_pos, dtype=float)
    n_seg = len(segment_times)
    n_coeff = poly_order + 1
    n_vars_1d = n_seg * n_coeff

    x_sol = solution_pos[0:n_vars_1d]
    y_sol = solution_pos[n_vars_1d:2 * n_vars_1d]
    z_sol = solution_pos[2 * n_vars_1d:3 * n_vars_1d]

    return x_sol, y_sol, z_sol

def generate_trajectory(waypoints, yaws, segment_times, corridor_widths, max_speed):
    best_total_time, best_T, history, info = find_feasible_total_time(
        waypoints_xyz=waypoints,
        yaws=yaws,
        initial_total_time=sum(segment_times),
        corridor_widths=corridor_widths,
        v_max_per_segment=max_speed,
        poly_order=7,
        n_corridor_samples=8,
        n_velocity_samples=10,
        max_iterations=7,
    )

    qp = build_qp(
        waypoints_xyz=waypoints,
        yaws=yaws,
        segment_times=best_T,
        corridor_widths=corridor_widths,
        v_max_per_segment=max_speed,
        poly_order=7,
        n_corridor_samples=8,
        n_velocity_samples=8,
    )

    P_pos, q_pos, A_pos, l_pos, u_pos = qp["position"]
    sol_pos, res_pos = solve_osqp_qp(P_pos, q_pos, A_pos, l_pos, u_pos, verbose=True)

    P_yaw, q_yaw, A_yaw, l_yaw, u_yaw = qp["yaw"]
    sol_yaw, res_yaw = solve_osqp_qp(P_yaw, q_yaw, A_yaw, l_yaw, u_yaw, verbose=True)

    print("P_pos:", P_pos.shape)
    print("A_pos:", A_pos.shape)
    print("sol_pos:", sol_pos.shape)

    return sol_pos, sol_yaw, best_total_time, best_T
