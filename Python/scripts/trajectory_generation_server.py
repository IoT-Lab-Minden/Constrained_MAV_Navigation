"""Small HTTP server to generate minimum-snap trajectories from JSON.

Endpoints
- POST /generate
  - Accepts JSON with keys:
    - waypoints: list of objects {time: float, position: [x,y,z], velocity?, acceleration?, jerk?, snap?}
    - degree: int (polynomial degree)
    - idx_minimized_orders: optional list of derivative orders to minimize (default [4])
    - num_continuous_orders: optional int (default 3)
    - algorithm: optional string 'closed-form' or 'constrained' (default 'closed-form')
    - optimize_options: optional dict forwarded to solver
  - Returns JSON with keys: time_reference, durations, coefficients (nested lists)
"""

import numpy as np
from flask import Flask, request, jsonify
from . import minimum_snap_generation as qpgen

app = Flask(__name__)

def format_trajectory_solution(
    sol_pos,
    sol_yaw,
    segment_times,
    poly_order=7,
):
    """
    Converts the OSQP solutions into a structured format:

    Output:
        [
            {
                "segment_time": T_i,
                "position_coeffs": {
                    "x": [...],
                    "y": [...],
                    "z": [...]
                },
                "yaw_coeffs": [...]
            },
            ...
        ]
    """
    import numpy as np

    sol_pos = np.asarray(sol_pos, dtype=float)
    sol_yaw = np.asarray(sol_yaw, dtype=float)
    segment_times = np.asarray(segment_times, dtype=float)

    n_seg = len(segment_times)
    n_coeff = poly_order + 1
    n_vars_1d = n_seg * n_coeff

    # --- Split 3D solution ---
    x_sol = sol_pos[0:n_vars_1d]
    y_sol = sol_pos[n_vars_1d:2 * n_vars_1d]
    z_sol = sol_pos[2 * n_vars_1d:3 * n_vars_1d]

    trajectory = []

    for seg_idx in range(n_seg):
        start = seg_idx * n_coeff
        end = (seg_idx + 1) * n_coeff

        segment_dict = {
            "segment_time": float(segment_times[seg_idx]),
            "position_coeffs": {
                "x": x_sol[start:end].tolist(),
                "y": y_sol[start:end].tolist(),
                "z": z_sol[start:end].tolist(),
            },
            "yaw_coeffs": sol_yaw[start:end].tolist(),
        }

        trajectory.append(segment_dict)

    return trajectory

@app.route("/generate", methods=["POST"])
def generate():
    try:
        data = request.get_json(force=True)
    except Exception as exc:
        return jsonify({"error": f"Invalid JSON: {exc}"}), 400

    if data is None:
        return jsonify({"error": "Missing JSON body"}), 400

    try:
        waypoints_data = data["waypoints"]
    except KeyError as ke:
        return jsonify({"error": f"Missing required field: {ke}"}), 400

    # Expect a list of dicts with keys 'time' and 'position'
    if not isinstance(waypoints_data, list):
        return jsonify({"error": "'waypoints' must be a list"}), 400
    waypoints = []
    for i, wp in enumerate(waypoints_data):
        if not isinstance(wp, dict):
            return jsonify({"error": f"waypoint {i} is not an object"}), 400
        if "time" not in wp or "position" not in wp:
            return jsonify({"error": f"waypoint {i} missing 'time' or 'position'"}), 400
        waypoints.append(wp)

    try:
        # Build arrays expected by the QP generator from plain dict waypoints
        positions = [wp["position"] for wp in waypoints]
        yaws = [float(wp.get("yaw", 0.0)) for wp in waypoints]
        times = [float(wp["time"]) for wp in waypoints]

        positions = [list(map(float, np.asarray(p))) for p in positions]
        yaws = [float(v) for v in yaws]
        times = np.asarray(times, dtype=float)
        seg_times = list(np.diff(times))

        corridor_widths = [float(wp.get("corridor_width", 0.5)) for wp in waypoints[1:]]
        max_speeds = [float(wp.get("max_speed", 0.5)) for wp in waypoints[1:]]

        # Call the QP-based generator
        res = qpgen.generate_trajectory(positions, yaws, seg_times, corridor_widths, max_speeds)
        if res is None:
            return jsonify({"error": "Generator returned no result"}), 500

        sol_pos, sol_yaw, best_total_time, best_T = res

        traj = format_trajectory_solution(
            sol_pos,
            sol_yaw,
            best_T,
            poly_order=7,
        )

        print("segment times:", best_T)
        for i, seg in enumerate(traj):
            print(f"\nSegment {i}")
            print("T:", seg["segment_time"])
            print("x coeffs:", seg["position_coeffs"]["x"])
            print("y coeffs:", seg["position_coeffs"]["y"])
            print("z coeffs:", seg["position_coeffs"]["z"])
            print("yaw coeffs:", seg["yaw_coeffs"])

        return jsonify(traj), 200
    except Exception as exc:
        print(exc)
        return jsonify({"error": f"QP trajectory generation failed: {exc}"}), 500

if __name__ == "__main__":
    # Run development server
    app.run(host="0.0.0.0", port=5000, debug=True)
