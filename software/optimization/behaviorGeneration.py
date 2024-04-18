import matplotlib.pyplot as plt
import numpy as np
from pydrake.all import MathematicalProgram, MixedIntegerBranchAndBound, SnoptSolver, IpoptSolver, NloptSolver, GurobiSolver
import time

def mppc(Hopper, PARKOUR, x_start, xg, goal_tolerance, N):
    t_start_setup = time.time()
    
    g = Hopper.gravity
    r = Hopper.leg.r_takeoff

    dx_hip_foot = Hopper.dx_hip_foot
    dz_hip_foot = Hopper.dz_hip_foot
    dx_foot_knee = Hopper.dx_foot_knee
    dz_foot_knee = Hopper.dz_foot_knee

    # ################################################### CREATE PARKOUR ####################################################
    obstacle_start = 0
    obstacle_end = 0
    res_start = 0
    res_end = 0
    z_start = 0
    for i in range(len(PARKOUR["position"])):
            if PARKOUR["position"][i] - PARKOUR["width"][i]/2 <= x_start:
                if x_start <= PARKOUR["position"][i] + PARKOUR["width"][i]/2:
                    z_start = PARKOUR["height"][i]
                else:
                    obstacle_start += 1
            if PARKOUR["position"][i] + PARKOUR["width"][i]/2 < xg - goal_tolerance:
                obstacle_end += 1
    obstacles = np.arange(obstacle_start, obstacle_end, 1)

    for i in range(len(PARKOUR["res_position"])):
            if PARKOUR["res_position"][i] + PARKOUR["res_width"][i]/2 <= x_start:
                res_start += 1
            if PARKOUR["res_position"][i] + PARKOUR["res_width"][i]/2 < xg - goal_tolerance:
                res_end += 1
    restricted_areas = np.arange(res_start, res_end, 1)

    # ################################################### CREATE PROGRAM ####################################################
    prog = MathematicalProgram()
    t = prog.NewContinuousVariables(N, "t")
    prog.AddBoundingBoxConstraint(Hopper.limit.t_min, Hopper.limit.t_max, t)
    v = prog.NewContinuousVariables(N, "v")
    prog.AddBoundingBoxConstraint(Hopper.limit.v_min, Hopper.limit.v_max, v)
    theta = prog.NewContinuousVariables(N, r"\theta")
    prog.AddBoundingBoxConstraint(np.radians(Hopper.limit.theta_min),
                                  np.radians(Hopper.limit.theta_max), theta)

    binarys_front = [None] * len(PARKOUR["position"])
    binarys_back = [None] * len(PARKOUR["position"])
    for i in obstacles:
        name_front = f'delta_{i+1}_hurdle_front'
        name_back = f'delta_{i+1}_hurdle_back'
        binarys_front[i] = prog.NewBinaryVariables(N, name_front)
        binarys_back[i] = prog.NewBinaryVariables(N, name_back)
    PARKOUR["binarys_hurdle_front"] = binarys_front
    PARKOUR["binarys_hurdle_back"] = binarys_back

    # ################################################### SET CONSTRAINTS ####################################################
    x0 = x_start + r * np.cos(theta[0]) + dx_hip_foot
    z0 = z_start + r * np.sin(theta[0]) + dz_hip_foot
    x0_knee = x0 + dx_foot_knee
    z0_knee = z0 + dz_foot_knee
    cost = 0
    for i in range(N):
        vx = v[i] * np.cos(theta[i])
        vz = v[i] * np.sin(theta[i])
        x = x0 + vx * t[i]
        z = z0 + vz * t[i] - 0.5 * g * t[i] * t[i]
        cost += abs(x - xg)
        parkour_height = 0
        for k in obstacles:
            x_position_hurdle_front = PARKOUR["position"][k] - PARKOUR["width"][k] / 2
            x_position_hurdle_back = PARKOUR["position"][k] + PARKOUR["width"][k] / 2

            # front/back binary for obstacle 0 before x_pos and 1 after
            prog.AddConstraint(PARKOUR["binarys_hurdle_front"][k][i] * (x_position_hurdle_front - x) <= 0)
            prog.AddConstraint((1 - PARKOUR["binarys_hurdle_front"][k][i]) * (x - x_position_hurdle_front) <= 0)
            prog.AddConstraint(PARKOUR["binarys_hurdle_back"][k][i] * (x_position_hurdle_back - x) <= 0)
            prog.AddConstraint((1 - PARKOUR["binarys_hurdle_back"][k][i]) * (x - x_position_hurdle_back) <= 0)

            # landing height on platform
            parkour_height += PARKOUR["height"][k] * (PARKOUR["binarys_hurdle_front"][k][i] -
                                                      PARKOUR["binarys_hurdle_back"][k][i])

            # z_pos of foot/knee at hurdle front/back
            z_foot_hurdle_front = (- 0.5 * g * ((x_position_hurdle_front - x0) / vx) ** 2
                                   + vz * ((x_position_hurdle_front - x0) / vx) + z0)
            z_knee_hurdle_back = (- 0.5 * g * ((x_position_hurdle_back - x0_knee) / vx) ** 2
                                  + vz * ((x_position_hurdle_back - x0_knee) / vx) + z0_knee)

            # margin vertical
            if i == 0:
                if PARKOUR["position"][k] - PARKOUR["width"][k] / 2 <= x_start and x_start <= PARKOUR["position"][k] + PARKOUR["width"][k] / 2:
                    prog.AddConstraint((PARKOUR["height"][k] + PARKOUR["margin_obstacle_vertical"] 
                                        - z_knee_hurdle_back) * PARKOUR["binarys_hurdle_back"][k][i] <= 0)
                else:
                    prog.AddConstraint((PARKOUR["height"][k] + PARKOUR["margin_obstacle_vertical"] 
                                        - z_foot_hurdle_front) * PARKOUR["binarys_hurdle_front"][k][i] <= 0)
                    prog.AddConstraint((PARKOUR["height"][k] + PARKOUR["margin_obstacle_vertical"] 
                                        - z_knee_hurdle_back) * PARKOUR["binarys_hurdle_back"][k][i] <= 0)
            else:
                prog.AddConstraint((PARKOUR["height"][k] + PARKOUR["margin_obstacle_vertical"] - z_foot_hurdle_front) *
                                   (PARKOUR["binarys_hurdle_front"][k][i] - PARKOUR["binarys_hurdle_front"][k][i-1]) <= 0)
                prog.AddConstraint((PARKOUR["height"][k] + PARKOUR["margin_obstacle_vertical"] - z_knee_hurdle_back) *
                                   (PARKOUR["binarys_hurdle_back"][k][i] - PARKOUR["binarys_hurdle_back"][k][i-1]) <= 0)
            
            # margin horizontal
            prog.AddConstraint(abs(x - x_position_hurdle_front) >= PARKOUR["margin_obstacle_horizontal"])
            prog.AddConstraint(abs(x - x_position_hurdle_back) >= PARKOUR["margin_obstacle_horizontal"])
            
        # restricted areas
        for j in restricted_areas:
            prog.AddConstraint(abs(x - PARKOUR["res_position"][j]) >= PARKOUR["res_width"][j] / 2)

        prog.AddConstraint(z == parkour_height)
        if i == N - 1:
            prog.AddConstraint(x == xg)
        else:
            x0 = x + r * np.cos(theta[i+1]) + dx_hip_foot
            z0 = z + r * np.sin(theta[i+1]) + dz_hip_foot
            x0_knee = x0 + dx_foot_knee
            z0_knee = z0 + dz_foot_knee

    # cost
    prog.AddCost(sum(t))

    # ################################################### INITIAL GUESS ####################################################
    ig_theta = np.radians(65)
    dx = (xg - x_start) / N - (np.cos(ig_theta) * r + dx_hip_foot)
    dz = -(np.sin(ig_theta) * r + Hopper.dz_hip_foot)  # - np.sin(theta_f) * r_f)
    ig_t = np.sqrt((-dz + dx * np.tan(ig_theta)) / (0.5 * g))
    ig_v = dx / (ig_t * np.cos(ig_theta))

    prog.SetInitialGuess(theta, np.ones(N) * ig_theta)
    prog.SetInitialGuess(t, np.ones(N) * ig_t)
    prog.SetInitialGuess(v, np.ones(N) * ig_v)
    t_setup = time.time() - t_start_setup
    t_start_solve = time.time()

    try:
        if obstacles.size == 0:
            result = SnoptSolver().Solve(prog)
            assert result.is_success()
        else:
            result = MixedIntegerBranchAndBound(prog, SnoptSolver().solver_id())
        t_solve = time.time() - t_start_solve

        res_dict = {"x_start": x_start,
                    "z_start": z_start,
                    "obstacles": obstacles,
                    "areas": restricted_areas,
                    "t": result.GetSolution(t),
                    "v": result.GetSolution(v),
                    "theta": result.GetSolution(theta),
                    "setup_time": t_setup,
                    "solve_time": t_solve,
                    "mppc_time": []}
        print(f"Solution found with {N} jumps")

        return res_dict, obstacles
    except:
        print(f"No solution found with {N} jumps")
        return None, None