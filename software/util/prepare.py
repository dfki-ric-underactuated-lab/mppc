import numpy as np

def number_of_jumps(Hopper, dist):
    t_min = 2 * Hopper.limit.v_min * np.sin(np.radians(Hopper.limit.theta_min)) / Hopper.gravity
    x_min = (Hopper.limit.v_min * np.cos(np.radians(Hopper.limit.theta_min)) * t_min 
             + Hopper.leg.r_takeoff * np.cos(np.radians(Hopper.limit.theta_min)) + Hopper.dx_hip_foot) # 0.0662

    t_max = 2 * Hopper.limit.v_max * np.sin(np.radians(Hopper.limit.theta_min)) / Hopper.gravity
    x_max = (Hopper.limit.v_max * np.cos(np.radians(Hopper.limit.theta_min)) * t_max 
             + Hopper.leg.r_takeoff * np.cos(np.radians(Hopper.limit.theta_min)) + Hopper.dx_hip_foot) # 0.7058

    N_min = int(np.ceil((dist/x_max)))
    N_max = int(max(np.floor((dist/x_min)), 1))
    return np.arange(N_min, N_max+1, 1)


def starting_height(Parkour, x_start):
    for i in range(len(Parkour["position"])):
            if Parkour["position"][i] - Parkour["width"][i]/2 <= x_start:
                if x_start <= Parkour["position"][i] + Parkour["width"][i]/2:
                    return Parkour["height"][i]
    return 0


def prep_contact(Hopper, res_dict):
    x_start = res_dict["x_start"]
    z_start = res_dict["z_start"]
    t = res_dict["t"]
    v = res_dict["v"]
    theta = res_dict["theta"]
    N = len(t)
    contact_x = np.zeros(N + 1)
    contact_x[0] = x_start
    contact_z = np.zeros(N + 1)
    contact_z[0] = z_start
    for i in range(N):
        contact_x[i+1] = (contact_x[i] + Hopper.leg.r_takeoff * np.cos(theta[i]) 
                          + Hopper.dx_hip_foot + v[i] * np.cos(theta[i]) * t[i])
        contact_z[i+1] = (contact_z[i] + Hopper.leg.r_takeoff * np.sin(theta[i]) 
                          + Hopper.dz_hip_foot + v[i] * np.sin(theta[i]) * t[i] 
                          - 0.5 * Hopper.gravity * t[i] ** 2)
    return contact_x, contact_z


def disturb_jump(g, t, v, theta, deviation=0.00):
     dx = np.random.rand(1)[0] * 2 * deviation - deviation
     print(f"Disturbed Contact Point in X: {dx} m")
     x = v * np.cos(theta) * t + dx
     z = v * np.sin(theta) * t - 0.5 * g * t**2
     t_new = np.sqrt(2 * (x * np.tan(theta) - z) / g)
     v_new = x / (t_new * np.cos(theta))
     return dx, t_new, v_new