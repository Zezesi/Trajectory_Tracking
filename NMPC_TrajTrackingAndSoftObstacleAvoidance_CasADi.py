import casadi as ca
import numpy as np
import math
from casadi import *
import matplotlib.pyplot as plt

Ts = 0.1  # sampling time [s]
tau1 = Ts  # time delay [s]
tau2 = Ts  # time delay [s]
Lf = 1.2  # front wheel center to c.o.g. [m]
Lr = 1.6  # rear wheel center to c.o.g. [m]
L = Lf + Lr  # wheelbase [m]
Wt = 1.365  # wheel track [m]
MAX_STEER = np.deg2rad(30) # max front steering angle
MAX_ACCEL = 4.0  # max acceleration
MAX_DECEL = -5.0  # max deceleration
# for drawing
Vw = 1.580  # vehicle width [m]
Lh = 2  # c.o.g. to the head [m]
Lt = 2.4  # c.o.g. to the tail [m]
Tr = 0.2888  # tire radius 185/60 R 14
Tw = 0.185  # tire width
Length = Lh + Lt  # vehicle length
road_w = 6.0  # road_width [m]


def normalize_angle(angle):
    if angle > math.pi:
        angle -= 2 * math.pi
    if angle < -math.pi:
        angle += 2 * math.pi
    return angle

def normalize_yaw_rate(yaw_rate):
    while yaw_rate > math.pi/Ts:
        yaw_rate -= 2 * math.pi/Ts
    while yaw_rate < -math.pi/Ts:
        yaw_rate += 2 * math.pi/Ts
    return yaw_rate


def vehicle_kinematics(x, u):  # this is the actual vehicle model, inside the mpc controller, usually a simplified model is used
    u1, u2 = u  # u1 is the desired acceleration, u2 is the desired steering angle
    beta = np.atan(Lr / L * np.tan(x[5]))
    x_next = np.zeros(6)
    x_next[0] = x[0] + Ts * (x[3] * cos(x[2] + beta))
    x_next[1] = x[1] + Ts * (x[3] * sin(x[2] + beta))
    x_next[2] = normalize_angle(x[2] + Ts * x[3] / Lr * sin(beta))
    x_next[3] = x[3] + Ts * x[4]
    x_next[4] = x[4] - Ts * (x[4] / tau1 - u1 / tau1)
    x_next[5] = x[5] - Ts * (x[5] / tau2 - u2 / tau2)
    return x_next


def draw_vehicle(x, y, yaw, steer, ax, colort='black', colorb='red'):
    vehicle_outline = np.array(
        [[-Lt, Lh, Lh, -Lt, -Lt],
         [Vw / 2, Vw / 2, -Vw / 2, -Vw / 2, Vw / 2]])

    wheel = np.array([[-Tr, Tr, Tr, -Tr, -Tr],
                      [Tw / 2, Tw / 2, -Tw / 2, -Tw / 2,
                       Tw / 2]])

    rr_wheel = wheel.copy()  # rear right
    rl_wheel = wheel.copy()  # rear left
    fr_wheel = wheel.copy()  # front right
    fl_wheel = wheel.copy()  # front left

    # steering angle rotation matrix
    rot1 = np.array([[np.cos(steer), -np.sin(steer)],
                     [np.sin(steer), np.cos(steer)]])

    # yaw angle rotation matrix
    rot2 = np.array([[np.cos(yaw), -np.sin(yaw)],
                     [np.sin(yaw), np.cos(yaw)]])
    fr_wheel = np.dot(rot1, fr_wheel)
    fl_wheel = np.dot(rot1, fl_wheel)
    fr_wheel += np.array([[Lf], [-Wt / 2]])
    fl_wheel += np.array([[Lf], [Wt / 2]])
    rr_wheel += np.array([[-Lr], [-Wt / 2]])
    rl_wheel += np.array([[-Lr], [Wt / 2]])

    fr_wheel = np.dot(rot2, fr_wheel)
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    fl_wheel = np.dot(rot2, fl_wheel)
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rr_wheel = np.dot(rot2, rr_wheel)
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    rl_wheel = np.dot(rot2, rl_wheel)
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y
    vehicle_outline = np.dot(rot2, vehicle_outline)
    vehicle_outline[0, :] += x
    vehicle_outline[1, :] += y

    ax.plot(fr_wheel[0, :], fr_wheel[1, :], colort)
    ax.plot(rr_wheel[0, :], rr_wheel[1, :], colort)
    ax.plot(fl_wheel[0, :], fl_wheel[1, :], colort)
    ax.plot(rl_wheel[0, :], rl_wheel[1, :], colort)

    ax.plot(vehicle_outline[0, :], vehicle_outline[1, :], colorb)


def generate_global_reference_trajectory(Ts=0.1, steps=750,desired_velocity=10):
    # reference trajectory states constains Xr, Yr, psir vr, ar and delta_fr,
    # ar and delta_fr can be set to 0 or reference values


    # Re-initialize state and storage
    state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [x, y, yaw, v, a, delta_f]
    trajectory = []

    for i in range(steps):
        cof1 = 40
        cof2 = 250
        if i % cof2 == 0:
            i = i - 1
        trajectory.append(state.copy())

        # Control logic
        u1 = np.clip((desired_velocity - state[3]) / 2, MAX_DECEL, MAX_ACCEL)
        if i <= cof2:
            u2 = -0.2 * np.tanh(np.square(cof1 / (cof2 - i)))
        elif i <= 2 * cof2 and i > cof2:
            u2 = +0.2 * np.tanh(np.square(cof1 / (cof2 - i)))
        elif i <= 2.2 * cof2 and i > 2 * cof2:
            u2 = -0.2 * np.tanh(np.square(cof1 / (cof2 - (i - cof2))))
        else:
            u2 = +0.2 * np.tanh(np.square(cof1 / (cof2 - (i - 2 * cof2))))

        u = np.array([u1, u2])
        new_state = vehicle_kinematics(state, u)
        state = new_state

    '''
    Time= steps*Ts
    step=np.linspace(0,Time,steps)

    a = 0.5 * np.sin(2 * np.pi * step / Time)
    delta_f = MAX_STEER * 0.8 * (1 / (1 + np.exp(-10 * (step / Time - 0.5))) - 0.5)
    delta_f = np.clip(delta_f, -MAX_STEER * 0.8, MAX_STEER * 0.8)

    state = np.array([0.0, 0.0, 0.0, 0.0,0.0,0.0])  # [x, y, yaw, v, a, delta_f]
    trajectory = []

    for i in range(steps):
        trajectory.append(state.copy())
        u = np.array([a[i], delta_f[i]])
        new_state = vehicle_kinematics(state, u)
        state = new_state
    '''

    trajectory = np.array(trajectory)
    x_ref = trajectory[:, 0]
    y_ref = trajectory[:, 1]
    yaw_ref = trajectory[:, 2]
    v_ref = trajectory[:, 3] # np.full(x_ref.shape, desired_velocity)
    a_ref = np.full(x_ref.shape, 0.0)
    delta_f_ref = np.full(x_ref.shape, 0.0)

    return np.column_stack([x_ref, y_ref, yaw_ref, v_ref, a_ref, delta_f_ref])


class InstantPlanner:
    def __init__(self, ref_trajectory, Np=10):
        self.ref_trajectory = ref_trajectory
        self.Np = Np
        self.prev_idx = 0  # for recording the last search index

    def get_local_trajectory(self, current_pose):

        # start from the last search index
        search_window = self.ref_trajectory[self.prev_idx:, :2]

        distances = np.linalg.norm(search_window - current_pose[:2], axis=1)
        relative_idx = np.argmin(distances)
        min_distance = distances[relative_idx]
        nearest_idx = self.prev_idx + relative_idx

        nearest_idx = max(self.prev_idx, nearest_idx)
        self.prev_idx = nearest_idx

        remaining = len(self.ref_trajectory) - nearest_idx - 1
        horizon = min(self.Np, remaining)
        if horizon < 1:
            local_trajectory = self.ref_trajectory[-1, :]
            local_trajectory = local_trajectory.reshape(1, -1)
        else:
            local_trajectory = self.ref_trajectory[nearest_idx:nearest_idx + horizon + 1]

        if self.Np + 1 > len(local_trajectory):
            local_trajectory = np.concatenate(
                [local_trajectory, [local_trajectory[-1]] * (self.Np + 1 - len(local_trajectory))], axis=0)

        return local_trajectory, nearest_idx, min_distance


if __name__ == "__main__":
    Ts = 0.1  # sampling time [s]
    N = 15  # predict horizon length
    steps = 500  # total control length
    road_w = 6.0  # road width [m]
    tau1 = Ts  # time delay [s]
    tau2 = Ts  # time delay [s]
    Lf = 1.2  # front wheel center to c.o.g. [m]
    Lr = 1.6  # rear wheel center to c.o.g. [m]
    L = Lf + Lr  # wheelbase [m]
    cte_max = road_w / 3  # max cross track error [m]
    cte_min = -road_w / 3  # min cross track error [m]

    v_max = 20.0  # max longitudinal velocity
    v_min = 0.0  # min longitudinal velocity

    u1_max = 4.0  # max acceleration
    u1_min = -5.0  # max deceleration
    u2_max = np.deg2rad(30) # max steering angle
    u2_min = -np.deg2rad(30) # min steering angle
    delta_a_max = 0.8 # max acceleration change to ensure smooth acceleration transition
    delta_a_min = -0.8
    delta_delta_f_max = np.deg2rad(5) # max steering angle change to ensure smooth steering transition
    delta_delta_f_min = -np.deg2rad(5)
    desired_velocity=10 # [m/s]
    global_trajectory = generate_global_reference_trajectory(Ts, steps=steps, desired_velocity=desired_velocity)
    planner = InstantPlanner(global_trajectory, Np=N)
    Lh = 2  # c.o.g. to the head [m]
    Lt = 2.4  # c.o.g. to the tail [m]
    Length = Lh + Lt  # vehicle length
    Obs_diam=0.5 # obstacle diameter [m]
    total_diam=Length+Obs_diam/2 # total diameter [m]
    margin=0.1 # safe margin [m]
    safe_diam=total_diam+margin

    X_obs=[78.6,151.3]
    Y_obs=[-10.3,-131.5]

    # visual settings
    show_animation = True
    skip_frame = 2

    left_bound = np.array([
        global_trajectory[1:-1, 0] - (road_w / 2) * np.sin(global_trajectory[1:-1, 2]),
        global_trajectory[1:-1, 1] + (road_w / 2) * np.cos(global_trajectory[1:-1, 2])
    ]).T
    right_bound = np.array([
        global_trajectory[1:-1, 0] + (road_w / 2) * np.sin(global_trajectory[1:-1, 2]),
        global_trajectory[1:-1, 1] - (road_w / 2) * np.cos(global_trajectory[1:-1, 2])
    ]).T

    opti = ca.Opti()

    delta_control_states = opti.variable(N, 2)
    delta_a = delta_control_states[:, 0]  # delta action 1: for smooth acceleration transition
    delta_delta_f = delta_control_states[:, 1]  # delta action 2: for smooth steering angle transition

    opt_controls = opti.variable(N, 2)
    u1 = opt_controls[:, 0]  # action 1: desired acceleration
    u2 = opt_controls[:, 1]  # action 2: desired steering angle

    # vehicle states
    veh_states = opti.variable(N + 1, 6)
    X = veh_states[:, 0]
    Y = veh_states[:, 1]
    psi = veh_states[:, 2]
    v = veh_states[:, 3]
    a = veh_states[:, 4]
    delta_f = veh_states[:, 5]

    # vehicle states need to be optimized
    opt_states = opti.variable(N + 1, 5)
    cte = opt_states[:, 0]
    he = opt_states[:, 1]
    ve = opt_states[:, 2]
    ae = opt_states[:, 3]
    delta_fe = opt_states[:, 4]


    # parameters, these parameters are the reference trajectories and the actual initial vehicle states
    opt_x_ref = opti.parameter(N + 1, 6)
    # and the actual initial vehicle states
    init_v_states = opti.parameter(1, 6)

    # vehicle model
    f = lambda x, u: ca.vertcat(*[x[3] * ca.cos(x[2] + ca.atan(Lr / L * ca.tan(x[5]))),
                                  x[3] * ca.sin(x[2] + ca.atan(Lr / L * ca.tan(x[5]))),
                                  x[3] / Lr * ca.sin(ca.atan(Lr / L * ca.tan(x[5]))),
                                  x[4],
                                  u[0] / tau1 - x[4] / tau1,
                                  u[1] / tau2 - x[5] / tau2])

    # for assigning opti.variable, must use opti.subject_to; for assigning opti.parameter, use opti.set_value,but not during the optimization process; for others use =
    opti.subject_to(veh_states[0, :] == init_v_states)
    opti.subject_to(opt_states[0, 0] == (opt_x_ref[0, 1] - veh_states[0, 1]) * ca.cos(opt_x_ref[0, 2]) - (
                opt_x_ref[0, 0] - veh_states[0, 0]) * ca.sin(opt_x_ref[0, 2]))
    opti.subject_to(opt_states[0, 1:] == opt_x_ref[0, 2:] - veh_states[0, 2:])
    for i in range(N):
        next_veh_states = veh_states[i, :] + f(veh_states[i, :], opt_controls[i, :]).T * Ts
        opti.subject_to(veh_states[i + 1, :] == next_veh_states)
        opti.subject_to(delta_control_states[i, :] == next_veh_states[4:] - veh_states[i, 4:])
        opti.subject_to(opt_states[i + 1, 0] == (opt_x_ref[i + 1, 1] - next_veh_states[1]) * ca.cos(opt_x_ref[i + 1, 2]) - (
                    opt_x_ref[i + 1, 0] - next_veh_states[0]) * ca.sin(opt_x_ref[i + 1, 2]))
        opti.subject_to(opt_states[i + 1, 1:] == opt_x_ref[i + 1, 2:] - next_veh_states[2:])




    # weight matrix
    Q = np.diag([1e1, 1e-3, 1e1, 1e1, 1e1])
    R = np.diag([1e1, 1e1])  # (delta_a, delta_delta_f)
    P = np.diag([1e1, 1e-3, 1e1, 1e1, 1e1])  # weights for final states
    O=1e1

    # cost function
    obj = 0
    for i in range(N):
        state_error = opt_states[i, :]
        control_change = delta_control_states[i, :]
        obj +=ca.mtimes([state_error, Q, state_error.T]) + ca.mtimes([control_change, R, control_change.T])
    state_error_final = opt_states[N, :]
    obj +=ca.mtimes([state_error_final, P, state_error_final.T])

    for i in range(N+1):
        for j in range(len(X_obs)):
            soft_error = 10/((veh_states[i, 0]-X_obs[j])**2+(veh_states[i, 1]-Y_obs[j])**2+0.0001)
            obj+=ca.mtimes([soft_error, O, soft_error.T])
    opti.minimize(obj)

    # state and action constraints
    opti.subject_to(opti.bounded(cte_min, cte, cte_max))
    opti.subject_to(opti.bounded(v_min, v, v_max))
    opti.subject_to(opti.bounded(u1_min, u1, u1_max))
    opti.subject_to(opti.bounded(u2_min, u2, u2_max))
    opti.subject_to(opti.bounded(delta_a_min, delta_a, delta_a_max))
    opti.subject_to(opti.bounded(delta_delta_f_min, delta_delta_f, delta_delta_f_max))

    # remember the cost function and the hard constraints can be replaced by soft functions

    opts_setting = {'ipopt.max_iter': 2000,
                    'ipopt.print_level': 0,
                    'print_time': 0,
                    'ipopt.acceptable_tol': 1e-8,
                    'ipopt.acceptable_obj_change_tol': 1e-6}

    opti.solver('ipopt', opts_setting)

    current_state = global_trajectory[0] # initial vehicle state
    opt_controls0 = np.zeros((N, 2))  # initial optimized actions guess
    delta_controls0 = np.zeros((N, 2))  # initial action change guess
    init_trajectories, nearest_idx, min_distance = planner.get_local_trajectory(
        current_state)  # set the initial trajectories for the vehicle
    init_states = np.tile(current_state, N + 1).reshape(N + 1, -1) # set the initial states for the vehicle
    init_error = np.zeros(5) # set the states need to be optimized for the vehicle
    init_error[0] = (init_trajectories[0, 1] - current_state[1]) * np.cos(init_trajectories[0, 2]) - (
                init_trajectories[0, 0] - current_state[0]) * np.sin(init_trajectories[0, 2])
    init_error[1:] = init_trajectories[0, 2:] - current_state[2:]
    init_errors = np.tile(init_error, N + 1).reshape(N + 1, -1)

    # contains the history of the states and the action
    X_h = []
    Y_h = []
    psi_h = []
    v_h = []
    Xr_h = []
    Yr_h = []
    psir_h = []
    vr_h = []
    a_h = []
    delta_f_h = []
    u1_h = []
    u2_h = []
    cte_h = []
    he_h=[]
    yaw_rate_h=[]
    X_h.append(current_state[0])
    Y_h.append(current_state[1])
    psi_h.append(current_state[2])
    v_h.append(current_state[3])
    Xr_h.append(init_trajectories[0,0])
    Yr_h.append(init_trajectories[0,1])
    psir_h.append(init_trajectories[0,2])
    vr_h.append(init_trajectories[0,3])
    a_h.append(current_state[4])
    delta_f_h.append(current_state[5])
    u1_h.append(opt_controls0[0, 0])
    u2_h.append(opt_controls0[0, 1])
    cte_h.append(init_error[0])
    he_h.append(normalize_angle(init_error[1]))

    # start NMPC loop
    while True:
        if show_animation and nearest_idx % skip_frame == 0:
            plt.cla()
            current_accel = a_h[-1]
            current_steer = delta_f_h[-1]
            plt.fill(
                np.concatenate([left_bound[:, 0], right_bound[::-1, 0]]),
                np.concatenate([left_bound[:, 1], right_bound[::-1, 1]]),
                color='gray', alpha=0.1, zorder=1
            )
            plt.plot(global_trajectory[:, 0], global_trajectory[:, 1], 'k-', alpha=1,
                     label='Global Path', dashes=(3, 4))
            plt.plot(left_bound[:, 0], left_bound[:, 1], 'k-', linewidth=1, alpha=1)
            plt.plot(right_bound[:, 0], right_bound[:, 1], 'k-', linewidth=1, alpha=1)
            plt.plot(init_trajectories[:, 0], init_trajectories[:, 1], 'g-', linewidth=1, alpha=1,
                     label='Local Reference')
            plt.plot(Xr_h[-1], Yr_h[-1], 'go', markersize=3,
                     label='Closest Point')
            plt.gca().add_patch(plt.Circle((X_obs[0], Y_obs[0]), Obs_diam/2,color='r', fill=False))
            plt.gca().add_patch(plt.Circle((X_obs[1], Y_obs[1]), Obs_diam/2,color='r', fill=False))
            draw_vehicle(current_state[0], current_state[1], current_state[2], current_state[-1], plt.gca())
            plt.grid(True)
            plt.axis('equal')
            plt.legend()
            plt.xlabel('X [m]')
            plt.ylabel('Y [m]')
            plt.title(
                f'NMPC Trajectory Tracking | Accel:{current_accel:.3f}m/s^2 | Steering:{np.rad2deg(current_steer):.3f}'+chr(176))
            plt.pause(0.001)

        # set parameters which are the local reference trajectories, opti.set_value only works for an opti.parameter not an opti.variable
        opti.set_value(opt_x_ref, init_trajectories)
        opti.set_value(init_v_states, current_state)

        # provide the initial guess of the optimization targets, opti.set_initial only takes place once
        opti.set_initial(delta_control_states, delta_controls0.reshape(N, 2))
        opti.set_initial(opt_controls, opt_controls0.reshape(N, 2))
        opti.set_initial(veh_states, init_states.reshape(N + 1, 6))
        opti.set_initial(opt_states, init_errors.reshape(N + 1, 5))

        # solve the problem once again
        sol = opti.solve()
        # obtain the control input
        u_res = sol.value(opt_controls)
        # print(u_res)
        u1_h.append(u_res[0, 0])
        u2_h.append(u_res[0, 1])
        next_state = vehicle_kinematics(current_state, u_res[0, :])
        yaw_rate_h.append(normalize_yaw_rate((next_state[2]-current_state[2])/Ts))
        current_state = next_state
        init_states = np.tile(current_state, N + 1).reshape(N + 1, -1)
        X_h.append(current_state[0])
        Y_h.append(current_state[1])
        psi_h.append(current_state[2])
        v_h.append(current_state[3])
        a_h.append(current_state[4])
        delta_f_h.append(current_state[5])
        # obtain the new local trajectories
        init_trajectories, nearest_idx, min_distance = planner.get_local_trajectory(current_state)
        Xr_h.append(init_trajectories[0, 0])
        Yr_h.append(init_trajectories[0, 1])
        psir_h.append(init_trajectories[0, 2])
        vr_h.append(init_trajectories[0, 3])
        cte_h.append((init_trajectories[0, 1] - current_state[1]) * np.cos(init_trajectories[0, 2]) - (
                init_trajectories[0, 0] - current_state[0]) * np.sin(init_trajectories[0, 2]))
        he_h.append(normalize_angle(init_trajectories[0, 2]-current_state[2]))
        if nearest_idx == len(global_trajectory) -1:
            break

    ## after loop
    plt.figure(figsize=(20, 10))
    plt.plot(X_h, Y_h, 'b--', label='Actual Trajectory')
    plt.plot(left_bound[:, 0], left_bound[:, 1], 'k-', linewidth=1, alpha=1,label='Left Road Boundary')
    plt.plot(right_bound[:, 0], right_bound[:, 1], 'k-', linewidth=1, alpha=1,label='Right Road Boundary')
    plt.plot(Xr_h, Yr_h, 'g--',linewidth=2, alpha=1, label='Reference Trajectory')
    plt.gca().add_patch(plt.Circle((X_obs[0], Y_obs[0]), Obs_diam / 2, color='r', fill=False))
    plt.gca().add_patch(plt.Circle((X_obs[1], Y_obs[1]), Obs_diam / 2, color='r', fill=False))
    plt.ylabel('Y [m]')
    plt.xlabel('X [m]')
    plt.grid(True)
    plt.axis('equal')
    plt.title('Trajectory Tracking Comparison',fontsize=20)
    plt.legend(fontsize=20)
    plt.tight_layout()
    plt.savefig('1.jpg')

    time_axis = np.arange(len(X_h)) * Ts

    plt.figure(figsize=(10, 5))
    plt.plot(time_axis, cte_h, 'b-', label='Cross Track Error')
    plt.grid(True)
    plt.title('Cross Track Error During Tracking')
    plt.ylabel('Distance Error (m)')

    plt.figure(figsize=(10, 5))
    plt.plot(time_axis, np.degrees(he_h), 'g-', label='Heading error')
    plt.grid(True)
    plt.title('Heading Error During Tracking')
    plt.ylabel('Angle (°)')

    plt.figure(figsize=(10, 5))
    plt.plot(time_axis, np.degrees(psi_h), 'g-', label='Actual Heading Angle')
    plt.plot(time_axis, np.degrees(psir_h), 'm--', alpha=0.6, label='Reference Heading Angle')
    plt.grid(True)
    plt.title('Change In Heading Angle')
    plt.ylabel('Angle (°)')
    plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(10, 5))
    plt.plot(time_axis, v_h, 'b-', label='Actual Speed')
    plt.grid(True)
    plt.plot(time_axis, vr_h, 'm--', alpha=0.6, label='Reference Speed')
    plt.title('Speed Tracking')
    plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(10, 5))
    plt.plot(time_axis, a_h, 'r-', label='Acceleration Input')
    plt.grid(True)
    plt.title('Acceleration Control History')
    plt.ylabel('Acceleration (m/s²)')
    plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(10, 5))
    plt.plot(time_axis, delta_f_h, 'b-', label='Steering Angle Input')
    plt.grid(True)
    plt.title('Steering Control History')
    plt.ylabel('Steering Angle (rad)')
    plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(10, 5))
    plt.plot(time_axis[0:-1], np.diff(a_h) / Ts, 'r-', label='Acceleration Derivative ')
    plt.grid(True)
    plt.title('Jerk History')
    plt.ylabel('Jerk (m/s^3)')
    plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(10, 5))
    plt.plot(time_axis[0:-1], np.diff(delta_f_h) / Ts, 'r-', label='Steering Angle Derivative ')
    plt.grid(True)
    plt.title('Steering Angle Rate History')
    plt.ylabel('Steering Angle Rate (rad/s)')
    plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(20, 10))
    plt.subplot(2,2,1)
    plt.plot(time_axis, np.rad2deg(delta_f_h), 'b-')
    plt.grid(True)
    plt.title('Steering Control History',fontsize=20)
    #plt.xlabel('Time [s]',fontsize=12)
    plt.ylabel('Steering Angle ['+chr(176)+']',fontsize=20)

    plt.subplot(2, 2, 2)
    plt.plot(time_axis[:-1], np.rad2deg(yaw_rate_h), 'b-')
    plt.grid(True)
    plt.title('Yaw Rate History',fontsize=20)
    #plt.xlabel('Time [s]',fontsize=12)
    plt.ylabel('Yaw Rate [' + chr(176) + '/s]',fontsize=20)

    plt.subplot(2, 2, 3)
    plt.plot(time_axis, np.rad2deg(he_h), 'b-')
    plt.grid(True)
    plt.title('Heading Error History',fontsize=20)
    plt.xlabel('Time [s]',fontsize=20)
    plt.ylabel('Heading Angle Error [' + chr(176) + ']',fontsize=20)

    plt.subplot(2, 2, 4)
    plt.plot(time_axis, cte_h, 'b-')
    plt.grid(True)
    plt.title('Cross Track Error During Tracking',fontsize=20)
    plt.xlabel('Time [s]',fontsize=20)
    plt.ylabel('Distance Error (m)',fontsize=20)
    plt.savefig('2.jpg')


    plt.show()