import numpy as np
import math
import matplotlib.pyplot as plt
import control as ct
from scipy.io import savemat

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



def vehicle_kinematics(x, u):  # this is the actual vehicle model, inside the mpc controller, usually a simplified model is used
    u1,u2 = u # u1 is the desired acceleration, u2 is the desired steering angle
    beta = np.atan(Lr / L * np.tan(x[5]))
    x_next = np.zeros(6)
    x_next[0] = x[0] + Ts * (x[3] * np.cos(x[2] + beta))
    x_next[1] = x[1] + Ts * (x[3] * np.sin(x[2] + beta))
    x_next[2] = x[2] + Ts * x[3] / Lr * np.sin(beta)
    x_next[3] = x[3] + Ts * x[4]
    x_next[4] = x[4] - Ts * (x[4] / tau1 - u1 / tau1)
    x_next[5] = x[5] - Ts * (x[5] / tau2 - u2 / tau2)
    return x_next

def lqr_vehicle_kinematic(xr):
    # X_dot=A*X+B*U
    Xr,Yr,psir,Vr,ar,delta_fr,u1r,u2r=xr
    C_beta=Lr/L*(1+np.square(np.tan(delta_fr)))/(1+np.square(Lr/L*np.tan(delta_fr)))
    if Vr<=0.1:
        Vr=0.1
    '''
    # continuous
    a11=0;a12=0;a13=Vr*-np.sin(psir+C_beta*delta_fr);a14=np.cos(psir+C_beta*delta_fr);a15=0;a16=Vr*C_beta*-np.sin(psir+C_beta*delta_fr)
    a21 = 0;a22 = 0;a23 = Vr * np.cos(psir + C_beta * delta_fr);a24 = np.sin(psir + C_beta * delta_fr);a25 = 0;a26 = Vr * C_beta * np.cos(psir + C_beta * delta_fr)
    a31 = 0;a32 = 0;a33 = 0;a34 = np.sin(C_beta * delta_fr)/L;a35 = 0;a36 = Vr/L * C_beta * np.cos(C_beta * delta_fr)
    a41 = 0;a42 = 0;a43 = 0;a44 =0;a45 = 1;a46 = 0
    a51 = 0;a52 = 0;a53 = 0;a54 = 0;a55 = -1/tau1;a56 = 0
    a61 = 0;a62 = 0;a63 = 0;a64 = 0;a65 = 0;a66 = -1/tau2
    b11=0;b12=0;b21=0;b22=0;b31=0;b32=0;b41=0;b42=0;b51=1/tau1;b52=0;b61=0;b62=1/tau2
    '''
    # discrete
    a11 = 1;a12 = 0;a13 = Ts*Vr * -np.sin(psir + C_beta * delta_fr);a14 = Ts*np.cos(psir + C_beta * delta_fr);a15 = 0;a16 = Ts*Vr * C_beta * -np.sin(psir + C_beta * delta_fr)
    a21 = 0;a22 = 1;a23 = Ts*Vr * np.cos(psir + C_beta * delta_fr);a24 = Ts*np.sin(psir + C_beta * delta_fr);a25 = 0;a26 = Ts*Vr * C_beta * np.cos(psir + C_beta * delta_fr)
    a31 = 0;a32 = 0;a33 = 1;a34 = Ts*np.sin(C_beta * delta_fr) / L;a35 = 0;a36 = Ts*Vr / L * C_beta * np.cos(C_beta * delta_fr)
    a41 = 0;a42 = 0;a43 = 0;a44 = 1;a45 = Ts;a46 = 0
    a51 = 0;a52 = 0;a53 = 0;a54 = 0;a55 = 1-Ts / tau1;a56 = 0
    a61 = 0;a62 = 0;a63 = 0;a64 = 0;a65 = 0;a66 = 1-Ts / tau2
    b11 = 0;b12 = 0;b21 = 0;b22 = 0;b31 = 0;b32 = 0;b41 = 0;b42 = 0;b51 = Ts / tau1;b52 = 0;b61 = 0;b62 = Ts / tau2

    A=np.array([[a11,a12,a13,a14,a15,a16],[a21,a22,a23,a24,a25,a26],[a31,a32,a33,a34,a35,a36],
               [a41,a42,a43,a44,a45,a46],[a51,a52,a53,a54,a55,a56],[a61,a62,a63,a64,a65,a66]])
    B=np.array([[b11,b12],[b21,b22],[b31,b32],[b41,b42],[b51,b52],[b61,b62]])
    return A,B

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
    u1_ref = np.full(x_ref.shape, 0.0)
    u2_ref = np.full(x_ref.shape, 0.0)

    return np.column_stack([x_ref, y_ref, yaw_ref, v_ref, a_ref, delta_f_ref,u1_ref,u2_ref])


class InstantPlanner:
    def __init__(self, ref_trajectory):
        self.ref_trajectory = ref_trajectory
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
        if remaining < 1:
            local_trajectory = self.ref_trajectory[-1]
        else:
            local_trajectory = self.ref_trajectory[nearest_idx]

        return local_trajectory, nearest_idx, min_distance


if __name__ == "__main__":
    Ts = 0.1  # sampling time [s]
    steps = 500  # total control length
    road_w = 6.0  # road width [m]
    tau1 = Ts  # time delay [s]
    tau2 = Ts  # time delay [s]
    Lf = 1.2  # front wheel center to c.o.g. [m]
    Lr = 1.6  # rear wheel center to c.o.g. [m]
    L = Lf + Lr  # wheelbase [m]
    u1_max = 4.0  # max acceleration
    u1_min = -5.0  # max deceleration
    u2_max = np.deg2rad(30) # max steering angle
    u2_min = -np.deg2rad(30) # min steering angle
    desired_velocity=10 # [m/s]
    global_trajectory = generate_global_reference_trajectory(Ts, steps=steps, desired_velocity=desired_velocity)
    planner = InstantPlanner(global_trajectory)

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



    Q = np.diag([1e2, 1e2, 1e2, 1e1, 1e1,1e1])
    R = np.diag([1e1, 1e1])


    current_state = global_trajectory[0,0:6].copy() # initial vehicle state, use copy() to make sure global_trajectory[0,:6] is not modified if current_state is modified
    current_state[3]=0.1 # initial velocity
    nearest_trajectory, nearest_idx, min_distance = planner.get_local_trajectory(
        current_state)  # set the initial trajectories for the vehicle
    Ad,Bd=lqr_vehicle_kinematic(nearest_trajectory)
    #print(Ad)
    #print(Bd)
    dX=np.array([current_state[0:]-nearest_trajectory[0:6]]).T
    #print(dX)
    [K,S,E]=ct.dlqr(Ad,Bd,Q,R)
    #print(K)
    u_opt=-K@dX
    u1=u_opt[0,0]
    u2=u_opt[1,0]
    u1=np.clip(u1,u1_min,u1_max)
    u2=np.clip(u2,u2_min,u2_max)
    u=u1,u2
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
    Xr_h.append(nearest_trajectory[0])
    Yr_h.append(nearest_trajectory[1])
    psir_h.append(nearest_trajectory[2])
    vr_h.append(nearest_trajectory[3])
    a_h.append(current_state[4])
    delta_f_h.append(current_state[5])
    u1_h.append(u[0])
    u2_h.append(u[1])
    cte_h.append((nearest_trajectory[1] - current_state[1]) * np.cos(nearest_trajectory[2]) - (
            nearest_trajectory[0] - current_state[0]) * np.sin(nearest_trajectory[2]))
    he_h.append(nearest_trajectory[2] - current_state[2])

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
            plt.plot(Xr_h[-1], Yr_h[-1], 'go', markersize=3,
                     label='Closest Point')
            draw_vehicle(current_state[0], current_state[1], current_state[2], current_state[-1], plt.gca())
            plt.grid(True)
            plt.axis('equal')
            plt.legend()
            plt.xlabel('X [m]')
            plt.ylabel('Y [m]')
            plt.title(
                f'LQR Trajectory Tracking | Accel:{current_accel:.3f}m/s^2 | Steering:{np.rad2deg(current_steer):.3f}'+chr(176))
            plt.pause(0.001)

        # set parameters which are the local reference trajectories, opti.set_value only works for an opti.parameter not an opti.variable

        next_state = vehicle_kinematics(current_state, u)
        yaw_rate_h.append((next_state[2]-current_state[2])/Ts)
        current_state = next_state
        nearest_trajectory, nearest_idx, min_distance = planner.get_local_trajectory(
            current_state)
        Ad, Bd = lqr_vehicle_kinematic(nearest_trajectory)
        # print(Ad)
        # print(Bd)
        dX = np.array([current_state[0:] - nearest_trajectory[0:6]]).T
        # print(dX)
        [K, S, E] = ct.dlqr(Ad, Bd, Q, R)
        # print(K)
        u_opt = -K @ dX
        u1 = u_opt[0, 0]
        u2 = u_opt[1, 0]
        u1 = np.clip(u1, u1_min, u1_max)
        u2 = np.clip(u2, u2_min, u2_max)
        u = u1, u2
        #print(u)
        X_h.append(current_state[0])
        Y_h.append(current_state[1])
        psi_h.append(current_state[2])
        v_h.append(current_state[3])
        Xr_h.append(nearest_trajectory[0])
        Yr_h.append(nearest_trajectory[1])
        psir_h.append(nearest_trajectory[2])
        vr_h.append(nearest_trajectory[3])
        a_h.append(current_state[4])
        delta_f_h.append(current_state[5])
        u1_h.append(u[0])
        u2_h.append(u[1])
        cte_h.append((nearest_trajectory[1] - current_state[1]) * np.cos(nearest_trajectory[2]) - (
                nearest_trajectory[0] - current_state[0]) * np.sin(nearest_trajectory[2]))
        he_h.append(nearest_trajectory[2] - current_state[2])
        if nearest_idx == len(global_trajectory) -1:
            break

    ## after loop
    plt.figure(figsize=(20, 10))
    plt.plot(X_h, Y_h, 'b--', label='Actual Trajectory')
    plt.plot(left_bound[:, 0], left_bound[:, 1], 'k-', linewidth=1, alpha=1, label='Left Road Boundary')
    plt.plot(right_bound[:, 0], right_bound[:, 1], 'k-', linewidth=1, alpha=1, label='Right Road Boundary')
    plt.plot(Xr_h, Yr_h, 'g--', linewidth=2, alpha=1, label='Reference Trajectory')
    plt.ylabel('Y [m]')
    plt.xlabel('X [m]')
    plt.grid(True)
    plt.axis('equal')
    plt.title('Trajectory Tracking Comparison', fontsize=20)
    plt.legend(fontsize=20)
    plt.tight_layout()
    plt.savefig('lqr1.jpg')

    time_axis = np.arange(len(X_h)) * Ts

    plt.figure(figsize=(10, 5))
    plt.plot(time_axis, cte_h, 'b-', label='Cross Track Error')
    plt.grid(True)
    plt.title('Cross Track Error During Tracking')
    plt.ylabel('Distance Error [m]')

    plt.figure(figsize=(10, 5))
    plt.plot(time_axis, np.degrees(he_h), 'g-', label='Heading error')
    plt.grid(True)
    plt.title('Heading Error During Tracking')
    plt.ylabel('Angle [°]')

    plt.figure(figsize=(10, 5))
    plt.plot(time_axis, np.degrees(psi_h), 'g-', label='Actual Heading Angle')
    plt.plot(time_axis, np.degrees(psir_h), 'm--', alpha=0.6, label='Reference Heading Angle')
    plt.grid(True)
    plt.title('Change In Heading Angle')
    plt.ylabel('Angle [°]')
    plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(10, 5))
    plt.plot(time_axis, v_h, 'b-', label='Actual Speed')
    plt.grid(True)
    plt.plot(time_axis, vr_h, 'm--', alpha=0.6, label='Reference Speed')
    plt.title('Speed Tracking')
    plt.ylabel('Speed [m/s]')
    plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(10, 5))
    plt.plot(time_axis, a_h, 'r-', label='Acceleration Input')
    plt.grid(True)
    plt.title('Acceleration Control History')
    plt.ylabel('Acceleration [m/s^2]')
    plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(10, 5))
    plt.plot(time_axis, delta_f_h, 'b-', label='Steering Angle Input')
    plt.grid(True)
    plt.title('Steering Control History')
    plt.ylabel('Steering Angle [rad]')
    plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(10, 5))
    plt.plot(time_axis[0:-1], np.diff(a_h) / Ts, 'r-', label='Acceleration Derivative ')
    plt.grid(True)
    plt.title('Jerk History')
    plt.ylabel('Jerk [m/s^3]')
    plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(10, 5))
    plt.plot(time_axis[0:-1], np.diff(delta_f_h) / Ts, 'r-', label='Steering Angle Derivative ')
    plt.grid(True)
    plt.title('Steering Angle Rate History')
    plt.ylabel('Steering Angle Rate [rad/s]')
    plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(20, 10))
    plt.subplot(2, 2, 1)
    plt.plot(time_axis, np.rad2deg(delta_f_h), 'b-')
    plt.grid(True)
    plt.title('Steering Control History', fontsize=20)
    # plt.xlabel('Time [s]',fontsize=12)
    plt.ylabel('Steering Angle [' + chr(176) + ']', fontsize=20)

    plt.subplot(2, 2, 2)
    plt.plot(time_axis[:-1], np.rad2deg(yaw_rate_h), 'b-')
    plt.grid(True)
    plt.title('Yaw Rate History', fontsize=20)
    # plt.xlabel('Time [s]',fontsize=12)
    plt.ylabel('Yaw Rate [' + chr(176) + '/s]', fontsize=20)

    plt.subplot(2, 2, 3)
    plt.plot(time_axis, np.rad2deg(he_h), 'b-')
    plt.grid(True)
    plt.title('Heading Error History', fontsize=20)
    plt.xlabel('Time [s]', fontsize=20)
    plt.ylabel('Heading Angle Error [' + chr(176) + ']', fontsize=20)

    plt.subplot(2, 2, 4)
    plt.plot(time_axis, cte_h, 'b-')
    plt.grid(True)
    plt.title('Cross Track Error During Tracking', fontsize=20)
    plt.xlabel('Time [s]', fontsize=20)
    plt.ylabel('Distance Error [m]', fontsize=20)
    plt.savefig('lqr2.jpg')

    plt.show()


    savemat('Xlqr.mat', {'Xlqr': np.array(X_h)})
    savemat('Ylqr.mat', {'Ylqr': np.array(Y_h)})
    savemat('delta_flqr.mat', {'delta_flqr': np.array(delta_f_h)})
    savemat('yaw_ratelqr.mat', {'yaw_ratelqr': np.array(yaw_rate_h)})
    savemat('helqr.mat', {'helqr': np.array(he_h)})
    savemat('ctelqr.mat', {'ctelqr': np.array(cte_h)})