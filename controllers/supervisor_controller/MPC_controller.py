import numpy as np
import cvxpy
import os


from Utilits.CubicSpline import cubic_spline_planner, spline_continuity
from Utilits.utils import read_waypoints

"""
MPC controller for webots environment
"""


class MPC_controller:
    def __init__(self, MPC_horizon, dt, state_weight, control_weight, x_init) -> None:
        self.N = MPC_horizon
        self.dt = dt
        self.Q = state_weight
        self.R = control_weight
        self.Qf = self.Q
        self.v_lim = [-10.0, 10.0]
        self.u_max = 30.0
        self.x_init = x_init

        self.state_dimension = 4
        self.control_dimension = 2
        self.goal_dist = 0.2

    def get_nparray_from_matrix(self, x):
        return np.array(x).flatten()

    def mpc_control(self, x_ref, x_current):
        """
        Input: x_ref: reference trajectory

        Output: State Trajectory and Control Trajectory for the next N steps
        """
        x = cvxpy.Variable(
            (self.state_dimension, self.N + 1)
        )  # [pos_1, vel_1, pos_2, vel_2]
        u = cvxpy.Variable(
            (self.control_dimension, self.N)
        )  # [acceleration_1, acceleration_2]

        # System dynamics
        A = np.zeros((self.state_dimension, self.state_dimension))
        B = np.zeros((self.state_dimension, self.control_dimension))

        A[0, 0] = 1.0
        A[0, 1] = self.dt
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[2, 3] = self.dt
        A[3, 3] = 1.0

        B[0, 0] = 0.5 * self.dt**2
        B[1, 0] = self.dt
        B[2, 1] = 0.5 * self.dt**2
        B[3, 1] = self.dt

        cost = 0.0
        constraints = []

        for t in range(self.N):
            # define Cost function
            cost += cvxpy.quad_form(u[:, t], self.R)
            cost += cvxpy.quad_form(x_ref[:, t] - x[:, t], self.Q)

            # Dynamics Constraint
            constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t]]

        cost += cvxpy.quad_form(x_ref[:, self.N] - x[:, self.N], self.Qf)

        constraints += [x[:, 0] == x_current]
        constraints += [x[1, :] >= self.v_lim[0]]
        constraints += [x[1, :] <= self.v_lim[1]]

        constraints += [cvxpy.abs(u[0, :]) <= self.u_max]
        constraints += [cvxpy.abs(u[1, :]) <= self.u_max]

        MPC_problem = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        MPC_problem.solve(verbose=False)

        if (
            MPC_problem.status == cvxpy.OPTIMAL
            or MPC_problem.status == cvxpy.OPTIMAL_INACCURATE
        ):
            x1_traj = self.get_nparray_from_matrix(x.value[0, :])
            x2_traj = self.get_nparray_from_matrix(x.value[1, :])
            x3_traj = self.get_nparray_from_matrix(x.value[2, :])
            x4_traj = self.get_nparray_from_matrix(x.value[3, :])
            u1_traj = self.get_nparray_from_matrix(u.value[0, :])
            u2_traj = self.get_nparray_from_matrix(u.value[1, :])

            return x1_traj, x2_traj, x3_traj, x4_traj, u1_traj, u2_traj
        else:
            print("Cannot find control")
            return [None] * (self.state_dimension + self.control_dimension)

    def intergrate_dynamics_with_euler(self, x_current, u_current):
        # Integrate over a single time step dt using euler method
        # ok to use euler method for linear system
        x_next = x_current

        x_next[0] = (
            x_current[0] + x_current[1] * self.dt + 0.5 * u_current[0] * self.dt**2
        )
        x_next[1] = x_current[1] + u_current[0] * self.dt
        x_next[2] = (
            x_current[2] + x_current[3] * self.dt + 0.5 * u_current[1] * self.dt**2
        )
        x_next[3] = x_current[3] + u_current[1] * self.dt

        return x_next

    def within_goal(self, x_current, x_goal):
        return np.linalg.norm(x_current - x_goal) <= self.goal_dist

    def nearest_interpolated_point(self, x_ref, x_current, target_index):
        # find nearest point on the interpolated trajectory
        N_indx_search = 8  # search for the next 10 point, this value should be close to the MPC horizon
        dx = [
            x_current[0] - icx
            for icx in x_ref[0][target_index : target_index + N_indx_search]
        ]
        dy = [
            x_current[2] - icy
            for icy in x_ref[2][target_index : target_index + N_indx_search]
        ]
        d = [idx**2 + idy**2 for (idx, idy) in zip(dx, dy)]
        mind = min(d)
        ind = d.index(mind) + target_index

        return ind

    def calculate_local_reference(self, x_ref, x_current, target_index=0):
        x_local_ref = np.zeros((self.state_dimension, self.N + 1))
        total_interpolated_points_len = len(x_ref[0])

        ind = self.nearest_interpolated_point(x_ref, x_current, target_index)

        if target_index >= ind:
            ind = target_index

        x_local_ref[0, 0] = x_ref[0, ind]
        x_local_ref[1, 0] = x_ref[1, ind]
        x_local_ref[2, 0] = x_ref[2, ind]
        x_local_ref[3, 0] = x_ref[3, ind]

        for i in range(self.N + 1):
            if (ind + self.N) < total_interpolated_points_len:
                x_local_ref[0, i] = x_ref[0, ind + i]
                x_local_ref[1, i] = x_ref[1, ind + i]
                x_local_ref[2, i] = x_ref[2, ind + i]
                x_local_ref[3, i] = x_ref[3, ind + i]
            else:
                x_local_ref[0, i] = x_ref[0, -1]
                x_local_ref[1, i] = x_ref[1, -1]
                x_local_ref[2, i] = x_ref[2, -1]
                x_local_ref[3, i] = x_ref[3, -1]

        return x_local_ref, ind

    def calculate_velocity_between_interpolated_positions(
        self, interpolated_x, interpolated_y, target_speed
    ):
        v_interpolated_x = np.zeros(len(interpolated_x))
        v_interpolated_y = np.zeros(len(interpolated_y))

        for i in range(len(interpolated_x) - 1):
            delta_interpolated_x = interpolated_x[i + 1] - interpolated_x[i]
            delta_interpolated_y = interpolated_y[i + 1] - interpolated_y[i]
            angle = np.arctan2(delta_interpolated_y, delta_interpolated_x)
            v_interpolated_x[i] = target_speed * np.cos(angle)
            v_interpolated_y[i] = target_speed * np.sin(angle)

        return v_interpolated_x, v_interpolated_y

    def waypoints_to_x_ref(
        self, waypoints, interpolated_dist, target_speed, interpolation_type="linear"
    ):
        if interpolation_type == "linear":
            rx, ry = [], []
            sp = spline_continuity.Spline2D(
                x=waypoints[:, [0]].flatten(),
                y=waypoints[:, [1]].flatten(),
                kind="linear",
            )
            s = np.arange(0, sp.s[-1], interpolated_dist)
            for i_s in s:
                ix, iy = sp.calc_position(i_s)
                rx.append(ix)
                ry.append(iy)

            interpolated_x1 = np.array(rx)
            interpolated_x3 = np.array(ry)

        elif interpolation_type == "cubic":
            (
                interpolated_x1,
                interpolated_x3,
                _,
                _,
                _,
            ) = cubic_spline_planner.calc_spline_course(
                x=waypoints[:, [0]].flatten(),
                y=waypoints[:, [1]].flatten(),
                ds=interpolated_dist,
            )  # ds is the distance between interpolated points

        x_ref = np.zeros((self.state_dimension, len(interpolated_x1)))
        # Fill reference trajectory with interpolated positions in x1 and x3
        x_ref[0, :] = interpolated_x1[:]
        x_ref[2, :] = interpolated_x3[:]

        # Calculate Velocity between interpolated positions and fill into the x_ref
        v_x2, v_x4 = self.calculate_velocity_between_interpolated_positions(
            interpolated_x1, interpolated_x3, target_speed
        )
        x_ref[1, :] = v_x2[:]
        x_ref[3, :] = v_x4[:]

        # Set the speed at the goal to be zero
        x_ref[1, -1] = 0.0
        x_ref[3, -1] = 0.0

        return x_ref
