from flow.controllers import BaseController
from flow.envs import Env
import numpy as np

class PIDHeadwayController(BaseController):

    def __init__(self,
                 veh_id,
                 car_following_params,
                 v_max=30,
                 acc_max=2,
                 decel_max=2,
                 desired_headway=5,
                 fail_safe=["obey_speed_limit", "feasible_accel"]
                 ):

        print('Desired headway= ' + str(desired_headway))
        super().__init__(veh_id, car_following_params, fail_safe=fail_safe)
        self.v_max = v_max
        self.acc_max = acc_max
        self.decel_max = decel_max
        self.prev_headway_error = 0
        self.prev_integral_value = 0
        self.desired_headway = desired_headway
        self.k_p = .1
        self.k_d = .07
        self.k_i = .0005

        # self.k_p = 2.5
        # self.k_d = 2.2
        # self.k_i = 0.001

        # self.k_p = 5
        # self.k_d = 2.4
        # self.k_i = 0.1

    def get_accel(self, env: Env):

        v = env.k.vehicle.get_speed(self.veh_id)
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        h = env.k.vehicle.get_headway(self.veh_id)
        car_ids = env.k.vehicle.get_ids()
        velocities = env.k.vehicle.get_speed(car_ids)
        realised_accels = env.k.vehicle.get_realized_accel(car_ids)
        headways = env.k.vehicle.get_headway(car_ids)

        headway_error = h - self.desired_headway
        self.prev_headway_error = headway_error



        integral_value = env.k.simulation.sim_step * headway_error + self.prev_integral_value
        self.prev_integral_value = integral_value

        acc = self.k_p * headway_error + self.k_d * (headway_error - self.prev_headway_error) / env.k.simulation.sim_step + self.k_i * integral_value

        print(self.veh_id + ' acc: ' + str(acc))

        return acc

class ConsensusController(BaseController):

    def __init__(self,
                 veh_id,
                 car_following_params,
                 v_max=30,
                 acc_max=2,
                 decel_max=2,
                 v_inc=0,
                 n_hops=-1,
                 fail_safe=["obey_speed_limit", "feasible_accel"]
                 ):

        super().__init__(veh_id, car_following_params, fail_safe=fail_safe)

        self.v_max = v_max
        self.acc_max = acc_max
        self.decel_max = decel_max
        self.v_inc = v_inc
        self.n_hops = n_hops

    def get_accel(self, env: Env):

        v = env.k.vehicle.get_speed(self.veh_id)
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        h = env.k.vehicle.get_headway(self.veh_id)

        num_vehicles = env.k.vehicle.num_vehicles

        car_ids = []
        if self.n_hops == -1 or self.n_hops >= num_vehicles - 1:
            car_ids = env.k.vehicle.get_ids()
        else:
            assert self.n_hops > 0
            car_ids.append(self.veh_id)
            curr_id_l = self.veh_id
            curr_id_f = self.veh_id
            for i in range(self.n_hops):
                curr_id_l = env.k.vehicle.get_leader(curr_id_l)
                curr_id_f = env.k.vehicle.get_follower(curr_id_f)
                car_ids.append(curr_id_l)
                car_ids.append(curr_id_f)

        # car_ids.sort()

        velocities = env.k.vehicle.get_speed(car_ids)
        realised_accels = env.k.vehicle.get_realized_accel(car_ids)
        headways = env.k.vehicle.get_headway(car_ids)

        goal_acc = np.mean(realised_accels)

        goal_velocity = np.mean(velocities) + self.v_inc
        acc = (goal_velocity - v) / env.k.simulation.sim_step

        return acc