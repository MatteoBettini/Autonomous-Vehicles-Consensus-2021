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
                 desired_headway=3,
                 k_p=.01,
                 k_d=.07,
                 k_i=.0005,
                 crash_faults=False,
                 byzantine_faults=False,
                 fail_safe=["obey_speed_limit", "feasible_accel"]
                 ):
        super().__init__(veh_id, car_following_params, fail_safe=fail_safe)

        self.v_max = v_max
        self.acc_max = acc_max
        self.decel_max = decel_max
        self.desired_headway = desired_headway

        self.prev_headway_error = 0
        self.prev_integral_value = 0

        self.k_p = k_p
        self.k_d = k_d
        self.k_i = k_i

        self.crash_faults = crash_faults
        self.byzantine_faults = byzantine_faults

        self.leader_id = None

    def get_accel(self, env: Env):
        if env.k.simulation.time == 0:
            self.leader_id = env.k.vehicle.get_leader(self.veh_id)

        # v = env.k.vehicle.get_speed(self.veh_id)

        h = env.k.vehicle.get_headway(self.veh_id)
        # v_lead = env.k.vehicle.get_speed(self.leader_id)
        # car_ids = env.k.vehicle.get_ids()
        # velocities = env.k.vehicle.get_speed(car_ids)
        # realised_accels = env.k.vehicle.get_realized_accel(car_ids)
        # headways = env.k.vehicle.get_headway(car_ids)

        headway_error = self.desired_headway - h
        integral_value = env.k.simulation.sim_step * headway_error + self.prev_integral_value

        acc = - self.k_p * headway_error - self.k_d * (headway_error - self.prev_headway_error) / env.k.simulation.sim_step - self.k_i * integral_value

        # print(f'VVehicel: {self.veh_id}, acc: {acc}, error: {headway_error}')
        self.prev_headway_error = headway_error
        self.prev_integral_value = integral_value

        time_in_seconds = round(env.k.simulation.time)

        if self.crash_faults and 100 < time_in_seconds < 150:
            acc = - self.decel_max

        return acc