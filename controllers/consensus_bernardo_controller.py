from flow.controllers import BaseController
from flow.envs import Env
import numpy as np

from utils import DefaultParams


class ConsensusBernardoController(BaseController):

    def __init__(self,
                 veh_id,
                 car_following_params,
                 v_max=30,
                 acc_max=2,
                 decel_max=2,
                 s0=2,
                 n_hops=-1,
                 crash_faults=False,
                 byzantine_faults=False,
                 fail_safe=["obey_speed_limit", "feasible_accel"]
                 ):

        super().__init__(veh_id, car_following_params, fail_safe=fail_safe)

        self.v_max = v_max
        self.acc_max = acc_max
        self.decel_max = decel_max
        self.decel_max_leader = decel_max
        self.s0 = s0
        self.n_hops = n_hops

        self.neighbours_ids = []
        self.leader_id = None
        self.follower_id = None

        self.crash_faults = crash_faults
        self.byzantine_faults = byzantine_faults

    def get_accel(self, env: Env):

        if env.k.simulation.time == 0:
            num_vehicles = env.k.vehicle.num_vehicles

            self.leader_id = env.k.vehicle.get_leader(self.veh_id)
            self.follower_id = env.k.vehicle.get_follower(self.veh_id)

            if self.n_hops == -1 or self.n_hops > (num_vehicles - 1) / 2:
                self.neighbours_ids = env.k.vehicle.get_ids().copy()
                self.neighbours_ids.remove(self.veh_id)
            else:
                assert self.n_hops > 0
                curr_id_l = self.veh_id
                curr_id_f = self.veh_id
                for i in range(self.n_hops):
                    curr_id_l = env.k.vehicle.get_leader(curr_id_l)
                    curr_id_f = env.k.vehicle.get_follower(curr_id_f)
                    #  print(f'Vehicle {self.veh_id}, leader is {curr_id_l}, follower is {curr_id_f}')
                    self.neighbours_ids.append(curr_id_l)
                    self.neighbours_ids.append(curr_id_f)

            assert None not in self.neighbours_ids
            assert self.leader_id is not None
            assert self.follower_id is not None


        v = env.k.vehicle.get_speed(self.veh_id)
        # d = env.k.vehicle.get_distance(self.veh_id)
        # h = env.k.vehicle.get_headway(self.veh_id)

        if 'leader_0' in self.neighbours_ids:
            self.neighbours_ids.remove('leader_0')

        leader_speed = env.k.vehicle.get_speed('leader_0')

        # velocities = env.k.vehicle.get_speed(car_ids)
        # realised_accels = env.k.vehicle.get_realized_accel(car_ids)
        # distances = env.k.vehicle.get_distance(car_ids)
        headways = env.k.vehicle.get_headway(self.neighbours_ids)

        for headway in headways:
            assert headway >= 0


        acc = - 0.1 * (v - leader_speed) - 0.05 * (np.mean(headways) + env.k.vehicle.get_length(self.veh_id) - 0.5 * self.v_max)

        print(f'Vehicle {self.veh_id}, headway error {(np.mean(headways) + env.k.vehicle.get_length(self.veh_id) - 1 * self.v_max)}')

        time_in_seconds = round(env.k.simulation.time)

        if self.crash_faults and 100 < time_in_seconds < 150:
            acc = -self.decel_max

        return acc
