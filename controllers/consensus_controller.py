from flow.controllers import BaseController
from flow.envs import Env
import numpy as np

from utils import DefaultParams


class ConsensusController(BaseController):

    def __init__(self,
                 veh_id,
                 car_following_params,
                 v_max=30,
                 acc_max=2,
                 decel_max=2,
                 s0=2,
                 v_inc=0.1,
                 n_hops=-1,
                 tau=0.12,
                 crash_faults=False,
                 byzantine_faults=False,
                 fail_safe=["obey_speed_limit", "feasible_accel"]
                 ):

        super().__init__(veh_id, car_following_params, fail_safe=fail_safe)

        self.v_max = v_max
        self.acc_max = acc_max
        self.decel_max = -decel_max
        self.decel_max_leader = -decel_max
        self.v_inc = v_inc
        self.s0 = s0
        self.tau = tau
        self.n_hops = n_hops

        self.crash_faults = crash_faults
        self.byzantine_faults = byzantine_faults

        self.neighbours_ids = []
        self.leader_id = None
        self.follower_id = None

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
        # h = env.k.vehicle.get_headway(self.veh_id)
        #
        assert v >= 0
        # assert h >= 0

        velocities = env.k.vehicle.get_speed(self.neighbours_ids)
        # realised_accels = env.k.vehicle.get_realized_accel(car_ids)
        # headways = env.k.vehicle.get_headway(car_ids)

        for v in velocities:
            assert v >= 0
        # for a in realised_accels:
        #     assert v >= 0
        # for h in headways:
        #     assert v >= 0

        velocity = np.mean(velocities) + self.v_inc
        # velocity = v + 0.1 * np.sum(np.array(velocities) - v) + 0.3 * (self.v_max - v)

        time_in_seconds = round(env.k.simulation.time)

        if self.byzantine_faults:
            print(time_in_seconds)
            if int(time_in_seconds / 10) % 2 == 0:
                velocity = velocity + 1
            else:
                velocity = velocity - 1

        if self.crash_faults and 100 < time_in_seconds < 150:
            velocity = 0

        acc = self.get_gipps_accel(env, velocity)

        return acc

    def get_gipps_accel(self, env, goal_velocity):
        v = env.k.vehicle.get_speed(self.veh_id)
        h = env.k.vehicle.get_headway(self.veh_id)
        v_l = env.k.vehicle.get_speed(self.leader_id)

        assert v >= 0
        assert h >= 0
        assert v_l >= 0

        # get velocity dynamics
        v_safe = (self.tau * self.decel_max) + np.sqrt(
            ((self.tau ** 2) * (self.decel_max ** 2)) - (
                        self.decel_max * ((2 * (h - self.s0)) - (self.tau * v) - (
                            (v_l ** 2) / self.decel_max_leader))))

        assert not np.isnan(v_safe)
        # if np.isnan(v_safe):
        #     v_safe = 0

        v_next = max(0, min(goal_velocity, v_safe, self.v_max))

        return (v_next - v) / env.sim_step
