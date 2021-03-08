from flow.controllers import BaseController
from flow.envs import Env
import numpy as np

class ConsensusLeaderController(BaseController):

    def __init__(self, veh_id, car_following_params):
        super().__init__(veh_id, car_following_params, fail_safe=["obey_speed_limit", "feasible_accel", "safe_velocity"])

    def get_accel(self, env: Env):
        return 0.03


class ConsensusFollowerController(BaseController):

    def __init__(self, veh_id, car_following_params):
        super().__init__(veh_id, car_following_params, fail_safe=["obey_speed_limit", "feasible_accel", "safe_velocity"])

    def get_accel(self, env: Env):
        return np.mean([env.k.vehicle.get_accel(vehicle_id) for vehicle_id in env.k.vehicle.get_ids() if env.k.vehicle.get_accel(vehicle_id) is not None]) if np.mean([env.k.vehicle.get_accel(vehicle_id) for vehicle_id in env.k.vehicle.get_ids() if env.k.vehicle.get_accel(vehicle_id) is not None]) > 0 else 0