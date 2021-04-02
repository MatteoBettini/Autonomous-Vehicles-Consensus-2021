from flow.controllers import BaseController
from flow.envs import Env
import numpy as np



class BandoController(BaseController):
    """Bando follow-the-leader controller.

    Usage
    -----
    See BaseController for usage example.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.params.SumoCarFollowingParams
        see parent class
    alpha : float
        gain on desired velocity to current velocity difference
        (default: 0.6)
    v_max : float
        max velocity (default: 30)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 car_following_params,
                 alpha=2,
                 v_max=30,
                 dec_max=2,
                 want_max_accel=True,
                 time_delay=0.1,
                 noise=0,
                 crash_faults=False,
                 byzantine_faults=False,
                 fail_safe=None,
                 display_warnings=True):
        """Instantiate an Bando controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )
        self.veh_id = veh_id
        self.v_max = v_max
        self.alpha = alpha
        self.want_max_accel = want_max_accel
        self.leader_id = None
        self.dec_max = dec_max

        self.crash_faults = crash_faults
        self.byzantine_faults = byzantine_faults

    def get_accel(self, env: Env):
        """See parent class."""

        if env.k.simulation.time == 0:
            self.leader_id = env.k.vehicle.get_leader(self.veh_id)

        if not self.leader_id:  # no car ahead
            if self.want_max_accel:
                return self.max_accel

        v = env.k.vehicle.get_speed(self.veh_id)
        s = env.k.vehicle.get_headway(self.veh_id)

        assert v >= 0
        assert s >= 0

        accel = self.alpha * (self.optimal_velocity_function(s + env.k.vehicle.get_length(self.veh_id)) - v)

        time_in_seconds = round(env.k.simulation.time)


        if self.crash_faults and 100 < time_in_seconds < 150:
            accel = -self.dec_max


        return accel

    def optimal_velocity_function(self, s):
        """Compute the acceleration function."""
        v_optimal = np.tanh(s-2) + np.tanh(2)
        return v_optimal