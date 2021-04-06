"""Used as an example of ring experiment.

This example consists of 22 IDM cars on a ring creating shockwaves.
"""

from flow.controllers import *
from flow.core.params import SumoParams, EnvParams, InitialConfig, NetParams, SumoCarFollowingParams
from flow.core.params import VehicleParams
from flow.envs.ring.accel import AccelEnv, ADDITIONAL_ENV_PARAMS
from flow.networks.ring import RingNetwork, ADDITIONAL_NET_PARAMS
import numpy as np

from controllers.pid_headway_controller import PIDHeadwayController
from utils import DefaultParams

from controllers.consensus_controller import *



vehicles = VehicleParams()

vehicles.add(
    veh_id="follower",
    acceleration_controller=(ConsensusController, {
        'v_max': DefaultParams.TARGET_SPEED,
        'acc_max': DefaultParams.MAX_ACCEL,
        'decel_max': DefaultParams.MAX_DECEL,
        'v_inc': 0,
        'n_hops': DefaultParams.N_HOPS,
        'crash_faults': False,
        'byzantine_faults': False,
        'fail_safe': DefaultParams.FAIL_SAFES
    }),
    routing_controller=(ContinuousRouter, {}),
    num_vehicles=DefaultParams.N_VEHICLES - 1,
    car_following_params=SumoCarFollowingParams(
        speed_dev=0,
        max_speed=DefaultParams.MAX_SPEED,
        accel=DefaultParams.MAX_ACCEL,
        decel=DefaultParams.MAX_DECEL,
        speed_mode="aggressive"
    ),
)

vehicles.add(
    veh_id="leader",
    acceleration_controller=(PIDHeadwayController, {
        'v_max': DefaultParams.TARGET_SPEED,
        'acc_max': DefaultParams.MAX_ACCEL,
        'decel_max': DefaultParams.MAX_DECEL,
        'desired_headway': DefaultParams.TARGET_HEADWAY,
        'crash_faults': DefaultParams.N_BROKEN_VEHICLES > 0,
        'k_p': 0.2,
        'k_d': 1.3,
        'k_i': 0.0005,
        'fail_safe': DefaultParams.FAIL_SAFES
    }),
    routing_controller=(ContinuousRouter, {}),
    num_vehicles=1,
    car_following_params=SumoCarFollowingParams(
        speed_dev=0,
        max_speed=DefaultParams.MAX_SPEED,
        accel=DefaultParams.MAX_ACCEL,
        decel=DefaultParams.MAX_DECEL,
        speed_mode="aggressive"
    ),
    color='green'
)



flow_params = dict(
    # name of the experiment
    exp_tag='ring_ConsesusPIDHeadwayLead',

    # name of the flow environment the experiment is running on
    env_name=AccelEnv,

    # name of the network class the experiment is running on
    network=RingNetwork,

    # simulator that is used by the experiment
    simulator='traci',

    # sumo-related parameters (see flow.core.params.SumoParams)
    sim=SumoParams(
        render=True,
        sim_step=DefaultParams.SIM_STEP,
    ),

    # environment related parameters (see flow.core.params.EnvParams)
    env=EnvParams(
        horizon=int(DefaultParams.DURATION / DefaultParams.SIM_STEP),  # Nu,ber of steps
        additional_params={
            # maximum acceleration for autonomous vehicles, in m/s^2
            'max_accel': DefaultParams.MAX_ACCEL,
            # maximum deceleration for autonomous vehicles, in m/s^2
            'max_decel': DefaultParams.MAX_DECEL,
            # desired velocity for all vehicles in the network, in m/s
            'target_velocity': DefaultParams.TARGET_SPEED,
            # specifies whether vehicles are to be sorted by position during a
            # simulation step. If set to True, the environment parameter
            # self.sorted_ids will return a list of all vehicles sorted in accordance
            # with the environment
            'sort_vehicles': False
        },
    ),

    # network-related parameters (see flow.core.params.NetParams and the
    # network's documentation or ADDITIONAL_NET_PARAMS component)
    net=NetParams(
        additional_params={
            # length of the ring road
            "length": 2 * np.pi * DefaultParams.RING_RADIUS,
            # number of lanes
            "lanes": 1,
            # speed limit for all edges
            "speed_limit": DefaultParams.MAX_SPEED,
            # resolution of the curves on the ring
            "resolution": 40
        },
    ),

    # vehicles to be placed in the network at the start of a rollout (see
    # flow.core.params.VehicleParams)
    veh=vehicles,

    # parameters specifying the positioning of vehicles upon initialization/
    # reset (see flow.core.params.InitialConfig)
    initial=InitialConfig(
        spacing="uniform",
        # specifies the positioning of vehicles in the network relative to one another. May be one of: "uniform", "random", or "custom"
        perturbation=DefaultParams.PERTURBATION,
        bunching=DefaultParams.BUNCHING
    )
)
