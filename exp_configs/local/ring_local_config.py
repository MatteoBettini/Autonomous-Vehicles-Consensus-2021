"""Used as an example of ring experiment.

This example consists of 22 IDM cars on a ring creating shockwaves.
"""

from flow.controllers import IDMController, ContinuousRouter, FollowerStopper, NonLocalFollowerStopper, SimCarFollowingController
from flow.core.params import SumoParams, EnvParams, InitialConfig, NetParams, SumoCarFollowingParams
from flow.core.params import VehicleParams
from flow.envs.ring.accel import AccelEnv, ADDITIONAL_ENV_PARAMS
from flow.networks.ring import RingNetwork, ADDITIONAL_NET_PARAMS
import numpy as np

from controllers.consensus_controller import ConsensusLeaderController, ConsensusFollowerController

RING_RADIUS = 50
MAX_SPEED = 30
TARGET_SPEED = MAX_SPEED
MAX_ACCEL = 2
MAX_DECEL = MAX_ACCEL
N_VEHICLES = 22



vehicles = VehicleParams()
vehicles.add(
    veh_id="follower",
    acceleration_controller=(IDMController,{

    }),
    routing_controller=(ContinuousRouter, {}),
    num_vehicles=N_VEHICLES - 1,
    car_following_params=SumoCarFollowingParams(
        speed_dev=0,
        max_speed=MAX_SPEED,
        accel=MAX_ACCEL,
        decel=MAX_DECEL,
        speed_mode="aggressive"
    ),
)

vehicles.add(
    veh_id="leader",
    acceleration_controller=(IDMController, {
       
    }),
    routing_controller=(ContinuousRouter, {}),
    num_vehicles=1,
    car_following_params=SumoCarFollowingParams(
        speed_dev=0,
        max_speed=MAX_SPEED,
        accel=MAX_ACCEL,
        decel=MAX_DECEL,
        speed_mode="aggressive"
    ),
)


flow_params = dict(
    # name of the experiment
    exp_tag='ring_local',

    # name of the flow environment the experiment is running on
    env_name=AccelEnv,

    # name of the network class the experiment is running on
    network=RingNetwork,

    # simulator that is used by the experiment
    simulator='traci',

    # sumo-related parameters (see flow.core.params.SumoParams)
    sim=SumoParams(
        render=True,
        sim_step=0.1,
    ),

    # environment related parameters (see flow.core.params.EnvParams)
    env=EnvParams(
        horizon=5000,
        additional_params={
                # maximum acceleration for autonomous vehicles, in m/s^2
                'max_accel': MAX_ACCEL,
                # maximum deceleration for autonomous vehicles, in m/s^2
                'max_decel': MAX_DECEL,
                # desired velocity for all vehicles in the network, in m/s
                'target_velocity': TARGET_SPEED,
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
            "length": 2 * np.pi * RING_RADIUS,
            # number of lanes
            "lanes": 1,
            # speed limit for all edges
            "speed_limit": MAX_SPEED,
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
        spacing="uniform", # specifies the positioning of vehicles in the network relative to one another. May be one of: "uniform", "random", or "custom"
        perturbation=0.0,
        bunching=50
       )
)

