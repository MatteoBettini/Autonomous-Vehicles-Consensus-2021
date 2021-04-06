"""Runner script for non-RL simulations in flow.

Usage
    python simulate.py EXP_CONFIG --no_render
"""
import argparse
import glob
import sys
import json
import os
from flow.core.experiment import Experiment


from flow.utils.rllib import FlowParamsEncoder

from plot_sim_results import plot_sim_results
from utils import PathUtils, get_project_root


def parse_args(args):
    """Parse training options user can specify in command line.

    Returns
    -------
    argparse.Namespace
        the output parser object
    """
    parser = argparse.ArgumentParser(
        description="Parse argument used when running a Flow simulation.",
        epilog="python run_ring.py EXP_CONFIG --num_runs INT --no_render")

    # required input parameters
    parser.add_argument(
        '--exp_config', type=str, default='ring_consensus_leaderless_config',
        help='Name of the experiment configuration file, as located in '
             'exp_configs/local.')

    # optional input parameters
    parser.add_argument(
        '--num_runs', type=int, default=1,
        help='Number of simulations to run. Defaults to 1.')
    parser.add_argument(
        '--no_render',
        action='store_true',
        help='Specifies whether to run the simulation during runtime.')
    parser.add_argument(
        '--aimsun',
        action='store_true',
        help='Specifies whether to run the simulation using the simulator '
             'Aimsun. If not specified, the simulator used is SUMO.')
    parser.add_argument(
        '--no_gen_emission',
        action='store_true',
        help='Specifies whether to generate an emission file from the '
             'simulation.')
    parser.add_argument(
        '--no_plot_outcome',
        action='store_true',
        help='Specifies wheather to plot the speed and accellerations of the experiment'
    )

    return parser.parse_known_args(args)[0]


if __name__ == "__main__":
    flags = parse_args(sys.argv[1:])

    os.chdir(str(PathUtils.data_folder))
    files = glob.glob(str(PathUtils.data_folder) + '/*.csv')
    for file in files:
        os.remove(file)
    os.chdir(str(get_project_root()))

    # Get the flow_params object.
    module = __import__("exp_configs.ring", fromlist=[flags.exp_config])
    flow_params = getattr(module, flags.exp_config).flow_params

    # Get the custom callables for the runner.
    if hasattr(getattr(module, flags.exp_config), "custom_callables"):
        callables = getattr(module, flags.exp_config).custom_callables
    else:
        callables = None

    flow_params['sim'].render = not flags.no_render
    flow_params['simulator'] = 'traci'


    # Specify an emission path if they are meant to be generated.
    if not flags.no_gen_emission:
        flow_params['sim'].emission_path = "./data"

        # Create the flow_params object
        fp_ = flow_params['exp_tag']
        dir_ = flow_params['sim'].emission_path
        with open(os.path.join(dir_, "{}.json".format('ring')), 'w') as outfile:
            json.dump(flow_params, outfile,
                      cls=FlowParamsEncoder, sort_keys=True, indent=4)
            outfile.close()
        with open(os.path.join(dir_, "{}.json".format('ring')), 'r') as outfile:
            params_dict = json.load(outfile)
            outfile.close()

    # Create the experiment object.
    exp = Experiment(flow_params, callables)

    # Run for the specified number of rollouts.
    returns = exp.run(flags.num_runs, convert_to_csv=not flags.no_gen_emission)

    print(params_dict['exp_tag'])

    if not flags.no_gen_emission:
        plot_sim_results(str(PathUtils.data_folder) + '/' + returns["emissions_file_name"], params_dict, folder_to_save=str(PathUtils.data_folder), plot=not flags.no_plot_outcome)
