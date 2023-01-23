import glob
import json
import subprocess

from utils import PathUtils
from plot_sim_results import plot_multiple_results

EXP_CONFIGS = ['ring_local_config',
               'ring_consensus_config']

if __name__ == '__main__':

    ring_configs = glob.glob(str(PathUtils.exp_configs_folder) + '/ring' + '/*.py')

    for config in ring_configs:

        config = config.rsplit('/', 1)[1].rsplit('.',1)[0]

        command = ['python',
                   PathUtils.run_ring_file,
                   '--exp_config', config,
                   #'--no_render',
                   '--no_plot_outcome'
                   ]

        process = subprocess.Popen(command,
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)

        output, error = process.communicate()

        if output:
            print(output.decode())
        if error:
            print(error.decode())


    with open(str(PathUtils.ring_json_file), 'r') as f:
        params_dict = json.load(f)
        f.close()

    plot_multiple_results(params_dict)