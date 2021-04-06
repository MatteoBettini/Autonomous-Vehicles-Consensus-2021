import errno
import glob
import os
import pickle

from IPython import get_ipython

from utils import *

try:
    from matplotlib import pyplot as plt
except ImportError:
    import matplotlib

    matplotlib.use('TkAgg')
    from matplotlib import pyplot as plt

import numpy as np
import pandas as pd

try:
    get_ipython().run_line_magic('matplotlib', 'inline')
except:
    pass

plt.rcParams['figure.figsize'] = (5, 4)
plt.rcParams['figure.facecolor'] = 'white'
plt.rcParams['figure.subplot.bottom'] = 0.125
plt.rcParams['figure.edgecolor'] = 'white'

plt.rcParams["savefig.dpi"] = 300
plt.rcParams["savefig.bbox"] = 'tight'
plt.rcParams["savefig.pad_inches"] = 0.1

plt.rcParams['font.size'] = 7
plt.rcParams['axes.labelsize'] = 7
plt.rcParams['xtick.labelsize'] = 7
plt.rcParams['ytick.labelsize'] = 7


def plot_sim_results(csv_file: str, params_dict, plot=True, folder_to_save=None):
    df = get_datframe(csv_file)

    if df['speed'].mean() < 0:
        return

    title = get_title(params_dict)

    exp_tag = params_dict['exp_tag']
    accell_controller = exp_tag.rsplit('_', 1)[1]

    directory_path = str(PathUtils.data_folder) + '/' + title
    create_directory(directory_path)

    with open(directory_path + '/' + exp_tag + '.pkl', "wb") as f:
        pickle.dump(df, f)

    if plot:

        fig, axs = plt.subplots(1, 2)

        axs[0].set_title('Average speed')
        axs[1].set_title('Average accelleration')

        axs[0].set(xlabel='Seconds', ylabel='m/s')
        axs[1].set(xlabel='Seconds', ylabel='m/$s^2$')

        # color = next(axs[0, 0]._get_lines.prop_cycler)['color']

        axs[0].plot(df['speed'], label=accell_controller)
        axs[1].plot(df['realized_accel'], label=accell_controller)

        for ax_v in axs:
            ax_v.legend()

        fig.suptitle(title, fontsize=12)
        fig.canvas.set_window_title(title)

        if folder_to_save:
            fig.savefig(folder_to_save + '/' + title + '/' + 'result.pdf')

        plt.show()


def plot_multiple_results(params_dict):
    title = get_title(params_dict)

    controllers = glob.glob(str(PathUtils.data_folder) + '/' + title + '/*.pkl')

    fig, axs = plt.subplots(1, 1)

    axs.set_title('Average headway')
    # axs[1].set_title('Avrage accelleration')

    axs.set(xlabel='Seconds', ylabel='m/s')
    # axs[1].set(xlabel='Seconds', ylabel='m/$s^2$')

    for controller in controllers:

        with open(controller, 'rb') as f:
            df = pickle.load(f)
            f.close()

        controller_name = controller.rsplit('/', 1)[1].rsplit('.', 1)[0].rsplit('_', 1)[1]

        if controller_name == 'LLVC' or controller_name == 'PIDHeadway':
            controller_name += ' (our)'

        axs.plot(df['speed'], label=controller_name)
        # axs.plot(df['headway'], label=controller_name)
        # axs[1].plot(df['realized_accel'], label=controller_name)

    files = glob.glob(str(PathUtils.data_folder) + '/' + title + '/*.pkl')
    for file in files:
        os.remove(file)

    # for ax_v in axs:
    axs.legend(loc='lower right', prop={'size': 8})

    # fig.suptitle(title, fontsize=12)
    fig.canvas.set_window_title(title)

    fig.savefig(str(PathUtils.data_folder) + '/' + title + '/' + 'result.pdf')

    plt.show()


def get_title(params_dict):
    # Get params
    max_accel = params_dict['veh'][0]['car_following_params']['controller_params']['accel']
    max_decel = params_dict['veh'][0]['car_following_params']['controller_params']['decel']
    accell_controller = params_dict['veh'][0]['acceleration_controller'][0]
    num_vehicles = 0
    for vehicle in params_dict['veh']:
        num_vehicles += vehicle['num_vehicles']
    max_speed = params_dict['veh'][0]['car_following_params']['controller_params']['maxSpeed']
    target_speed = params_dict['env']['additional_params']['target_velocity']
    network_name = params_dict["network"].split('.')[-1]
    radius = params_dict['net']['additional_params']['length'] / (2 * np.pi)

    title = f'Network={network_name}, Radius={round(radius, 1)}, Max speed={max_speed}, Max accel-decel={max_accel}, Num vehicles={num_vehicles}'

    return title


def get_datframe(csv_name: str):
    df = pd.read_csv(csv_name)
    df_mean = df.groupby("time")[["speed", "realized_accel"]].mean()
    df_headway = df[df["id"] != "fault_vehicle_0"][["time", "headway", "speed"]]
    df_headway = df_headway.groupby("time")[["headway", "speed"]].mean()
    df_mean = df_mean.join(df_headway["headway"])

    return df_mean.iloc[1:]


def create_directory(path: str):
    try:
        os.mkdir(path)
    except OSError as exc:
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
