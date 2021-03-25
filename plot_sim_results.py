import errno
import glob
import os
import pickle

from utils import *

try:
    from matplotlib import pyplot as plt
except ImportError:
    import matplotlib
    matplotlib.use('TkAgg')
    from matplotlib import pyplot as plt

import numpy as np
import pandas as pd

plt.rcParams['figure.figsize'] = (11.0, 5)
plt.rcParams['figure.facecolor'] = 'white'
plt.rcParams['figure.subplot.bottom'] = 0.125
plt.rcParams['figure.edgecolor'] = 'white'

plt.rcParams["savefig.dpi"] = 300
plt.rcParams["savefig.bbox"] = 'tight'
plt.rcParams["savefig.pad_inches"] = 0.1

plt.rcParams['font.size'] = 10
plt.rcParams['axes.labelsize'] = 10
plt.rcParams['xtick.labelsize'] = 10
plt.rcParams['ytick.labelsize'] = 10



def plot_sim_results(csv_file: str, params_dict, plot=True, folder_to_save=None):

    df, df_headway = get_datframe(csv_file)

    if df['speed'].mean() < 0:
        return

    title = get_title(params_dict)

    exp_tag = params_dict['exp_tag']
    accell_controller = exp_tag.rsplit('_',1)[1]

    directory_path = str(PathUtils.data_folder) + '/' + title
    create_directory(directory_path)

    with open(directory_path + '/' + exp_tag + '.pkl', "wb") as f:
        pickle.dump(df, f)

    if plot:

        columns = 2
        if not df_headway.empty:
            columns += 1

        fig, axs = plt.subplots(1, columns)

        axs[0].set_title('Avrage speed')
        axs[1].set_title('Avrage accelleration')

        axs[0].set(xlabel='Seconds', ylabel='m/s')
        axs[1].set(xlabel='Seconds', ylabel='m/$s^2$')



        if not df_headway.empty:
            axs[2].set_title('Leader headway error')
            axs[2].set(xlabel='Seconds', ylabel='Error')
            axs[2].plot(df_headway['headway'] - DefaultParams.TARGET_HEADWAY, label=accell_controller)

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

    fig, axs = plt.subplots(1, 2)

    axs[0].set_title('Avrage speed')
    axs[1].set_title('Avrage accelleration')

    axs[0].set(xlabel='Seconds', ylabel='m/s')
    axs[1].set(xlabel='Seconds', ylabel='m/$s^2$')

    for controller in controllers:

        with open(controller,'rb') as f:
            df = pickle.load(f)
            f.close()

        controller_name = controller.rsplit('/', 1)[1].rsplit('.', 1)[0].rsplit('_',1)[1]

        axs[0].plot(df['speed'], label=controller_name)
        axs[1].plot(df['realized_accel'], label=controller_name)


    files = glob.glob(str(PathUtils.data_folder) + '/' + title + '/*.pkl')
    for file in files:
        os.remove(file)


    for ax_v in axs:
        ax_v.legend()

    fig.suptitle(title, fontsize=12)
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
    df_mean = df.groupby("time")["speed","realized_accel","headway"].mean()
    df_headway_leader = df[df["id"] == "leader_0"][["time","headway"]]

    return df_mean.iloc[1:], df_headway_leader

def create_directory(path: str):
    try:
        os.mkdir(path)
    except OSError as exc:
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
