# - path to collocation trajectory (solution of NLP)
# - path to sampled trajectory

from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
import argparse

def main(args):
    # todo: support different algorithm and problem names
    filename_collocation_state_traj = "collocation-state-traj-trapezoidal-cartpole.csv"
    filename_collocation_ctrl_traj = "collocation-ctrl-traj-trapezoidal-cartpole.csv"
    filename_sample_state_traj = "sample-state-traj-trapezoidal-cartpole.csv"
    filename_sample_ctrl_traj = "sample-ctrl-traj-trapezoidal-cartpole.csv"

    # read data
    data_dir = Path(args.traj_data_dir)
    df_coll_state_traj = pd.read_csv(data_dir / filename_collocation_state_traj)
    df_sample_state_traj = pd.read_csv(data_dir / filename_sample_state_traj)

    num_joints = 2
    fig_ax_tuples = []
    joint_label = ["q", "dq", "ddq"]
    joint_title_names = ["Position", "Velocity", "Acceleration"]
    for i in range(len(joint_label)):
        fig, ax = plt.subplots()
        for j in range(num_joints):
            line, = ax.plot(df_coll_state_traj.time, df_coll_state_traj.iloc[:, i*num_joints + j + 1], 'o', label=joint_label[i] + str(j))
            ax.plot(df_sample_state_traj.time, df_sample_state_traj.iloc[:, i*num_joints + j + 1], color=line.get_color())

        ax.set_title(f"Trapezoidal Collocation Joint {joint_title_names[i]} Trajectory")
        ax.set_xlabel("time [s]")
        ax.grid(True)
        ax.legend()
        fig_ax_tuples.append((fig, ax))
    
    plt.show()


if __name__ == "__main__":
    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--traj-data-dir", help="Path to directory containing trajectory data files.", type=str, required=True)
    args = parser.parse_args()
    
    # run main with arguments
    main(args)
