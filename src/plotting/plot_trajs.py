# - path to collocation trajectory (solution of NLP)
# - path to sampled trajectory

from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
import argparse

def plot_state_traj(path_collocation_state_traj: Path,
                    path_sample_state_traj: Path,
                    algorithm_name: str):
    """Plot joint state trajectories."""
    # read data
    df_coll_state_traj = pd.read_csv(path_collocation_state_traj)
    df_sample_state_traj = pd.read_csv(path_sample_state_traj)

    # each row is: [time, q, dq, ddq] where q, dq, and ddq are row vectors, each
    # with as many elements as there are joints
    
    num_joints = int((len(df_coll_state_traj.columns) - 1) / 3)
    
    fig_ax_tuples = []
    joint_label = ["q", "dq", "ddq"]
    joint_title_names = ["Position", "Velocity", "Acceleration"]
    for i in range(len(joint_label)):
        fig, ax = plt.subplots()
        for j in range(num_joints):
            line, = ax.plot(df_coll_state_traj.time, df_coll_state_traj.iloc[:, i*num_joints + j + 1], 'o', label="{}[{}]".format(joint_label[i], j))
            ax.plot(df_sample_state_traj.time, df_sample_state_traj.iloc[:, i*num_joints + j + 1], color=line.get_color())

        ax.set_title(f"{algorithm_name} Collocation Joint {joint_title_names[i]} Trajectory")
        ax.set_xlabel("time [s]")
        ax.grid(True)
        ax.legend()
        fig_ax_tuples.append((fig, ax))
    return fig_ax_tuples
    

def plot_ctrl_traj(path_collocation_ctrl_traj,
                   path_sample_ctrl_traj,
                   algorithm_name):
    df_coll_ctrl_traj = pd.read_csv(path_collocation_ctrl_traj)
    df_sample_ctrl_traj = pd.read_csv(path_sample_ctrl_traj)

    # each row is: [time, u] where u is a row vector with as many elements as
    # there are control input elements (no more than the number of joints)
    num_ctrls = len(df_coll_ctrl_traj.columns) - 1
    
    fig, ax = plt.subplots()
    for i in range(num_ctrls):
        line, = ax.plot(df_coll_ctrl_traj.time, df_coll_ctrl_traj.iloc[:, i+1], 'o', label="u[{}]".format(i))
        ax.plot(df_sample_ctrl_traj.time, df_sample_ctrl_traj.iloc[:, i+1], color=line.get_color())
    ax.set_title(f"{algorithm_name} Collocation Joint Control Trajectory")
    ax.set_xlabel("time [s]")
    ax.grid(True)
    ax.legend()
    return (fig, ax)


def main():
    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--traj-data-dir", help="Path to directory containing trajectory data files.", type=Path, required=True)
    parser.add_argument("--algorithm", help="Type of algorithm.", type=str, choices=["Trapezoidal", "Hermite-Simpson"], default="Trapezoidal")
    parser.add_argument("--model", help="Type of model.", type=str, choices=["cartpole", "so101"], default="Trapezoidal")
    args = parser.parse_args()

    # todo: support different algorithm and problem names
    algs_lower = args.algorithm.lower()
    filename_collocation_state_traj = f"collocation-state-traj-{algs_lower}-{args.model}.csv"
    filename_collocation_ctrl_traj = f"collocation-ctrl-traj-{algs_lower}-{args.model}.csv"
    filename_sample_state_traj = f"sample-state-traj-{algs_lower}-{args.model}.csv"
    filename_sample_ctrl_traj = f"sample-ctrl-traj-{algs_lower}-{args.model}.csv"

    fig_ax_tuples = plot_state_traj(
        path_collocation_state_traj=args.traj_data_dir / filename_collocation_state_traj,
        path_sample_state_traj=args.traj_data_dir / filename_sample_state_traj,
        algorithm_name=args.algorithm
    )

    fig_ax_tuples += plot_ctrl_traj(
        path_collocation_ctrl_traj=args.traj_data_dir / filename_collocation_ctrl_traj,
        path_sample_ctrl_traj=args.traj_data_dir / filename_sample_ctrl_traj,
        algorithm_name=args.algorithm
    )
    
    plt.show()


if __name__ == "__main__":    
    main()
