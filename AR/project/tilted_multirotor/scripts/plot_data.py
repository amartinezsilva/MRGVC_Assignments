import sys
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import numpy as np

def quaternion_to_euler_angles(q):
    # Quaternion to Euler angles conversion
    # Adapted from: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

    sinr_cosp = 2.0 * (q[0] * q[1] + q[2] * q[3])
    cosr_cosp = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q[0] * q[2] - q[3] * q[1])
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2])
    cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def plot_3d_scatter(data_frame_ref, data_frame_experiment, cols_ref, cols_experiment):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(np.array(data_frame_ref[cols_ref[0]]), np.array(data_frame_ref[cols_ref[1]]), abs(np.array(data_frame_ref[cols_ref[2]])), c='r', label = 'reference')
    ax.plot(np.array(data_frame_experiment[cols_experiment[0]]), np.array(data_frame_experiment[cols_experiment[1]]), abs(np.array(data_frame_experiment[cols_experiment[2]])), c='b', label = 'UAV pose')

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend()

    plt.savefig('pose_3D.png', bbox_inches='tight')

    plt.show()

def plot_pose_temporal_evolution(df_ref, df_experiment, cols_ref, cols_experiment):
    

    fig, axes = plt.subplots(3, 1, figsize=(10, 15))
    plt.suptitle("Temporal Evolution of 3D Pose")

    # Subtract the first element of the timestamp column to start from 0
    df_experiment[cols_experiment[0]] -= df_experiment[cols_experiment[0]].iloc[0]
    df_ref[cols_ref[0]] -= df_ref[cols_ref[0]].iloc[0]

    # Subtract the first element of the timestamp column to start from 0
    df_experiment[cols_experiment[0]] *= 1e-6
    df_ref[cols_ref[0]] *= 1e-6

    axes[0].plot(np.array(df_experiment[cols_experiment[0]]), np.array(df_experiment[cols_experiment[1]]), c='b', label = 'state')
    axes[0].plot(np.array(df_ref[cols_ref[0]]), np.array(df_ref[cols_ref[1]]), c='r', label = 'reference')
    axes[0].legend()
    #plt.xlabel("Timestamp")
    axes[0].set_ylabel("X(m)")
    axes[0].grid()
    axes[1].plot(np.array(df_experiment[cols_experiment[0]]), np.array(df_experiment[cols_experiment[2]]), c='b')
    axes[1].plot(np.array(df_ref[cols_ref[0]]), np.array(df_ref[cols_ref[2]]), c='r')
    axes[1].set_ylabel("Y(m)")
    axes[1].grid()

    axes[2].plot(np.array(df_experiment[cols_experiment[0]]), np.array(abs(df_experiment[cols_experiment[3]])), c='b')
    axes[2].plot(np.array(df_ref[cols_ref[0]]), np.array(abs(df_ref[cols_ref[3]])), c='r')
    axes[2].set_ylabel("Z(m)")
    axes[2].grid()

    axes[2].set_xlabel("Time(s)")


    plt.savefig('pose_t.png', bbox_inches='tight')

    plt.show()


def plot_velocity_temporal_evolution(df_ref, df_experiment, cols_ref, cols_experiment):
    

    fig, axes = plt.subplots(3, 1, figsize=(10, 15))
    plt.suptitle("Temporal Evolution of velocities")

    # Subtract the first element of the timestamp column to start from 0
    df_experiment[cols_experiment[0]] -= df_experiment[cols_experiment[0]].iloc[0]
    df_ref[cols_ref[0]] -= df_ref[cols_ref[0]].iloc[0]

    # Subtract the first element of the timestamp column to start from 0
    df_experiment[cols_experiment[0]] *= 1e-6
    df_ref[cols_ref[0]] *= 1e-6

    axes[0].plot(np.array(df_experiment[cols_experiment[0]]), np.array(df_experiment[cols_experiment[1]]), c='b', label = 'state')
    axes[0].plot(np.array(df_ref[cols_ref[0]]), np.array(df_ref[cols_ref[1]]), c='r', label = 'reference')
    axes[0].legend()
    #plt.xlabel("Timestamp")
    axes[0].set_ylabel("vx(m/s)")
    axes[0].grid()
    axes[1].plot(np.array(df_experiment[cols_experiment[0]]), np.array(df_experiment[cols_experiment[2]]), c='b')
    axes[1].plot(np.array(df_ref[cols_ref[0]]), np.array(df_ref[cols_ref[2]]), c='r')
    axes[1].set_ylabel("vy(m/s)")
    axes[1].grid()

    axes[2].plot(np.array(df_experiment[cols_experiment[0]]), np.array(abs(df_experiment[cols_experiment[3]])), c='b')
    axes[2].plot(np.array(df_ref[cols_ref[0]]), np.array(abs(df_ref[cols_ref[3]])), c='r')
    axes[2].set_ylabel("vz(m/s)")
    axes[2].grid()

    axes[2].set_xlabel("Time(s)")

    plt.savefig('velocity.png', bbox_inches='tight')

    plt.show()


def plot_attitude_temporal_evolution(df_ref_rp, df_ref_yaw, df_experiment, cols_ref_rp, cols_ref_yaw, cols_experiment):
    

    fig, axes = plt.subplots(3, 1, figsize=(10, 15))
    plt.suptitle("Temporal Evolution of attitude")

    # Subtract the first element of the timestamp column to start from 0
    df_experiment[cols_experiment[0]] -= df_experiment[cols_experiment[0]].iloc[0]
    df_ref_rp[cols_ref_rp[0]] -= df_ref_rp[cols_ref_rp[0]].iloc[0]
    df_ref_yaw[cols_ref_yaw[0]] -= df_ref_yaw[cols_ref_yaw[0]].iloc[0]


    # Subtract the first element of the timestamp column to start from 0
    df_experiment[cols_experiment[0]] *= 1e-6
    df_ref_rp[cols_ref_rp[0]] *= 1e-6
    df_ref_yaw[cols_ref_yaw[0]] *= 1e-6

    axes[0].plot(np.array(df_experiment[cols_experiment[0]]), np.array(df_experiment[cols_experiment[1]]), c='b', label = 'state')
    axes[0].plot(np.array(df_ref_rp[cols_ref_rp[0]]), np.array(df_ref_rp[cols_ref_rp[1]]), c='r', label = 'reference')
    axes[0].legend()
    #plt.xlabel("Timestamp")
    axes[0].set_ylabel("Roll(rad)")
    axes[0].grid()
    axes[1].plot(np.array(df_experiment[cols_experiment[0]]), np.array(df_experiment[cols_experiment[2]]), c='b')
    axes[1].plot(np.array(df_ref_rp[cols_ref_rp[0]]), np.array(df_ref_rp[cols_ref_rp[2]]), c='r')
    axes[1].set_ylabel("Pitch(rad)")
    axes[1].grid()

    axes[2].plot(np.array(df_experiment[cols_experiment[0]]), np.array(df_experiment[cols_experiment[3]]), c='b')
    axes[2].plot(np.array(df_ref_yaw[cols_ref_yaw[0]]), np.array(df_ref_yaw[cols_ref_yaw[1]]), c='r')
    axes[2].set_ylabel("Yaw(rad)")
    axes[2].grid()

    axes[2].set_xlabel("Time(s)")

    plt.savefig('attitude_t.png', bbox_inches='tight')

    plt.show() 

def plot_experiment_data(path_interp, path_experiment_data):

    #Get column names from interpolated trajectory file 
    x_interp = 'x'
    y_interp = 'y'
    z_interp = 'z'
    
    #Get names of the topics we want to plot
    time_data = '/fmu/vehicle_odometry/out/timestamp'
    x_data = '/fmu/vehicle_odometry/out/x'
    y_data = '/fmu/vehicle_odometry/out/y'
    z_data = '/fmu/vehicle_odometry/out/z'
    q0 = '/fmu/vehicle_odometry/out/q.0'
    q1 = '/fmu/vehicle_odometry/out/q.1'
    q2 = '/fmu/vehicle_odometry/out/q.2'
    q3 = '/fmu/vehicle_odometry/out/q.3'
    vx_data = '/fmu/vehicle_odometry/out/vx'
    vy_data = '/fmu/vehicle_odometry/out/vy'
    vz_data = '/fmu/vehicle_odometry/out/vz'

    time_data_setpoint = '/fmu/trajectory_setpoint/in/timestamp'
    x_ref = '/fmu/trajectory_setpoint/in/x'
    y_ref = '/fmu/trajectory_setpoint/in/y'
    z_ref = '/fmu/trajectory_setpoint/in/z'
    yaw_ref = '/fmu/trajectory_setpoint/in/yaw'
    vx_ref = '/fmu/trajectory_setpoint/in/vx'
    vy_ref = '/fmu/trajectory_setpoint/in/vy'
    vz_ref = '/fmu/trajectory_setpoint/in/vz'


    time_data_tilt = '/fmu/tilting_mc_desired_angles/in/timestamp'
    roll_ref = '/fmu/tilting_mc_desired_angles/in/roll_body'
    pitch_ref = '/fmu/tilting_mc_desired_angles/in/pitch_body'

    # Plot 3D representation

    columns_3D_experiment_data = [x_data, y_data, z_data]
    columns_3D_interp = [x_interp, y_interp, z_interp]

    experiment_data_df = read_pandas_df(path_experiment_data, columns_3D_experiment_data)
    interp_trajectory_df = read_pandas_df(path_interp, columns_3D_interp)

    plot_3d_scatter(interp_trajectory_df, experiment_data_df, columns_3D_interp, columns_3D_experiment_data)

    
    #Plot position vs time
    columns_position_ref = [time_data_setpoint, x_ref, y_ref, z_ref]
    columns_position = [time_data, x_data, y_data, z_data]
    position_df = read_pandas_df(path_experiment_data, columns_position)
    position_setpoint_df = read_pandas_df(path_experiment_data, columns_position_ref)
    plot_pose_temporal_evolution(position_setpoint_df, position_df, columns_position_ref, columns_position)

    #Plot attitude vs time
    columns_attitude_rollpitch_ref = [time_data_tilt, roll_ref, pitch_ref]
    columns_attitude_yaw_ref = [time_data_setpoint, yaw_ref]
    columns_attitude_exp = [time_data, q0, q1, q2, q3]
    attitude_data_df = read_pandas_df(path_experiment_data, columns_attitude_exp)
    rpy_attitude_data_df = attitude_data_df[[q0, q1, q2, q3]].apply(lambda row: pd.Series(quaternion_to_euler_angles(row), index=['roll', 'pitch', 'yaw']), axis=1)    
    attitude_data_df = pd.concat([attitude_data_df, rpy_attitude_data_df], axis=1)
    columns_attitude_exp = [time_data, 'roll', 'pitch', 'yaw']
    
    attitude_setpoint_rollpitch_df = read_pandas_df(path_experiment_data, columns_attitude_rollpitch_ref)
    attitude_setpoint_yaw_df = read_pandas_df(path_experiment_data, columns_attitude_yaw_ref)

    plot_attitude_temporal_evolution(attitude_setpoint_rollpitch_df, attitude_setpoint_yaw_df, attitude_data_df, columns_attitude_rollpitch_ref, columns_attitude_yaw_ref, columns_attitude_exp)

    #Plot velocity vs time
    columns_velocity = [time_data, vx_data, vy_data, vz_data]
    columns_velocity_ref = [time_data_setpoint, vx_ref, vy_ref, vz_ref]

    velocity_setpoint_df = read_pandas_df(path_experiment_data, columns_velocity_ref)
    velocity_df = read_pandas_df(path_experiment_data, columns_velocity)

    plot_velocity_temporal_evolution(velocity_setpoint_df, velocity_df, columns_velocity_ref, columns_velocity)


def read_pandas_df(path, columns):

    try:
            # Read CSV file into a Pandas DataFrame
            df = pd.read_csv(path, usecols = columns)
            df = df.dropna()

            # Check if the specified columns exist in the DataFrame
            for column in columns:
                if column not in df.columns:
                    raise ValueError("One or more specified columns not found in the CSV file.")

    except FileNotFoundError:
        print(f"Error: File '{path}' not found.")
    except pd.errors.EmptyDataError:
        print(f"Error: File '{path}' is empty.")
    except pd.errors.ParserError:
        print(f"Error: Unable to parse file '{path}'. Make sure it's a valid CSV file.")
    except ValueError as ve:
        print(f"Error: {ve}")

    return df

def main():

    if len(sys.argv) != 2:
        print("Usage: python script.py <csv_file_path>")
        sys.exit(1)

    path_to_data = sys.argv[1]

    bag_csv_path = path_to_data + "/experiment_data.csv"
    interp_txt_path = path_to_data + "/interpolated_trajectory.csv"

    plot_experiment_data(interp_txt_path,bag_csv_path)


if __name__ == "__main__":
    main()