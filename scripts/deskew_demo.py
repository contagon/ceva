import os
import glob
import numpy as np
from scipy.spatial.transform import Rotation

from rosbags.highlevel import AnyReader

# python wrapper for spline
from ceva import Ceva

# pcd interface
import pypcd4 as pypcd
from pathlib import Path

# Link to rosbag
rosbag_file = "../example/deskew_example.bag"

# Folder to export the pointclouds to
exported_dir = "exported_pcds"

# Ground truth spline log
spline_log = "../example/spline.csv"

# Extrinsic parameters of the ouster lidar, downloadable at https://mcdviral.github.io/Download.html#calibration
T_B_L = np.array(
    [
        [
            0.999934655205122900,
            0.003477624535771754,
            -0.010889970036688295,
            -0.060649229060416594,
        ],
        [
            0.003587143302461965,
            -0.999943027982117100,
            0.010053516443599904,
            -0.012837544242408117,
        ],
        [
            -0.010854387257665576,
            -0.010091923381711220,
            -0.999890161647627000,
            -0.020492606896077407,
        ],
        [
            0.000000000000000000,
            0.000000000000000000,
            0.000000000000000000,
            0.000000000000000000,
        ],
    ]
)

# Minimum range
min_range = 0.75

# Load the ground truth

# Read the spline
log = open(spline_log, "r")

# Extract some settings in the header
log_header = log.readline()
log.close()

# Read the dt from header
dt = float(log_header.split(sep=",")[0].replace("Dt: ", ""))
# Read the spline order
order = int(log_header.split(sep=",")[1].replace("Order: ", ""))
# Read the number of knots
knots = int(log_header.split(sep=",")[2].replace("Knots: ", ""))
# Read the start time in header
start_time = float(log_header.split(sep=",")[3].replace("MinTime: ", ""))
# Calculate the end time
final_time = start_time + dt * (knots - order + 1)

# print("Seq: ", log)
# print("dt: \t\t", dt)
# print("order: \t\t", order)
# print("start_time: \t", start_time)
# print("final_time: \t", final_time)
# print("Duration: \t", final_time - start_time)

# Read the spline log in text
knots = np.loadtxt(spline_log, delimiter=",", skiprows=1)
spline = Ceva(order, dt, start_time, spline_log)

# Create folders
os.makedirs(exported_dir + "/cloud_inB_deskewed", exist_ok=True)
os.makedirs(exported_dir + "/cloud_inB_distorted", exist_ok=True)

# Read the pointclouds from rosbag and deskew them

# Extract the rotation and translation extrinsics for convinience
R_B_L = T_B_L[:3, :3]
t_B_L = T_B_L[:3, 3]

# Create interface with rosbag
bag = AnyReader([Path(rosbag_file)])
bag.open()

# Cloud index
cloud_idx = -1

# Iterate through the message
for connection, timestamp, rawdata in bag.messages():
    if connection.topic == "/os_cloud_node/points":
        msg = bag.deserialize(rawdata, connection.msgtype)

        cloud_idx += 1

        # Ouster lidar timestamp is incident with the last point
        sweeptime = 0.1
        timestamp = msg.header.stamp
        print(timestamp)
        timestamp = timestamp.sec + timestamp.nanosec / 1.0e9
        print(timestamp)

        # Make sure the pointcloud is in the valid time period, adding some padding for certainty
        if (
            timestamp < spline.minTime() + sweeptime + 10e-3
            or timestamp > spline.maxTime() - sweeptime - 10e-3
        ):
            continue

        # Convert the msg to np array
        pc_in = np.squeeze(
            np.reshape(
                pypcd.PointCloud.from_msg(msg).pc_data, (msg.height * msg.width, 1)
            )
        )

        # Adjust the sweep time to be more precise
        sweeptime = (max(pc_in["t"]) - min(pc_in["t"])) / 1.0e9

        # Time stamp at the beginning
        timestamp_beginning = timestamp - sweeptime

        # Obtain the pose at the beginning of the scan
        pose_ts = spline.getPose(timestamp_beginning)[0]  # t, x, y, z, qx, qy, qz, qw
        R_W_Bs = Rotation.from_quat(pose_ts[4:8]).as_matrix()
        t_W_Bs = pose_ts[1:4].T

        # Remove the zero points
        nonzero_idx = (pc_in["range"] / 1000.0 > min_range).nonzero()[0]
        pc_in = pc_in[nonzero_idx]

        # Extract the xyz parts
        pc_xyz_inL = np.array([list(p) for p in pc_in[["x", "y", "z"]]])

        # Extract the intensity
        intensity = pc_in["intensity"].reshape((len(pc_in["intensity"]), 1))

        # Extract the point timestamp relative to the beginning
        tr = pc_in["t"]

        # Absolute time of the points
        th = timestamp_beginning
        # ta = th + tr / 1.0e9

        # Transform pointcloud from lidar to body frame
        pc_xyz_inB = np.dot(R_B_L, pc_xyz_inL.T).T + t_B_L

        # Transform the distorted pointcloud to the world frame
        # pc_inB_distorted = np.dot(R_W_Bs, pc_xyz_inB.T).T + t_W_Bs

        # Put the intensity back
        # pc_inB_distorted = np.concatenate([ pc_inB_distorted, intensity ], axis=1)

        # Deskew the pointcloud, all points are referenced to the time at the start of the scan
        pc_xyz_inB_deskewed = spline.deskewCloudInBody(pc_xyz_inB, tr / 1.0e9, th)
        print(type(pc_xyz_inB_deskewed))

        # Put the intensity back
        # pc_inB_deskewed = np.concatenate([ pc_xyz_inB_deskewed, intensity ], axis=1)

        # Save the deskewed pointcloud
        pc_inB_deskewed_filename = (
            exported_dir
            + "/cloud_inB_deskewed/cloud"
            + "_"
            + str(msg.header.seq).zfill(4)
            + "_"
            + f"{timestamp_beginning}.pcd"
        )

        pypcd.PointCloud.from_xyz_points(pc_xyz_inB_deskewed).save(
            pc_inB_deskewed_filename
        )

        # Save the original distorted pointcloud
        pc_inB_distorted_filename = (
            exported_dir
            + "/cloud_inB_distorted/cloud"
            + "_"
            + str(msg.header.seq).zfill(4)
            + "_"
            + f"{timestamp_beginning}.pcd"
        )
        pypcd.PointCloud.from_xyz_points(pc_xyz_inB_deskewed).save(
            pc_inB_distorted_filename
        )

# Close the rosbag
bag.close()


# Visualize the exported pointclouds (it's better to load the pcd files into cloudcompare and view from there, we only use open3d here for a quick view)

pc_inB_deskewed = None
dekewed_pcd = glob.glob(exported_dir + "/cloud_inB_deskewed/*.pcd")
for pcd in dekewed_pcd:
    cloud = pypcd.PointCloud.from_path(pcd).pc_data
    pc_inB_deskewed = (
        cloud if pc_inB_deskewed is None else np.concatenate((pc_inB_deskewed, cloud))
    )
    break

pc_inB_distorted = None
distorted_pcd = glob.glob(exported_dir + "/cloud_inB_distorted/*.pcd")
for pcd in distorted_pcd:
    cloud = pypcd.PointCloud.from_path(pcd).pc_data
    pc_inB_distorted = (
        cloud if pc_inB_distorted is None else np.concatenate((pc_inB_distorted, cloud))
    )
    break

import rerun as rr

rr.init("ceva")
rr.connect_tcp("0.0.0.0:9876")

# View the deskewed pointcloud
pc_o3d = rr.Points3D(
    np.column_stack((pc_inB_deskewed["x"], pc_inB_deskewed["y"], pc_inB_deskewed["z"]))
)
rr.log("deskewed", pc_o3d)

# View the distorted pointcloud
pc_o3d = rr.Points3D(
    np.column_stack(
        (pc_inB_distorted["x"], pc_inB_distorted["y"], pc_inB_distorted["z"])
    )
)
rr.log("original", pc_o3d)
