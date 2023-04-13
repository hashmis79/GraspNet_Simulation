#!/home/saad/env/bin/python
import os
import sys
import numpy as np
import open3d as o3d
import argparse
import importlib
import scipy.io as scio
from PIL import Image
import rospy
import ros_numpy as rnp
from manipulator_1.srv import grasp_planner_service, grasp_planner_serviceResponse
import torch
from graspnetAPI import GraspGroup
from cv_bridge import CvBridge, CvBridgeError
import cv2
from scipy.spatial.transform import Rotation as R

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, 'models'))
sys.path.append(os.path.join(ROOT_DIR, 'dataset'))
sys.path.append(os.path.join(ROOT_DIR, 'utils'))

from graspnet import GraspNet, pred_decode
from graspnet_dataset import GraspNetDataset
from collision_detector import ModelFreeCollisionDetector
from data_utils import CameraInfo, create_point_cloud_from_depth_image
from geometry_msgs.msg import Pose

parser = argparse.ArgumentParser()
# parser.add_argument('--checkpoint_path', type= validate_file, default="logs/log_rs/checkpoint.tar", help='Model checkpoint path')
parser.add_argument('--num_point', type=int, default=20000, help='Point Number [default: 20000]')
parser.add_argument('--num_view', type=int, default=300, help='View Number [default: 300]')
parser.add_argument('--collision_thresh', type=float, default=0.01, help='Collision Threshold in collision detection [default: 0.01]')
parser.add_argument('--voxel_size', type=float, default=0.01, help='Voxel Size to process point clouds before collision detection [default: 0.01]')
cfgs = parser.parse_args()
data_dir = 'doc/example_data'


def get_net():
    # Init the model
    net = GraspNet(input_feature_dim=0, num_view=cfgs.num_view, num_angle=12, num_depth=4,
            cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01,0.02,0.03,0.04], is_training=False)
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    net.to(device)
    # Load checkpoint
    checkpoint = torch.load(os.path.join(ROOT_DIR, 'logs/log_rs/checkpoint.tar'))
    net.load_state_dict(checkpoint['model_state_dict'])
    start_epoch = checkpoint['epoch']
    print("-> loaded checkpoint %s (epoch: %d)"%('logs/log_rs/checkpoint.tar', start_epoch))
    # set model to eval mode
    net.eval()
    return net

def get_and_process_data(color_im,depth_im):
    # load data
    color = rnp.image.image_to_numpy(color_im)/255.0
    depth = rnp.image.image_to_numpy(depth_im)*63/0.5403085
    workspace_mask = np.ones((480,640),dtype=bool)
    meta = scio.loadmat(os.path.join(ROOT_DIR, data_dir, 'meta.mat'))
    intrinsic = meta['intrinsic_matrix']
    factor_depth = meta['factor_depth']

    # generate cloud
    camera = CameraInfo(640.0, 480.0, intrinsic[0][0], intrinsic[1][1], intrinsic[0][2], intrinsic[1][2], factor_depth)
    cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)

    # get valid points
    mask = (workspace_mask & (depth > 0))
    cloud_masked = cloud[mask]
    color_masked = color[mask]

    # sample points
    if len(cloud_masked) >= cfgs.num_point:
        idxs = np.random.choice(len(cloud_masked), cfgs.num_point, replace=False)
    else:
        idxs1 = np.arange(len(cloud_masked))
        idxs2 = np.random.choice(len(cloud_masked), cfgs.num_point-len(cloud_masked), replace=True)
        idxs = np.concatenate([idxs1, idxs2], axis=0)
    cloud_sampled = cloud_masked[idxs]
    color_sampled = color_masked[idxs]

    # convert data
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
    cloud.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))
    end_points = dict()
    cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32))
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    cloud_sampled = cloud_sampled.to(device)
    end_points['point_clouds'] = cloud_sampled
    end_points['cloud_colors'] = color_sampled

    return end_points, cloud

def get_grasps(net, end_points):
    # Forward pass
    with torch.no_grad():
        end_points = net(end_points)
        grasp_preds = pred_decode(end_points)
    gg_array = grasp_preds[0].detach().cpu().numpy()
    gg = GraspGroup(gg_array)
    return gg

def collision_detection(gg, cloud):
    mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=cfgs.voxel_size)
    collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=cfgs.collision_thresh)
    gg = gg[~collision_mask]
    return gg

def vis_grasps(gg, cloud):
    gg.nms()
    gg.sort_by_score()
    gg = gg[:50]
    grippers = gg.to_open3d_geometry_list()
    o3d.visualization.draw_geometries([cloud, *grippers])

# def read_images(req):
#     raw_color = req.color_image
#     raw_depth = req.depth_image
#     x = CvBridge()
#     try:
#         color_im = x.imgmsg_to_cv2(raw_color, "rgb8")
#         depth_im = (x.imgmsg_to_cv2(raw_depth, desired_encoding="passthrough"))
        
#     except CvBridgeError as cv_bridge_exception:
#         rospy.logerr(cv_bridge_exception)
    
#     return raw_color, raw_depth

def plan_grasp(req):
    #Used for calculating grasp poses
    net = get_net()
    pose = Pose()
    # color_im, depth_im = read_images(req)
    end_points, cloud = get_and_process_data(req.color_image, req.depth_image)
    gg = get_grasps(net, end_points)
    if cfgs.collision_thresh > 0:
        gg = collision_detection(gg, np.array(cloud.points))
    print(gg)
    vis_grasps(gg, cloud)
    cv = gg[1]
    print(cv.translation)
    trans = gg[1].translation
    x = R.from_matrix(gg[1].rotation_matrix)
    rot = x.as_quat()
    pose.orientation.x = rot[0]
    pose.orientation.y = rot[1]
    pose.orientation.z = rot[2]
    pose.orientation.w = rot[3]
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    print(pose.orientation.x)
    return grasp_planner_serviceResponse(pose)

if __name__=='__main__':
    rospy.init_node("graspnet")
    grasp_planning_service = rospy.Service("grasp_planner", grasp_planner_service, plan_grasp)
    rospy.spin()
