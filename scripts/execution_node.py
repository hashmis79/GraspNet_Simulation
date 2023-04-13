#!/usr/bin/env python3

#Imports for MoveIt Gazebo Integration 
import sys
import rospy
import rosservice
import os
import sensor_msgs
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as nps
import cv2
from autolab_core import RigidTransform
from message_filters import ApproximateTimeSynchronizer, Subscriber
from manipulator_1.srv import grasp_planner_service
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped, Pose
# from visualization import Visualizer2D as vis
# Transform Listener
import tf

class grasp_net():
    def __init__(self):
        # Service Client 
        self.flag=1
        print("Waiting for Service..")
        rospy.wait_for_service("/grasp_planner")
        self.grasp_planning_srv = rospy.ServiceProxy("/grasp_planner", grasp_planner_service)	
        self.depth_image_sub = Subscriber("/camera2/depth/image_raw", Image)
        self.color_image_sub = Subscriber("/camera2/color/image_raw", Image)
        # self.camera_info_sub = Subscriber("/r200/camera/color/camera_info", CameraInfo)	
        print ("Subscriber information recieved")
    
    def sync_subscriber(self):
        # Sync Subscribers
        self.ats = ApproximateTimeSynchronizer(
            [
                self.color_image_sub,
                self.depth_image_sub,
                # self.camera_info_sub
            ],
            queue_size=5,
            slop=1,
            allow_headerless=True
        )
        self.ats.registerCallback(self.msg_filter_callback)

    def msg_filter_callback(self,color_image,depth_image):
	# Message Callback
        if self.flag == 1:
            self.grasp = self.grasp_planning_srv(color_image, depth_image)
            self.pose_dexnet = self.grasp.pose_msg
            self.flag = 0


    def T_gripper_camera(self):
        # Offsets: Distance between Link8 centre and color frame (Camera intrinsic Frame) wrt to world frame
        self.pose_dexnet.position.x = self.pose_dexnet.position.x - 0.030
        self.pose_dexnet.position.y = self.pose_dexnet.position.y + 0.015
        self.pose_dexnet.position.z = self.pose_dexnet.position.z - 0.060
        # Transformation from Approach Grasp Frame to Camera Frame
        return RigidTransform.from_pose_msg(self.pose_dexnet,
                        from_frame='grasp', to_frame='depth_camera_link')

    def pre_processing(self):
        # Transforms using Listener
        listener = tf.TransformListener()
        while True:
            try:
               (self.tcr,rcr) = listener.lookupTransform('/panda_link0',
							 '/depth_camera_link', 
							 rospy.Time(0))
               (self.tref,rref) = listener.lookupTransform('/panda_link0',
							   '/panda_link8', 
							   rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
               continue
            break
	
	    # Reordering for RigidTransform function in autolab core
        self.qref = [rref[3],rref[0],rref[1],rref[2]]
        self.qcr = [rcr[3],rcr[0],rcr[1],rcr[2]]

    def T_camera_robot(self):
	# Transformation from Camera to Robot link 0 (base) Frame
        return RigidTransform(rotation = self.qcr , translation = self.tcr, from_frame='depth_camera_link',to_frame='panda_link0')

    def ready_pose(self):
	# Ready Position
        return RigidTransform(rotation = self.qref , translation = self.tref, from_frame='panda_link8', to_frame='panda_link0').pose_msg

    def T_eef_gripper(self):
	    # Transformation from End-Effector/Tool Frame (Panda Link 8) to Grasp Frame	
	    # YZX = [90,45,0] in degrees (Can be visualized in Rviz by enabling POSE Tab)
        return RigidTransform(rotation = [ 0.6532815 ,0.2705981, 0.6532815, 0.2705981], translation = [0,0,0], from_frame='eef', to_frame='grasp')

    def final_transform(self):
        """
        Transformation : Panda_link8 -> Grasp -> Camera -> Panda_link0
        """
        # Listener 
        self.pre_processing()
        # Transformation from End-Effector/Tool Frame (Panda Link 8) to Robot Base Frame
        self.T_gripper_robot = self.T_camera_robot() * self.T_gripper_camera() * self.T_eef_gripper()

    def pose_set(self,transform):
	# Set pose of Arm using Moveit Commander
        pose_goal = Pose()
        pose_goal.orientation.x = transform.quaternion[1]
        pose_goal.orientation.y = transform.quaternion[2]
        pose_goal.orientation.z = transform.quaternion[3]
        pose_goal.orientation.w = transform.quaternion[0]
        pose_goal.position.x = transform.translation[0] 
        pose_goal.position.y = transform.translation[1]
        pose_goal.position.z = transform.translation[2]
        return pose_goal

    def open_gripper(self):
	# open gripper 
        self.group_h.set_joint_value_target([0.04,0.04])
        self.group_h.go(wait=True)
        self.group_h.stop()
        self.group_h.clear_pose_targets()

    def close_gripper(self):
        # close gripper 
        self.group_h.set_joint_value_target([0.00,0.00])
        self.group_h.go(wait=True)
        self.group_h.stop()
        self.group_h.clear_pose_targets()

    def group_action(self):
	# pose setting and clearning
        self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets() 

    def pre_grasp_action(self):
        # set arm to pre grasp position
        grasp_pose = self.pose_set(self.T_gripper_robot)
        grasp_pose.position.z = grasp_pose.position.z + 0.1
        self.group.set_pose_target(grasp_pose)
        self.group_action()

    def grasping_action(self):
        # set arm to grasp position
        grasp_pose = self.pose_set(self.T_gripper_robot)
        # offset to avoid collision with table
        grasp_pose.position.z = grasp_pose.position.z + 0.045
        self.group.set_pose_target(grasp_pose)
        self.group_action()

    def post_grasp_action(self):
        # set arm to post grasp position
        grasp_pose = self.pose_set(self.T_gripper_robot)
        grasp_pose.position.z = grasp_pose.position.z + 0.2 
        self.group.set_pose_target(grasp_pose)
        self.group_action()

    def auto_reset(self):
        # set to ready state
        self.open_gripper()
        self.group.set_pose_target(self.ready_pose())
        self.group_action()
        rospy.sleep(1)
        self.close_gripper()

    def moveit_setup(self):  
	# Moveit setup for planning
        print("In Moveit_Setup")
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.group_h = moveit_commander.MoveGroupCommander("gripper")
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                        moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        planning_frame = self.group.get_planning_frame()  

    def go_home(self):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = -0.785
        joint_goal[1] = -0.785
        joint_goal[2] = 0
        joint_goal[3] = -2.356
        joint_goal[4] = 0
        joint_goal[5] = 1.571 
        joint_goal[6] = 0.785
        self.group.go(joint_goal, wait=True)
        self.group.stop()

    def execution(self): 
        # Final execution sequence
        self.moveit_setup()
        print("Moving towards Object")
        self.go_home()
        # self.open_gripper()
        print("Sync")
        self.sync_subscriber()
        rospy.sleep(15)
        py = input("Enter any number to proceed (To be done after closing the visualizations)")
        self.final_transform()
        self.pre_grasp_action()
        rospy.sleep(5) 
        print("Grasping Object")
        self.grasping_action()
        rospy.sleep(5)
        self.close_gripper()
        rospy.sleep(5)
        print("Moving away with Object")
        self.post_grasp_action()	
        rospy.sleep(15) 
        # This motion could be jerky 
        print("Returning to ready position!!")
        self.auto_reset() 
        
def main(args):       
    print("===============EXECUTION STARTED ====================")
    rospy.init_node('simulation_graspnet', anonymous=True)
    gd = grasp_net()
    rospy.sleep(15) 
    gd.execution()
    print("===================== EXECUTION HAS ENDED ============================")
    rospy.spin()

if __name__ == '__main__':
	main(sys.argv)