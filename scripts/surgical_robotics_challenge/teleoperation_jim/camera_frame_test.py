from ambf_client import Client
import argparse
import crtk
import crtk_msgs.msg
import dvrk
from enum import Enum
import geometry_msgs.msg
import json
import math
import numpy
import os
import PyKDL
import rospy
import sensor_msgs.msg
import std_msgs.msg
import sys
import time

class camera_frame_test:

    def __init__(self, ral):
        self.ral = ral

        self.ambf_client = Client()
        self.ambf_client.connect()
        self.cameraframe_obj = self.ambf_client.get_obj_handle('/CameraFrame')
        self.psmbase_obj = self.ambf_client.get_obj_handle('psm2/baselink')
        self.psmtool_obj = self.ambf_client.get_obj_handle('psm2/toolyawlink')
        self.camera_pub = rospy.Publisher('ambf_camera', geometry_msgs.msg.PoseStamped, queue_size=10)
        self.psmbase_pub = rospy.Publisher('ambf_psmbase', geometry_msgs.msg.PoseStamped, queue_size=10)
        self.psmtool_pub = rospy.Publisher('ambf_psmtool', geometry_msgs.msg.PoseStamped, queue_size=10)

    def run(self):
        while not rospy.is_shutdown():
            T_w_c_frame = PyKDL.Frame()
            T_w_c_pos = self.cameraframe_obj.get_pos()
            T_w_c_rot = self.cameraframe_obj.get_rot()
            T_w_c_frame.p = PyKDL.Vector(T_w_c_pos.x, T_w_c_pos.y, T_w_c_pos.z)
            T_w_c_frame.M = PyKDL.Rotation.Quaternion(T_w_c_rot.x, T_w_c_rot.y, T_w_c_rot.z, T_w_c_rot.w)
            # T_w_c = rospy.wait_for_message('/ECM/measured_cp', geometry_msgs.msg.PoseStamped)
            T_c_psmbase = rospy.wait_for_message(f'/SUJ/PSM2/measured_cp', geometry_msgs.msg.PoseStamped)
            psm_js = rospy.wait_for_message(f'/PSM2/measured_js', sensor_msgs.msg.JointState)
            psm_jaw_js = rospy.wait_for_message(f'/PSM2/jaw/measured_js', sensor_msgs.msg.JointState)

            y_pi = PyKDL.Frame(PyKDL.Rotation.RotY(math.pi), PyKDL.Vector())
            T_w_c_adjusted = crtk.msg_conversions.FrameToPoseMsg(T_w_c_frame * y_pi)
            
            # cameraframe_obj.set_pos(T_w_c.pose.position.x, T_w_c.pose.position.y, T_w_c.pose.position.z)
            # cameraframe_obj.set_rot([T_w_c.pose.orientation.x, T_w_c.pose.orientation.y, T_w_c.pose.orientation.z, T_w_c.pose.orientation.w])
            T_w_psmbase = crtk.msg_conversions.FrameToPoseMsg(
                T_w_c_frame * y_pi * crtk.msg_conversions.FrameFromPoseMsg(T_c_psmbase.pose))
            self.psmbase_obj.set_pos(T_w_psmbase.position.x, T_w_psmbase.position.y, T_w_psmbase.position.z)
            self.psmbase_obj.set_rot([T_w_psmbase.orientation.x, T_w_psmbase.orientation.y, T_w_psmbase.orientation.z, T_w_psmbase.orientation.w])
            for i in range(6):
                self.psmbase_obj.set_joint_pos(i, psm_js.position[i])
            self.psmbase_obj.set_joint_pos(6, psm_jaw_js.position[0]/2)
            self.psmbase_obj.set_joint_pos(7, psm_jaw_js.position[0]/2)

            T_w_psmtool_pos = self.psmtool_obj.get_pos()
            T_w_psmtool_rot = self.psmtool_obj.get_rot()

            p = geometry_msgs.msg.PoseStamped()
            p.header.frame_id = 'Cart'
            p.header.stamp = rospy.Time.now()
            p.pose = T_w_c_adjusted
            self.camera_pub.publish(p)

            p.header.stamp = rospy.Time.now()
            p.pose = T_w_psmbase
            self.psmbase_pub.publish(p)

            p.header.stamp = rospy.Time.now()
            p.pose.position = T_w_psmtool_pos
            p.pose.orientation = T_w_psmtool_rot
            self.psmtool_pub.publish(p)


    
if __name__ == '__main__':
    # extract ros arguments (e.g. __ns:= for namespace)
    argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

    # parse arguments
    parser = argparse.ArgumentParser(description = __doc__,
                                     formatter_class = argparse.RawTextHelpFormatter)
    args = parser.parse_args(argv)

    ral = crtk.ral('camera_frame_test')
    application = camera_frame_test(ral)
    ral.spin_and_execute(application.run)
