#!/usr/bin/env python

import moveit_commander
import rospy, sys
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np


class Grasping:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur5_grasping', anonymous=True)
        
        self.scene = moveit_commander.PlanningSceneInterface()
        self.box_name = ""
        
        rospy.sleep(1)
        
        self.move_group = moveit_commander.MoveGroupCommander('manipulator')
        self.move_group.set_goal_joint_tolerance(0.001)
        self.move_group.set_max_acceleration_scaling_factor(0.5)
        self.move_group.set_max_velocity_scaling_factor(0.5)
        self.move_group.allow_replanning(True)
        self.move_group.set_planning_time(10)
        
        if self.add_box():
            print('OK!')
        else:
            print('Failed!')

    def homing(self):
        move_group = self.move_group
        move_group.set_named_target('home')
        move_group.go()
        rospy.sleep(1)
        
    def move_to_goal(self, pose_goal):
        move_group = self.move_group
        
        move_group.set_start_state_to_current_state()
        move_group.set_pose_target(pose_goal)
        
        plan = move_group.go()        
        move_group.stop()
        move_group.clear_pose_targets()
        
    def is_ok(self, box_is_known=False, box_is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        
        box_name = self.box_name
        scene = self.scene
        
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()
            
        return False

    def add_box(self, timeout=4):
        box_name = self.box_name
        scene = self.scene
        
        # Add table
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.4
        box_pose.pose.position.y = 0
        box_pose.pose.position.z = 0
        box_name = "table"
        scene.add_box(box_name, box_pose, size=(0.3, 0.3, 0.2))
        
        # Add floor
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0
        box_name = "floor"
        scene.add_box(box_name, box_pose, size=(2.5, 2.5, 0.01))
  
        self.box_name=box_name
        return self.is_ok(box_is_known=True, timeout=timeout)
       

def callback(data):
    global goal
    goal = data
        
if __name__ == '__main__':
    
    location_sub = rospy.Subscriber('/recognition/location', geometry_msgs.msg.Pose, callback)
    
    demo = Grasping()
    demo.homing()
    
    demo.move_to_goal(goal)
    
