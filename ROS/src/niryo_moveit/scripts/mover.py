#!/usr/bin/env python

from __future__ import print_function

import rospy

import sys
import copy
import math
import moveit_commander #moveit을 이용
#moveit 패키지를 사용
import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from niryo_moveit.srv import MoverService, MoverServiceRequest, MoverServiceResponse
#joint 6개
joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Between Melodic and Noetic, the return type of plan() changed. moveit_commander has no __version__ variable, so checking the python version as a proxy
if sys.version_info >= (3, 0):
    def planCompat(plan):
        return plan[1] #plan의 첫번째 원소를 가져온다?
else:
    def planCompat(plan):
        return plan
        
"""
    Given the start angles of the robot, plan a trajectory that ends at the destination pose.
"""
def plan_trajectory(move_group, destination_pose, start_joint_angles): 
    #현재 joint 정보 이름과 angle을 입력
    current_joint_state = JointState() 
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles
    #현재 로봇 정보와 joint 정보를 입력
    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    #move_group에 현재 robot state와 destination_pose 입력
    move_group.set_start_state(moveit_robot_state)
    move_group.set_pose_target(destination_pose)
    #move_group의 plan을 가져옴
    plan = move_group.plan()

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(destination_pose, destination_pose)
        raise Exception(exception_str)

    return planCompat(plan)


"""
    Creates a pick and place plan using the four states below.
    
    1. Pre Grasp - position gripper directly above target object
    2. Grasp - lower gripper so that fingers are on either side of object
    3. Pick Up - raise gripper back to the pre grasp position
    4. Place - move gripper to desired placement position

    Gripper behaviour is handled outside of this trajectory planning.
        - Gripper close occurs after 'grasp' position has been achieved
        - Gripper open occurs after 'place' position has been achieved

    https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py
"""
def plan_pick_and_place(req):
    response = MoverServiceResponse()
    #group_name 지정
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    #current_robot_joint_configuration 가져오기
    current_robot_joint_configuration = req.joints_input.joints

    # Pre grasp - position gripper directly above target object
    pre_grasp_pose = plan_trajectory(move_group, req.pick_pose, current_robot_joint_configuration)
    
    # If the trajectory has no points, planning has failed and we return an empty response
    if not pre_grasp_pose.joint_trajectory.points: #has no points
        return response 

    previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions #pre_grasp_pose의 마지막 원소

    # Grasp - lower gripper so that fingers are on either side of object
    pick_pose = copy.deepcopy(req.pick_pose)
    pick_pose.position.z -= 0.05  # Static value coming from Unity, TODO: pass along with request
    grasp_pose = plan_trajectory(move_group, pick_pose, previous_ending_joint_angles) #각도가 이전각도가 들어간다?
    
    if not grasp_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions #grasp_pose의 마지막 원소

    # Pick Up - raise gripper back to the pre grasp position
    pick_up_pose = plan_trajectory(move_group, req.pick_pose, previous_ending_joint_angles)
    
    if not pick_up_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = pick_up_pose.joint_trajectory.points[-1].positions #pick_up_pose의 마지막 원소

    # Place - move gripper to desired placement position
    place_pose = plan_trajectory(move_group, req.place_pose, previous_ending_joint_angles) #2번째 값으로 place_pose가 들어감

    if not place_pose.joint_trajectory.points:
        return response

    # If trajectory planning worked for all pick and place stages, add plan to response
    response.trajectories.append(pre_grasp_pose)
    response.trajectories.append(grasp_pose)
    response.trajectories.append(pick_up_pose)
    response.trajectories.append(place_pose)

    move_group.clear_pose_targets() #move_group clear?

    return response


def moveit_server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('niryo_moveit_server')

    s = rospy.Service('niryo_moveit', MoverService, plan_pick_and_place)
    print("Ready to plan")
    rospy.spin()


if __name__ == "__main__":
    moveit_server()
