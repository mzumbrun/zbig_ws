#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def create_pos_stamped(navigator, position_x, position_y, orientation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0  
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

def main():
    rclpy.init()
    nav = BasicNavigator()
    
    # wait for Nav2
    nav.waitUntilNav2Active
    
    # set initial position and pose
    initial_pose = create_pos_stamped(nav, 1.8, 3.5, -1.57)
    # nav.setInitialPose(initial_pose)
    # while not nav.isTaskComplete():
    #     feedback = nav.getFeedback()
    
    print(nav.getResult())

    
    # send goals
    goal_pose1 = create_pos_stamped(nav, 1.8, 3.5, -1.57)
    goal_pose2 = create_pos_stamped(nav, 1.8, 2.5, -1.57)
    goal_pose3 = create_pos_stamped(nav, 1.8, 2.5, -1.57)

    nav.goToPose(goal_pose1)
    
    # while not nav.isTaskComplete():
    #     feedback = nav.getFeedback()
    #   #  print(feedback)

    # waypoints = [goal_pose1, goal_pose2, goal_pose3]
    # nav.followWaypoints(waypoints)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
      #  print(feedback)
    
    
    print(nav.getResult())
    
    
    
    rclpy.shutdown()
            
if __name__ =='__main__':
    main()