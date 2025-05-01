#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time

class SequentialGoalSenderSync:
    def __init__(self):
        rospy.init_node('sequential_goal_sender', anonymous=True)

        # 读取 waypoints 参数
        if not rospy.has_param("waypoints"):
            rospy.logerr("No 'waypoints' parameter found.")
            return

        self.waypoints = rospy.get_param("waypoints")
        if len(self.waypoints) < 1:
            rospy.logerr("Need at least one waypoint.")
            return

        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server.")

        self.send_goals()

    def send_goals(self):
        for idx, wp in enumerate(self.waypoints):
            rospy.loginfo(f"Sending goal {idx+1}/{len(self.waypoints)}: {wp}")
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = wp[0]
            goal.target_pose.pose.position.y = wp[1]
            goal.target_pose.pose.orientation.w = 1.0  # facing forward

            self.client.send_goal(goal)
            self.client.wait_for_result()

            state = self.client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal reached!")
            else:
                rospy.logwarn(f"Goal failed with state {state}")
                break  # 可选：失败就停止整个导航

            rospy.sleep(1.0)  # 停止1秒再继续

        rospy.loginfo("All goals processed.")

if __name__ == "__main__":
    try:
        SequentialGoalSenderSync()
    except rospy.ROSInterruptException:
        pass
