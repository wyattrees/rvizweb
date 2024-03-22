#!/usr/bin/python3
import math

import rospy
from std_msgs.msg import String
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryActionGoal, JointTolerance
from control_msgs.msg import FollowJointTrajectoryActionFeedback, FollowJointTrajectoryActionResult

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

pub_topic = '/arm_controller/follow_joint_trajectory/goal'
feedback_topic = '/arm_controller/follow_joint_trajectory/feedback'
result_topic = '/arm_controller/follow_joint_trajectory/result'
node_name = 'traj_manager'

class TrajManager:
    def __init__(self):
        self.joint_states = []
        rospy.init_node(node_name)
        self.traj_pub = rospy.Publisher(pub_topic, FollowJointTrajectoryActionGoal, queue_size=10)
        self.joint_states_sub = rospy.Subscriber('/joint_states', JointState, callback=self.update_joint_states, queue_size=10)
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
      'wrist_3_joint']
        self.updated_joints = False
        self.feedback_sub = rospy.Subscriber(feedback_topic, FollowJointTrajectoryActionFeedback,
                                             callback=self.print_subscriber, queue_size=10)
        self.result_sub = rospy.Subscriber(result_topic, FollowJointTrajectoryActionResult,
                                           callback=self.print_subscriber, queue_size=10)

    def print_subscriber(self, msg):
        print(msg)

    def update_joint_states(self, msg):
        print(msg)
        self.joint_states = msg.position
        self.updated_joints = True


    def traj_publisher(self):
        rate = rospy.Rate(10) # 10hz
        published = False
        while not rospy.is_shutdown():
            if not published:
                traj_goal = self.home_joint_trajectory()
                self.traj_pub.publish(traj_goal)
            rospy.sleep(0.1)


    def home_joint_trajectory(self):
        # init header
        jt_action_goal = FollowJointTrajectoryActionGoal()
        jt_action_goal.header.seq = 0
        jt_action_goal.header.stamp = rospy.Time.now()
        jt_action_goal.header.frame_id = ''
        jt_action_goal.goal_id.stamp.secs = 0
        jt_action_goal.goal_id.stamp.nsecs = 0
        jt_action_goal.goal_id.id = 'home'
        # set up goal
        goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj.points = []
        start_point = JointTrajectoryPoint()
        start_point.positions = self.joint_states
        start_point.time_from_start.nsecs = 500000
        traj.points.append(start_point)
        end_goal = [0, math.pi / 2, 0, 0, 0, 0]
        n_points = 100
        s_counter = 1
        # slowly increment joints till goal
        for i in range(n_points):
            new_point = JointTrajectoryPoint()
            new_point.positions = []
            for x in end_goal:
                new_point.positions.append(x*(i+1)/n_points)
                new_point.time_from_start.secs = s_counter
                s_counter += 1
            traj.points.append(new_point)
        
        goal.trajectory = traj

        tolerance = []
        for joint in self.joint_names:
            tol = JointTolerance()
            tol.name = joint
            tol.position = 0
            tol.velocity = 0
            tol.acceleration = 0
            tolerance.append(tol)
        goal.path_tolerance = tolerance
        goal.goal_tolerance = tolerance

        goal.goal_time_tolerance.secs = 5

        jt_action_goal.goal = goal
        return jt_action_goal



def main():
    try:
        tm = TrajManager()
        tm.traj_publisher()
    except rospy.ROSInterruptException:
        pass