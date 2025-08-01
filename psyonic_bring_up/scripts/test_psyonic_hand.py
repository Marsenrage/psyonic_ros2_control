#!/usr/bin/env python3
"""
Simple Psyonic Hand Position Command Test for ROS2 Jazzy
Tests basic position commands using trajectory controller
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class SimplePsyonicTest(Node):
    
    def __init__(self):
        super().__init__('simple_psyonic_test')
        
        # Joint names
        self.joint_names = [
            'Left_Hand_index_q1',
            'Left_Hand_middle_q1', 
            'Left_Hand_ring_q1',
            'Left_Hand_pinky_q1',
            'Left_Hand_thumb_q2',
            'Left_Hand_thumb_q1'
        ]
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/left_hand/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Action client for sending position commands
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/left_hand/psyonic_left_hand/follow_joint_trajectory'
        )
        
        self.current_positions = None
        
    def joint_state_callback(self, msg):
        """Get current joint positions"""
        if len(msg.name) >= 6:
            # Create position dictionary
            pos_dict = {}
            for i, name in enumerate(msg.name):
                if name in self.joint_names:
                    pos_dict[name] = msg.position[i]
            
            # Store positions in correct order
            self.current_positions = [pos_dict.get(name, 0.0) for name in self.joint_names]
    
    def send_position_command(self, positions, duration=2.0):
        """Send position command to hand"""
        
        # Wait for action server
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return False
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        trajectory.points = [point]
        
        # Create and send goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        self.get_logger().info(f"Sending position command: {[f'{p:.2f}' for p in positions]}")
        
        future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        
        if future.result() is None:
            self.get_logger().error("Failed to send goal")
            return False
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected")
            return False
        
        self.get_logger().info("Goal accepted, waiting for completion...")
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 2.0)
        
        if result_future.result() is None:
            self.get_logger().error("Goal execution timed out")
            return False
        
        result = result_future.result().result
        if result.error_code == 0:
            self.get_logger().info("✓ Position command completed successfully!")
            return True
        else:
            self.get_logger().error(f"Goal failed with error code: {result.error_code}")
            return False
    
    def run_test(self):
        """Run simple position test"""
        self.get_logger().info("Starting simple position test...")
        
        # Wait for joint states
        self.get_logger().info("Waiting for joint states...")
        while self.current_positions is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        self.get_logger().info(f"Current positions: {[f'{p:.2f}' for p in self.current_positions]}")
        
        # Test 1: Small flex
        self.get_logger().info("\n--- Test 1: Small finger flex ---")
        small_flex = [0.3, 0.3, 0.3, 0.3, 0.3, -0.3]  # Small safe movements
        if not self.send_position_command(small_flex, duration=3.0):
            return False
        
        time.sleep(1.0)
        
        # Test 2: Return to neutral
        self.get_logger().info("\n--- Test 2: Return to neutral ---")
        neutral = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if not self.send_position_command(neutral, duration=3.0):
            return False
        
        time.sleep(1.0)
        
        # Test 3: Individual finger test
        self.get_logger().info("\n--- Test 3: Individual finger test ---")
        for i, joint_name in enumerate(self.joint_names):
            self.get_logger().info(f"Testing {joint_name}")
            
            # Create position array with only one joint moving
            test_pos = [0.0] * 6
            if i == 5:  # thumb_q1 has negative range
                test_pos[i] = -0.5
            else:
                test_pos[i] = 0.5
            
            if not self.send_position_command(test_pos, duration=2.0):
                return False
            
            time.sleep(0.5)
            
            # Return to neutral
            neutral = [0.0] * 6
            if not self.send_position_command(neutral, duration=2.0):
                return False
            
            time.sleep(0.5)
        
        self.get_logger().info("\n🎉 All position tests completed successfully!")
        return True

def main():
    rclpy.init()
    
    node = SimplePsyonicTest()
    
    try:
        # Give time for connections
        time.sleep(2.0)
        
        # Run the test
        success = node.run_test()
        
        if success:
            node.get_logger().info("✓ Test PASSED")
        else:
            node.get_logger().error("✗ Test FAILED")
            
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted")
    except Exception as e:
        node.get_logger().error(f"Test error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()