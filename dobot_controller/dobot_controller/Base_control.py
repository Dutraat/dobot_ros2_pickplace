import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from dobot_msgs.action import PointToPoint # Importa a definição da Action Trajectory
from dobot_msgs.srv import GripperControl


class DobotPointToPointClient(Node):
    def __init__(self):
        super().__init__('dobot_p2p_client')
        self._action_client = ActionClient(self,PointToPoint, 'PTP_action')
        self._gripper_client = self.create_client(GripperControl,'dobot_gripper_service')
        self.ok = True
        while not self._gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gripper not available, waiting again...')
    
    def send_gripper(self,status):
        req = GripperControl.Request()
        req.gripper_state = status 
        req.keep_compressor_running = False
        return self._gripper_client.call_async(req)


    def send_goal(self, position):
        if not self.ok:
            print()
            return

        goal_msg = PointToPoint.Goal()
        goal_msg.motion_type = goal_msg.MOTION_TYPE_MOVL_XYZ
        goal_msg.target_pose = position

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
      

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0},{1},{2},{3}'.format(*result.achieved_pose))
        #rclpy.shutdown()
        return result
         
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0:.3f}, {1:.3f}, {2:.3f}, {3:.3f}'.format(*feedback.current_pose))


def main(args=None):
    import time
    rclpy.init(args=args)

    action_client = DobotPointToPointClient()

    future = action_client.send_gripper('open')
    rclpy.spin_until_future_complete(action_client, future)
    time.sleep(1)
    future = action_client.send_goal([-60.0,250.0,10.0,20.0])
    rclpy.spin(action_client, future)
    time.sleep(1)
    future = action_client.send_gripper('close')
    rclpy.spin_until_future_complete(action_client, future)
    time.sleep(1)
    future = action_client.send_goal([168.0,200.0,5.0,50.0])
    rclpy.spin(action_client, future)
    time.sleep(1)
    future = action_client.send_gripper('open')
    rclpy.spin_until_future_complete(action_client, future)

    #future = action_client.send_goal([250.0,0.0,50.0,0.0])
    #future = action_client.send_goal([168.0,200.0,5.0,50.0])
    #future = action_client.send_goal([-60.0,250.0,50.0,20.0])


    #rclpy.spin(action_client, future)

    #future = action_client.send_gripper('open')
    #rclpy.spin_until_future_complete(action_client, future)

    #response = future.result()
    #action_client.get_logger().info(f'{response.success}: {response.message}')
    #time.sleep(1)
    #future = action_client.send_gripper('close')
    #rclpy.spin_until_future_complete(action_client, future)

    #response = future.result()
    #action_client.get_logger().info(f'{response.success}: {response.message}')

    
    action_client.destroy_node()
    rclpy.shutdown()
        

if __name__ == '__main__':
    main()