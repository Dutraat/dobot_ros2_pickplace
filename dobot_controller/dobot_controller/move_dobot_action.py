import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import GripperControl
import time

class DobotRelativeClient(Node):
    def __init__(self):
        super().__init__('dobot_relative_client')
        self._action_client = ActionClient(self, PointToPoint, 'PTP_action')
        self._gripper_client = self.create_client(GripperControl, 'dobot_gripper_service')
        
        self.current_pose = [0.0, 0.0, 0.0, 0.0]  # Inicializa posição atual
        self.ok = True

        while not self._gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gripper service not available, waiting...')

    def send_gripper(self, status):
        req = GripperControl.Request()
        req.gripper_state = status
        req.keep_compressor_running = False
        return self._gripper_client.call_async(req)

    def send_relative_goal(self, delta):
        """Envia um movimento relativo em XYZ usando MOVL_INC"""
        goal_msg = PointToPoint.Goal()
        goal_msg.motion_type = 7  # MOVL_INC
        goal_msg.target_pose = [
            self.current_pose[0] + delta[0],
            self.current_pose[1] + delta[1],
            self.current_pose[2] + delta[2],
            self.current_pose[3] + delta[3]
        ]

        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        return future

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.current_pose = feedback.current_pose
        self.get_logger().info(
            'Current pose: x={0:.2f}, y={1:.2f}, z={2:.2f}, r={3:.2f}'.format(*self.current_pose)
        )

    def wait_for_result(self, goal_future):
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.current_pose = result.achieved_pose
        return result

def main(args=None):
    rclpy.init(args=args)
    client = DobotRelativeClient()

    # 1️⃣ Abrir gripper
    client.get_logger().info('Opening gripper...')
    future = client.send_gripper('open')
    rclpy.spin_until_future_complete(client, future)
    time.sleep(1)

    # 2️⃣ Mover para posição de pegar (relativo)
    client.get_logger().info('Moving to pick position (relative)...')
    goal_future = client.send_relative_goal([50.0, 0.0, -50.0, 0.0])  # exemplo: frente 50mm, baixo 50mm
    client.wait_for_result(goal_future)
    time.sleep(1)

    # 3️⃣ Fechar gripper para pegar objeto
    client.get_logger().info('Closing gripper...')
    future = client.send_gripper('close')
    rclpy.spin_until_future_complete(client, future)
    time.sleep(1)

    # 4️⃣ Levantar objeto (relativo)
    client.get_logger().info('Lifting object...')
    goal_future = client.send_relative_goal([0.0, 0.0, 50.0, 0.0])  # sobe 50mm
    client.wait_for_result(goal_future)
    time.sleep(1)

    # 5️⃣ Mover para posição de soltar (relativo)
    client.get_logger().info('Moving to place position (relative)...')
    goal_future = client.send_relative_goal([100.0, 50.0, 0.0, 0.0])  # deslocamento XY
    client.wait_for_result(goal_future)
    time.sleep(1)

    # 6️⃣ Abrir gripper para soltar objeto
    client.get_logger().info('Opening gripper to release object...')
    future = client.send_gripper('open')
    rclpy.spin_until_future_complete(client, future)
    time.sleep(1)

    # 7️⃣ Opcional: voltar para home (relativo)
    client.get_logger().info('Returning to initial relative position...')
    goal_future = client.send_relative_goal([-150.0, -50.0, 0.0, 0.0])
    client.wait_for_result(goal_future)
    time.sleep(1)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

