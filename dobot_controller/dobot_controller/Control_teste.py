import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from dobot_msgs.action import PointToPoint # Importa a definição da Action Trajectory
from dobot_msgs.srv import GripperControl

class DobotPointToPointClient(Node):
    def __init__(self):
        super().__init__('dobot_p2p_client')
        self._action_client = ActionClient(self, PointToPoint, 'PTP_action')
        
        self._gripper_client = self.create_client(GripperControl, 'dobot_gripper_service')
        
        self.ok = True
        
        while not self._gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gripper not available, waiting again...')
    
    def send_gripper(self, status):
        req = GripperControl.Request()
        req.gripper_state = status 
        req.keep_compressor_running = False
        return self._gripper_client.call_async(req)
    
    def wait_for_result(self, goal_future):
        """Aguarda a conclusão de uma action"""
        # Primeiro aguarda a aceitação do goal
        rclpy.spin_until_future_complete(self, goal_future)
        
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return None
        
        self.get_logger().info('Goal accepted :)')
        
        # Agora aguarda o resultado
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        self.get_logger().info('Result: {0},{1},{2},{3}'.format(*result.achieved_pose))
        return result
        
    def send_goal(self, position):
        if not self.ok:
            print("Client not ready")
            return None
            
        goal_msg = PointToPoint.Goal()
        goal_msg.motion_type = goal_msg.MOTION_TYPE_MOVL_XYZ
        goal_msg.target_pose = position
        
        self._action_client.wait_for_server()
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        return self._send_goal_future
      
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
        return result
         
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0:.3f}, {1:.3f}, {2:.3f}, {3:.3f}'.format(*feedback.current_pose))

def main(args=None):
    import time
    rclpy.init(args=args)
    
    action_client = DobotPointToPointClient()
    
    # Gripper open
    action_client.get_logger().info('Opening gripper...')
    future = action_client.send_gripper('open')
    rclpy.spin_until_future_complete(action_client, future)
    response = future.result()
    action_client.get_logger().info(f'Gripper open response: {response.success}: {response.message}')
    time.sleep(2)  # Aumentei o tempo para garantir que a operação complete
    
    # Primeiro movimento - Home 
    action_client.get_logger().info('Primeira Posição...')
    goal_future = action_client.send_goal([200.0, 0.0, 100.0, 0.0])
    if goal_future:
        action_client.wait_for_result(goal_future)
    time.sleep(1)
    
    # Segunda posição - Pegar o objeto  
    action_client.get_logger().info('Primeira Posição...')
    goal_future = action_client.send_goal([168.0, 200.0, 2.0, 0.0])
    if goal_future:
        action_client.wait_for_result(goal_future)
    time.sleep(1)
    
    # Gripper close  
    action_client.get_logger().info('Closing gripper...')
    future = action_client.send_gripper('close')
    rclpy.spin_until_future_complete(action_client, future)  # ADICIONADO: aguarda a resposta
    response = future.result()
    action_client.get_logger().info(f'Gripper close response: {response.success}: {response.message}')
    time.sleep(2)  
    
    # Segundo movimento
    action_client.get_logger().info('Segunda Posição...')
    goal_future = action_client.send_goal([-60.0,250.0,10.0,20.0])
    if goal_future:
        action_client.wait_for_result(goal_future)
    time.sleep(1)
    
    # Gripper open
    action_client.get_logger().info('Opening gripper again...')
    future = action_client.send_gripper('open')
    rclpy.spin_until_future_complete(action_client, future)
    response = future.result()
    action_client.get_logger().info(f'Gripper final open response: {response.success}: {response.message}')
    
    action_client.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
