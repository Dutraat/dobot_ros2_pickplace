import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool


class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')

        # Publica alvos de movimento
        self.publisher_ = self.create_publisher(Float64MultiArray, '/dobot_target_pose', 10)

        # Escuta feedback de conclusão do movimento
        self.create_subscription(Bool, '/dobot_motion_done', self.feedback_callback, 10)

        # --- Ações da garra ---
        self.GRIPPER_OPEN = 1.0
        self.GRIPPER_CLOSE = 0.0

        # --- Lista de posições e ações correspondentes ---
        self.target_poses = [
            [200.0, 0.0, 100.0, 0.0],   # Posição inicial
            [168.0, 200.0, 2.0, 0.0],   # Pegar objeto
            [-60.0, 250.0, 10.0, 20.0]  # Soltar objeto
        ]
        self.gripper_actions = [
            self.GRIPPER_OPEN,
            self.GRIPPER_CLOSE,
            self.GRIPPER_OPEN
        ]

        # Estado interno - comando para esperar o movimento terminar ou começar 
        self.current_index = 0 # posição da lista de movimentos em que o braço esta realizando 
        self.motion_in_progress = False # Se estiver ainda em processo ele espera

        # Timer para publicar o primeiro alvo
        self.timer = self.create_timer(2.0, self.initial_publish)

        self.get_logger().info(' Nó intermediário iniciado — aguardando feedbacks do robô.')

    def initial_publish(self):
        """Publica o primeiro alvo e desativa o timer."""
        self.publish_next_target()
        self.timer.cancel()

    def publish_next_target(self):
        """Envia o próximo comando de movimento + garra."""
        if self.current_index >= len(self.target_poses):
            self.get_logger().info(' Todos os alvos foram enviados.')
            return

        pose = self.target_poses[self.current_index]
        gripper = self.gripper_actions[self.current_index]

        msg = Float64MultiArray()
        msg.data = pose + [gripper]

        self.publisher_.publish(msg)
        self.get_logger().info(
            f' Publicando Alvo {self.current_index + 1}: {pose} | Garra: {"Abrir" if gripper == 1.0 else "Fechar"}'
        )

        self.motion_in_progress = True
        self.current_index += 1

    def feedback_callback(self, msg: Bool):
        """Quando o controle confirma que o movimento terminou, publica o próximo."""
        if msg.data and self.motion_in_progress:
            self.motion_in_progress = False
            self.get_logger().info('Movimento concluído — enviando próximo alvo...')
            self.publish_next_target()


def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

