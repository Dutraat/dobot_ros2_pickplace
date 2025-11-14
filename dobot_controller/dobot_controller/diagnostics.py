import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

class DobotLedMonitor(Node):
    def __init__(self):
        super().__init__('dobot_led_monitor')
        self.subscription = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.callback,
            10
        )
        self.last_states = {}
        self.get_logger().info('🤖 Monitor de LED do Dobot iniciado')
        self.get_logger().info('🟢 Aguardando mensagens de diagnóstico...')

    def callback(self, msg: DiagnosticArray):
        for status in msg.status:
            name = status.name
            level = status.level
            message = status.message
            
            # Mapeia WARN e ERROR para estado "crítico"
            critical = level in [DiagnosticStatus.WARN, DiagnosticStatus.ERROR]
            
            # Verifica se já temos um estado anterior
            if name in self.last_states:
                last_level = self.last_states[name]
                last_critical = last_level in [DiagnosticStatus.WARN, DiagnosticStatus.ERROR]
                
                # Só loga se houve mudança de estado
                if last_critical != critical:
                    if critical:
                        self.get_logger().error(
                            f"🔴 [ALERTA] {name} → LED VERMELHO (fora dos limites): {message}"
                        )
                    else:
                        self.get_logger().info(
                            f"🟢 [OK] {name} → LED VERDE (dentro dos limites): {message}"
                        )
            else:
                # Primeira vez vendo este componente - mostra estado inicial
                if critical:
                    self.get_logger().warn(f"🔴 {name}: Estado inicial CRÍTICO - {message}")
                else:
                    self.get_logger().info(f"🟢 {name}: Estado inicial OK - {message}")
            
            # Atualiza o estado
            self.last_states[name] = level

def main(args=None):
    rclpy.init(args=args)
    node = DobotLedMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Monitor encerrado pelo usuário')
    except Exception as e:
        node.get_logger().error(f'Erro inesperado: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
