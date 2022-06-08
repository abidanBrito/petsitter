#################################################################################
# Autor: Abidán Brito
# Fecha: 25/05/2022
# Nombre del fichero: initial_pose_pub.py
# Descripción: Publicador de la posición inicial del robot (en el entorno real).
#################################################################################

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    """
  	Nodo que utiliza un publisher para publicar la posicion inicial del robot en el topic initialpose

  	Attributes:
    	publisher (publisher): publisher
    	timer (timer): Cuenta que indica cada cuanto tiempo se ejecuta el callback
  
  	Methods:
    	callback(): Introduce los valores de la posicion en un mensaje de tipo PoseWithCovarianceStamped() y lo publica

  	"""

    def __init__(self):
        """
        Crea el publisher y le asigna un callback con un temporizador
        """
        super().__init__('initial_pose_pub_node')
        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, 
            'initialpose',
            QoSProfile(depth = 1, reliability = ReliabilityPolicy.BEST_EFFORT))
        timer_period = 0.5  # seconds
        self.i = 0.0
        self.timer_ = self.create_timer(timer_period, self.callback)

    def callback(self):
        """
        Coloca los valores de la posicion inicial en el mensaje y lo publica
        """
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        
        # Position
        msg.pose.pose.position.x = -0.75
        msg.pose.pose.position.y = -0.95
        
        # Orientation
        msg.pose.pose.orientation.z = 0.42
        msg.pose.pose.orientation.w = 0.90
        self.get_logger().info('Publishing  Initial Position  \n Position X = -0.75 \n Position Y = -0.95 \n Orientation Z = 0.42 \n Orientation W = 0.90 ')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    publisher = InitialPosePublisher()
    try:
        rclpy.spin_once(publisher)
    except KeyboardInterrupt:
        publisher.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
