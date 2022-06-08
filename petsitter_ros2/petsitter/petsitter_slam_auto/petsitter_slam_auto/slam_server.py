#################################################################################
# Autor: Abidán Brito
# Fecha: 25/05/2022
# Nombre del fichero: slam_server.py
# Descripción: Definición de la clase SlamService, que crea el nodo para el 
#              SLAM autónomo. for the autonomous SLAM.
#################################################################################

# Librerías de mensajes
from petsitter_custom_interface.srv import ScanMsg
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Liberías de python para ROS2
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

# Importar temporizador
from threading import Timer
import os

class SlamService(Node):
    """
  	Nodo que crea un servicio para el SLAM autónomo del entorno.

  	Attributes:
    	srv (service): servicio del servidor.
    	_scanning (bool): bandera de escaneo.
        _angular_velocity (float): velocidad angular por defecto.
        _linear_velocity_max (float): velocidad lineal máxima aplicable.
        _collision_max (int): máximo de colisiones antes de resetear.
        _collision_counter (int): contador de colisiones detectadas.
        _publisher (publisher): publisher para aplicar velocidades en '/cmd_vel'.
  
  	Methods:
    	listener_scan_callback(): callback de la suscripción al topic '/scan'.
        service_callback(): callback del servicio. Comienza el escaneo al recibir el request 'scan'.
        _found_collision(): detecta colisiones para un rango de distancias.
        _publish_twist_msg(): publica un mensaje de tipo twist con las velocidades suministradas.
        _subscribe_to_scan_topic(): se suscribe al topic '/scan' y enlaza 'listener_scan_callback()'.
  	"""

    def __init__(self):
        """
        Crea el servicio del servidor de SLAM autónomo, se suscribe al topic '/scan' y
        crea el objeto publicador 'self._publisher'.
        """
        super().__init__('slam_server') 

        # Creamos el servicio (tipo de mensaje, nombre del servicio y callback)
        self.get_logger().info('--- Firing up autonomous_slam service ---')
        self.srv = self.create_service(ScanMsg, 'autonomous_slam', self.service_callback)

        # Banderas
        self._scanning = False

        # Velocidad
        self._angular_velocity = 1.6
        self._linear_velocity_max = 0.2 # NOTA(abi): no pasar de 0.2 para el robot real

        # Colisiones
        self._collision_max = 15
        self._collision_counter = self._collision_max

        # NOTA(abi): hace falta suscribirse al topic /scan para detectar colisiones
        self._subscribe_to_scan_topic()

        # Objeto publicador (tipo de mensaje, nombre del topic y tamaño de cola)
        self._publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # NOTA(abi): este es el tiempo (en segundos) que tiene el servicio para escanear el entorno.
        self._countdown = Timer(90, self._end_scan_pass, args=[])

    def listener_scan_callback(self, msg):
        """ 
        Callback del listener de la suscripción al topic '/scan'. Si la bandera de escaneado está
        levantada publica un mensaje de tipo Twist y comienza el SLAM. En caso de encontrar
        colisiones decrementa el contador y a las 15 colisiones lo resetea, junto con la velocidad
        angular.
        
        Args:
            msg: mensaje que recibe del topic, con un array de distancias.
       """
        # Lado derecho (0 - 15 º)
        msg_distance_array = msg.ranges[0:15]

        # Left side (345 - 360 º)
        msg_distance_array.extend(msg.ranges[345:359])

        if self._scanning and self._found_collision(msg_distance_array, 0.3):
            # Colisión encontrada      
            self._collision_counter -= 1

            # A las 15 colisiones reseteamos el contador 
            if self._collision_counter == 0:
                self._collision_counter = self._collision_max
                self._angular_velocity = -self._angular_velocity

            # Parar robot
            self._publish_twist_msg(0.0, self._angular_velocity)

        elif self._scanning:
            self._publish_twist_msg(self._linear_velocity_max, 0.0)

    def service_callback(self, request, response):
        """ 
        Si se recibe la orden de escanear, aplica velocidad lineal al robot (publicando un mensaje de tipo Twist) 
        y levanta la bandera 'self._scanning'.
        
        Args:
            request (string): Contiene la orden de comenzar el escaneado.
            response (bool): Indica si se ha obtenido la orden y es válida.
       """
        if request.scan == "start":
            self.get_logger().info('--- Received start order ---')
            self._publish_twist_msg(self._linear_velocity_max, 0.0)
            self._scanning = True
            response.success = True
        
        elif request.scan == "countdown":
            self.get_logger().info('--- Received countdown order ---')
            self._publish_twist_msg(self._linear_velocity_max, 0.0)
            self._countdown.start()
            self._scanning = True
            response.success = True

        elif request.scan == "stop":
            self.get_logger().info('--- Received stop order ---')
            self._publish_twist_msg(0.0, 0.0)
            self._scanning = False
            response.success = True

        else:
            response.success = False

        return response

    def _found_collision(self, distance_range, collision_threshold):
        """ Detecta posibles colisiones con un pequeño margen, para evitar que se produzca el
        choque.

        Args:
            distance_range (float): Array de distancias para los 360º del robot, o menos, si se el rango.
            collision_range (float): Determina la distancia mínima a partir de la cual se considera colisión.
       """
        is_collision = False

        for d in distance_range:
            if d <= collision_threshold:
                is_collision = True
                #self.get_logger().info('--- Collision detected at ' + str(d) + 'm ---')
                break

        return is_collision
    
    def _publish_twist_msg(self, linear, angular):
        """
        Crea un mensaje de tipo Twist con los valores de velocidad lineal y angular y lo publica en el
        topic cmd_vel a través del publisher. 
        """
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular

        self._publisher.publish(msg)

    def _subscribe_to_scan_topic(self):
        """
        Crea una suscripción al topic /scan con la mejor QoS y la enlaza a listener_scan_callback().
        """
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_scan_callback,
            QoSProfile(depth = 10, reliability = ReliabilityPolicy.BEST_EFFORT))

        self.subscriber

    def _end_scan_pass(self):
        """
        Termina el escaneado del entorno, guarda el mapa generado y lo sube a Firebase Database.
        """
        self.get_logger().info('--- Ending SLAM pass ---')
        self._scanning = False
        self._publish_twist_msg(0.0, 0.0)
       
        self.get_logger().info('--- Saving out generated map ---')
        os.system("ros2 run nav2_map_server map_saver_cli -f $HOME/turtlebot3_ws/src/petsitter/petsitter_ros2/petsitter/petsitter_nav2_system/map/map_auto")
        os.system("python $HOME/turtlebot3_ws/src/petsitter/petsitter_ros2/petsitter/petsitter_slam_auto/petsitter_slam_auto/upload_map_firestore.py")

def main(args=None):
    """
    Crea un objeto de tipo SlamService, levanta el nodo del servicio y, al recibir la orden por teclado, lo destruye.
    """
    rclpy.init(args=args)
    slam_service = SlamService()

    try:
        rclpy.spin(slam_service)

    except KeyboardInterrupt:
        slam_service.get_logger().info('--- Destroying autonomous_slam service node ---')

    finally:
        slam_service.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()