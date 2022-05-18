from http import client
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from custom_interface.srv import NavMessage

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import sys

class NavToPoseActionClient(Node):
    """
    Cliente de la accion NavigateToPose

    Attributes:
        action_client (ActionClient): El cliente de la acción NavigateToPose
        srv (service): servidor del servicio send_pose
  
    Methods:
        my_first_service_callback(): envia la posición a la que tiene que dirigirse el robot cada vez que recibe una peticion
        send_goal(): manda la posicion final 
        goal_response_callback(): escribe mensajes por la terminal dependiendo de si se ha aceptado la posición final o no
        get_result_callback(): informa de si se ha llegado a la meta y de no ser el caso, continua publicando la posicion final
        feedback_callback(): envía mensajes durante el trayecto hacia la meta

    """
    goal_pose = NavigateToPose.Goal()
	
    def __init__(self):
        """
        Crea el cliente de la accion y el servidor del servicio asociado a él
        """
        super().__init__('action_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.srv = self.create_service(NavMessage, 'send_pose', self.my_first_service_callback)
        
    def my_first_service_callback(self, request, response):
       """ Envia la posición a la que tiene que dirigirse el robot cada vez que recibe una peticion
       Args:
        request (float): Contiene las coordenadas x e y de la posición final
        response (bool): Indica si se ha mandado la posición y si es válida
        
       """
       # recibe los parametros de esta clase
       #  recibe el mensaje request
       # devuelve el mensaje response
       self.goal_pose.pose.header.frame_id = 'map'
       self.goal_pose.pose.pose.position.x = request.x
       self.goal_pose.pose.pose.position.y = request.y
       self.goal_pose.pose.pose.position.z = 0.0
       self.goal_pose.pose.pose.orientation.x = 0.0
       self.goal_pose.pose.pose.orientation.y = 0.0
       self.goal_pose.pose.pose.orientation.z = 0.0
       self.goal_pose.pose.pose.orientation.w = 1.0

       if self.goal_pose.pose.pose.position.x != 0 or self.goal_pose.pose.pose.position.y != 0:
           # imprime mensaje informando del movimiento
           self.get_logger().info('Se recibió la nueva posicion')
           self.get_logger().info('Goal creado :)')

           self._action_client.wait_for_server()

           self.get_logger().info('Acción activa :)')

           self._send_goal_future = self._action_client.send_goal_async(
                self.goal_pose,
                feedback_callback=self.feedback_callback)

           self._send_goal_future.add_done_callback(self.goal_response_callback)
           self.get_logger().info('Goal lanzado :)')
           # devuelve la respuesta
           response.success = True
       else:
           # estado de la respuesta
           # si no se ha dado ningun caso anterior
           response.success = False

       # devuelve la respuesta
       return response

    def send_goal(self):
        """ 
        Envia la posición a la que tiene que dirigirse el robot
        """
        self.get_logger().info('Goal debug :)')

        self._action_client.wait_for_server()

        self.get_logger().info('Acción activa :)')

        self._send_goal_future = self._action_client.send_goal_async(
            self.goal_pose,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info('Goal lanzado :)')

    def goal_response_callback(self, future):
        """ 
        Escribe mensajes por la terminal dependiendo de si se ha aceptado la posición final o no
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
            
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """ 
        Informa de si se ha llegado a la meta y de no ser el caso, continua publicando la posicion final
        """

        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded! ')
        else:
            self.get_logger().info('Navigation failed with status: {0}'.format(status))
            self.get_logger().info('Try again...')
            self.send_goal()


    def feedback_callback(self, feedback_msg):
        """ 
        Envía mensajes durante el trayecto hacia la meta
        """
        feedback = feedback_msg.feedback

def main(args=None):
    rclpy.init(args=args)

    action_client = NavToPoseActionClient()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
