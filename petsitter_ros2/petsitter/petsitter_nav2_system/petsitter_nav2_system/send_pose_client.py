#importamos el mensaje
from custom_interface.srv import MyMoveMsg
#importamos la bib ROS2 para python
import rclpy
from rclpy.node import Node
#importamos la bib sys para poder usar los arg de entrada
import sys
from custom_interface.srv import NavMessage

class ClientAsync(Node):
    """
    Cliente del servicio send_pose

    Attributes:
        client (client): El cliente del servicio send_pose
        req (NavMessage.Request()): La petición del servicio send_pose
  
    Methods:
        send_request(): envia la posición introducida por la terminal como request al servicio send_pose

    """

    def __init__(self):
        """
        Crea el cliente del servicio
        """
        #inicializa el nodo cliente
        super().__init__('send_pose_client')
        #crea el objeto cliente
        self.client = self.create_client(NavMessage, 'send_pose')
        #cada segundo revisa si el servicio esta activo
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('el servicio no esta activo, prueba de nuevo...')
        
        #crea el mensaje 
        self.req = NavMessage.Request()

    def send_request(self):
        """
        Almacena los valores introducidos por la terminal en la request del servicio y los manda a la parte del servidor del servicio
        """
        # usa sys.argv para tener acceso a los argumentos introducidos en la
        # llamada al programa por consola
        self.req.x = float(sys.argv[1])
        self.req.y = float(sys.argv[2])
        #envia la peticion del servicio
        self.future = self.client.call_async(self.req)


def main(args=None):
    #inicializa la comunicacion ROS2
    rclpy.init(args=args)
    #declara el constructor del objeto cliente
    client = ClientAsync()
    #ejecuta el metodo de peticion de servicio
    client.send_request()

    while rclpy.ok():
        #deja el nodo abierto hasta recibir ctrl+c
        rclpy.spin_once(client)
        #si se ha enviado el mensaje future
        if client.future.done():
            try:
                # chequea el mensaje future
                # si se ha enviado una respuesta 
                # la recoge
                response = client.future.result()
            except Exception as e:
                client.get_logger().info('La llamada al servicio ha fallado %r' % (e,))
        else:
            client.get_logger().info('Respuesta del servicio %r' % (response.success,))
        break
    client.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()