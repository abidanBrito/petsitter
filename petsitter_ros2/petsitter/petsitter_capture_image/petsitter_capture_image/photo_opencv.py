import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

#Firebase
import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore
from firebase_admin import storage
import pyrebase
from datetime import datetime

# Importar mensajes
from geometry_msgs.msg import Twist
from petsitter_custom_interface.srv import OpencvMsg


# credenciales para Firestore con firebase_admin
cred = credentials.Certificate("/home/ruben/turtlebot3_ws/src/petsitter-main/petsitter_ros2/petsitter/petsitter_capture_image/petsitter_capture_image/firebase-sdk.json")
firebase_admin.initialize_app(cred)



        # credenciales para Storage con pyrebase

config = {
            'apiKey': "AIzaSyABOCK6NTYF1jsv1WT5SiDZdFgUuhXLH2o",

            'authDomain': "petsitter-42488.firebaseapp.com",

            'projectId': "petsitter-42488",

            'storageBucket': "petsitter-42488.appspot.com",

            'messagingSenderId': "590637458117",

            'appId': "1:590637458117:web:e0681e78d84b708a3c46ac",

            "databaseURL": ""
}

firebase = pyrebase.initialize_app(config)

class ServiceOpencv(Node):

    """
  	Nodo que crea un servicio para tomar una foto y analizarla

  	Attributes:
    	srv (service): servicio del servidor
    	resultado (float): valor final del escaneado (0/33/66/100)%
        bridge: puente
        image_sub: lanza el topic y la funcion camera_callback
        analize (bool): indica cuando hay que analizar la imagen
  
  	Methods:
    	camera_callback(): callback de la suscripción al topic '/camera/image_raw'
        my_firts_service_callback(): callback del servicio. Comienza a analizar al recibir el request 'analize = True'

  	"""

    def __init__(self):
        #constructor con el nombre del nodo
        super().__init__('Ros2OpenCVImageConverter') 
       
        self.resultado = 10
        #image para el topic real
        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(Image,'/camera/image_raw',self.camera_callback,QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.srv = self.create_service(OpencvMsg, 'analize', self.my_first_service_callback)

        self.analize = False

    #print("Entra en el servicio")

    def camera_callback(self,data):
        #print("Entra al callback")
        """ 
        Callback del listener de la suscripción al topic '/camera/image_raw' Si analize = True entonces toma una 
        foto y comienza a analizarla con opencv. Por una parte mira la diferencia entre un cuenco vacio y el actual 
        y por otra la cantidad de marron que hay, es decir la cantidad de pienso del cuenco. A firebase le enviamos el
        menor valor de los 2 ya que suponemos que a en el caso de que uno de los resultados sea erroneo se envia mas bajo
        para así rellenar la comida cuanto antes
        
        Args:
            ---
       """

        if self.analize == True:
            print("Analize es true")

            try:
                # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
                cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
                print("Se ejecuta")

            except CvBridgeError as e:
                print(e)

            cv2.imshow("Imagen capturada por el robot", cv_image)
                
            cv2.waitKey(1)    
            cv2.imwrite("/home/ruben/turtlebot3_ws/src/petsitter-main/petsitter_web/imgs/Imagen_cuenco.jpg", cv_image)
        
            #Opencv
            #Cargamos las dos imágenes
            img1 = cv2.imread("/home/ruben/turtlebot3_ws/src/petsitter-main/petsitter_web/imgs/Imagen_cuenco.jpg")
            img2 = cv2.imread("/home/ruben/turtlebot3_ws/src/petsitter-main/petsitter_web/imgs/Imagen_cuenco.jpg")

            #Comparamos cada imagen con la original mediate subtract: sustrae cada píxel y valor de color
            diff_12 = cv2.subtract(img1,img2)

            #Sumamos todos los elementos del array diferencia para obtener un número representativo de cómo de similares o diferentes son
            print("--------------------------------------------------------")
            print("DIFERENCIA: ")
            resto = np.sum(diff_12)
            print(resto)

            if(resto < 5000000):
                valor_diferencia = 0
                print("0%")
            elif(resto >= 5000000 and resto <= 7000000):
                valor_diferencia = 33
                print("33%")
            elif(resto > 7000001 and resto <= 10075000):
                valor_diferencia = 66
                print("66%")
            elif(resto > 10075001 and resto <= 10700000):
                valor_diferencia = 100
                print("100%")
            else:
                print("Hay demasiada cantidad de comida")
                valor_diferencia = 100000

        #--------------------------------------------------------------------------------------------------
            hsv = cv2.cvtColor(img2,cv2.COLOR_BGR2HSV)
            dark_blue=np.uint8([[[255,0,0]]])
            hsv_dark_blue = cv2.cvtColor(dark_blue,cv2.COLOR_BGR2HSV)

            lower_brown = np.array([0,140,0])#rango de color marrón que se capta
            upper_brown = np.array([255,255,255])

            cv2.imshow('hsv',hsv)

            mask = cv2.inRange(hsv, lower_brown, upper_brown)
            cv2.imshow('mask',mask)

            res = cv2.bitwise_and(img2, img2, mask= mask)
            cv2.imshow('res',res)

            #cantidad de pixeles marrones
            total = res.size
            #print(total) #píxeles verticales y horizontales

            num_black_px = np.sum(res == 0)
            #print(num_black_px)#pixeles negros

            px_marron = total - num_black_px
            print("--------------------------------------------------------")
            print("PIXELES MARRONES: ")
            print(px_marron)#pixeles marrones

            if(px_marron < 50000):
                valor_color = 0
                print("0%")
            elif(px_marron >= 50000 and px_marron <= 90000):
                valor_color = 33
                print("33%")
            elif(px_marron > 90001 and px_marron <= 110350):
                valor_color = 66
                print("66%")
            elif(px_marron > 110351 and px_marron <= 130000):
                valor_color = 100
                print("100%")
            else:
                print("Hay demasiada cantidad de comida")
                valor_color = 100000

            print("--------------------------------------------------------")

            if(valor_diferencia == valor_color):
                valor_final = valor_color
            elif(valor_diferencia > valor_color):
                valor_final = valor_color
            else:
                valor_final = valor_diferencia

            print("VALOR FIABLE:")
            print(valor_final, "%")
            print("--------------------------------------------------------")

        #cv2.waitKey(0) #aprieta una tecla 
        #cv2.destroyAllWindows()

            self.resultado = valor_final
            print(self.resultado)

            def get_resultado(self):
                print(" resultado del metodo")
                print(self.resultado)

                return self.resultado

        # subir imagen a Storage
            storage = firebase.storage()

            my_image = "/home/ruben/turtlebot3_ws/src/petsitter-main/petsitter_web/imgs/Imagen_cuenco.jpg"
            image_path = "/images/Imagen_cuenco.jpg" # ruta donde se guarda la imagen

            # Upload image
            storage.child(image_path).put(my_image)
            img_url = storage.child(image_path).get_url(None)

            fecha_actual = datetime.now()
            fecha = fecha_actual.strftime("%d/%m/%Y %H:%M")

            # subir documento a Firestore
            db = firestore.client()

            doc_ref = db.collection(u'Imagenes').document(u'cuenco')
            doc_ref.set({
                u'url' : img_url,
                u'cantidad' : self.resultado,
                u'fecha' : fecha
            })

            self.analize = False




    def my_first_service_callback(self, request, response):
        """ 
        Si se recibe la orden de analizar, se le poner self.analize a true
        
        Args:
            request (string): Contiene la orden de comenzar a analizar
            response (bool): Indica si se ha obtenido la orden y es válida
        """
        if request.analize == True:
            #print("Detecta pienso")
            self.analize = True
            response.success = True
        else:
            response.success = False
            self.analize = False
        return response
        

def main(args=None):
    # inicializa la comunicacion ROS2
    rclpy.init(args=args)
    # creamos el nodo
    service = ServiceOpencv()
    try:
        #dejamos abierto el servicio
        rclpy.spin(service)
    except KeyboardInterrupt:
        service.destroy_node()
        service.get_logger().info('Cerrando el nodo service')
    finally:
        #destruimos el nodo
        service.destroy_node()
        #cerramos la comunicacion
        rclpy.shutdown()

cv2.destroyAllWindows()
    
#definimos el ejecutable
if __name__=='__main__':
    main()
