from tensorflow.keras.datasets.mnist import load_data
from tensorflow.keras.utils import to_categorical
from tensorflow.keras.models import load_model
from keras_preprocessing.image import ImageDataGenerator
from getpass import getuser
import rclpy
import cv2
import numpy as np
import time
from absl import logging
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from petsitter_custom_interface.srv import DetectMsg
import tensorflow as tf
from .models import (
    YoloV3, YoloV3Tiny
)
from .dataset import transform_images, load_tfrecord_dataset
from .utils import draw_outputs

import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore
from firebase_admin import storage

import pyrebase
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

from getpass import getuser

# credenciales para Firestore con firebase_admin

cred = credentials.Certificate("/home/ivan/prog_python_ws/firestore_credentials/firebase-sdk.json")
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

class Service(Node): 
    """
    Servicio que se encarga de analizar las imágenes que muestra el robot y cuando se encuentre con un perro o un gato, subirá la imagen a la base de datos
    indicando que tipo de animal es, y si se trata de un perro, también se indicará si se trata de un pastor alemán o no

    Attributes:
        detection (bool): Indica si el servicio está identficando a las mascotas o no
        bridge_object (CvBridge): permite convertir imágenes de ROS en imágenes de OpenCV
        image_sub (Subscriber): Suscriptor al topic que contiene las imágenes que publica la cámara del robot
        srv (Service): Servicio que analiza las imágenes para ver si se trata de perros o gatos
  
    Methods:
        camera_callback(): Analiza las imágenes de la cámara del robot y si se trata de perros o gatos, las sube a la base de datos
        service_callback(): modifica el atributo detection dependiendo del request que recibe

    """

    detection = False

    def __init__(self):

        super().__init__('Ros2OpenCVImageConverter')
        
        self.detection = False
        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(Image,'/camera/image_raw',self.camera_callback,QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.srv = self.create_service(DetectMsg, 'detect', self.service_callback)
        
    def camera_callback(self,data):
        """ Esta función utiliza los modelos de detección de mascotas para determinar si se trata de un perro, un gato o ninguno de los 2, y de tratarse de un perro,
        trata de determinar si se trata de un pastor aleman o no. De ser el caso, sube la imagen a la base de datos junto con la información de la detección

        Args: 
            data (ROS image): la imagen que capta la cámara del robot en el formato nativo que utiliza ROS

        """

        if self.detection == True:
            try:
                # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
                cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            except CvBridgeError as e:
                print(e)

            cv2.imshow("Imagen capturada por el robot", cv_image)
                    
            cv2.waitKey(1)    
            cv2.imwrite("/home/" +  str(getuser()) + "/turtlebot3_ws/src/Petsitter_Sprint3/petsitter/petsitter_ros2/petsitter/petsitter_detect_pets/corpus/test/deteccion.jpg", cv_image)
            
            BATCH_SIZE = 10
            WIDTH      = 250
            HEIGHT     = 250

            test_datagen = ImageDataGenerator(
                rescale = 1./255)

            test_generator = test_datagen.flow_from_directory(
                "/home/" +  str(getuser()) + "/turtlebot3_ws/src/Petsitter_Sprint3/petsitter/petsitter_ros2/petsitter/petsitter_detect_pets/corpus",
                batch_size=BATCH_SIZE,
                #color_mode="grayscale",
                target_size=(WIDTH, HEIGHT))
                
            #Importamos el modelo entrenado, que está en este directorio
            model = load_model("/home/" +  str(getuser()) + "/turtlebot3_ws/src/Petsitter_Sprint3/petsitter/petsitter_ros2/petsitter/petsitter_detect_pets/petsitter_model_real")

            predictions = model.predict(test_generator)
            #print(predictions.size / 3)
            for i in range(int(predictions.size / 3)):
                #print(i, predictions[i])
                if np.argmax(predictions[i]) == 0:
                    print("Parece que es un gato")

                    # subir imagen a Storage
                    storage = firebase.storage()

                    #my_image = "/home/" +  str(getuser()) + "/digitos/corpus_final_mejorado/training/gato/00000000.jpg" # ruta local de la imagen
                    my_image = "/home/" +  str(getuser()) + "/turtlebot3_ws/src/Petsitter_Sprint3/petsitter/petsitter_ros2/petsitter/petsitter_detect_pets/corpus/test/deteccion.jpg"
                    image_path = "/images/mascota.jpg" # ruta donde se guarda la imagen

                    # Upload image
                    storage.child(image_path).put(my_image)
                    img_url = storage.child(image_path).get_url(None)

                    # subir documento a Firestore
                    db = firestore.client()

                    doc_ref = db.collection(u'Imagenes').document(u'mascota')
                    doc_ref.set({
                        u'url' : img_url,
                        u'tipo de mascota': "Gato"
                    })
                elif np.argmax(predictions[i]) == 1:
                    print("Parece que es un perro")

                    classes_path = '/home/' + str(getuser()) + '/turtlebot3_ws/src/Petsitter_Sprint3/petsitter/petsitter_ros2/petsitter/petsitter_detect_pets/german_sheperd_model/data/new_names.names' 
                    weights = '/home/' + str(getuser()) + '/turtlebot3_ws/src/Petsitter_Sprint3/petsitter/petsitter_ros2/petsitter/petsitter_detect_pets/german_sheperd_model/checkpoints/yolov3_train_17.tf' 
                    tiny = False #yolov3 or yolov3-tiny
                    size = 416 #resize images to
                    image = '/home/' + str(getuser()) + '/turtlebot3_ws/src/Petsitter_Sprint3/petsitter/petsitter_ros2/petsitter/petsitter_detect_pets/corpus/test/deteccion.jpg' 
                    tfrecord = None 
                    output = '/home/' + str(getuser()) + '/turtlebot3_ws/src/Petsitter_Sprint3/petsitter/petsitter_ros2/petsitter/petsitter_detect_pets/german_sheperd_model/test/output/deteccion.jpg' 
                    num_classes = 1 

                    physical_devices = tf.config.experimental.list_physical_devices('GPU')
                    
                    if len(physical_devices) > 0:
                        tf.config.experimental.set_memory_growth(physical_devices[0], True)

                    if tiny:
                        yolo = YoloV3Tiny(classes=num_classes)
                    else:
                        yolo = YoloV3(classes=num_classes)

                    yolo.load_weights(weights).expect_partial()
                    logging.info('weights loaded')

                    class_names = [c.strip() for c in open(classes_path).readlines()]
                    logging.info('classes loaded')

                    if tfrecord:
                        dataset = load_tfrecord_dataset(
                            tfrecord, classes_path, size)
                        dataset = dataset.shuffle(512)
                        img_raw, _label = next(iter(dataset.take(1)))
                    else:
                        img_raw = tf.image.decode_image(
                            open(image, 'rb').read(), channels=3)

                    img = tf.expand_dims(img_raw, 0)
                    img = transform_images(img, size)

                    t1 = time.time()
                    boxes, scores, classes, nums = yolo(img)
                    t2 = time.time()
                    logging.info('time: {}'.format(t2 - t1))

                    for i in range(nums[0]):
                        if np.array(scores[0][i]) >= 0.8:
                            print("Con bastante seguridad se trata de un pastor aleman")

                            # subir imagen a Storage
                            storage = firebase.storage()

                            my_image = "/home/" +  str(getuser()) + "/turtlebot3_ws/src/Petsitter_Sprint3/petsitter/petsitter_ros2/petsitter/petsitter_detect_pets/corpus/test/deteccion.jpg"
                            image_path = "/images/mascota.jpg" # ruta donde se guarda la imagen

                            # Upload image
                            storage.child(image_path).put(my_image)
                            img_url = storage.child(image_path).get_url(None)

                            # subir documento a Firestore
                            db = firestore.client()

                            doc_ref = db.collection(u'Imagenes').document(u'mascota')
                            doc_ref.set({
                                u'url' : img_url,
                                u'tipo de mascota': "Perro",
                                u'pastor aleman': "Sí"
                            })

                        elif np.array(scores[0][i]) < 0.8 and np.array(scores[0][i]) > 0.4:
                            print("Es posible que sea un pastor alemán u otra raza de perro")

                            # subir imagen a Storage
                            storage = firebase.storage()

                            my_image = "/home/" +  str(getuser()) + "/turtlebot3_ws/src/Petsitter_Sprint3/petsitter/petsitter_ros2/petsitter/petsitter_detect_pets/corpus/test/deteccion.jpg"
                            image_path = "/images/mascota.jpg" # ruta donde se guarda la imagen

                            # Upload image
                            storage.child(image_path).put(my_image)
                            img_url = storage.child(image_path).get_url(None)

                            # subir documento a Firestore
                            db = firestore.client()

                            doc_ref = db.collection(u'Imagenes').document(u'mascota')
                            doc_ref.set({
                                u'url' : img_url,
                                u'tipo de mascota': "Perro",
                                u'pastor aleman': "Es posible"
                            })

                        else:
                            print("No es un pastor alemán")

                            # subir imagen a Storage
                            storage = firebase.storage()

                            my_image = "/home/" +  str(getuser()) + "/turtlebot3_ws/src/Petsitter_Sprint3/petsitter/petsitter_ros2/petsitter/petsitter_detect_pets/corpus/test/deteccion.jpg"
                            image_path = "/images/mascota.jpg" # ruta donde se guarda la imagen

                            # Upload image
                            storage.child(image_path).put(my_image)
                            img_url = storage.child(image_path).get_url(None)

                            # subir documento a Firestore
                            db = firestore.client()

                            doc_ref = db.collection(u'Imagenes').document(u'mascota')
                            doc_ref.set({
                                u'url' : img_url,
                                u'tipo de mascota': "Perro",
                                u'pastor aleman': "No"
                            })

                    img = cv2.cvtColor(img_raw.numpy(), cv2.COLOR_RGB2BGR)
                    img = draw_outputs(img, (boxes, scores, classes, nums), class_names)
                    cv2.imwrite(output, img)

                else:
                    print("Parece que no hay animales en la foto")


        
    def service_callback(self, request, response):
        """ Esta función es un servicio que se encarga de activar o desactivar la variable booleana que se utiliza para saber si se debe realizar 
        la identificación de las mascotas o no

        Args: 
            request (DetectMsg [bool]): indica si se inicia el proceso de identificación o se detiene
            response (DetectMsg [bool]): indica el mismo valor que la variable request

        Returns:
            response (DetectMsg [bool]): indica el mismo valor que la variable request

        """

        if request.detect == True:
            self.detection = True
            self.get_logger().info('Se está rastreando a la mascota')
            # devuelve la respuesta
            response.success = True
        else:
            self.detection = False
            self.get_logger().info('Se ha detenido la búsqueda')
            response.success = False

        # devuelve la respuesta
        return response
        
def main(args=None):

    rclpy.init(args=args)    
    img_converter_object = Service()    
       
    try:
        print("Nueva imagen")
        rclpy.spin(img_converter_object)
        
    except KeyboardInterrupt:
        img_converter_object.destroy_node()
        print("Fin del programa!")
    
    cv2.destroyAllWindows() 
    
