#################################################################################
# Autor: Abidán Brito
# Fecha: 06/06/2022
# Nombre del fichero: upload_map_firestore.py
# Descripción: Convierte el mapa a base64 (UTF-8) y lo sube a Firestore.
#################################################################################

import firebase_admin
from firebase_admin import credentials, firestore
import os
from PIL import Image
import base64
import yaml


def initialize_firebase():
    """
    Inicializa la aplicación del proyecto en Firebase. Para ello utiliza una clave con credenciales, guardada de 
    forma local.
    """
    cred = credentials.Certificate("/home/abi/firebase_keys/petsitter-42488-firebase-adminsdk-kbpr3-d2b96d00bb.json")
    firebase_admin.initialize_app(cred)


def main(args=None):
    """
    Convierte el mapa pgm a base64, obtiene la resolución de su yaml y sube ambos datos a la instancia de Firestore.
    """
    # Get Firestore instance
    initialize_firebase()
    db = firestore.client()   

    path_to_map = '/home/abi/turtlebot3_ws/src/petsitter/petsitter_ros2/petsitter/petsitter_nav2_system/map/'
    map_str = ''

    # Skim through all files in 'map' directory
    for f in os.listdir(path_to_map):
        filename, extension  = os.path.splitext(f)

        # Filter out unwanted maps (Gazebo virtual worlds)
        if filename == 'map_auto' and extension == '.pgm':

            # Convert to png and save it out
            map_png = "{}.png".format(filename)
            with Image.open(path_to_map + f) as im:
                im.save(map_png)

            # Load png and convert to base64, then delete png
            map_base64 = base64.b64encode(open(map_png, "rb").read())
            map_str = map_base64.decode('utf-8')
            os.remove(map_png)

    # Get cartographer resolution from the yaml config file
    with open(path_to_map + "map_auto.yaml", 'r') as map_yaml:
        config_file = yaml.safe_load(map_yaml)
        resolution = config_file['resolution']

    # Upload map in base64, along with its resolution, to Firestore
    data = {
        u'base64': map_str,
        u'resolution': resolution
    }
    db.collection(u'maps').document(u'robot-500').set(data)


if __name__ == '__main__':
    main()