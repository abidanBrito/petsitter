from getpass import getuser        
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
import cv2

import time
from absl import app, flags, logging
from absl.flags import FLAGS
import numpy as np
import tensorflow as tf
from .models import (
    YoloV3, YoloV3Tiny
)
from .dataset import transform_images, load_tfrecord_dataset
from .utils import draw_outputs

def main(args=None):

    rclpy.init(args=args)

    classes_path = '/home/' + str(getuser()) + '/turtlebot3_ws/src/Petsitter_Sprint3/petsitter/petsitter_ros2/petsitter/petsitter_detect_pets/german_sheperd_model/data/new_names.names' 
    weights = '/home/' + str(getuser()) + '/turtlebot3_ws/src/Petsitter_Sprint3/petsitter/petsitter_ros2/petsitter/petsitter_detect_pets/german_sheperd_model/checkpoints/yolov3_train_20.tf' 
    tiny = False #yolov3 or yolov3-tiny
    size = 416 #resize images to
    image = '/home/' + str(getuser()) + '/turtlebot3_ws/src/Petsitter_Sprint3/petsitter/petsitter_ros2/petsitter/petsitter_detect_pets/german_sheperd_model/test/input/pastor_aleman5.jpeg' 
    tfrecord = None 
    output = '/home/' + str(getuser()) + '/turtlebot3_ws/src/Petsitter_Sprint3/petsitter/petsitter_ros2/petsitter/petsitter_detect_pets/german_sheperd_model/test/output/pastor_aleman5.jpeg' 
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

    print('detections:')
    for i in range(nums[0]):
    	if np.array(scores[0][i]) >= 0.8:
    		print("Con bastante seguridad se trata de un pastor aleman")

    	elif np.array(scores[0][i]) < 0.8 and np.array(scores[0][i]) > 0.4:
    		print("Es posible que sea un pastor alemán u otra raza de perro")

    img = cv2.cvtColor(img_raw.numpy(), cv2.COLOR_RGB2BGR)
    img = draw_outputs(img, (boxes, scores, classes, nums), class_names)
    cv2.imwrite(output, img)
    #print('output saved to: {}'.format(output))
    
    

if __name__ == '__main__':
    main()

