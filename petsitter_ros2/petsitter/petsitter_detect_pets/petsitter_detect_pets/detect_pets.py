from tensorflow.keras.datasets.mnist import load_data
from tensorflow.keras.utils import to_categorical
from tensorflow.keras.models import load_model
from keras_preprocessing.image import ImageDataGenerator
import numpy
from getpass import getuser

def main():
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
	model = load_model("/home/" +  str(getuser()) + "/turtlebot3_ws/src/Petsitter_Sprint3/petsitter/petsitter_ros2/petsitter/petsitter_detect_pets/petsitter_model")

	predictions = model.predict(test_generator)
	print(predictions.size / 3)
	for i in range(int(predictions.size / 3)):
	    print(i, predictions[i])
	    if numpy.argmax(predictions[i]) == 0:
	    	print("Parece que es un gato")
	    elif numpy.argmax(predictions[i]) == 1:
	    	print("Parece que es un perro")
	    else:
	    	print("Parece que no hay animales en la foto")
    	
