import tensorflow as tf
import time
from absl import app, flags, logging
from absl.flags import FLAGS
import cv2
import numpy as np
import tensorflow as tf

def main(_argv):
	physical_devices = tf.config.experimental.list_physical_devices('GPU')
	# Recreate the exact same model, including its weights and the optimizer
	new_model = tf.keras.models.load_model('yolo.h5')

	# Show the model architecture
	new_model.summary()