
from keras.layers import Input, Lambda, Dense, Flatten
from keras.models import Model
from keras.applications.vgg16 import VGG16
from keras.applications.vgg16 import preprocess_input
from keras.preprocessing import image
from keras.preprocessing.image import ImageDataGenerator
from keras.models import Sequential
import numpy as np
from sklearn.metrics import confusion_matrix
import itertools
import os
import shutil
import random
import glob
import matplotlib.pyplot as plt
import warnings
from keras.models import load_model

os.chdir('../OpenCV/myCopterFaceImages')
if os.path.isdir('train') is False:
    os.makedirs('train')
    os.makedirs('valid')
    os.makedirs('test')
    
    for i in random.sample(glob.glob('*.jpg'), 500):
        shutil.move(i, 'train')     
    for i in random.sample(glob.glob('*.jpg'), 100):
        shutil.move(i, 'valid')   
    for i in random.sample(glob.glob('*.jpg'), 50):
        shutil.move(i, 'test')

os.chdir('../../Keras')

#resize images for VGG16 model. This is what it requires to process input
IMAGE_SIZE = [224, 224]

train_path = 'train'
valid_path = 'test'

vgg = VGG16(input_shape=IMAGE_SIZE + [3], weights='imagenet', include_top=False)

for layer in vgg.layers:
  layer.trainable = False
  
#uncomment next line if multiple classes exists
#folders = glob('Datasets/Train/*')
  
x = Flatten()(vgg.output)

prediction = Dense(1), activation='softmax')(x)

model = Model(inputs=vgg.input, outputs=prediction)

model.compile(
  loss='categorical_crossentropy',
  optimizer='adam',
  metrics=['accuracy']
)

train_datagen = ImageDataGenerator(rescale = 1./255,
                                   shear_range = 0.2,
                                   zoom_range = 0.2,
                                   horizontal_flip = True)

test_datagen = ImageDataGenerator(rescale = 1./255)

training_set = train_datagen.flow_from_directory(train_path,
                                                 target_size = (224, 224),
                                                 batch_size = 32,
                                                 class_mode = 'categorical')

test_set = test_datagen.flow_from_directory(valid_path,
                                            target_size = (224, 224),
                                            batch_size = 32,
                                            class_mode = 'categorical')


model.fit_generator(
  training_set,
  validation_data=test_set,
  epochs=5,
  steps_per_epoch=len(training_set),
  validation_steps=len(test_set), 
  verbose = 2
)

model.save('face_features_model.h5')
