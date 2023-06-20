#!/usr/bin/env python

import cv2
import os
import pyrealsense2 as rs
import numpy as np
import tensorflow as tf
import tensorflow_hub as hub
import matplotlib.pyplot as plt
import rospy

from tensorflow.keras.applications.inception_v3 import InceptionV3, preprocess_input, decode_predictions
from tensorflow.keras.preprocessing import image
from std_msgs.msg import Bool
from std_msgs.msg import String
from mtcnn import MTCNN
from tensorflow.keras.models import load_model

## Benötigte Installationen
# pip install tensorflow
# sudo pip install mtcnn
# pip install pyrealsense2 opencv-python

## ROS
# roscore
# rostopic echo /Img_detect 
whileloop=0

# Konfiguration der RealSense-Kamera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30) 
pipeline.start(config)

# Parameter festlegen
class_threshold = 5
predict_class1 = 0
predict_class2 = 0
predict_false = 0

# Pfad zum vortrainierten Haar Cascade-Modell für die Gesichtserkennung
cascade_path = '/home/user/Schreibtisch/Fre23/haarcascade_frontalface_default.xml'
face_cascade = cv2.CascadeClassifier(cascade_path)

# Netz von Haag laden
haag_path = '/home/user/Schreibtisch/Fre23/3class_model.h5'
#model_haag = tf.keras.models.load_model((haag_path), custom_objects={'KerasLayer':hub.KerasLayer})
#model_haag.summary()

# VGGFace2 Modell laden
detector = MTCNN()

# Pfad zum vortrainierten InceptionV3-Modell
inception_model = InceptionV3(weights='imagenet')

# Objekterkennung mit Haag
def classify_haag(img):
    resized_img = cv2.resize(img, (224, 224))
    x = image.img_to_array(resized_img)
    x = np.expand_dims(x, axis=0)
    x = x / 255.0
    predictions = model_haag.predict(x)
    result = np.argmax(predictions[0])
    return result

def classify_face_cascade(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    return faces

# Funktion zur Gesichtserkennung mit VGGFace2
def classify_face_vgg(data_dir):
    img = plt.imread(data_dir)
    faces = detector.detect_faces(img)
    return faces

# Funktion zur Klassifizierung des Bildes mit InceptionV3
def classify_deer(data_dir):
    img = image .load_img(data_dir, target_size=(299, 299))
    x = image.img_to_array(img)
    x = preprocess_input(x)
    x = x.reshape((1, x.shape[0], x.shape[1], x.shape[2]))
    preds = inception_model.predict(x)
    deer = decode_predictions(preds, top=1)[0]
    return deer


def detection():
    print("defection")
    predict_class2 =0
    predict_class1 =0
    predict_false =0
    class_threshold=5
    while True:
        ## Get live image
        print("in while")
        if rospy.is_shutdown():
            print("ros shutdown ")
            break
        
        if pipeline.try_wait_for_frames():

            frames = pipeline.wait_for_frames()
        else:
            print("wait for frames failed")
            break

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            print("no depth")
            continue

        color_image = np.asanyarray(color_frame.get_data())
        color_image_rgb = color_image[:, :, ::-1]
        color_image_rgb_crop = color_image_rgb[30:230, 220:420]
        #cv2.imshow('Live Color Image', color_image_rgb)
        cv2.waitKey(1)
        img_path = '/home/user/Schreibtisch/Fre23/live.jpeg'

        cv2.imwrite(img_path, color_image_rgb_crop)

        ## Classify Object
        # Both with Haag
        #haag_pred = classify_haag(color_image_rgb_crop)
        
        # Deer with Cascade-Modell
        image_classification = classify_deer(img_path)
        class_name = image_classification[0][1]
        prob = image_classification[0][2]
        #print(f'Erkanntes Objekt: {class_name} mit Wahrscheinlichkeit: {prob*100}%')

        # Human with Cascada or VGGFace2
        faces_cascade = classify_face_cascade(color_image_rgb_crop)
        faces_vgg = classify_face_vgg(img_path)

        if class_name in ['impala', 'deer', 'gazelle', 'wallaby', 'Italian_greyhound', 'ibex']:# or haag_pred == 0:        
            predict_class1 += 1
            #print("Deer")

        elif len(faces_vgg) > 0 and len(faces_cascade) > 0:# and haag_pred == 1:
            predict_class2 += 1
            #print("Human")
            
        else:
            predict_false += 1

        if predict_class1 >= class_threshold:
            predict_class1 = 0
            predict_class2 = 0
            predict_false = 0
            print("Class: Deer, Publish: True")
            print("Wait for restart after Deer")
            pub.publish(1)
            pub_string.publish("Deer")
            cv2.waitKey(5000)
            whileloop =0
            

            break

        elif predict_class2 >= class_threshold:
            predict_class1 = 0
            predict_class2 = 0
            predict_false = 0
            print("Class: Human, Publish: True")
            print("Wait for restart after Human")
            pub.publish(1)
            pub_string.publish("Human")
            cv2.waitKey(5000)
            whileloop =0
            break

        elif predict_false >= class_threshold:
            predict_class1 = 0
            predict_class2 = 0
            predict_false = 0
            print("Class: Otherwise, Publish: False")
            print("Wait for restart after otherwise")
            pub.publish(0)
            pub_string.publish("Otherwise")
            cv2.waitKey(5000)
            whileloop =0
            break
    print("schleife durch")      

#callback
def obstacle_cb(data):
    print("callback called")
    #pipeline.start(config)

    print(data.data)
    if data.data==1:
        print("data ist true")
        whileloop =1
        detection()
        
    elif data.data!=1 :
        whileloop =0
        print("data ist false")
    #pipeline.stop()


pub = rospy.Publisher('Img_bool', Bool, queue_size=10)
rospy.init_node('Img_detect', anonymous=True)
pub_string = rospy.Publisher('detect_class', String, queue_size=10)
rospy.Subscriber("show_obstacle", Bool, obstacle_cb)
rospy.spin()
pipeline.stop()
