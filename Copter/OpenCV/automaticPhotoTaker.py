import cv2
import numpy as np
import os

display_width=640
display_height=480
flip=3

camSet='nvarguscamerasrc ! video/x-raw(memory:NVMM), width=3264, height=2464, format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(display_width)+', height='+str(display_height)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
cam= cv2.VideoCapture(camSet)
count = 0

face_classifier = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')


def face_extractor(img):

    faces = face_classifier.detectMultiScale(img, 1.3, 5)
    
    if faces is ():
        return None
    

    for (x,y,w,h) in faces:
        x=x-10
        y=y-10
        cropped_face = img[y:y+h+50, x:x+w+50]

    return cropped_face

if os.path.isdir('myCopterFaceImages') is False:
    os.makedirs('myCopterFaceImages')

while True:

    ret, frame = cam.read()
    if face_extractor(frame) is not None:
        count += 1
        face = cv2.resize(face_extractor(frame), (400, 400))

        file_name_path = './myCopterFaceImages/' + str(count) + '.jpg'
        cv2.imwrite(file_name_path, face)

        
        cv2.putText(face, str(count), (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 2)
        cv2.imshow('Face Cropper', face)
        
    else:
        print("Face not found")
        pass

    if cv2.waitKey(1) == 13 or count == 100: #13 is the Enter Key
        break
        
cam.release()
cv2.destroyAllWindows()      
print("Collecting Samples Complete")