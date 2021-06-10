"""
Este modulo sirve para que se puedan ejecutar funciones usando videofeed y QR
"""

import cv2
from pyzbar.pyzbar import decode
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy

resultData = None

def decodeFileFromPath(path : str) -> list:
    return decode(cv2.imread(path))

def decodeFileFromFrame(f) -> list:
    return decode(f)

def webCameraQR() -> str:
    WINDOW_WIDTH    = 640
    WINDOW_HEIGHT   = 480
    cap = cv2.VideoCapture(0)
    cap.set(3,WINDOW_WIDTH)     #width=640
    cap.set(4,WINDOW_HEIGHT)    #height=480

    cv2.namedWindow('video_feed')

    frame = None
    result = []

    while(cap.isOpened()):
        ret, frame = cap.read()
        if frame is None:
            break
        # Display the resulting frame
        cv2.imshow('video_feed',frame)
        result = decodeFileFromFrame(frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if len(result) > 0:
            break

    cap.release()
    return result[0].data.decode("UTF-8")

def dronCameraCallback(image):
    global resultData
    br = CvBridge()
    try:
        cv_image = br.imgmsg_to_cv2(image, "bgr8")
    except CvBridgeError as e:
        print(e)
    if cv_image is not None:
        result = decodeFileFromFrame(cv_image)
        if len(result) > 0:
            resultData = result[0].data.decode("UTF-8")
        else:
            resultData = None

def droneCameraQR():
    global resultData
    rospy.init_node("qrcodeReader")
    image_sub = rospy.Subscriber("/gs1/vi_sensor/camera_depth/camera/image_raw", Image, dronCameraCallback)

    while not rospy.core.is_shutdown():
        if resultData is None:
            rospy.rostime.wallsleep(0.5)
        else:
            break
    
    return resultData

if __name__ == "__main__":
    r = webCameraQR()
    print(r)