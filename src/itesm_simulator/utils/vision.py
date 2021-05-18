import cv2
from pyzbar.pyzbar import decode

def decodeFileFromPath(path : str) -> list:
    return decode(cv2.imread(path))

def decodeFileFromFrame(f) -> list:
    return decode(f)

def lookForQR():
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

if __name__ == "__main__":
    r = lookForQR()
    print(r)