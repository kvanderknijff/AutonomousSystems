import cv2
import math
import numpy as np
import matplotlib.pyplot as plt

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detector = cv2.aruco.ArucoDetector(dictionary)

def arUcoDetection(frame: np.ndarray):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, markerIDs, rejected = detector.detectMarkers(frame)
    
    information = []

    if markerIDs is not None:
        for corner, markerID in zip(corners, markerIDs):
            corner = corner.reshape(4, 2)
            corner = corner.astype(int)

            topLeft, topRight, bottomRight, bottomLeft = corner

            cv2.putText(frame, f"id: {markerID[0]}", tuple(topLeft), cv2.FONT_HERSHEY_PLAIN, 1.3, (255, 0, 255), 2)

            centerX = (topLeft[0] + bottomRight[0]) // 2
            centerY = (topLeft[1] + bottomRight[1]) // 2

            center = (centerX, centerY)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            topMiddleX = (topLeft[0] + topRight[0]) // 2
            topMiddleY = (topLeft[1] + topRight[1]) // 2
            
            direction = math.degrees(math.atan2(topMiddleY - centerY, topMiddleX - centerX))
            cv2.putText(frame, f"dir: {direction}", tuple(topRight), cv2.FONT_HERSHEY_PLAIN, 1.3, (255, 0, 255), 2)            
            information.append([int(markerID[0]), center, direction])
            
    return information, frame

def LEDDetection(frame: np.ndarray):
    frameRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    r, g, b = cv2.split(frameRGB)
    b_g = cv2.subtract(b,g)
    r_b = cv2.subtract(r,b)
    
    

def videoProcessing(file: str, record: bool, camera: bool):
    capture = cv2.VideoCapture(file)

    if not capture.isOpened():
        print("Error: Could not open video")
        return

    if record:
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

        if camera:
            width, height = height, width

        fps = capture.get(cv2.CAP_PROP_FPS)

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter('output.mp4', fourcc, fps, (width, height), isColor=False)

    while True:
        try:
            ret, frame = capture.read()

            if not ret:
                print("End of video")
                break
            
            if camera:
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
                
            arUcoInformation, arUcoFrame = arUcoDetection(frame)
            # LEDDetection(frame)

            cv2.imshow("ArUcos", arUcoFrame)
            if record:
                out.write(arUcoFrame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        except Exception as e:
            print("Program ran into an exception: ", e)

    capture.release()
    if record:
        out.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    #videoProcessing("http://145.137.57.80:8080/video", record=True, camera=True)
    videoProcessing("C:/Vakken TI/Jaar 3/TINLAB - Autonomous Systems/Object detection AS/aruco test/arucoturntest.mp4", record=False, camera=False)