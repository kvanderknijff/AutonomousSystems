import cv2
import math
import numpy as np

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

def detect_pix (frame: np.ndarray, frameRGB: np.ndarray, color: str, method: int):
    """
    Function's base made by: Soufiane Lemkaddem, Hogeschool Rotterdam
    """
    ledPositions = []
    Contours, hierarchy = cv2.findContours(frame, method, cv2.CHAIN_APPROX_SIMPLE)
    for contour in Contours:
        area = cv2.contourArea(contour)
        if area > 500:
            x, y, w, h = cv2.boundingRect(contour)
            ledPositions.append((x + 0.5 * w, y + 0.5* h))
            cv2.rectangle(frameRGB, (x, y), (x + w, y + h), (255, 0, 0), 2)
            print(color, np.round(x+(w/2)),np.round(y+(h/2)))

    if not ledPositions:
        ledPositions.append((-1,-1))

    return ledPositions, frameRGB

def ledDetection(frameBGR: np.ndarray):
    """
    Function's base made by: Soufiane Lemkaddem, Hogeschool Rotterdam
    """
    
    ledPositions = []

    frameRGB = cv2.cvtColor(frameBGR, cv2.COLOR_BGR2RGB)
    frameHSV = cv2.cvtColor(frameBGR, cv2.COLOR_BGR2HSV)

    frameR, frameG, frameB = cv2.split(frameRGB)

    frameBG = cv2.subtract(frameB, frameG)
    frameRB = cv2.subtract(frameR, frameB)

    # BLUE
    ret, frameBG = cv2.threshold(frameBG, 37, 255, cv2.THRESH_BINARY)
    blueLedPositions, blueLedframeRGB = detect_pix(frameBG, frameRGB, "Blue", cv2.RETR_TREE)
    ledPositions.append(blueLedPositions)
    
    """
    # RED
    ret, frameRB = cv2.threshold(frameRB, 60, 255, cv2.THRESH_BINARY)
    redLedPositions, frameRGB = detect_pix(frameRB, frameRGB, "Red", cv2.RETR_TREE)
    ledPositions.append(redLedPositions)
    
    # GREEN
    lowerGreen = np.array([40, 50, 50])
    upperGreen = np.array([85, 255, 255])
    maskG = cv2.inRange(frameHSV, lowerGreen, upperGreen)
    greenLedPositions, frameRGB = detect_pix(maskG, frameRGB, "Green", cv2.RETR_EXTERNAL)
    ledPositions.append(greenLedPositions)

    # WHITE
    lowerWhite = np.array([0, 0, 200])
    upperWhite = np.array([180, 120, 255])
    maskW = cv2.inRange(frameHSV, lowerWhite, upperWhite)
    whiteLedPositions, frameRGB = detect_pix(maskW, frameRGB, "White", cv2.RETR_EXTERNAL)
    ledPositions.append(whiteLedPositions)
    """

    return ledPositions, frameRGB

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
        out = cv2.VideoWriter('output.mp4', fourcc, fps, (width, height)) #, isColor=False)

    while True:
        try:
            ret, frame = capture.read()

            if not ret:
                print("End of video")
                break

            if camera:
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

            arUcoInformation, arUcoFrame = arUcoDetection(frame)
            """
            arUcoInformation =
            [[id1, center1, dir1],[id2,center2,dir2],...]
            """
            ledPositions, frameRGB = ledDetection(frame)
            """
            ledPositions =
            [[[blueled1x,blueled1y],[blueled2x,blueled2y],...]
            [[redled1x,redled1y],[redled2x,redled2y],...]
            [[greenled1x,greenled1y],[greenled2x,greenled2y],...]
            [[whiteled1x,whiteled1y],[whiteled2x,whiteled2y],...]]
            """

            cv2.imshow("frame", frameRGB)
            if record:
                out.write(frameRGB)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except Exception as e:
            print("Program ran into an exception: ", e)

    capture.release()
    if record:
        out.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    #videoProcessing("http://192.168.1.108:8080/video", record=True, camera=True)
    #videoProcessing("C:/Vakken TI/Jaar 3/TINLAB - Autonomous Systems/Object detection AS/aruco test/arucoturntest.mp4", record=False, camera=False)
    videoProcessing("C:/Vakken TI/Jaar 3/TINLAB - Autonomous Systems/Object detection AS/leds test/led_lightson_openwindow.mp4", record=False, camera=False)