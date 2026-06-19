from controller import Robot, Camera
import numpy as np
from flask import Flask, Response
import cv2
# Create the Robot instance.
robot = Robot()

# Time step of the simulation in milliseconds.
timeStep = int(robot.getBasicTimeStep())

# Get the camera device.
camera = robot.getDevice("camera")
camera.enable(timeStep)

# Initialize Flask app
app = Flask(__name__)

# Frame to be served
frame = None

@app.route('/video_feed')
def video_feed():
    global frame
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

def generate():
    global frame
    while True:
        if frame is not None:
            ret, jpeg = cv2.imencode('.jpg', frame)
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n\r\n')

# Function to update the frame
def update_frame():
    global frame
    while robot.step(timeStep) != -1:
        # Capture image from the camera.
        image = camera.getImage()

        if image:
            width = camera.getWidth()
            height = camera.getHeight()
            
            # Convert the image to a numpy array.
            image_array = np.frombuffer(image, np.uint8).reshape((height, width, 4))
            
            # Convert the BGRA image to BGR for OpenCV processing.
            frame = cv2.cvtColor(image_array, cv2.COLOR_BGRA2BGR)
        
        # Perform other robot tasks here...

# Start the frame update in a separate thread
import threading
thread = threading.Thread(target=update_frame)
thread.start()

# Run the Flask app
if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5005)
