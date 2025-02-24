from djitellopy import Tello
import cv2
import time
import numpy as np

car_classifier = cv2.CascadeClassifier(r'haarcascade_car.xml')

"""
Working File 
"""

def detect_car(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cars = car_classifier.detectMultiScale(gray, 1.1, 4)

    if len(cars) == 0:
        return img, [[0, 0], 0]

    max_area = 0
    max_index = 0
    for i, (x, y, w, h) in enumerate(cars):
        area = w * h
        if area > max_area:
            max_area = area
            max_index = i

    x, y, w, h = cars[max_index]
    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3)
    center_x = x + w // 2
    center_y = y + h // 2
    cv2.circle(img, (center_x, center_y), 5, (0, 0, 0), cv2.FILLED)

    return img, [[center_x, center_y], max_area]

def trackCar(drone, info, w, pid, pError):
    fbrange = [5000, 6000]
    area = info[1]
    x, y = info[0]
    fb = 0

    # Define the bounds for the dead zone in the center of the screen
    left_bound = w * 0.4  # 40% of the screen width from the left
    right_bound = w * 0.6  # 60% of the screen width from the left

    # Calculate error based on the dead zone
    if x < left_bound:
        error = x - left_bound  # Negative error - Object is to the left
    elif x > right_bound:
        error = x - right_bound  # Positive error - Object is to the right
    else:
        error = 0  # Object is within the dead zone, no error

    speed = pid[0] * error + pid[1] * (error - pError)
    speed = int(np.clip(speed, -100, 100))

    if area > fbrange[0] and area < fbrange[1]:
        fb = 0
    elif area > fbrange[1]:
        fb = -200  # Adjusted to -20 for safety, you can change it back to -200 based on your testing
    elif area < fbrange[0] and area != 0:
        fb = 200  # Adjusted to 20 for safety, you can change it back to 200 based on your testing

    if x == 0:
        speed = 0
        error = 0

    drone.send_rc_control(0, fb, 0, speed)
    return error


def main():
    tello = Tello()
    w, h = 360, 240
    pid = [0.4, 0.4, 0]
    pError = 0

    tello.connect()
    print(tello.get_battery())

    tello.takeoff()
    tello.send_rc_control(0, 0, 35, 0)
    time.sleep(6)
    tello.streamon()
    frame_read = tello.get_frame_read()

    try:
        while True:
            img = frame_read.frame
            img = cv2.resize(img, (w, h))

            img, info = detect_car(img)
            print("Center: ", info[0], "Area: ", info[1])

            pError = trackCar(tello, info, w, pid, pError)

            display_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            cv2.imshow("Drone Camera", display_img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        tello.land()
        tello.streamoff()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
