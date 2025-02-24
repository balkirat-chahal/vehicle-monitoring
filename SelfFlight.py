from djitellopy import Tello
import cv2
import time
import numpy as np

face_classifier = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def detect_face(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_classifier.detectMultiScale(gray, 1.1, 4)

    if len(faces) == 0:
        return img, [[0, 0], 0]

    max_area = 0
    max_index = 0

    for i, (x, y, w, h) in enumerate(faces):
        area = w * h
        if area > max_area:
            max_area = area
            max_index = i

    x, y, w, h = faces[max_index]
    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3)
    center_x = x + w // 2
    center_y = y + h // 2
    cv2.circle(img, (center_x, center_y), 5, (0, 0, 0), cv2.FILLED)

    return img, [[center_x, center_y], max_area]

def trackFace(drone,info,w,pid,pError):
    fbrange = [6200,6800]
    area = info[1]
    x,y = info[0]
    fb= 0
    pid = [0.4,0.4,0]

    error = info[0][0] - w//2
    speed = pid[0]*error + pid[1]*(error-pError)
    speed = int(np.clip(speed,-100,100))
 

    if area> fbrange[0] and area <fbrange[1]:
        fb = 0
    elif area >fbrange[1]:
        fb = -20
    elif area <fbrange[0] and area!=0:
        fb = 20
    

    if info[0][0] !=0:
        yaw_velocity = speed
    else:
        for_back_velocity = 0
        left_right_velocity = 0
        up_down_velocity = 0
        yaw_velocity = 0
        error = 0
    
    if x==0:
        speed = 0
        error = 0
    
    drone.send_rc_control(0,fb,0,speed)
    return error

def main():
    tello = Tello()
    w,h = 360,240
    pid = [0.4,0.4,0]
    pError = 0
    startCounter = 0

    try:

        tello.connect()

        battery_level = tello.get_battery()
        print(battery_level)



        tello.takeoff()
        tello.send_rc_control(0,0,25,0)
        time.sleep(6)
        tello.streamon()
        frame_read = tello.get_frame_read()

        while True:

            img = frame_read.frame
            img = cv2.resize(img,(w,h))

            img, info = detect_face(img)
            print("Center: ", info[0], "Area: ", info[1])

            pError = trackFace(tello,info,w,pid,pError)

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
