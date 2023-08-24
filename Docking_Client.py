import cv2 as cv
from cv2 import aruco
import numpy as np
import math

# SOCKET
import socket


HEADER = 64
PORT = 5791
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"
SERVER = "192.168.1.102"
ADDR = (SERVER, PORT)
k=5


client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(ADDR)

def send(msg):
    message = msg.encode(FORMAT)
    msg_length = len(message)
    send_length = str(msg_length).encode(FORMAT)
    send_length += b' '*(HEADER - len(send_length))
    client.send(send_length)
    client.send(message)
    print(client.recv(2048).decode(FORMAT))
    
    
# ARUCO
calib_data_path = "calib_data/MultiMatrix.npz"

calib_data = np.load(calib_data_path)
print(calib_data.files)

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

MARKER_SIZE = 2.5  # centimeters

marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

param_markers = aruco.DetectorParameters_create()

cap = cv.VideoCapture(1, cv.CAP_DSHOW)
# cap.set(cv.CAP_PROP_FRAME_WIDTH, 1000)
# cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1000)
# cap.set(cv.CAP_PROP_BRIGHTNESS, 150)  # giá trị mặc định của độ sáng
# cap.set(cv.CAP_PROP_BRIGHTNESS, 150)  # giá trị mặc định của độ sáng
# cap.set(cv.CAP_PROP_SATURATION,  80)  # giá trị mặc định của độ bão hòa màu
# cap.set(cv.CAP_PROP_CONTRAST,  20)  # giá trị mặc định của độ tương phản
# cap.set(cv.CAP_PROP_FPS,  30)  # giá trị mặc định của tốc độ khung hình
# cap.set(cv.CAP_PROP_TEMPERATURE,  200)

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentify = np.dot(Rt,R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentify)
    return n < 1e-6

def rotationMatrixToEulerAngles(R):
    assert(isRotationMatrix(R))
    sy = math.sqrt(R[0,0]*R[0,0] + R[1,0]*R[1,0])
    singular = sy < 1e-6
    
    if not singular:
        x = math.atan2(R[2,1], R[2,2])
        y = math.atan2(-R[2,0],sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x,y,z])

while True:
    ret, frame = cap.read()
    
    if not ret:
        break
    
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(gray_frame, marker_dict, parameters=param_markers)

    if marker_IDs is not None:
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corners, MARKER_SIZE, cam_mat, dist_coef)
        total_markers = range(0, marker_IDs.size)
        
        
        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            
            top_right = corners[0].ravel()
            top_left = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()

            # Since there was mistake in calculating the distance approach point-outed in the Video Tutorial's comment
            # so I have rectified that mistake, I have test that out it increase the accuracy overall.
            
            ###### ANGLE #####
            rvec = rVec[0][0]
            tvec = tVec[0][0]
            rvec_flipped = rvec*-1
            tvec_flipped = tvec*-1
            rotation_matrix, jacobian = cv.Rodrigues(rvec_flipped)
            realworld_tvec = np.dot(rotation_matrix, tvec_flipped)
            
            pitch, roll, yaw = rotationMatrixToEulerAngles(rotation_matrix)
            
            x = round(tVec[i][0][0],1)
            z = round(tVec[i][0][2],1)
            
            data = str(x) + ',' + str(z)
            send(data)
            
            # if(x > 1.5 and z > 25):
            #     k = '1'
            #     send(k)
            # elif(x < -1.5 and z > 25):
            #     k = '2'
            #     send(k)
            # elif( (x > -1.5 and x < 1.5) and z > 10):
            #     k = '3'
            #     send(k)
            # else:
            #     k = '5'
            #     send(k)
        
            
            # Draw the pose of the marker
            point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
            cv.putText(frame,f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} z:{round(tVec[i][0][2],1)} ",(10,35),cv.FONT_HERSHEY_PLAIN,1.8,(255, 0, 255),2,cv.LINE_AA,)
            cv.putText(frame,f"roll:{round(math.degrees(roll),1)} pitch:{round(math.degrees(pitch),1)} yaw:{round(math.degrees(yaw),1)}",(10,65),cv.FONT_HERSHEY_PLAIN,1.8,(255, 0, 255),2,cv.LINE_AA,)
        
        
    cv.imshow("frame", frame)
    key = cv.waitKey(1)
    if key == ord("q"):
        break
cap.release()
cv.destroyAllWindows()