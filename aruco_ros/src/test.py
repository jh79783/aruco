#!/usr/bin/env python
import numpy as np
import cv2
import glob
import math
import os
import rospy
from multi_robot.msg import Float32Multi
from std_msgs.msg import Float32
from rospy.numpy_msg import numpy_msg

def cal():
    rospy.INFO("====================START CAMERA CALIBRATION==========================")
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6 * 9, 3), np.float32)
    objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    images = glob.glob('/home/mun/catkin_ws/src/aruco_ros/src/camera_cal_img/*.png')

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, (9, 6), corners, ret)
            # cv2.imshow("img", img)
            # cv2.waitKey(500)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    rospy.INFO("====================END CAMERA CALIBRATION==========================")
    return mtx, dist


def arucomark(mtx, dist):
    rospy.INFO("====================START DETECTING==========================")
    rvecs_pub = rospy.Publisher('/rvecs_msg', numpy_msg(Float32Multi), queue_size=10)
    tvecs_pub = rospy.Publisher('/tvecs_msg', numpy_msg(Float32Multi), queue_size=10)
    id_pub = rospy.Publisher('/id_msg',numpy_msg(Float32Multi), queue_size=10)
    dddd= Float32Multi()

    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    param = cv2.aruco.DetectorParameters_create()
    param.adaptiveThreshConstant = 10
    os.system('sudo modprobe bcm2835-v4l2')
    cam = cv2.VideoCapture(0)
    trim_img = None
    if cam.isOpened():
        while True:
            _, frame = cam.read()
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            coners, ids, point = cv2.aruco.detectMarkers(gray_frame, aruco_dict, parameters=param)
            #print("ids", ids)

            if np.all(ids != None):
                # rvecs, tvecs, objpoint = cv2.aruco.estimatePoseSingleMarkers(coners, 0.05, mtx, dist)
                rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(coners, 0.05, mtx, dist)
                rvecs_pub.publish(rvecs)
                tvecs_pub.publish(tvecs)
                id_pub.publish(ids)
                for i in range(0, ids.size):
                    print("Ddd",[rvecs, tvecs, ids])
                    
                    a= [list(rvecs[i][0] * 100),list(tvecs[i][0]),list(ids[i])]

                    # print(rvecs[i][0][2] * 100)
                    # print(tvecs[i][0][2])
                    # print(ids)
                    # print(i,a)

                    # rvecs_pub.publish(rvecs[i][0][2]*100)
                    # tvecs_pub.publish(tvecs[i][0][2])

                    
                    frame = cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)

                frame = cv2.aruco.drawDetectedMarkers(frame, coners, ids)
                frame = cv2.aruco.drawDetectedMarkers(frame, point, borderColor=(0, 255, 0))

            cv2.imshow("result", frame)
            k = cv2.waitKey(100)
            if k == ord('q'):
                break
    cam.release()
    cv2.destroyAllWindows()


def main():
    rospy.init_node("dfdfdf")
    mtx, dist = cal()
    arucomark(mtx, dist)


if __name__ == "__main__":
    main()
