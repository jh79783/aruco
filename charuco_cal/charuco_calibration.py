import cv2
import glob
import os
import numpy as np
from cv2 import aruco
import rospy
from multi_robot.msg import aruco_msgs

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
board = aruco.CharucoBoard_create(7, 5, 1, .8, aruco_dict)


def cal():
    """
    Charuco base pose estimation.
    """
    rospy.loginfo("POSE ESTIMATION STARTS:")

    allCorners = []
    allIds = []
    decimator = 0
    # SUB PIXEL CORNER DETECTION CRITERION
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)
    images = glob.glob('cal*.png')

    for im in images:
        print("=> Processing image {0}".format(im))
        frame = cv2.imread(im)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)

        if len(corners)>0:
            # SUB PIXEL DETECTION
            for corner in corners:
                cv2.cornerSubPix(gray, corner,
                                 winSize = (3,3),
                                 zeroZone = (-1,-1),
                                 criteria = criteria)
            res2 = cv2.aruco.interpolateCornersCharuco(corners,ids,gray,board)
            if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%1==0:
                allCorners.append(res2[1])
                allIds.append(res2[2])

        decimator+=1

    imsize = gray.shape
    return allCorners,allIds,imsize

def calibrate_charuco(allCorners,allIds,imsize):
    """
    Calibrates the camera using the dected corners.
    """
    rospy.loginfo("CAMERA CALIBRATION")

    cameraMatrixInit = np.array([[ 1000.,    0., imsize[0]/2.],
                                 [    0., 1000., imsize[1]/2.],
                                 [    0.,    0.,           1.]])

    distCoeffsInit = np.zeros((5,1))
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
    #flags = (cv2.CALIB_RATIONAL_MODEL)
    (ret, camera_matrix, distortion_coefficients0,
     rotation_vectors, translation_vectors,
     stdDeviationsIntrinsics, stdDeviationsExtrinsics,
     perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
                      charucoCorners=allCorners,
                      charucoIds=allIds,
                      board=board,
                      imageSize=imsize,
                      cameraMatrix=cameraMatrixInit,
                      distCoeffs=distCoeffsInit,
                      flags=flags,
                      criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))
    rospy.loginfo("END CALIBRATION")
    return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors


def detect_marker(mtx, dist):
    rospy.loginfo("START DETECT MARKER")
    os.system('sudo modprobe bcm2835-v4l2')
    cam = cv2.VideoCapture(-1)
    param = cv2.aruco.DetectorParameters_create()
    aruco_pub = rospy.Publisher('aruco_msg', aruco_msgs, queue_size=10)
    aruco = aruco_msgs()
    if cam.isOpened():
        while not rospy.is_shutdown:
            _, frame = cam.read()
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            coners, ids, point = cv2.aruco.detectMarkers(gray_frame, aruco_dict, parameters=param)
            if np.all(ids != None):
                rvecs, tvecs, objpoint = cv2.aruco.estimatePoseSingleMarkers(coners, 0.05, mtx, dist)
                frame = cv2.aruco.drawAxis(frame, mtx, dist, rvecs[0], tvecs[0], 0.05)
                rvecs_msg = rvecs.tolist()
                tvecs_msg = tvecs.tolist()
                rvecs_msg_x = rvecs_msg[0][0][0]
                rvecs_msg_y = rvecs_msg[0][0][1]
                rvecs_msg_z = rvecs_msg[0][0][2]
                tvecs_msg_x = tvecs_msg[0][0][0]
                tvecs_msg_y = tvecs_msg[0][0][1]
                tvecs_msg_z = tvecs_msg[0][0][2]
                aruco.r_x = rvecs_msg_x
                aruco.r_y = rvecs_msg_y
                aruco.r_z = rvecs_msg_z
                aruco.t_x = tvecs_msg_x
                aruco.t_y = tvecs_msg_y
                aruco.t_z = tvecs_msg_z
                aruco.id = int(ids[i])
                rospy.loginfo(aruco)
                aruco_pub.publish(aruco)
            frame = cv2.aruco.drawDetectedMarkers(frame, coners, ids)
            #cv2.imshow("result", frame)
            k = cv2.waitKey(33)
            #if k == ord('q'):
            #    break
    #cam.release()
    #cv2.destroyAllWindows()


def main():
    allCorners, allIds, imsize = cal()
    print("Calibration is Completed. Starting tracking marker.")
    ret, mtx, dist, rvec, tvec = calibrate_charuco(allCorners, allIds, imsize)
    detect_marker(mtx, dist)


if __name__ =="__main__":
    main()

