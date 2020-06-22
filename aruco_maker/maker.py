import cv2
import numpy as np


aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

markerImage = np.zeros((200, 200), dtype=np.uint8)
markerImage = cv2.aruco.drawMarker(aruco_dict, 3, 200, markerImage, 1)

cv2.imwrite("marker3.png", markerImage)
