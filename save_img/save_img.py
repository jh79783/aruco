import cv2
import numpy as np

def save():
    cam = cv2.VideoCapture(0)
    count = 1
    while True:
        _, frame = cam.read()

        if not cam.isOpened():
            print("camera error")
        else:
            cv2.imshow("frame", frame)
            k = cv2.waitKey(10)
            if k == ord('s'):
                cv2.imwrite(f"cal{count}.png", frame)
                print(count)
                count += 1
            if k == ord('q'):
                break
    cam.release()
    cv2.destroyAllWindows()


def main():
    save()

if __name__ == '__main__':
    main()