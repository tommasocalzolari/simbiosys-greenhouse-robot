import sys

import cv2


def detect(image_path):
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"Kan foto niet laden: {image_path}")
        return

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    corners, ids, rejected = detector.detectMarkers(frame)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        for i, tag_id in enumerate(ids.flatten()):
            cx = int(corners[i][0][:, 0].mean())
            cy = int(corners[i][0][:, 1].mean())
            print(f"Tag ID: {tag_id}, center: ({cx}, {cy})")
    else:
        print("Geen tags gedetecteerd")

    cv2.imshow("AprilTag detectie", frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def test_apriltag_detector_helper_imports():
    assert callable(detect)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 test_april.py <image_path>")
        sys.exit(1)

    detect(sys.argv[1])
