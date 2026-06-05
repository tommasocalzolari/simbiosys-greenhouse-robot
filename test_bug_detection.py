import sys
from pathlib import Path

import cv2
import numpy as np


TEMPLATE_PATH = (
    Path(__file__).parent
    / "src"
    / "simbiosys_perception"
    / "simbiosys_perception"
    / "models"
    / "bug_template.png"
)
MIN_MATCHES = 10
RATIO_THRESHOLD = 0.75
MAX_DISPLAY_WIDTH = 1200


def resize_for_display(image):
    height, width = image.shape[:2]
    if width <= MAX_DISPLAY_WIDTH:
        return image

    scale = MAX_DISPLAY_WIDTH / width
    return cv2.resize(image, (MAX_DISPLAY_WIDTH, int(height * scale)))


def detect_bug(image_path):
    template = cv2.imread(str(TEMPLATE_PATH), cv2.IMREAD_GRAYSCALE)
    if template is None:
        print(f"Could not load template: {TEMPLATE_PATH}")
        return

    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        print(f"Could not load image: {image_path}")
        return
    result = cv2.imread(image_path)

    sift = cv2.SIFT_create()
    template_keypoints, template_descriptors = sift.detectAndCompute(template, None)
    image_keypoints, image_descriptors = sift.detectAndCompute(image, None)

    good_matches = []
    if template_descriptors is not None and image_descriptors is not None:
        matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
        matches = matcher.knnMatch(template_descriptors, image_descriptors, k=2)

        for match_pair in matches:
            if len(match_pair) != 2:
                continue

            first, second = match_pair
            if first.distance < RATIO_THRESHOLD * second.distance:
                good_matches.append(first)

    bug_detected = len(good_matches) >= MIN_MATCHES
    if bug_detected:
        source_points = np.float32(
            [template_keypoints[match.queryIdx].pt for match in good_matches]
        ).reshape(-1, 1, 2)
        destination_points = np.float32(
            [image_keypoints[match.trainIdx].pt for match in good_matches]
        ).reshape(-1, 1, 2)
        homography, _ = cv2.findHomography(
            source_points,
            destination_points,
            cv2.RANSAC,
            5.0,
        )
        if homography is not None:
            height, width = template.shape
            corners = np.float32(
                [[0, 0], [width, 0], [width, height], [0, height]]
            ).reshape(-1, 1, 2)
            transformed_corners = cv2.perspectiveTransform(corners, homography)
            cv2.polylines(
                result,
                [np.int32(transformed_corners)],
                True,
                (0, 0, 255),
                3,
            )
        print(f"Bug detected! Good matches: {len(good_matches)}")
    else:
        print(f"No bug detected. Good matches: {len(good_matches)}")

    cv2.imshow("Bug detection", resize_for_display(result))
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def test_bug_detection_sift_helper_imports():
    assert callable(detect_bug)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 test_bug_detection_sift.py <image_path>")
        sys.exit(1)

    detect_bug(sys.argv[1])
