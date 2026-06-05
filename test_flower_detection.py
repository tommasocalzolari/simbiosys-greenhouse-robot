import sys
from pathlib import Path

import cv2
import numpy as np


MODEL_PATH = (
    Path(__file__).parent
    / "src"
    / "simbiosys_perception"
    / "simbiosys_perception"
    / "models"
    / "flower_model (Copy).pt"
)
YOLO_FLOWER_LABELS = {
    0: "magenta",
    1: "white",
    2: "light_pink",
}
YOLO_BUG_CLASS_ID = 3
HSV_RANGES = {
    "magenta": ([165, 150, 70], [180, 255, 180]),
    "light": ([0, 3, 175], [180, 90, 255]),
}

MORPH_KERNEL_SIZE = 7
MIN_CONTOUR_AREA = 500
MAX_CONTOUR_AREA = 50000
MAX_BBOX_SIZE_FRACTION = 0.50
MAX_DISPLAY_WIDTH = 1200
MAX_BBOX_CENTER_Y_FRACTION = 0.35
MIN_ASPECT_RATIO = 0.4
MAX_ASPECT_RATIO = 2.5


def print_hsv_on_click(event, x, y, _flags, user_data):
    if event != cv2.EVENT_LBUTTONDOWN:
        return

    hsv_image = user_data["hsv"]
    scale = user_data["scale"]
    original_x = min(hsv_image.shape[1] - 1, int(x / scale))
    original_y = min(hsv_image.shape[0] - 1, int(y / scale))
    hue, saturation, value = hsv_image[original_y, original_x]
    print(
        f"clicked pixel x={original_x}, y={original_y}, "
        f"HSV=[{int(hue)}, {int(saturation)}, {int(value)}]"
    )


def find_valid_contours(mask, image_shape):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    image_height, image_width = image_shape[:2]
    valid_contours = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_CONTOUR_AREA or area > MAX_CONTOUR_AREA:
            continue
        _x, y, width, height = cv2.boundingRect(contour)
        center_y = y + height / 2.0
        if center_y >= image_height * MAX_BBOX_CENTER_Y_FRACTION:
            continue
        if width > image_width * MAX_BBOX_SIZE_FRACTION:
            continue
        if height > image_height * MAX_BBOX_SIZE_FRACTION:
            continue
        aspect_ratio = width / float(height)
        if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
            continue
        valid_contours.append(contour)

    return valid_contours


def contour_top_pixel(contour):
    points = contour.reshape(-1, 2)
    top_y = int(points[:, 1].min())
    top_candidates = points[points[:, 1] == top_y]
    top_x = int(np.median(top_candidates[:, 0]))
    return top_x, top_y


def resize_for_display(image):
    height, width = image.shape[:2]
    if width <= MAX_DISPLAY_WIDTH:
        return image, 1.0

    scale = MAX_DISPLAY_WIDTH / width
    resized = cv2.resize(image, (MAX_DISPLAY_WIDTH, int(height * scale)))
    return resized, scale


def draw_detection(result, label, bbox, center, top_pixel, confidence=None):
    x, y, width, height = bbox
    cv2.rectangle(result, (x, y), (x + width, y + height), (0, 255, 0), 3)
    cv2.circle(result, center, 7, (0, 0, 255), -1)
    cv2.circle(result, top_pixel, 7, (255, 0, 0), -1)
    text = label
    if confidence is not None:
        text = f"{label} {confidence:.2f}"
    cv2.putText(
        result,
        text,
        (x, max(25, y - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2,
    )


def print_detections(detections):
    if not detections:
        print("No flower detected")
        return

    print(f"detected flowers: {len(detections)}")
    for index, detection in enumerate(detections, start=1):
        print(f"{index}. color: {detection['color']}")
        print(f"   bbox: {detection['bbox']}")
        print(f"   center point: {detection['center']}")
        print(f"   top pixel: {detection['top_pixel']}")
        if "confidence" in detection:
            print(f"   confidence: {detection['confidence']:.3f}")
        if "area" in detection:
            print(f"   area: {detection['area']:.0f}")


def detect_with_yolo(image):
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        print(f"ultralytics not available ({exc}); falling back to HSV")
        return None

    try:
        model = YOLO(str(MODEL_PATH))
    except Exception as exc:  # noqa: BLE001 - keep the test helper usable.
        print(f"Could not load YOLO model ({exc}); falling back to HSV")
        return None
    result = image.copy()
    detections = []
    bug_detected = False

    for yolo_result in model.predict(image, verbose=False):
        boxes = yolo_result.boxes
        if boxes is None:
            continue

        for box in boxes:
            class_id = int(box.cls[0])
            if class_id == YOLO_BUG_CLASS_ID:
                bug_detected = True
                continue

            color_name = YOLO_FLOWER_LABELS.get(class_id)
            if color_name is None:
                continue

            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            x = max(0, int(round(x1)))
            y = max(0, int(round(y1)))
            width = max(1, int(round(x2 - x1)))
            height = max(1, int(round(y2 - y1)))
            center = (int(round(x + width / 2.0)), int(round(y + height / 2.0)))
            top_pixel = (center[0], y)
            confidence = float(box.conf[0]) if box.conf is not None else 0.0

            draw_detection(
                result,
                color_name,
                (x, y, width, height),
                center,
                top_pixel,
                confidence,
            )
            detections.append(
                {
                    "color": color_name,
                    "confidence": confidence,
                    "bbox": (x, y, width, height),
                    "center": center,
                    "top_pixel": top_pixel,
                }
            )

    detections.sort(key=lambda detection: detection["confidence"], reverse=True)
    print(f"Using YOLO model: {MODEL_PATH}")
    print(f"bug detected: {bug_detected}")
    print_detections(detections)
    return result


def detect_with_hsv(image):
    print(
        "Using detection settings: "
        f"min_contour_area={MIN_CONTOUR_AREA}, "
        f"max_contour_area={MAX_CONTOUR_AREA}, "
        f"max_bbox_width={MAX_BBOX_SIZE_FRACTION * 100:.0f}%, "
        f"max_bbox_height={MAX_BBOX_SIZE_FRACTION * 100:.0f}%"
    )

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    kernel = np.ones((MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE), np.uint8)
    result = image.copy()
    detections = []

    for color_name, (lower, upper) in HSV_RANGES.items():
        mask = cv2.inRange(
            hsv,
            np.array(lower, dtype=np.uint8),
            np.array(upper, dtype=np.uint8),
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        for contour in find_valid_contours(mask, image.shape):
            area = cv2.contourArea(contour)
            x, y, width, height = cv2.boundingRect(contour)
            center = (x + width // 2, y + height // 2)
            top_pixel = contour_top_pixel(contour)

            draw_detection(
                result,
                color_name,
                (x, y, width, height),
                center,
                top_pixel,
            )

            detections.append(
                {
                    "color": color_name,
                    "area": area,
                    "bbox": (x, y, width, height),
                    "center": center,
                    "top_pixel": top_pixel,
                }
            )

    detections.sort(key=lambda detection: detection["area"], reverse=True)
    print_detections(detections)
    return result, hsv


def detect(image_path):
    image = cv2.imread(image_path)
    if image is None:
        print(f"Could not load image: {image_path}")
        return

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    result = None
    if MODEL_PATH.exists():
        result = detect_with_yolo(image)

    if result is None:
        result, hsv = detect_with_hsv(image)

    display_image, display_scale = resize_for_display(result)
    cv2.namedWindow("Flower detection")
    cv2.setMouseCallback(
        "Flower detection",
        print_hsv_on_click,
        {"hsv": hsv, "scale": display_scale},
    )
    cv2.imshow("Flower detection", display_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 test_flower_detection.py <image_path>")
        sys.exit(1)

    detect(sys.argv[1])
