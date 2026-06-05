import sys
from pathlib import Path

import cv2
from ultralytics import YOLO


MODEL_PATH = (
    Path(__file__).parent
    / "src"
    / "simbiosys_perception"
    / "simbiosys_perception"
    / "models"
    / "flower_model (Copy).pt"
)
BUG_CLASS_ID = 3
MAX_DISPLAY_WIDTH = 1200


def resize_for_display(image):
    height, width = image.shape[:2]
    if width <= MAX_DISPLAY_WIDTH:
        return image

    scale = MAX_DISPLAY_WIDTH / width
    return cv2.resize(image, (MAX_DISPLAY_WIDTH, int(height * scale)))


def detect_bug(image_path):
    image = cv2.imread(image_path)
    if image is None:
        print(f"Could not load image: {image_path}")
        return

    model = YOLO(str(MODEL_PATH))
    result = image.copy()
    detections = []

    for yolo_result in model(image, conf=0.4, iou=0.5, verbose=False):
        boxes = yolo_result.boxes
        if boxes is None:
            continue

        for box in boxes:
            class_id = int(box.cls[0])
            if class_id != BUG_CLASS_ID:
                continue

            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            x1 = max(0, int(round(x1)))
            y1 = max(0, int(round(y1)))
            x2 = max(0, int(round(x2)))
            y2 = max(0, int(round(y2)))
            confidence = float(box.conf[0]) if box.conf is not None else 0.0

            cv2.rectangle(result, (x1, y1), (x2, y2), (0, 0, 255), 3)
            cv2.putText(
                result,
                f"bug {confidence:.2f}",
                (x1, max(25, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )
            detections.append((x1, y1, x2, y2, confidence))

    if detections:
        print(f"Bug detections: {len(detections)}")
        for index, (x1, y1, x2, y2, confidence) in enumerate(detections, start=1):
            print(
                f"{index}. bbox=({x1}, {y1}, {x2}, {y2}); "
                f"confidence={confidence:.3f}"
            )
    else:
        print("No bug detected")

    cv2.imshow("Bug detection", resize_for_display(result))
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 test_bug_detection.py <image_path>")
        sys.exit(1)

    detect_bug(sys.argv[1])
