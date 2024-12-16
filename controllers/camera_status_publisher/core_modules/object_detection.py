import cv2
import numpy as np
from ultralytics import YOLO

class ObjectDetection:
    def __init__(self, model_path, object_codes):
        self.model = YOLO(model_path)
        self.object_codes = object_codes  
    
    def detect_objects(self, camera):
        raw_image = camera.getImage()
        width = camera.getWidth()
        height = camera.getHeight()
        image = np.frombuffer(raw_image, dtype=np.uint8).reshape((height, width, 4)) 
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)

        results = self.model(image_rgb)
        detected_codes = []

        for result in results:
            if result.boxes:
                for box in result.boxes:
                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    label = self.model.names[class_id] if self.model.names else f"class_{class_id}"

                    if confidence > 0.5:  
                        code = self.object_codes.get(label, -1)
                        if code != -1:
                            detected_codes.append(code)

        return list(set(detected_codes)) 
