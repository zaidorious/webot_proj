from ultralytics import YOLO

# Load trained YOLOv8 model
model_path = '/home/zaid/webot_proj/controllers/camera_status_publisher/new_dataset/runs/detect/yolov8_custom/weights/best.pt'
model = YOLO(model_path)

# Run inference on an image
results = model('/home/zaid/webot_proj/controllers/camera_status_publisher/new_dataset/test/images/image_0001_png.rf.9f4407a36cce0c33439dc89861059ee0.jpg')

# Iterate over the results list and display each result
for result in results:
    result.plot()  # `plot()` saves the annotated image to memory
    result.show()  # Display the detections visually
