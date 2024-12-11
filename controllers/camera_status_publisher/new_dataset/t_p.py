from ultralytics import YOLO

# Path to your data.yaml file
data_yaml_path = '/home/zaid/webot_proj/controllers/camera_status_publisher/new_dataset/data.yaml'

# Initialize YOLOv8 model (e.g., yolov8n for nano, yolov8s for small, etc.)
model = YOLO('yolov8s.pt')  # Pretrained YOLOv8 model

# Train the model
model.train(
    data=data_yaml_path,
    epochs=50,  # Number of epochs to train
    imgsz=640,  # Image size for training
    batch=16,   # Batch size
    name='yolov8_custom',  # Experiment name
    device=0   # Use GPU (set to 'cpu' if no GPU is available)
)
