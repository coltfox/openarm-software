import cv2
from ultralytics import YOLO

# Load pre-trained YOLOv8 model
model = YOLO("yolov8n.pt")  # Use 'yolov8n.pt' for a lightweight model

# Initialize webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Perform inference
    results = model(frame)

    # Render results on the frame
    frame = results[0].plot()

    # Extract predictions
    predictions = results[0].boxes
    if predictions is not None:
        # Get top 3 predictions by confidence score
        top_predictions = sorted(zip(predictions.cls, predictions.conf), key=lambda x: x[1], reverse=True)[:3]

        # Annotate frame with top 3 predictions
        for cls, conf in top_predictions:
            label = f"{model.names[int(cls)]}: {conf:.2f}"
            print(label)

    # Display the frame
    cv2.imshow("YOLOv8 Object Detection", frame)

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
