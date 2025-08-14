import cv2

def stream_camera(camera_index=0):
    """
    Streams the camera feed from the specified camera index using OpenCV.
    :param camera_index: Index of the camera (default 0)
    """
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Error: Could not open camera with index {camera_index}")
        return

    print(f"Streaming camera {camera_index}. Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break
        cv2.imshow(f'Camera {camera_index} Stream', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        idx = int(input("Enter camera index (default 0): ") or "0")
    except ValueError:
        idx = 0
    stream_camera(idx)
