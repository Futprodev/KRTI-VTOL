import cv2
import threading
import time

class CameraThread(threading.Thread):
    def __init__(self, camera_id, name, fps=10):
        threading.Thread.__init__(self)
        self.camera_id = camera_id
        self.name = name
        self.fps = fps
        self.cap = cv2.VideoCapture(self.camera_id)
        self.running = True

    def run(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                break

            # Display the resulting frame
            cv2.imshow(self.name, frame)

            # Limiting FPS
            time.sleep(1 / self.fps)

            # Press 'q' to stop
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False

        self.cap.release()
        cv2.destroyAllWindows()

    def stop(self):
        self.running = False

def main():
    # Create camera threads
    cam1 = CameraThread(camera_id=0, name='Camera 1', fps=10)
    cam2 = CameraThread(camera_id=1, name='Camera 2', fps=10)

    # Start camera threads
    cam1.start()
    cam2.start()

    # Wait for the threads to finish
    cam1.join()
    cam2.join()

if __name__ == "__main__":
    main()