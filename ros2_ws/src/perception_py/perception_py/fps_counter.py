import time
import cv2
class FPSCounter:
    def __init__(self, avg_window=30):
        self.avg_window = avg_window
        self.times = []
        self.last = time.time()

    def tick(self):
        now = time.time()
        self.times.append(now - self.last)
        self.last = now
        if len(self.times) > self.avg_window:
            self.times.pop(0)

    def fps(self):
        if not self.times:
            return 0.0
        return 1.0 / (sum(self.times) / len(self.times))

    def draw(self, frame):
        fps_text = f"FPS: {self.fps():.1f}"
        cv2.putText(frame, fps_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        return frame