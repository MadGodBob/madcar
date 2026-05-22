import cv2
from threading import Thread
import queue

class VideoStream:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
        solution = [320,240]
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, solution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, solution[1])
        self.cap.set(6,cv2.VideoWriter.fourcc('M','J','P','G'))
        self.cap.set(cv2.CAP_PROP_FPS, 90)
        self.q = queue.Queue(maxsize=1)  # 只保留最新帧
        self.thread = Thread(target=self._update, daemon=True)
        self.thread.start()

    def _update(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            if not self.q.empty():
                try:
                    self.q.get_nowait()  # 丢弃旧帧
                except queue.Empty:
                    pass
            self.q.put(frame)

    def read(self):
        return self.q.get()

# 使用
stream = VideoStream(0)
count = 1
while True:
    frame = stream.read()  # 获取最新帧（无缓冲）
    cv2.imshow("Camera", frame)
    count += 1
    print(count)
    if cv2.waitKey(1) == ord('q'):
        break