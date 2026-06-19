import cv2 as cv 
import numpy as np
import time

class LEDDetector:
    def __init__(self, lower_hsv=None, upper_hsv=None, min_area=50) -> None: 
        """
        Ініціалізує детектор LED-маяка.
        :param lower_hsv: Нижня межа кольору в HSV (list або tuple)
        :param upper_hsv: Верхня межа кольору в HSV (list або tuple)
        :param min_area: Мінімальна площа в пікселях для відсіювання шуму
        """
        # Дефолтні значення для зеленого кольору (з урахуванням можливих засвітів)
        if lower_hsv is None:
            self.lower_hsv = np.array([35, 50, 50])
        else:
            self.lower_hsv = np.array(lower_hsv)
            
        if upper_hsv is None:
            self.upper_hsv = np.array([90, 255, 255])
        else:
            self.upper_hsv = np.array(upper_hsv)
            
        self.min_area = min_area

    def detect(self, frame: np.ndarray) -> tuple[int | None, int | None, float]:
        """
        Знаходить LED-маяк на зображенні.
        
        :param frame: Зображення у форматі BGR (стандарт OpenCV)
        :return: Кортеж (center_x, center_y, area). Якщо нічого не знайдено, повертає (None, None, 0.0)
        """
        if frame is None or frame.size == 0:
            return None, None, 0.0

        # 1. Переводимо кадр у формат HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        
        # 2. Створюємо бінарну маску
        mask = cv.inRange(hsv, self.lower_hsv, self.upper_hsv)
        
        # 3. Прибираємо шум (морфологічні операції)
        # Erode прибирає дрібні крапки, Dilate повертає розмір основній плямі
        kernel = np.ones((3, 3), np.uint8)
        mask = cv.erode(mask, kernel, iterations=1)
        mask = cv.dilate(mask, kernel, iterations=2)
        
        # 4. Шукаємо контури
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Беремо контур з найбільшою площею
            largest_contour = max(contours, key=cv.contourArea)
            area = cv.contourArea(largest_contour)
            
            # Перевіряємо, чи це не просто піксельний шум
            if area > self.min_area:
                # 5. Рахуємо центр маси контуру (моменти)
                M = cv.moments(largest_contour)
                if M["m00"] != 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])
                    
                    return center_x, center_y, float(area)
                    
        # Маяк не знайдено або він занадто малий
        return None, None, 0.0

# === Приклад використання ===
if __name__ == "__main__":
    # Створюємо екземпляр детектора
    detector = LEDDetector(min_area=80)
    
    # Симуляція отримання кадру (замініть на реальне відео або топік ROS)
    cap = cv.VideoCapture(0)
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
            
        start_time = time.time()
        
        # Викликаємо ваш метод
        cx, cy, area = detector.detect(frame)
        
        fps = 1.0 / (time.time() - start_time)
        
        # Візуалізація результату
        if cx is not None:
            cv.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv.putText(frame, f"Area: {area:.1f}", (cx + 10, cy), 
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
        cv.putText(frame, f"FPS: {fps:.1f}", (10, 20), 
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                   
        cv.imshow("Drone Camera", frame)
        
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv.destroyAllWindows()