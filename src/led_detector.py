import cv2 as cv
import numpy as np

class LEDDetector:
    def __init__(self, rgba_color: list[float] | tuple[float, ...], min_area: int = 50) -> None: 
        self.min_area = min_area
        self.rgba_color = rgba_color
        self.calculate_hsv_bounds()

    def calculate_hsv_bounds(self) -> None:
        r = int(self.rgba_color[0] * 255)
        g = int(self.rgba_color[1] * 255)
        b = int(self.rgba_color[2] * 255)
        
        bgr_pixel = np.uint8([[[b, g, r]]])
        hsv_pixel = cv.cvtColor(bgr_pixel, cv.COLOR_BGR2HSV)[0][0]
        h, s, v = hsv_pixel[0], hsv_pixel[1], hsv_pixel[2]

        h_tolerance = 10
        s_tolerance = 150  
        v_tolerance = 150  

        self.lower_hsv = np.array([max(0, h - h_tolerance), max(50, s - s_tolerance), max(50, v - v_tolerance)])
        self.upper_hsv = np.array([min(180, h + h_tolerance), 255, 255])
        
        self.lower_hsv2, self.upper_hsv2 = None, None
        if h < h_tolerance:
            self.lower_hsv2 = np.array([180 - (h_tolerance - h), max(50, s - s_tolerance), max(50, v - v_tolerance)])
            self.upper_hsv2 = np.array([180, 255, 255])
        elif h > (180 - h_tolerance):
            self.lower_hsv2 = np.array([0, max(50, s - s_tolerance), max(50, v - v_tolerance)])
            self.upper_hsv2 = np.array([h_tolerance - (180 - h), 255, 255])

    def detect(self, frame: np.ndarray, drone_id: int) -> tuple[int | None, int | None, float]:
        if frame is None or frame.size == 0:
            return None, None, 0.0

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, self.lower_hsv, self.upper_hsv)
        
        if self.lower_hsv2 is not None and self.upper_hsv2 is not None:
            mask2 = cv.inRange(hsv, self.lower_hsv2, self.upper_hsv2)
            mask = cv.bitwise_or(mask, mask2)
        
        kernel = np.ones((3, 3), np.uint8)
        mask = cv.erode(mask, kernel, iterations=1)
        mask = cv.dilate(mask, kernel, iterations=2)
        
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        center_x, center_y, area = None, None, 0.0
        
        if contours:
            largest_contour = max(contours, key=cv.contourArea)
            current_area = cv.contourArea(largest_contour)
            
            if current_area > self.min_area:
                area = current_area
                M = cv.moments(largest_contour)
                if M["m00"] != 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])
                    
                    # Окреслюємо область
                    cv.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
                    cv.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                    cv.putText(frame, f"Area: {int(area)}", (center_x + 10, center_y - 10), 
                               cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # Рендеринг вікон
        cv.imshow(f"Drone {drone_id} Camera View", frame)
        cv.imshow(f"Drone {drone_id} Binary Mask", mask)
        cv.waitKey(1)
                    
        return center_x, center_y, float(area)