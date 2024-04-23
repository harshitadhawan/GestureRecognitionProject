import cv2
import numpy as np
import autopy
import time
from HandTrackingModule import handDetector


class VirtualMouse:
    def __init__(self, w_cam: int = 640, h_cam: int = 480, frame_reduction: int = 100, smoothening: int = 7):
        self.w_cam = w_cam
        self.h_cam = h_cam
        self.frame_reduction = frame_reduction
        self.smoothening = smoothening
        self.plocX, self.plocY = 0, 0
        self.clocX, self.clocY = 0, 0
        self.detector = None
        self.cap = None
        self.pTime=0
        self.cTime=0
        self.button=False

    def start(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, self.w_cam)
        self.cap.set(4, self.h_cam)
        self.detector = handDetector(maxHands=1)
        self.w_scr, self.h_scr = autopy.screen.size()

    def run_virtual_mouse(self):
        delay=20
        counter=0
        while True:
            success, img = self.cap.read()
            
            img = self.detector.findHands(img)
            lm_list, bbox = self.detector.findPosition(img)
            
            fingers = []
            if len(lm_list) != 0:
                x1, y1 = lm_list[8][1:]
                x2, y2 = lm_list[12][1:]
                fingers = self.detector.fingersUp()
                
                cv2.rectangle(img, (self.frame_reduction, self.frame_reduction), (self.w_cam - self.frame_reduction, self.h_cam - self.frame_reduction),
                            (255, 0, 255), 2)
                
                # finger up
                if fingers[1] == 1 and fingers[2] == 0:
                    x3 = np.interp(x1, (self.frame_reduction, self.w_cam - self.frame_reduction), (0, self.w_scr))
                    y3 = np.interp(y1, (self.frame_reduction, self.h_cam - self.frame_reduction), (0, self.h_scr))
                    
                    self.clocX = self.plocX + (x3 - self.plocX) / self.smoothening
                    self.clocY = self.plocY + (y3 - self.plocY) / self.smoothening
                    
                    if 0 <= self.clocX <= self.w_scr and 0 <= self.clocY <= self.h_scr:
                        autopy.mouse.move(self.w_scr - self.clocX, self.clocY)
                        cv2.circle(img, (x1, y1), 15, (255, 0, 255), cv2.FILLED)
                        self.plocX, self.plocY = self.clocX, self.clocY
                #mouse click
                if fingers[1] == 1 and fingers[2] == 1:
                    length, img, line_info = self.detector.findDistance(8, 12, img)
                    if length < 40 and self.button is False:
                        self.button=True
                        cv2.circle(img, (line_info[4], line_info[5]), 15, (0, 255, 0), cv2.FILLED)
                        autopy.mouse.click()
            if self.button:
                counter += 1
                if counter > delay:
                    counter = 0
                    self.button = False
                        
            
            self.cTime = time.time()
            fps = 1 / (self.cTime - self.pTime)
            self.pTime = self.cTime
            cv2.putText(img, str(int(fps)), (20, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 0), 3)
            
            cv2.imshow("Image", img)
            
            if cv2.waitKey(1) & fingers.count(1) == 5:
                break
        self.cap.release()
        cv2.destroyAllWindows()
    



    def end(self):
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    virtual_mouse = VirtualMouse()
    virtual_mouse.start()
    virtual_mouse.run_virtual_mouse()
    virtual_mouse.end()
