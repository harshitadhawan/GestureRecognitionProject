import cv2
import pyautogui as pg
from HandTrackingModule import handDetector

class Presentation:
    def __init__(self, width=1280, height=720, gesture_threshold=300):
        self.width = width
        self.height = height
        
        self.gesture_threshold = gesture_threshold
        self.buttonPressed = False
        self.counter = 0
        self.delayCounter = 15

    def start(self):
        cap = cv2.VideoCapture(0)
        self.detector = handDetector(detectionCon=0.8, maxHands=1)
        cap.set(3, self.width)
        cap.set(4, self.height)

        while True:
            success, img = cap.read()
            img = self.detector.findHands(img)
            lmList, _ = self.detector.findPosition(img)
            cv2.line(img, (0, self.gesture_threshold), (self.width, self.gesture_threshold), (0, 255, 0), 10)
            fingers=[]
            if lmList:
                cy = lmList[13][2]
                fingers = self.detector.fingersUp()

                if not self.buttonPressed and cy < self.gesture_threshold:
                    if fingers == [1, 0, 0, 0, 0]:
                        self.buttonPressed = True
                        pg.hotkey('left')
                    elif fingers == [0, 0, 0, 0, 1]:
                        self.buttonPressed = True
                        pg.hotkey('right')

            if self.buttonPressed:
                if self.counter > self.delayCounter:
                    self.counter = 0
                    self.buttonPressed = False
                else:
                    self.counter += 1

            cv2.imshow("Image", img)

            key = cv2.waitKey(1)
            if key and fingers==[1,1,1,1,1]:
                break

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    from HandTrackingModule import handDetector   
    controller = Presentation()    
    controller.start()
