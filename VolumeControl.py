import cv2
import time
import numpy as np
import HandTrackingModule as htm
from ctypes import cast, POINTER
from comtypes import CLSCTX_ALL
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume

class VolumeControl:
    def __init__(self, w_cam=640, h_cam=480, detection_con=0.7):
        self.w_cam = w_cam
        self.h_cam = h_cam
        self.detection_con = detection_con
        self.cap = None
        self.detector = None
        self.devices = None
        self.interface = None
        self.volume = None
        self.volRange = None
        self.minVol = None
        self.maxVol = None
        self.vol = 0
        self.volBar = 400
        self.volPer = 0
        self.area = 0
        self.colorVol = (255, 0, 0)
        self.pTime = 0

    def start(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, self.w_cam)
        self.cap.set(4, self.h_cam)
        self.detector = htm.handDetector(detectionCon=self.detection_con, maxHands=1)
        self.devices = AudioUtilities.GetSpeakers()
        self.interface = self.devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
        self.volume = cast(self.interface, POINTER(IAudioEndpointVolume))
        self.volRange = self.volume.GetVolumeRange()
        self.minVol = self.volRange[0]
        self.maxVol = self.volRange[1]

    def run(self):
        while True:
            success, img = self.cap.read()

            img = self.detector.findHands(img)
            lmList, bbox = self.detector.findPosition(img, draw=True)
            fingers=[]
            if len(lmList) != 0:
                fingers=self.detector.fingersUp()
                self.area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1]) // 100
                if 250 < self.area < 1000:
                    length, img, lineInfo = self.detector.findDistance(4, 8, img)

                    # Convert Volume
                    self.volBar = np.interp(length, [50, 200], [400, 150])
                    self.volPer = np.interp(length, [50, 200], [0, 100])

                    smoothness = 10
                    self.volPer = smoothness * round(self.volPer / smoothness)

                    fingers = self.detector.fingersUp()

                    # set volume
                    if not fingers[4]:
                        self.volume.SetMasterVolumeLevelScalar(self.volPer / 100, None)
                        cv2.circle(img, (lineInfo[4], lineInfo[5]), 15, (0, 255, 0), cv2.FILLED)
                        self.colorVol = (0, 255, 0)
                    else:
                        self.colorVol = (255, 0, 0)

            cv2.rectangle(img, (50, 150), (85, 400), (255, 0, 0), 3)
            cv2.rectangle(img, (50, int(self.volBar)), (85, 400), (255, 0, 0), cv2.FILLED)
            cv2.putText(img, f'{int(self.volPer)} %', (40, 450), cv2.FONT_HERSHEY_COMPLEX,
                        1, (255, 0, 0), 3)
            cVol = int(self.volume.GetMasterVolumeLevelScalar() * 100)
            cv2.putText(img, f'Vol Set: {int(cVol)}', (400, 50), cv2.FONT_HERSHEY_COMPLEX,
                        1, self.colorVol, 3)

            self.cTime = time.time()
            fps = 1 / (self.cTime - self.pTime)
            self.pTime = self.cTime
            cv2.putText(img, f'FPS: {int(fps)}', (40, 50), cv2.FONT_HERSHEY_COMPLEX,
                        1, (255, 0, 0), 3)

            cv2.imshow("Img", img)
            if cv2.waitKey(1) and fingers==[0,1,1,0,1]:
                break
        self.cap.release()
        cv2.destroyAllWindows()

    def end(self):
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    volume_control = VolumeControl()
    volume_control.start()
    volume_control.run()
    volume_control.end()

