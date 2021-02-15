import cv2

class OpenCVTracker:

    count = 0
    def __init__(self, bbox, img):
        self.tracker_type = 'CSRT'
        self.tracker = None
        if self.tracker_type == 'BOOSTING':
            self.tracker = cv2.TrackerBoosting_create()
        if self.tracker_type == 'MIL':
            self.tracker = cv2.TrackerMIL_create()
        if self.tracker_type == 'KCF':
            self.tracker = cv2.TrackerKCF_create()
        if self.tracker_type == 'TLD':
            self.tracker = cv2.TrackerTLD_create()
        if self.tracker_type == 'MEDIANFLOW':
            self.tracker = cv2.TrackerMedianFlow_create()
        if self.tracker_type == 'GOTURN':
            self.tracker = cv2.TrackerGOTURN_create()
        if self.tracker_type == "CSRT":
            self.tracker = cv2.TrackerCSRT_create()
    
        self.tracker.init(img, (bbox[0], bbox[1], bbox[2], bbox[3]))

        self.ok = True
        self.bbox = [bbox[0], bbox[1], bbox[2], bbox[3]]

    def update(self, bbox, img):    
        '''
        re-start the tracker with detected positions (it detector was active)
        '''
        self.tracker.init(img, (bbox[0], bbox[1], bbox[2], bbox[3]))

    def predict(self, img):
        ok, bbox = self.tracker.update(img)
        self.ok = ok
        self.bbox = bbox
    

    def get_state(self):
        return self.bbox