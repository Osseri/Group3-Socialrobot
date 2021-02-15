# Import python libraries
import numpy as np
from opencv_tracker import OpenCVTracker
from scipy.optimize import linear_sum_assignment

class Track(object):
    def __init__(self, bbox, center, img, trackIdCount):
        
        self.track_id = trackIdCount  # identification of each track object
        self.tracker = OpenCVTracker(bbox, img)  # KF instance to track this object
        self.bbox = bbox  # predicted bbox
        self.center = center
        self.skipped_frames = 0  # number of frames skipped undetected
        self.trace = []  # trace path
        self.status = ""
    

class Tracker(object):

    def __init__(self, dist_thresh = 160, max_frames_to_skip = 30 , max_trace_length =10, trackIdCount = 1):

        self.frame = 0
        self.dist_thresh = dist_thresh
        self.max_frames_to_skip = max_frames_to_skip
        self.max_trace_length = max_trace_length
        self.tracks = []
        self.trackIdCount = trackIdCount

    def assign_detections(self, detection, prediction):

        return


    def update(self, ids, centers, bboxes, img):
        '''
        Args: 
            centers: centroid array ((x1,y1), (x2,y2)...)
            bboxes: bounding box array ((x1,y1,w1,h1), ...)
            img: original image
        '''
        self.frame += 1
        assignment = []  # idx: index of track, value: assigned index of detections
        un_assigned_tracks = []
        del_tracks = []
        un_assigned_detects = []

        # if self.frame > 90:
        #     bboxes = [bboxes[0]]
        #     centers = [centers[0]]

        # Create tracker if no tracks vector found
        # print "frame=", self.frame
        # print "tracker number = ", len(self.tracks)
        # print "detections number = ", len(bboxes)

        # if no tracks, start new tracks
        if(len(self.tracks) == 0):
            for i, bbox in enumerate(bboxes):
                track = Track(bboxes[i], centers[i], img, ids[i])
                self.trackIdCount += 1
                self.tracks.append(track)

        # Calculate cost using sum of square distance between
        # predicted vs detected centroids
        N = len(self.tracks)
        M = len(bboxes)
        cost = np.zeros(shape=(N, M))   # Cost matrix
        for i, track in enumerate(self.tracks):
            for j, center in enumerate(centers):
                detection = np.array([[center[0]],[center[1]]])
                prediction = np.array([[track.center[0]],[track.center[1]]])
                diff = prediction - detection
                distance = np.sqrt(diff[0][0]*diff[0][0] +
                                    diff[1][0]*diff[1][0])                
                cost[i][j] = distance
                
        # Let's average the squared ERROR
        cost = (0.5) * cost

        # Using Hungarian Algorithm assign the correct detected measurements
        # to predicted tracks       

        for _ in range(N):
            assignment.append(-1)
        tracker_idx, detection_idx = linear_sum_assignment(cost)
        for i in range(len(tracker_idx)):
            assignment[tracker_idx[i]] = detection_idx[i]
            #update bbox
            self.tracks[i].status = "DETECTING"
            self.tracks[i].tracker.update(bboxes[detection_idx[i]], img)

        # Identify tracks with no assignment, if any
        for i in range(len(assignment)):
            if (assignment[i] != -1):
                # check for cost distance threshold.
                # If cost is very high then un_assign (delete) the track
                if (cost[i][assignment[i]] > self.dist_thresh):
                    assignment[i] = -1
                    un_assigned_tracks.append(i)
                pass
            else:
                # If tracker have not assigned detection
                self.tracks[i].skipped_frames += 1  
                un_assigned_tracks.append(i)  
                self.tracks[i].status = "TRACKING"

        #If prediction error occurred, add to del list
        for i in range(len(self.tracks)):
            if not self.tracks[i].tracker.ok:
                del_tracks.append(i)

        # Now look for un_assigned detects
        for i in range(len(bboxes)):
            if i not in assignment:
                un_assigned_detects.append(i)

        # Start new tracks
        if(len(un_assigned_detects) != 0):
            for i in range(len(un_assigned_detects)):
                track = Track(bboxes[un_assigned_detects[i]], centers[un_assigned_detects[i]], img, ids[un_assigned_detects[i]])
                self.trackIdCount += 1
                self.tracks.append(track)

        # Update position state, and tracks trace
        for i, track in enumerate(self.tracks):

            track.skipped_frames = 0   
            track.tracker.predict(img)
            track.bbox = track.tracker.bbox
            track.center = (track.bbox[0] + track.bbox[2]/2, track.bbox[1] + track.bbox[3]/2)

            if(len(track.trace) > self.max_trace_length):
                for j in range(len(track.trace) -
                               self.max_trace_length):
                    del track.trace[j]

            track.trace.append(track.center)

        # print assignment,un_assigned_detects,un_assigned_tracks,del_tracks

        #If tracks are not detected for long time, remove them
        if len(del_tracks) > 0:  # only when skipped frame exceeds max
            for id in del_tracks:
                if id < len(self.tracks):
                    del self.tracks[id]
                    del assignment[id]
                else:
                    print("ERROR: id is greater than length of tracks")

        return self.tracks