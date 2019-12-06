"""
CAMERA FEATURE POINTS TRACKER USING SIFT

Extracts feature points in two following images to compute the euler angles and the translation.
MPSYS Project Course for HRP, group 20
"""

import numpy as np
import cv2
from comon import draw_str
import math

# Add some parameters to the SIFT-extraction.
lk_params = dict(winSize=(15, 15),
                 maxLevel=5,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.05))

feature_params = dict(maxCorners=300,
                      qualityLevel=0.05,
                      minDistance=7,
                      blockSize=7)


class FeatureTracker:
    def __init__(self):
        self.track_len = 10
        self.detect_interval = 5
        self.tracks = []
        self.frame_idx = 0
        self.prev_gray = None
        self.euler_angles = [0,0,0]

    def isRotationMatrix(self, R):
        """
        Checks if the rotation matrix is vaild.
        @param: R, rotation matrix
        """
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    def rotationMatrixToEulerAngles(self, R):
        """
        Converts the rotation matrix to Euler angles.
        @param: R, rotation matrix
        """

        assert(self.isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def run(self, curr_img):
        """
        Computes the euler angles from two following pictures.
        @param: curr_img, the current image in the image stream.
        """

        frame_gray = cv2.cvtColor(curr_img, cv2.COLOR_BGR2GRAY)
        vis = frame_gray.copy()

        if len(self.tracks) > 0:
            img0, img1 = self.prev_gray, frame_gray
            p0 = np.float32([tr[-1] for tr in self.tracks]).reshape(-1, 1, 2)
            p1, st, err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **lk_params)
            p0r, st, err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **lk_params)

            d = abs(p0-p0r).reshape(-1, 2).max(-1)
            good = d < 1
            new_tracks = []
            for tr, (x, y), good_flag in zip(self.tracks, p1.reshape(-1, 2), good):
                if not good_flag:
                    continue
                tr.append((x, y))
                if len(tr) > self.track_len:
                    del tr[0]
                new_tracks.append(tr)
                cv2.circle(vis, (x, y), 2, (0, 255, 0), -1)

            self.tracks = new_tracks
            cv2.polylines(vis, [np.int32(tr) for tr in self.tracks], False, (0, 255, 0))
            draw_str(vis, (20, 20), 'track count: %d' % len(self.tracks))

        if self.frame_idx % self.detect_interval == 0:
            mask = np.zeros_like(frame_gray)
            mask[:] = 255
            for x, y in [np.int32(tr[-1]) for tr in self.tracks]:
                cv2.circle(mask, (x, y), 5, 0, -1)
            p = cv2.goodFeaturesToTrack(frame_gray, mask=mask, **feature_params)
            if p is not None:
                for x, y in np.float32(p).reshape(-1, 2):
                    self.tracks.append([(x, y)])

        # The calibrated camera parameters.
        K = np.array([[976.558219, 0, 637.122052], [0, 974.99846, 456.40230], [0, 0, 1]])

        # Begin the calcualtions when two of more frames are recieved.
        if self.frame_idx >= 2:

            # Extract the feature points that is used to compute the homeography.
            new_points = []
            old_points = []
            for i in range(len(self.tracks)):
                try:
                    new_points.append(self.tracks[i][-1])
                    old_points.append(self.tracks[i][-2])
                except IndexError:
                    continue
            new_points = np.array(new_points)
            old_points = np.array(old_points)

            try:
                # Compute the M matrx for the homogenious camera equations.
                M, mask = cv2.findHomography(old_points, new_points, cv2.RANSAC, 5.0)
            except:
                pass
            try:
                # Extract the rotation, Rs, and translation, Ts, from the M and K matrix.
                _, Rs, Ts, Ns = cv2.decomposeHomographyMat(M, K)
                for i in range(len(Rs)):
                    try:
                        # From the rotation matrix extract the euler angles, i.e. the difference in direction between the two frames.
                        euler_angles_frame = self.rotationMatrixToEulerAngles(Rs[i])

                        # Add the euler angles to the total rotation of the robot.
                        self.euler_angles += euler_angles_frame
                    except:
                        pass
            except:
                pass

        # Add one to the frame index.
        self.frame_idx += 1

        # Save the previous grayscale image.
        self.prev_gray = frame_gray
