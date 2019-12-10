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
        self.euler_angles = [0, 0, 0]

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

        frame_gray = curr_img
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
        K = np.array([[993, 0, 673], [0, 990, 455], [0, 0, 1]])

        # Begin the calcualtions when two of more frames are recieved.
        if self.frame_idx >= 2:

            # Extract the feature points that is used to compute the homeography.
            new_points = []
            old_points = []
            new_points_1 = []
            old_points_1 = []
            for i in range(len(self.tracks)):
                try:
                    vec1 = list(self.tracks[i][-1])
                    vec1.append(1)
                    new_points.append(vec1)
                    new_points_1.append(self.tracks[i][-1])
                    vec2 = list(self.tracks[i][-2])
                    vec2.append(1)
                    old_points.append(vec2)
                    old_points_1.append(self.tracks[i][-2])
                except IndexError:
                    continue
            new_points = np.array(new_points)
            old_points = np.array(old_points)
            new_points_1 = np.array(new_points_1)
            old_points_1 = np.array(old_points_1)

            try:
                M, mask = cv2.findHomography(old_points_1, new_points_1, cv2.RANSAC, 5.0)
            except:
                pass

            try:
                _, Rs, Ts, Ns = cv2.decomposeHomographyMat(M, K)

                neg_pts = []

                old_points = np.matmul(np.invert(K), np.transpose(old_points))
                new_points = np.matmul(np.invert(K), np.transpose(new_points))

                for i in range(len(Rs)):

                    counter = 0

                    for j in range(min([len(new_points), len(old_points)])):
                        P1 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]]
                        P2 = []
                        for k in range(3):
                            row = list(Rs[i][k][0:3])
                            row.append(Ts[i][k][0])
                            P2.append(row)

                        A = []
                        for m in range(3):
                            row = P1[m]
                            row.append(-old_points[j][m])
                            row.append(0)
                            A.append(row)
                        for m in range(3):
                            row = P2[m]
                            row.append(0)
                            row.append(-new_points[j][m])
                            A.append(row)

                        A = np.array(A)
                        [U, W, V] = np.linalg.svd(A)

                        V = V / V[3]

                        Xs = V[:, -1]

                        if Xs[4] < 0 or Xs[5] < 0:
                            counter += 1

                    neg_pts.append(counter)

                ind_R = neg_pts.index(min(neg_pts))

                euler_angles = self.rotationMatrixToEulerAngles(Rs[ind_R])
                self.euler_angles += euler_angles

            except:
                pass

        print(self.euler_angles[1])

        # Add one to the frame index.
        self.frame_idx += 1

        # Save the previous grayscale image.
        self.prev_gray = frame_gray
