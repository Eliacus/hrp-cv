"""
Camera feature points tracker.
Extracts feature points in two following images to compute the euler angles and the translation.
"""

import numpy as np
import cv2
from comon import draw_str
import math
#import rospy
#from std_msgs.msg import Float32, String
import matplotlib.animation as animation
import matplotlib.pyplot as plt

lk_params = dict(winSize=(15, 15),
                 maxLevel=5,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.05))

feature_params = dict(maxCorners=300,
                      qualityLevel=0.05,
                      minDistance=7,
                      blockSize=7)


class App:
    def __init__(self, camera_input):
        self.track_len = 10
        self.detect_interval = 5
        self.tracks = []
        self.cam = camera_input
        self.frame_idx = 0
        self.prev_gray = None

    # Checks if a matrix is a valid rotation matrix.
    def isRotationMatrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    def rotationMatrixToEulerAngles(self, R):

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

    def run(self):
        global yaw_left
        global yaw_right
        global translation_all
        translation_all = [[0.0001, 0.0001, 0.0001]]
        rotation = np.array([0.0001, 0.0001, 0.0001])
        translation = np.array([0.0001, 0.0001, 0.0001])

        while True:
            ret, frame = self.cam.read()
            print(type(frame))

            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
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

            ch = 0xFF & cv2.waitKey(1)
            if ch == 27:
                break

            K = np.array([[2593, 0, 960], [0, 2593, 540], [0, 0, 1]])

            if self.frame_idx >= 2:
                q1 = 0
                num_q1 = 0
                q2 = 0
                num_q2 = 0
                q3 = 0
                num_q3 = 0
                q4 = 0
                num_q4 = 0

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
                    M, mask = cv2.findHomography(old_points, new_points, cv2.RANSAC, 5.0)
                except:
                    print("No M")
                try:
                    _, Rs, Ts, Ns = cv2.decomposeHomographyMat(M, K)
                    for i in range(len(Rs)):
                        try:
                            euler_angles = self.rotationMatrixToEulerAngles(Rs[i])
                            rotation += euler_angles
                        except:
                            print("No euler angles")

                except:
                    print("No decompose")

                translation += np.array([Ts[0][0][0], Ts[0][1][0], Ts[0][2][0]])
                translation_all.append(translation)
                print(rotation)

                for i in range(len(self.tracks)):
                    try:
                        new_point = self.tracks[i][-1]
                        old_point = self.tracks[i][-2]

                        angle = np.arctan((new_point[1]-old_point[1])/(new_point[0]-old_point[0]))
                        left = 0
                        if new_point[0]-old_point[0] < 0:
                            left = 1
                        dist = np.sqrt((new_point[1]-old_point[1])**2 + (new_point[0]-old_point[0])**2)

                        if 0 < angle < np.pi/2 and left == 0:
                            q1 += dist
                            num_q1 += 1
                        elif 0 < angle < np.pi/2 and left == 1:
                            q2 += dist
                            num_q2 += 1
                        elif -np.pi/2 <= angle < 0 and left == 1:
                            q3 += dist
                            num_q3 += 1
                        elif -np.pi/2 <= angle < 0 and left == 0:
                            q4 += dist
                            num_q4 += 1

                    except IndexError:
                        continue
                if num_q1 > 0:
                    q1 = q1 / num_q1
                if num_q2 > 0:
                    q2 = q2 / num_q2
                if num_q3 > 0:
                    q3 = q3 / num_q3
                if num_q4 > 0:
                    q4 = q4 / num_q4

                if (q1 + q4) > 70*(q3+q2):
                    print("Yaw left")
                elif (q3 + q2) > 70*(q1 + q4):
                    print("Yaw right")

                yaw_left = q1 + q4
                yaw_right = q3 + q2

                cv2.arrowedLine(vis, (300, 100), (300 - (int(round(q2+q3))), 100), (0, 0, 255), 3, 8, 0, 0.1)
                cv2.arrowedLine(vis, (300, 100), (300 + (int(round(q1 + q4))), 100), (255, 0, 0), 3, 8, 0, 0.1)

            self.frame_idx += 1
            self.prev_gray = frame_gray
            cv2.imshow('lk_track', vis)


def animate(i):
    ax.clear()
    ax.plot(translation_all[0][:], translation_all[0][:], 'red')
    fig.show(ax)
    print("Hello")


def main():
    global ax
    global fig
    app = App(cv2.VideoCapture(0))
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    App.run(app)
    cv2.destroyAllWindows()


def fnc_callback():
    global varS
    varS = cv2.VideoCapture(0)
    app = App()
    App.run(app)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
    """ Add here the name of the ROS. In ROS, names are unique named.
    rospy.init_node('lk_node')
    # subscribe to a topic using rospy.Subscriber class
    sub = rospy.Subscriber('cam_stream', None, fnc_callback)
    # publish messages to a topic using rospy.Publisher class
    pub = rospy.Publisher('control', String, queue_size=1)
    rate = rospy.Rate(10)
    tol = 50

    while not rospy.is_shutdown():
        if yaw_left > tol*yaw_right:
            pub.publish('d')
        elif yaw_right > tol*yaw_left:
            pub.publish('a')
        else:
            pub.publish('w')
        rate.sleep()
"""