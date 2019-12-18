import numpy as np


class SensorFusion:
    def __init__(self, x_0, P_0, Q, R_c, R_o, T):
        """
        Define internal variables.
        :param x_0: The prior mean
        :param P_0: The prior variance
        :param Q: The motion model variance.
        :param R_c: The camera measurement model variance
        :param R_o: The odometer measurement model variance
        :param T: The time step.
        """
        self.x = x_0
        self.P = P_0
        self.Q = Q
        self.Rc = R_c
        self.Ro = R_o
        self.T = T
        self.A = np.array([[1, T], [0, 1]])
        self.H = np.array([1, 0])
        self.odom = 0

        # Some high-pass filter parameters.
        self.alpha = 0.05
        self.old_yc = 0
        self.old_yo = 0
        self.last_yc = 0
        self.last_yo = 0

    def prediction(self):
        """
        Computes the Kalman prediction, returns the prediction x and the variance P.
        :return: Updates self.x, self.P
        """
        self.x = np.dot(self.A, self.x)
        self.P = np.matmul(np.matmul(self.A,self.P),np.transpose(self.A)) + self.Q

    def update_cam(self, y_c):
        """
        Update x and P with the yaw measurement from the camera.
        :param y_c: The yaw measurement from the camera.
        :return: Updates self.x, self.P
        """

        v = y_c - np.matmul(self.H,self.x)
        S = np.matmul(np.matmul(self.H,self.P),np.transpose(self.H)) + self.Rc
        K = np.array([np.matmul(self.P,np.transpose(self.H))*(1/S)])

        self.x = self.x + K.transpose()*v
        self.P = self.P - np.matmul(np.matmul(K,S),np.transpose(K))

    def update_odometer(self, y_o):
        """
        Update x and P with the yaw measurement from the odometer.
        :param y_c: The yaw measurement from the odometer.
        :return: Updates self.x, self.P
        """

        v = y_o - np.matmul(self.H,self.x)
        S = np.matmul(np.matmul(self.H,self.P),np.transpose(self.H)) + self.Ro
        K = np.array([np.matmul(self.P,np.transpose(self.H))*(1/S)])

        self.x = self.x + K.transpose()*v
        self.P = self.P - np.matmul(np.matmul(K,S),np.transpose(K))

    def take_step(self, y_c):
        """
        Takes one update step using the measurements from both the camera and the odometer.
        :param y_c: Measurement from the camera.
        :param y_o: Measurement from the odometer.
        :return: Updates the mean and variance of the states, yaw and yaw-rate.
        """
        self.prediction()
        self.update_cam(y_c)
        self.update_odometer(self.odom)

    def update_odom(self, odom_yaw):
        """
        updates internal variables-
        :param odom_yaw:
        :return:
        """
        self.odom = odom_yaw
