import numpy as np
import scipy.linalg as sln
from math import pi
from functools import partial

class UKF:
    def __init__(self, d):
        '''
        input: d, a list of tuples (xi,yi) of length 4

        '''
        self.d0 = np.asarray(d[0])[:,None]
        self.d1 = np.asarray(d[1])[:,None]
        self.d2 = np.asarray(d[2])[:,None]
        self.d3 = np.asarray(d[3])[:,None]
    def motion_model(self,x,sampling_time=1):
        '''
        input: x, (n x 1), state vector, which contains:
                    *   px          X-position
                    *   py          Y-position
                    *   v           velocity
                    *   phi         heading
                    *   omega       turn-rate

        output:fx,(n x 1), motion model evaluated at state x
               Fx,(n x n), motion model Jacobian evalueated at state x
        '''
        # if x is of shape (5,), then add one dimention
        if len(x.shape) == 1:
            x = x[:,None]

        fx = np.asarray([
                        x[0] + sampling_time*x[2]*np.cos(x[4]),
                        x[1] + sampling_time*x[2]*np.sin(x[4]),
                        x[2],
                        x[3] + sampling_time*x[4],
                        x[4]
                        ])
        
        Fx = np.identity(5)
        Fx[0,2] = sampling_time*np.cos(x[3])
        Fx[0,3] = -sampling_time*x[2]*np.sin(x[3])
        Fx[1,2] = sampling_time*np.sin(x[3])
        Fx[1,3] = sampling_time*x[2]*np.cos(x[3])
        Fx[3,4] = sampling_time
        return fx,Fx

    def meas_model(self,x):
        '''
        input: x, (n x 1), state vector, which contains(initially):
                    *   px          X-position
                    *   py          Y-position
                    *   v           velocity
                    *   phi         heading
                    *   omega       turn-rate
                Land
        output:hx,(m x 1), motion model evaluated at state x(initially)
                    *   px          X-position/ by hrp
                    *   py          Y-position/ by hrp
                    *   v           velocity/ by hrp
                    *   d1~d4       dist to landmarks/ by camera

               Fx,(m x n), meas model Jacobian evaluated at state x
        '''
        if len(x.shape) == 1:
            x = x[:,None]
        h = np.asarray([x[0].item(),
                        x[1].item(),
                        x[2].item(),
             np.linalg.norm(self.d0-x[0:2]),
             np.linalg.norm(self.d1-x[0:2]),
             np.linalg.norm(self.d2-x[0:2]),
             np.linalg.norm(self.d3-x[0:2])])
        
        Dd0Dx =  -(self.d0[0]-x[0])*(np.linalg.norm(self.d0-x[0:2])**2)
        Dd0Dy =  -(self.d0[1]-x[1])*(np.linalg.norm(self.d0-x[0:2])**2)
        
        Dd1Dx =  -(self.d1[0]-x[0])*(np.linalg.norm(self.d1-x[0:2])**2)
        Dd1Dy =  -(self.d1[1]-x[1])*(np.linalg.norm(self.d1-x[0:2])**2)
        
        Dd2Dx =  -(self.d2[0]-x[0])*(np.linalg.norm(self.d2-x[0:2])**2)
        Dd2Dy =  -(self.d2[1]-x[1])*(np.linalg.norm(self.d2-x[0:2])**2)
        
        Dd3Dx =  -(self.d3[0]-x[0])*(np.linalg.norm(self.d3-x[0:2])**2)
        Dd3Dy =  -(self.d3[1]-x[1])*(np.linalg.norm(self.d3-x[0:2])**2)
        
        H = np.array([
                     [1,0,0,0,0],
                     [0,1,0,0,0],
                     [0,0,1,0,0],
                     [Dd0Dx,Dd0Dy,0,0,0],
                     [Dd1Dx,Dd1Dy,0,0,0],
                     [Dd2Dx,Dd2Dy,0,0,0],
                     [Dd3Dx,Dd3Dy,0,0,0],
        ])
        
        return h,H
    def genNonLinearStateSeq(self,x_0, P_0, Q, N,func=motion_model):
        '''
        input:x_0, (n x 1) prior mean
               P0, (n x n) prior covariance
               motion_model, function 
                Q, (n x n) process noise covariance
                N, scalar, number of states to generate
        output: X, (n x N+1) state vector sequence
        '''
        n = x_0.shape[0]
        X0 = np.random.multivariate_normal(x_0[:,-1],P_0)
        
        X = np.zeros((n,N+1));
        X[:,0] = X0
        for i in np.arange(N)+1:
            f,_ = func(X[:,i-1])
            X[:,i] = f[:,-1]
            X[:,i] = X[:,i] + np.random.multivariate_normal(np.zeros(n),Q)
        return X
    def genNonLinearMeasSeq(self,X,R,func=meas_model):
        '''
        input:  X, (n x N+1) State vector sequence
                R, (m x m) meas noise covariance
        output: Y, (m x N) state vector sequence
        '''
        N = X.shape[1] - 1
        Y = []
        for i in range(N):
            meas_i,_ = func(X[:,i+1])
            meas_i = meas_i +  np.random.multivariate_normal(np.zeros(meas_i.shape[0]),R)
            Y.append(meas_i)
        Y = np.asarray(Y).T 
        return Y
    
    def sigma_points(self,x,P):
        '''
        input:  x, (n x 1) State vector sequence
                P, (n x n) meas noise covariance
        output: SP,(n x 2n+1) UKF
                 W,(1 x 2n+1) UKF
        '''
        n = x.shape[0]
        sqrt_P = sln.sqrtm(P)

        W = np.zeros(2*n+1)
        SP = np.zeros((n,2*n+1))

        W[0]  = 1 - n/3
        W[1:] = (1 - W[0])/(2*n)
        SP[:,0] = x[:,-1]

        for i in range(n):
            SP[:,i+1]   = x[:,-1] + np.sqrt(n/(1-W[0]))*sqrt_P[:,i]
            SP[:,i+1+n] = 2*x[:,-1] - SP[:,i+1] 

        return SP,W
    def prediction(self,x,P,Q,func=motion_model):
        SP,W = self.sigma_points(x,P)
        n = x.shape[0]
        x = 0
        P = 0

        for i in range(2*n+1):
            fx,_ = func(SP[:,i])
            x = x + fx*W[i]
            
        if len(x.shape) == 1:
            x = x[:,None]
    
        for i in range(2*n+1):
            fx,_ = func(SP[:,i])
            P = P + (fx-x)@(fx-x).T*W[i]+(i==2*n)*Q
            
        return x,P
    def update(self,x,P,y,R,func=meas_model):
        if len(y.shape) == 1:
            y = y[:,None]
        SP,W = self.sigma_points(x,P)
        y_hat = 0
        Pxy = 0
        S = 0
        for i in range(2*n+1):
            hx,_ = func(SP[:,i])
            hx = hx[:,None]
            y_hat = y_hat + hx*W[i]
        
        if len(x.shape) == 1:
            x = x[:,None]
            
        for i in range(2*n+1):
            hx,_ = func(SP[:,i])
            hx = hx[:,None]
            SPi = SP[:,i][:,None]
            Pxy = Pxy + (SPi-x)@(hx-y_hat).T*W[i]
            S = S + (hx-y_hat)@(hx-y_hat).T*W[i]+(i==2*n)*R
        
      
        S_inv = np.linalg.inv(S)
        x = x + Pxy@S_inv@(y-y_hat)
        P = P - Pxy@S_inv@Pxy.T
            
        return x,P
    def ukf_pipeline(self,Y,x_0,P_0,Q,R,funcMotion=motion_model,funcMeas=meas_model):
        N = Y.shape[1]
        n = x.shape[0]
     
        xf = []
        Pf = []
        xp = []
        Pp = []
        
        for i in range(N):
            if i == 0:
                xpi,Ppi = self.prediction(x_0,P_0,Q,funcMotion)
                xfi,Pfi = self.update(xpi,Ppi,Y[:,i],R,funcMeas)
                xp.append(xpi)
                Pp.append(Ppi)
                xf.append(xfi)
                Pf.append(Pfi)
            else:
                xpi,Ppi = self.prediction(xf[i-1],Pf[i-1],Q,funcMotion)
                xfi,Pfi = self.update(xpi,Ppi,Y[:,i],R,funcMeas)
                xp.append(xpi)
                Pp.append(Ppi)
                xf.append(xfi)
                Pf.append(Pfi)
        xp = np.stack(xp,axis=1)
        Pp = np.stack(Pp,axis=2)
        xf = np.stack(xf,axis=1)
        Pf = np.stack(Pf,axis=2)
        
        xp = xp[:,:,-1]
        xf = xf[:,:,-1]
        return xp,Pp,xf,Pf