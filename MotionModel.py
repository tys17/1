import sys
import numpy as np
import math

class MotionModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 5]
    """
    def __init__(self):

        """
        TODO : Initialize Motion Model parameters here
        """
        self.a1 = 0
        self.a2 = 0
        self.a3 = 0
        self.a4 = 0

    def update(self, u_t0, u_t1, x_t0):
        """
        param[in] u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]   
        param[in] u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
        param[in] x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
        param[out] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        """

        """
        TODO : Add your code here
        """
        theta0 = u_t0[2];
        theta1 = u_t1[2];        
        dx = u_t1[0]-u_t0[0];
        dy = u_t1[1]-u_t0[1];        
        dtheta = u_t1[2]-u_t0[2];
        a1 = self.a1
        a2 = self.a2
        a3 = self.a3
        a4 = self.a4
        
        drot1 = np.arctan2(dy, dx) - theta0;
        dtrans = np.sqrt(dx*dx + dy*dy);
        drot2 = theta1 - theta0 - drot1;
        
        var_rot1 = a1*drot1 + a2*dtrans; 
        if var_rot1>np.pi:
            var_rot1 = np.pi
        elif var_rot1 < -np.pi:
            var_rot1 = -np.pi
        
        var_rot2 = a1*drot2 + a2*dtrans;
        if var_rot2>np.pi:
            var_rot2 = np.pi
        elif var_rot2 < -np.pi:
            var_rot2 = -np.pi

        var_trans = a3*dtrans+a4*(drot1+drot2);
        
        drot1_p = drot1 - np.random.normal(0, abs(var_rot1));
        dtrans_p = dtrans - np.random.normal(0, abs(var_trans));
        drot2_p = drot2 - np.random.normal(0, abs(var_rot2));
        
        x_t1 = np.ndarray(3);
        x_t1[0] = x_t0[0] + dtrans_p*np.cos(x_t0[2]+drot1_p);
        x_t1[1] = x_t0[1] + dtrans_p*np.sin(x_t0[2]+drot1_p);
        x_t1[2] = x_t0[2] + drot1_p + drot2_p;
        # x_t1[3] = x_t0[3]

        return x_t1

if __name__=="__main__":
    pass