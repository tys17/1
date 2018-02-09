import numpy as np
import math
import time
from matplotlib import pyplot as plt
from scipy.stats import norm
import pdb

from MapReader import MapReader

class SensorModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 6.3]
    """

    def __init__(self, occupancy_map):

        """
        TODO : Initialize Sensor Model parameters here
        """
        self.occupancy_map = occupancy_map
        self.z_hit = 0.4
        self.z_short = 0.1
        self.z_max = 0.1
        self.z_rand = 0.4
        self.sigma_hit = 100
        self.lambda_short = 0.1
        self.max_range = 800
        self.intrinsics_conv_th = 0.01
        self.stride=5
        self.sigma = 200
    def generate_normaldis(self,x,medium,sigma):
        x = np.divide(np.subtract(x, medium), sigma)
        return np.divide(1, np.exp(-358 * x / 23 + 111 * np.arctan(37 * x / 294)) + 1)

    def compute_Phit(self,z,zprime):
        tempt=np.divide(1,self.generate_normaldis(self.max_range,zprime,self.sigma_hit)-self.generate_normaldis(0,zprime,self.sigma_hit))
        p=np.multily(tempt,1.0/math.sqrt(2*math.pi*self.sigma_hit**2)*np.exp(-0.5*(z-zprime)**2/self.sigma_hit**2))
        return np.multiply(z<=self.max_range,p)
    def compute_Pshort(self,z,zprime):
        tempt=1.0/(1.0-math.e**(-1*self.lambda_short*zprime))
        p=np.multiply(tempt,self.lambda_short*math.e**(-1*self.lambda_short*z))
        return np.multiply(z<=zprime,p)
    def compute_Pmax(self,z):
        return (z==self.max_range).astype(dtype=np.float)
    def compute_Prand(self,z):
        return (z<self.max_range).astype(dtype=np.float)/self.max_range
    
    
    def laser_input(self,x,y,theta):
        x=np.round(x/10)
        y=np.round(y/10)
        laserdegree = np.floor((np.asarray(range(90, -90, -self._angle_step)) + theta * 180 / np.pi)) % 360
        degreenum=int((180 / self.stride))
        zs = np.zeros((degreenum,))
        cosset = np.cos(laserdegree*np.pi / 180)
        sinset = np.sin(laserdegree*np.pi / 180)
        for i in range(degreenum):
            searchx=x
            searchy=y
            pos=1
            while 0<=searchy<self.occupancy_map.shape[0] and 0<=searchx<self.occupancy_map.shape[1] and pos<=self.max_range:
                searchx=searchx+cosset[i]
                searchy=searchy+sinset[i]
                pos=pos+1
            zs[i]=pos
        return zs
    
    def compute_P(self,z,zprime):
        while True:
            phit=self.compute_Phit(z,zprime)
            pmax=self.compute_Pmax(z,zprime)
            pshort=self.compute_Pshort(z,zprime)
            prand=self.compute_Prand(z,zprime)
            eta=np.divide(1,phit+pmax+pshort+prand)
            eta_hit=np.multiply(phit, eta)
            eta_max=np.multiply(pmax, eta)
            eta_short=np.multiply(pshort, eta)
            eta_rand = np.multiply(prand, eta)
            z_hit=np.sum(eta_hit)/len(z)
            z_short=np.sum(eta_short)/len(z)
            if z_short==0:
                break
            z_max=np.sum(eta_max)/len(z)
            z_rand=np.sum(eta_rand)/len(z)
            sigma_hit = np.sqrt(np.sum(np.multiply(eta_hit,(z-zprime)**2))/np.sum(eta_hit))
            lambda_short = np.sum(eta_short) / np.sum(np.multiply(eta_short, z))
            if (self.z_hit - z_hit)**2<self.intrinsics_conv_th and(self.z_max - z_max) ** 2 < self.intrinsics_conv_th and (self.z_short - z_short) ** 2 < self.intrinsics_conv_th and (self.z_rand - z_rand) ** 2 < self.intrinsics_conv_th and (self.sigma_hit - sigma_hit) ** 2 < self.intrinsics_conv_th and (self.lambda_short - lambda_short) ** 2 < self.intrinsics_conv_th:
                break
            self.z_hit=z_hit
            self.z_max=z_max
            self.z_short=z_short
            self.z_rand=z_rand
            self.sigma_hit=sigma_hit
            self.lambda_short=lambda_short
        return self.z_hit*self.compute_Phit(z,zprime)+self.z_max*self.compute_Pmax(z,zprime)+self.z_short*self.compute_Pshort(z,zprime)+self.z_rand*self.compute_Prand(z,zprime)
            

    
    def beam_range_finder_model(self, z_t1_arr, x_t1):
        """
        param[in] z_t1_arr : laser range readings [array of 180 values] at time t
        param[in] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        param[out] prob_zt1 : likelihood of a range scan zt1 at time t
        """

        """
        TODO : Add your code here
        """
        degreenum=int((180 / self.stride))
        z_t1_arr_tempt = np.zeros((degreenum,))
        for i in range(degreenum):
            z_t1_arr_tempt[i]=z_t1_arr[self.stride*(i-1)+1]
        z_t1_arr_tempt=np.maximum(z_t1_arr_tempt,self.max_range)
        z_t1_prime=self.laser_input(x_t1[0],x_t1[1],x_t1[2])
        z_t1_prob=self.compute_P(z_t1_arr_tempt,z_t1_prime)
        q=np.sum(np.log(z_t1_prob*self.sigma))

        return q    
 
if __name__=='__main__':
    pass