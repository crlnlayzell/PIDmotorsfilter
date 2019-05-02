# -*- coding: utf-8 -*-
"""
Created on Sat Apr 27 16:13:55 2019

@author: Carol
"""
import numpy as np
import Fetch_Optic as opflow
import smbus
import pdb
import time as t
import numpy as np
from numpy.linalg import inv

bus = smbus.SMBus(1)    # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)

MD25_ADDRESS12 = 0x58      #7 bit address (will be left shifted to add the read write bit)
MD25_ADDRESS34 = 0x59 
SOFTWARE_REG = 0x0D
CMD = 0x10
RESET_ENCODERS = 0x20
SPEED1 = 0X00
SPEED2 = 0X01
MODE_SELECTOR = 0X0F
ACCELERATION = 0X0E
VOLT_READ = 0X0A

bus.write_byte_data(MD25_ADDRESS12, MODE_SELECTOR, 1)
t.sleep(0.05)
bus.write_byte_data(MD25_ADDRESS34, MODE_SELECTOR, 1)
t.sleep(0.05)

opflow.optic_setup()

def set_acceleration(x):
    bus.write_byte_data(MD25_ADDRESS12, ACCELERATION, x)
    t.sleep(0.05)
    bus.write_byte_data(MD25_ADDRESS34, ACCELERATION, x)
    print("acceleration set to %d" %(x))
    return

def get_current():
    c = bus.read_byte_data(MD25_ADDRESS12, 0x0B)
    c = c/10.0
    t.sleep(0.05)
    print("current = %dA" %(c))

def get_volt():
    volt58 = bus.read_byte_data(MD25_ADDRESS12, VOLT_READ)
    volt59 = bus.read_byte_data(MD25_ADDRESS34, VOLT_READ)
    volt58 = volt58/10.0
    volt59 = volt59/10.0
    
    print(volt58, volt59)
    if (volt58 < 11.0 or volt59 < 11.0):
        
        print("CHARGE BATTERY")
    
    return() 

def get_software():
    software58 = bus.read_byte_data(MD25_ADDRESS12, SOFTWARE_REG)
    t.sleep(0.05)
    software59 = bus.read_byte_data(MD25_ADDRESS34, SOFTWARE_REG)
    return(software58, software59)

def stop():
    bus.write_byte_data(MD25_ADDRESS12, SPEED1, 0)
    t.sleep(0.05)
    bus.write_byte_data(MD25_ADDRESS12, SPEED2, 0)
    t.sleep(0.05)
    bus.write_byte_data(MD25_ADDRESS34, SPEED1, 0)
    t.sleep(0.05)
    bus.write_byte_data(MD25_ADDRESS34, SPEED2, 0)
    t.sleep(0.05)
    return

def prediction(xp, yp, Vx, Vy):
    t = 0.25
    
    A = np.array([[1, t, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, t],
                  [0, 0, 0, 1]])
    
    X = np.array([[xp],
                  [Vx],
                  [yp],
                  [Vy]])
    
    X_prime = A.dot(X)
    return X_prime

def covariance4d(sigma1, sigma2, sigma3, sigma4):
    cov1_2 = sigma1 * sigma2
    cov2_1 = sigma2 * sigma1
    cov1_3 = sigma1 * sigma3
    cov3_1 = sigma3 * sigma1
    cov1_4 = sigma1 * sigma4
    cov4_1 = sigma4 * sigma1
    cov2_3 = sigma2 * sigma3
    cov3_2 = sigma3 * sigma2
    cov2_4 = sigma2 * sigma4
    cov4_2 = sigma4 * sigma2
    cov3_4 = sigma3 * sigma4
    cov4_3 = sigma4 * sigma3
    
    cov_matrix = np.array([[sigma1 ** 2, cov1_2, cov1_3, cov1_4],
                           [cov2_1, sigma2 ** 2, cov2_3, cov2_4],
                           [cov3_1, cov3_2, sigma3 ** 2, cov3_4],
                           [cov4_1, cov4_2, cov4_3, sigma4 ** 2]])
    
    return np.diag(np.diag(cov_matrix))

def covariance2d(sigma1, sigma2):
    cov1_2 = sigma1 * sigma2
    cov2_1 = sigma2 * sigma1
    
    cov_matrix = np.array([[sigma1 ** 2, cov1_2],
                           [cov2_1, sigma2 ** 2],
                           ])
    
    return np.diag(np.diag(cov_matrix))






def kalman(P, Q, Xpred, x1, y1):
    error_m_x = 0.1
    error_m_y = 0.1
    t = 0.25
    
    H = np.array([
            [1, 0, 0, 0],
            [0, 0, 1, 0]
            ])
    
    
    A = np.array([
        
            [1, t, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, t],
            [0, 0, 0, 1]

            ])
    
    
    meas = [x1, 0, y1, 0]
     
    #pdb.set_trace()
    #print(A)
    #print(Q)
    #print(P)
    P = np.diag(A.dot(P).dot(A.T)) + Q
    R = covariance2d(error_m_x, error_m_y)
    S = H.dot(P).dot(H.T) + R
    K = P.dot(H.T).dot(inv(S))
    #print(K)
    Z = H.dot(meas).reshape(2, -1)
    #print(Z)
    Xest = Xpred + K.dot(Z - H.dot(Xpred))
    P = (np.identity(len(K)) - K.dot(H)).dot(P)
    #print(P)
    return Xest, P
    



def move(xp, yp, x2, y2):
    opflow.reset()
    xe = x2 - xp
    ye = y2 - yp
    xpe = 0.0
    ype = 0.0
    xes = 0.0
    yes = 0.0
    
    Xpred = []
    #pdb.set_trace()
    
    error_est_x = 1
    error_est_Vx = 1
    error_est_y = 1
    error_est_Vy = 1
    
    count = 0
    
    Q = covariance4d(error_est_x, error_est_Vx, error_est_y, error_est_Vy)
    
    while(abs(xe) >= 5 or abs(ye) >= 5):
        Kp = 0.0010         #pid gains
        Kd = 0.007
        Ki = 0.000000375
        
        x1 = opflow.fetch()[0] + xp     #get opflow measure
        y1 = opflow.fetch()[1] + yp
        xo = x1
        yo = y1
        print("xo: %d  yo: %d" %(xo, yo))

        f = open("opticalflow.txt", "a+")
        f.write("%d,%d \n" %(x1, y1))
        f.close()
        
        #filter
        if (count > 0):
            if (count == 1):
                (Xest, P) = kalman(Q, Q, Xpred, x1, y1)
            else:
                (Xest, P) = kalman(P, Q, Xpred, x1, y1)
                x1 = Xest[0][0]
                y1 = Xest[2][0]
        
        
        f = open("KalmanOP.txt", "a+")
        f.write("%d,%d \n" %(x1, y1))
        f.close()
        
        xe = x2 - x1            #calc errors
        ye = y2 - y1
        print("x1: %d  y1: %d" %(x1, y1))
        print("xe: %d  ye: %d" %(xe, ye))
        
        
        V = Kp * (xe ** 2 + ye ** 2) ** 0.5 + Kd * (xpe ** 2 + ype ** 2) ** 0.5 + Ki * (xes ** 2 + yes ** 2) ** 0.5
        
        
        
        
        #Scale velocities for motors
        #V = 40 * max(min(1, V), 0)
        
        th = np.arctan2((y2 - y1),(x2 - x1))
        print(th)
        Vx = V * np.cos(th)
        Vy = V * np.sin(th)
        print(Vx, Vy)
        V1 = 30* max(min(1, 0.5 * ((Vx/np.cos(np.pi/4)) + (Vy/np.sin(np.pi/4)))), -1)
        V3 = 30* max(min(1, 0.5 * ((Vx/np.cos(np.pi/4)) + (Vy/np.sin(np.pi/4)))), -1)
        V2 = 30* max(min(1, 0.5 * ((-Vx/np.cos(np.pi/4)) + (Vy/np.sin(np.pi/4)))), -1)
        V4 = 30* max(min(1, 0.5 * ((-Vx/np.cos(np.pi/4)) + (Vy/np.sin(np.pi/4)))), -1)
        
        #prediction
        Vp = V
        #* 4.203
        Vxp = Vp * np.cos(th)
        Vyp = Vp * np.sin(th)
        Xpred = prediction(x1, y1, Vxp, Vyp)
        print(Xpred)
        drive(V1, V2, V3, V4)
        
        xpe = xe
        ype = ye
        xes += xe
        yes += ye
        t.sleep(0.05)
        count += 1
        
    stop()
    P=Q
    Vxp = 0
    Vyp = 0
    return x1,y1,

def drive(V1, V2, V3, V4):
    
    bus.write_byte_data(MD25_ADDRESS12, SPEED1, int(V1))
    t.sleep(0.05)
    bus.write_byte_data(MD25_ADDRESS12, SPEED2, int(V3))
    t.sleep(0.05)
    bus.write_byte_data(MD25_ADDRESS34, SPEED1, -int(V2))
    t.sleep(0.05)
    bus.write_byte_data(MD25_ADDRESS34, SPEED2, -int(V4))
    t.sleep(0.05)
    return
