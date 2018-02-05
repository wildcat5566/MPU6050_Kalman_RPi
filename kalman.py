import numpy as np

class Kalman():


    def __init__(self):
        self.P = np.matrix([[0., 0.],
                            [0., 0.]])
    def setKalmanAngle(self, angle):
        self.State = np.matrix([[angle],
                                [0.   ]])
        #print('0. set initial guess: \n', self.State)

    def getKalmanAngle(self, angle, gyro_rate, dt):
        R = 0.03
        Q = np.matrix([[0.001, 0.   ],
                       [0.,    0.003]])
        H = np.matrix( [1.,    0.   ])

        F = np.matrix([[1., -dt],
                       [0., 1. ]])
        B = np.matrix([[dt],
                       [0.]])
        #print('F= \n', F)
        #print('B= \n', B)
        #print(self.State)
        
        #(I). State prediction
        self.State = F * self.State + B * gyro_rate
        #print('I. State prediction: \n', self.State)

        #(II). Covariance prediction
        self.P = F * self.P * np.transpose(F) + Q
        #print('II. Covariance prediction P: \n', self.P)

        #(III). Innovation
        I = angle - H * self.State
        #print('III. Innovation I: \n', I)

        #(IV). Innovation covariance S
        S = H * self.P * np.transpose(H) + R
        #print('IV. Innovation covariance S: \n', S)

        #(V). Kalman gain KG
        KG = self.P * np.transpose(H) / S
        #print('V. Kalman Gain: \n', KG)

        #(VI). Update state
        self.State = self.State + KG * I
        #print('VI. Update State: \n', self.State)

        #(VII). Update covariance
        self.P = (np.eye(2) - KG * H) * self.P
        #print('VII. Update Covariance P: \n', self.P)

        return self.State.item(0)

#X = Kalman()
#X.setKalmanAngle(30)
#x1 = X.getKalmanAngle(30, 2, 0.01)
#print(x1)
