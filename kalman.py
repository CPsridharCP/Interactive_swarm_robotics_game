import numpy as np

def filter(x,P,dt,Z):

	F = np.matrix([[1.0,0.0,dt,0.0],[0.0,1.0,0.0,dt],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,1.0]]) # next state function

	# prediction
	x = (F * x) + u
	P = F*P*F.transpose() + Q

	# measurement update
	y = np.matrix(Z).transpose() - (H*x)
	S = H * P * H.transpose() + R
	K = P * H.transpose() * np.linalg.inv(S)
	x = x + (K * y)
	P = (I - (K * H)) * P

	return x,P

u = np.matrix([[0.], [0.], [0.], [0.]]) # external motion
H = np.matrix([[1.,0.0,0.0,0.0],[0.0,1.0,0.0,0.0]]) # measurement function
R = np.matrix([[0.001,0.0],[0.0,0.001]]) # measurement uncertainty #both were 0.003
Q = np.matrix([[0.001,0.0,0.0,0.0],[0.0,0.001,0.0,0.0],[0.0,0.0,0.001,0.0],[0.0,0.0,0.0,0.001]]) # process noise 0.001
I = np.matrix([[1.0,0.,0.,0.],[.0,1.0,.0,.0],[.0,.0,1.0,.0],[.0,.0,.0,1.0]]) # identity matrix