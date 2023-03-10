import numpy as np

#a= np.array([14.868,1102,0])
a = [32.868,1133,1]
b=np.array([[0.48,0.87,-1010],
            [-0.87,0.48,-515],
            [0,0,1]])

c = b.dot(a)

print(c)