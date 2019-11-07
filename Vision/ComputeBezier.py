import matplotlib.pyplot as plt
import numpy as np
import bezier


def compute_bezier(p_0, p_1, p_2):
	t = 0.5
	B0 = ((p_0[0] * t) + ((1-t) * p_1[0]), (p_0[1] * t) + ((1-t) * p_1[1]))
	B1 = ((p_1[0] * t) + ((1-t) * p_2[0]), (p_1[1] * t) + ((1-t) * p_2[1]))
	result = (B0[0] * t + (1-t)*B1[0], B0[1] * t + (1-t)*B1[1])
	return result

#Testing
p_0 = (1.3, 4)
p_1 = (2, 3)
p_2 = (1, 3)
plotx = [p_0[0], p_1[0], p_2[0]]
ploty = [p_0[1], p_1[1], p_2[1]]
plt.plot(plotx, ploty)
curve = compute_bezier(p_0, p_1, p_2)
print("P0: ", p_0)
print("P1: ", p_1)
print("P2: ", p_2)
print("curve point: ", curve)
plt.scatter(p_0[0], p_0[1])
plt.show()

#Option2: using bezier library: https://bezier.readthedocs.io/en/0.10.0/python/reference/bezier.curve.html
#test
nodes = np.asfortranarray([
    [0.0, 0.625, 1.0],
    [0.0, 0.5  , 0.5],
])
p_0 = (0, 0)
p_1 = (0.625, 0.5)
p_2 = (1.0, 0.5)
curvePred = compute_bezier(p_0, p_1, p_2)
curveActual = bezier.Curve(nodes, degree=1)
print("My function: ", curvePred)
print("Actual Function: ", curveActual)
