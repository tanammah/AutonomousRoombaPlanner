import math

def almostEqual(a, b, epsilon=0.001):
	if (abs(a-b) <= epsilon):
		return True
	return False

def euclid(p1, p2):
	x1, y1 = p1
	x2, y2 = p2

	return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)