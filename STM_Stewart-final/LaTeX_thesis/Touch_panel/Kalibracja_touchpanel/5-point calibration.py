class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def calc_display_x(self, A, B, C):
        return A * self.x + B * self.y + C

    def calc_display_y(self, D, E, F):
        return D * self.x + E * self.y + F

# number of points in algorithm
n = 5

# touch points (te z ')
pt1 = Point(622, 401)
pt2 = Point(647, 3670)
pt3 = Point(3498, 3686)
pt4 = Point(3512, 375)
pt5 = Point(2040, 2006)
touch_list = [pt1, pt2, pt3, pt4, pt5]

# display points (te bez apostrofu)
pd1 = Point(-7100, -5000)
pd2 = Point(7100, -5000)
pd3 = Point(7100, 5000)
pd4 = Point(-7100, 5000)
pd5 = Point(0, 0)
disp_list = [pd1, pd2, pd3, pd4, pd5]

# needed to calculate with Cramer's rule
a = 0
b = 0
c = 0
d = 0
e = 0

# Matrixes
X1 = 0
X2 = 0
X3 = 0
Y1 = 0
Y2 = 0
Y3 = 0

#calculating for every point
for i in range(len(touch_list)):
    # print(touch_list[i].x, touch_list[i].y, disp_list[i].x, disp_list[i].y)
    a += touch_list[i].x * touch_list[i].x
    b += touch_list[i].y * touch_list[i].y
    c += touch_list[i].x * touch_list[i].y
    d += touch_list[i].x
    e += touch_list[i].y

    X1 += touch_list[i].x * disp_list[i].x
    X2 += touch_list[i].y * disp_list[i].x
    X3 += disp_list[i].x

    Y1 += touch_list[i].x * disp_list[i].y
    Y2 += touch_list[i].y * disp_list[i].y
    Y3 += disp_list[i].y


delta = n * (a * b - c * c) + 2 * c * d * e - a * e * e - b * d * d

delta_x1 = n * (X1 * b - X2 * c) + e * (X2 * d - X1 * e) + X3 * (c * e - b * d)
delta_x2 = n * (X2 * a - X1 * c) + d * (X1 * e - X2 * d) + X3 * (c * d - a * e)
delta_x3 = X3 * (a * b - c * c) + X1 * (c * e - b * d) + X2 * (c * d - a * e)

delta_y1 = n * (Y1 * b - Y2 * c) + e * (Y2 * d - Y1 * e) + Y3 * (c * e - b * d)
delta_y2 = n * (Y2 * a - Y1 * c) + d * (Y1 * e - Y2 * d) + Y3 * (c * d - a * e)
delta_y3 = Y3 * (a * b - c * c) + Y1 * (c * e - b * d) + Y2 * (c * d - a * e)

alpha_x = delta_x1/delta
beta_x = delta_x2/delta
delta_X = delta_x3/delta

alpha_y = delta_y1/delta
beta_y = delta_y2/delta
delta_Y = delta_y3/delta

print('alpha_x', alpha_x)
print('beta_x', beta_x)
print('delta_X', delta_X)

print('alpha_y', alpha_y)
print('beta_y', beta_y)
print('delta_Y', delta_Y)



print('sprawdzenie pd1 - punktu srodkowego')
calc_pd1_x = pt1.x*alpha_x + pt1.y*beta_x + delta_X
calc_pd1_y = pt1.x*alpha_y + pt1.y*beta_y + delta_Y

print('Podane:',  'Wyliczone:')
print('x:', pd1.x, calc_pd1_x)
print('y', pd1.y, calc_pd1_y)