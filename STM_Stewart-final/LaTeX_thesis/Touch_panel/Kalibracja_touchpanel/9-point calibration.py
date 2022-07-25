"""
Nadal nie dziala jakos super
co zrobic:
- sprawdzic czy zmieniajac odczyty na abs (aby osX i osY rosly w tym samym kierunku)
- napisac na forum
- napisac do sprzedawcy

"""

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


# number of points in algorithm
n = 9

# BEZ ABS W PODAWANYCH WSPOLRZEDNYCH
# touch points (te z ')
pt1 = Point(370, 3264)
pt2 = Point(682, 3295)
pt3 = Point(934, 3366)
pt4 = Point(317, 2707)
pt5 = Point(590, 2799)
pt6 = Point(810, 2888)
pt7 = Point(275, 2356)
pt8 = Point(504, 2439)
pt9 = Point(692, 2561)

touch_list = [pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8, pt9]

# display points (te bez apostrofu)

pd1 = Point(30.75, 121.5)
pd2 = Point(61.5, 121.5)
pd3 = Point(92.25, 121.5)
pd4 = Point(30.75, 81)
pd5 = Point(61.5, 81)
pd6 = Point(92.75, 81)
pd7 = Point(30.75, 40.5)
pd8 = Point(61.5, 40.5)
pd9 = Point(92.75, 40.5)

disp_list = [pd1, pd2, pd3, pd4, pd5, pd6, pd7, pd8, pd9]

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

# calculating for every point
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

print('sprawdzenie pd1 - punktu srodkowego')
calc_pd5_x = pt5.x*alpha_x + pt5.y*beta_x + delta_X
calc_pd5_y = pt5.x*alpha_y + pt5.y*beta_y + delta_Y

print('Podane:',  'Wyliczone:')
print('x:', pd5.x, calc_pd5_x)
print('y', pd5.y, calc_pd5_y)