"""
Nadal nie dziala jakos super
co zrobic:
- sprawdzic czy zmieniajac odczyty na abs (aby osX i osY rosly w tym samym kierunku)
- napisac na forum
- napisac do sprzedawcy


zbadalem rezystancje panelu
czarny kabel z GND tworzy potencjometr 480-1000 ohm, ale z Vcc 480 ohm
niebieski z Vcc tworzy tez 480-1000 ohm, ale z GND ma stale 275 ohm

"""

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def get_display_x(self):
        return self.x*alpha_x + self.y*beta_x + delta_X


    def get_display_y(self):
        return self.x*alpha_y + self.y*beta_y + delta_Y
# number of points in algorithm
n = 13

# BEZ ABS W PODAWANYCH WSPOLRZEDNYCH
# touch points (te z ')
pt1 = Point(370, 3264)
pt2 = Point(515, 3290)
pt3 = Point(680, 3313)
pt4 = Point(800, 3341)
pt5 = Point(930, 3370)

pt6 = Point(345, 2955)
pt7 = Point(630, 3020)
pt8 = Point(853, 3109)

pt9 = Point(292, 2548)
pt10 = Point(317, 2738)
pt11 = Point(454, 2781)
pt12 = Point(587, 2800)
pt13 = Point(707, 2844)
pt14 = Point(775, 2932)
pt15 = Point(880, 2940)

pt16 = Point(292, 2531)
pt17 = Point(559, 2545)
pt18 = Point(749, 2722)

pt19 = Point(278, 2334)
pt20 = Point(389, 2380)
pt21 = Point(509, 2418)
pt22 = Point(600, 2473)
pt23 = Point(699, 2553)

touch_list = [pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8, pt9, pt10, pt11, pt12, pt13, pt14, pt15, pt16, pt17, pt18, pt19, pt20, pt21, pt22, pt23]

# display points (te bez apostrofu)
pd1 = Point(29, 123)
pd2 = Point(29+14.5, 123)
pd3 = Point(59, 123)
pd4 = Point(59+14.5, 123)
pd5 = Point(89, 123)

pd6 = Point(29+14.5, 82.5+19)
pd7 = Point(59, 82.5+19)
pd8 = Point(59+14.5, 82.5+19)

pd9 = Point(29-14.5, 82.5)
pd10 = Point(29, 82.5)
pd11 = Point(29+14.5, 82.5)
pd12 = Point(58, 82.5)
pd13 = Point(58+14.5, 82.5)
pd14 = Point(89, 82.5)
pd15 = Point(89+14.5, 82.5)

pd16 = Point(29+14.5, 82.5-19)
pd17 = Point(59, 82.5-19)
pd18 = Point(59+14.5, 82.5-19)

pd19 = Point(29, 41)
pd20 = Point(29+14.5, 41)
pd21 = Point(59, 41)
pd22 = Point(59+14.5, 41)
pd23 = Point(89, 41)

disp_list = [pd1, pd2, pd3, pd4, pd5, pd6, pd7, pd8, pd9, pd10, pd11, pd12, pd13, pd14, pd15, pd16, pd17, pd18, pd19, pd20, pd21, pd22, pd23]

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


# def calc_display_x(Point p):
#     return x*alpha_x + y*beta_x + delta_X
#
#
# def calc_display_y(x, y):
#     return x*alpha_y + y*beta_y + delta_Y


print('sprawdzenie pd1 - punktu srodkowego')
calc_pd12_x = pt12.get_display_x()
calc_pd12_y = pt12.get_display_y()

print('Podane:',  'Wyliczone:')
for i in range(len(touch_list)):
    print( i, 'x:', disp_list[i].x, touch_list[i].get_display_x(), 'deltaX:', disp_list[i].x - touch_list[i].get_display_x())
    print( i, 'y', disp_list[i].y, touch_list[i].get_display_y(), 'deltaY:', disp_list[i].y - touch_list[i].get_display_y())
    print()