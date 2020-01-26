# Majac zmierzone wspolrzedne
# oblicz Macierz M
# dzieki ktorej mozna zmapowac wartosci na rzeczywisty pomiar

# zmierzone 3 punkty: p0,p1,p2
# Trzeba je zmapowac do rzezcywistych 3 punktow pd0,pd1,pd2


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


def calc_display_x(x, y):
    return A*x + B*y + C


def calc_display_y(x, y):
    return D*x + E*y + F


# punkty dotykowe
p0 = Point(330, 3620)
p1 = Point(870, 2925)
p2 = Point(455, 2320)

# docelowe punkty rzeczywiste [mm]
pd0 = Point(20, 140)
pd1 = Point(104, 80)
pd2 = Point(62, 20)

K = (p0.x-p2.x)*(p1.y-p2.y) - (p1.x - p2.x)*(p0.y - p2.y)
A = ((pd0.x-pd2.x)*(p1.y-p2.y) -(pd1.x-pd2.x)*(p0.y-p2.y))/K
B = ((p0.x-p2.x)*(pd1.x-pd2.x)-(pd0.x-pd2.x)*(p1.x-p2.x))/K
C = (p0.y*(p2.x*pd1.x-p1.x*pd2.x) + p1.y*(p0.x*pd2.x-p2.x*pd0.x) + p2.y*(p1.x*pd0.x-p0.x*pd1.x))/K
D = ((pd0.y-pd2.y)*(p1.y-p2.y) - (pd1.y - pd2.y)*(p0.y-p2.y))/K
E = ((p0.x-p2.x)*(pd1.y-pd2.y)-(pd0.y-pd2.y)*(p1.x-p2.x))/K
F = (p0.y*(p2.x*pd1.y-p1.x*pd2.y) + p1.y*(p0.x*pd2.y - p2.x*pd0.y) + p2.y*(p1.x*pd0.y-p0.x*pd1.y))/K

print('K', K)
print('A', A)
print('B', B)
print('C', C)
print('D', D)
print('E', E)
print('F', F)

# Wyniki
print('sprawdzenie pd0')
xd0 = calc_display_x(p0.x, p0.y)
yd0 = calc_display_y(p0.x, p0.y)
print(xd0, pd0.x)
print(yd0, pd0.y)

print('sprawdzenie pd1')
xd1 = calc_display_x(p1.x, p1.y)
yd1 = calc_display_y(p1.x, p1.y)
print(xd1, pd1.x)
print(yd1, pd1.y)

print('sprawdzenie pd2')
xd2 = calc_display_x(p2.x, p2.y)
yd2 = calc_display_y(p2.x, p2.y)
print(xd2, pd2.x)
print(yd2, pd2.y)

# teraz majac takie rzutowanie mozemy sprawdzic, czy rzeczywiscie dobrze mapuje wspolrzedne
# zrobie pomiar na srodku i oblicze jego wspolrzedne "mechaniczne"
# 620,2800
p_srodek = Point(620, 2800)
p_srodek_d_x = calc_display_x(p_srodek.x, p_srodek.y)
p_srodek_d_y = calc_display_y(p_srodek.x, p_srodek.y)
print('obliczono wsp',p_srodek_d_x, 'X', p_srodek_d_y)

# Wniosek - troche ma to racji, moze trzeba zrobic dokladniejsze pomiary pktow sprawdzajacych
print ('-------------nowe podejscie---------------')
# teraz punkty oddzielone nie o 20mm, podzielony jest ekran na 4 czesci
# pomiary suwmiarka - Xmax = 123, Ymax = 162
p0new = Point(312, 2739)
p1new = Point(932, 3365)
p2new = Point(695, 2252)

pd0new = Point(30.75, 81)
pd1new = Point(92.25, 121.5)
pd2new = Point(92.75, 40.5)

Knew = (p0new.x - p2new.x) * (p1new.y - p2new.y) - (p1new.x - p2new.x) * (p0new.y - p2new.y)
Anew = ((pd0new.x - pd2new.x) * (p1new.y - p2new.y) - (pd1new.x - pd2new.x) * (p0new.y - p2new.y)) / Knew
Bnew = ((p0new.x - p2new.x) * (pd1new.x - pd2new.x) - (pd0new.x - pd2new.x) * (p1new.x - p2new.x)) / Knew
Cnew = (p0new.y * (p2new.x * pd1new.x - p1new.x * pd2new.x) + p1new.y * (p0new.x * pd2new.x - p2new.x * pd0new.x) + p2new.y * (p1new.x * pd0new.x - p0new.x * pd1new.x)) / Knew
Dnew = ((pd0new.y - pd2new.y) * (p1new.y - p2new.y) - (pd1new.y - pd2new.y) * (p0new.y - p2new.y)) / Knew
Enew = ((p0new.x - p2new.x) * (pd1new.y - pd2new.y) - (pd0new.y - pd2new.y) * (p1new.x - p2new.x)) / Knew
Fnew = (p0new.y * (p2new.x * pd1new.y - p1new.x * pd2new.y) + p1new.y * (p0new.x * pd2new.y - p2new.x * pd0new.y) + p2new.y * (p1new.x * pd0new.y - p0new.x * pd1new.y)) / Knew

def calc_display_x_new(x, y):
    return Anew * x + Bnew * y + Cnew


def calc_display_y_new(x, y):
    return Dnew * x + Enew * y + Fnew

# Wyniki
print('sprawdzenie pd0new')
xd0new = calc_display_x_new(p0new.x, p0new.y)
yd0new = calc_display_y_new(p0new.x, p0new.y)
print(xd0new, pd0new.x)
print(yd0new, pd0new.y)

print('sprawdzenie pd1new')
xd1new = calc_display_x_new(p1new.x, p1new.y)
yd1new = calc_display_y_new(p1new.x, p1new.y)
print(xd1new, pd1new.x)
print(yd1new, pd1new.y)

print('sprawdzenie pd2new')
xd2new = calc_display_x_new(p2new.x, p2new.y)
yd2new = calc_display_y_new(p2new.x, p2new.y)
print(xd2new, pd2new.x)
print(yd2new, pd2new.y)

p_srodek_new = Point(590, 2900)  # powinno byc 590x2800 - to wtedy idealnie by sie zgadzalo
p_srodek_d_new_x = calc_display_x_new(p_srodek_new.x, p_srodek_new.y)
p_srodek_d_new_y = calc_display_y_new(p_srodek_new.x, p_srodek_new.y)
print('obliczono wsp',p_srodek_d_new_x, 'X', p_srodek_d_new_y)
# powinno byc jak najblizej 61x81
# sprawdzic inne algorytmy kalibracji itp


