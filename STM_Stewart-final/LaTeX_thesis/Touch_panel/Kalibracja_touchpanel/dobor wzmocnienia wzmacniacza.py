# ma byc wzmocnienie 3.3/5 wiec 6.6/10
# najlepiej zrobic 6.6/10 k
print(6.6/10 * 5)

# dostepne rezystancje
resistorsk = [474, 474, 474, 220, 220, 220, 100, 100, 100, 100, 1000, 1000, 1000, 1000, 470,470]
u = 5
g1 = -1 * (220+100)/474
g2 = -1 * 1000/1000
print(u*g1*g2)

vsource= 4.7
vin = 4.7
v_pin2 = 1.63
vn =1.6

# dodam do kazdego rezystora rownolegle 100 ohm, zeby zmniejszyc R i sprawdzic czy wtedy zadziala
#Rpar = R1*R2/(R1+R2)
#tak wiec
Rf = (320000*100/(320000+100))
Rin = 472000*100/(472000+100)
print (Rf, Rin)


# Ok, proby samego, zwyklego dzielnika napiecia
rezPanel = 1000
# zbadalem rezystancje panelu
# czarny kabel z GND tworzy potencjometr 480-1000 ohm, ale z Vcc 480 ohm
# niebieski z Vcc tworzy tez 480-1000 ohm, ale z GND ma stale 275 ohm
rezMax = 5000
r2=27
r1=51


# trzeba brac pod uwage rzeczywiste wartości
# poszukac jakie mam rezystory rzedu kilkunastu kOhm
gainDiv = r1/(r1+r2)        # 24k i 47k BEST
print('gainDiv', gainDiv)
print(5*gainDiv)

# Zasilanie 5V:
# 5 -> 3.14
# 3 -> 2.18
print('vin = 5, vout = 3.15, gain=:', 3.15/5)
print('vin = 3.3, vout = 2.18, gain=:', 2.18/3.3)

# sprawdzic teraz zasilanie 3V:
# 5 -> 3.15
# 3 -> 2.18
# Czyli mozna zasilac OpAmp przez 3V -> moze to zapewnic nieprzekraczanie 3.3V co ochroni uC


