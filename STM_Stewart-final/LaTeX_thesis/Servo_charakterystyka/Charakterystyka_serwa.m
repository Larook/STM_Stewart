ticks_mierz=[505,532,560,690,880,1150,1300,1480,1680,1880,2060,2280,2500,2720];
katy_mierz=[0,15,30,45,60,75,90,105,120,135,150,165,180,195];

katy= 0:195;%prosta zastepcza
a_Pr=13.19; %prostej
b_Pr=115; %prostej
ticks=a_Pr*katy+b_Pr; %prosta



%plot(katy_mierz,ticks_mierz,katy,ticks)
%figure
%title('Relacja uzyskanego wychylenia serwa od zadanego wype³nienia');
plot(ticks_mierz/100,katy_mierz,ticks/100,katy);
title('Relacja uzyskanego wychylenia serwa od zadanego wype³nienia');
xlabel(['wype³nienie D[%]'],'interpreter','latex');
ylabel(['uzyskany k¹t obrotu serwa [deg]'],'interpreter','latex');