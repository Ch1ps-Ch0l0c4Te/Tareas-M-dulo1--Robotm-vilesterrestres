%Limpieza de pantalla
clear all
close all
clc
%1 TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf=60; 
ts=0.01; 
t=0:ts:tf; 
N= length(t); 
%2 CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Damos valores a nuestro punto inicial de posición y orientación
x1(1)=-27; %Posición inicial eje x
y1(1)=-13; %Posición inicial eje y (Z)
phi(1)=pi/2; %Orientación inicial del robot
%3 POSICION DESEADA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Damos valores a nuestra posición deseada
puntos = [-27,-13; -26,-13; -24,-12; -21,-13; -18,-15; -17,-15; -15,-14; -12,-14; -10,-12; ...
          -9,-11; -8,-11; -7,-11; -5,-14; -3,-16; 0,-14; 6,-15; 8,-18; 9,-15; 11,-13; ... 
          13,-9; 15,-6; 15,-2; 16,1; 17,3; 19,5; 22,6; 23,9; 25,13; 26,16; ... 
          24,15; 22,15; 19,15; 15,15; 13,16; 13,12; ... 
          12,8; 10,4; 8,0; 7,-3; 5,-2; ... 
          2,3; 1,0; 0,0; -1,0; -3,1; -1,-5; -1,-7; -4,-7; -7,-7; -10,-6; -13,-4; ... 
          -15,-2; -19,1; -20,-1; -22,-5; -24,-8; -26,-11; -27,-13]; 
a = 0.15;
idx = 1;
hxd=puntos(idx,1);
hyd=puntos(idx,2);
hx(1)= x1(1) + a*cos(phi(1)); 
hy(1)= y1(1) + a*sin(phi(1)); 
%4 CONTROL, BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:N
%a)Errores de control
hxe(k)=hxd-hx(k);
hye(k)=hyd-hy(k);
%Matriz de error
he= [hxe(k);hye(k)];
%Magnitud del error de posición
Error(k)= sqrt(hxe(k)^2 +hye(k)^2);
%b)Matriz Jacobiana
J=[cos(phi(k)) -a*sin(phi(k));... 
sin(phi(k)) a*cos(phi(k))];
%c)Matriz de Ganancias
K=[3 0;...
0 3];
%d)Ley de Control
qpRef= pinv(J)*K*he;
v(k)= qpRef(1); 
w(k)= qpRef(2); 
%5 APLICACIÓN DE CONTROL AL ROBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Aplico la integral a la velocidad angular para obtener el angulo "phi" de la orientación
phi(k+1)=phi(k)+w(k)*ts; 
%%%%%%%%%%%%%%%%%%%%% MODELO CINEMATICO %%%%%%%%%%%%%%%%%%%%%%%%%
xp1=v(k)*cos(phi(k));
yp1=v(k)*sin(phi(k));
%Aplico la integral a la velocidad lineal para obtener las cordenadas
%"x1" y "y1" de la posición
x1(k+1)=x1(k)+ xp1*ts; 
y1(k+1)=y1(k)+ yp1*ts; 
% Posicion del robot con respecto al punto de control
hx(k+1)=x1(k+1) + a*cos(phi(k+1));
hy(k+1)=y1(k+1) + a*sin(phi(k+1));
if Error(k) < 0.2 && idx < size(puntos,1)
    idx = idx + 1;
    hxd = puntos(idx,1);
    hyd = puntos(idx,2);
end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION VIRTUAL 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% a) Configuracion de escena
scene=figure; 
set(scene,'Color','white'); 
set(gca,'FontWeight','bold') ;
sizeScreen=get(0,'ScreenSize'); 
set(scene,'position',sizeScreen); 
camlight('headlight'); 
axis equal; 
grid on; 
box on; 
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); 
view([0 90]); 
axis([-30 30 -20 20 0 1]); 
% b) Graficar robots en la posicion inicial
scale = 0.5;
MobileRobot_5;
H1=MobilePlot_4(x1(1),y1(1),phi(1),scale);hold on;
% c) Graficar Trayectorias
H2=plot3(hx(1),hy(1),0,'r','lineWidth',2);
H3=plot3(puntos(:,1),puntos(:,2),zeros(size(puntos,1),1),'b.','markersize',10); 
H4=plot3(hx(1),hy(1),0,'go','lineWidth',2);
% d) Bucle de simulacion de movimiento del robot
step=15; 
for k=1:step:N
delete(H1);
delete(H2);
H1=MobilePlot_4(x1(k),y1(k),phi(k),scale);
H2=plot3(hx(1:k),hy(1:k),zeros(1,k),'r','lineWidth',2);
drawnow;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Graficas %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph=figure; 
set(graph,'position',sizeScreen); 
subplot(311)
plot(t,v,'b','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('m/s'),legend('Velocidad Lineal (v)');
subplot(312)
plot(t,w,'g','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[rad/s]'),legend('Velocidad Angular (w)');
subplot(313)
plot(t,Error,'r','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[metros]'),legend('Error de posición (m)');