%Limpieza de pantalla
clear all
close all
clc
%1 TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf=200; 
ts=0.01; 
t=0:ts:tf; 
N= length(t); 
%2 CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Damos valores a nuestro punto inicial de posición y orientación
x1(1)=0; %Posición inicial eje x
y1(1)=6.5; %Posición inicial eje y (Z)
phi(1)=pi/2; %Orientación inicial del robot
%3 POSICION DESEADA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Damos valores a nuestra posición deseada
puntos = [0,6.5; 0,8.5; 3,11.5; 5.5,11.5; 5.5,10.5; 3,8.5; 2,8.5; 2,6.5; 4,8.5; 6,8.5; 8,6.5; 8,10.5; 10,8.5; ... 
          9.5,7.2; 11,7.2; 11,6.2; 9.5,6.2; 9.5,7.2; 11,7.2; ... % Ojo Izquierdo
           12,6.2;11,5.2; 13,5.2;12,6.2; ... % Boca
          13,7.2; 13,6.2; 14.5,6.2; 14.5,7.2; 13,7.2; ... % Ojo Derecho
          10,8.5; ... % Retorno a 
          14,8.5; 16,10.5; 16,5.5; 14.5,4.5; 13,3.5; 11,3.5; 9.5,4.5; 8,5.5; 8,2.2; 9,2.2; 9,0.5; 6,0.5; ... 
          6,4.5; 4,4.5; 3,3.5; 3,2.2; 5,2.2; 5,0.5; 1,0.5; 0,6.5]; 
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
K=[0.6 0;...
0 0.6];
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
axis([-2 18 -1 13 0 1]); 
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