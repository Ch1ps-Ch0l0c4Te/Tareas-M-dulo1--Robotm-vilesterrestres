clear
close all
clc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TIEMPO
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf = 10; % Tiempo de simulacion
ts = 0.1; % Tiempo de muestreo en segundos (s)
t = 0: ts: tf; % Vector de tiempo
N = length(t); % Muestras
%%%%%%%%%%%%%%%%%%%%%%%% CONDICIONES INICIALES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1 = zeros (1,N+1); y1 = zeros (1,N+1); phi = zeros(1,
N+1);
x1(1) = 0; y1(1) = 0; phi(1) = 30*pi/180;
%%%%%%%%%%%%%%%%%%%%%%%%%%%% PUNTO DE CONTROL
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hx = zeros(1, N+1); hy = zeros(1, N+1);
hx(1) = x1(1); hy(1) = y1(1);
%%%%%%%%%%%%%%%%%%%%%% VELOCIDADES DE REFERENCIA
%%%%%%%%%%%%%%%%%%%%%%%%%%
u = zeros(1,N); w = zeros(1,N);
u(1:10) = 1; % Tramo 1
w(11:20) = ((120*pi)/180); % Giro 1
u(21:30) = 1; % Tramo 2
w(31:40) = -(13*pi/18); % Giro 2
u(41:50) = 1.5; % Tramo 3
w(51:60) = -(pi/2); % Giro 3
u(61:70) = 2.2; % Tramo 4
%%%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE SIMULACION%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:N
phi(k+1)=phi(k)+w(k)*ts;
xp1=u(k)*cos(phi(k+1));
yp1=u(k)*sin(phi(k+1));
x1(k+1)=x1(k) + xp1*ts ;
y1(k+1)=y1(k) + yp1*ts ;
hx(k+1)=x1(k+1);
hy(k+1)=y1(k+1);
end
% d) Bucle de simulacion de movimiento del robot
step=1; % pasos para simulacion
for k=1:step:N
delete(H1);
delete(H2);
H1=MobilePlot_4(x1(k),y1(k),phi(k),scale);
H2=plot3(hx(1:k),hy(1:k),zeros(1,k),'r','lineWidth',2);
pause(ts);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Graficas%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(211)
plot(t,u,'b','LineWidth',2),grid('on'),xlabel('Tiempo[s]'),ylabel('m/s'),legend('u');
subplot(212)
plot(t,w,'r','LineWidth',2),grid('on'),xlabel('Tiempo[s]'),ylabel('[rad/s]'),legend('w');