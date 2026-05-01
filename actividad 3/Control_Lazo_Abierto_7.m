clear
close all
clc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf = 24;             % Tiempo de simulacion
ts = 0.1;            % Tiempo de muestreo en segundos (s)
t = 0: ts: tf;       % Vector de tiempo
N = length(t);       % Muestras
%%%%%%%%%%%%%%%%%%%%%%%% CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1 = zeros (1,N+1);  % Posición en el centro del eje que une las ruedas (eje x) en metros (m)
y1 = zeros (1,N+1);  % Posición en el centro del eje que une las ruedas (eje y) en metros (m)
phi = zeros(1, N+1); % Orientacion del robot en radianes (rad)
x1(1) = 0;    % Posicion inicial eje x
y1(1) = 0;   % Posicion inicial eje y
phi(1) = pi/2;   % Orientacion inicial
%%%%%%%%%%%%%%%%%%%%%%%%%%%% PUNTO DE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%
hx = zeros(1, N+1);  % Posicion en el punto de control (eje x) en metros (m)
hy = zeros(1, N+1);  % Posicion en el punto de control (eje y) en metros (m)
hx(1) = x1(1); % Posicion en el punto de control del robot en el eje x
hy(1) = y1(1); % Posicion en el punto de control del robot en el eje y
%%%%%%%%%%%%%%%%%%%%%% VELOCIDADES DE REFERENCIA %%%%%%%%%%%%%%%%%%%%%%%%%%
u = zeros(1,N); % Inicializamos vector de velocidad lineal
w = zeros(1,N); % Inicializamos vector de velocidad angular

% Tramo 1
u(1:50) = 1;
% Giro 1
w(51:60) = -(pi/2);
% Tramo 2
u(61:110) = 1;
% Giro 2
w(111:120) = -(pi/2);
% Tramo 3
u(121:170) = 1;
% Giro 3
w(171:180) = -(pi/2);
% Tramo 4
u(181:230) = 1;
% Giro 4
w(231:240) = -(pi/2);

%%%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:N
  
   phi(k+1)=phi(k)+w(k)*ts; % Integral numérica (método de Euler)
  
   %%%%%%%%%%%%%%%%%%%%% MODELO CINEMATICO %%%%%%%%%%%%%%%%%%%%%%%%%
   %Aplicamos el modelo cinemático diferencial para obtener las
   %velocidades en x, y, phi
   xp1=u(k)*cos(phi(k+1));
   yp1=u(k)*sin(phi(k+1));
   phip= w(k);
   x1(k+1)=x1(k) + xp1*ts ; % Integral numérica (método de Euler)
   y1(k+1)=y1(k) + yp1*ts ; % Integral numérica (método de Euler)
  
  
   % Posicion del robot con respecto al punto de control
   hx(k+1)=x1(k+1);
   hy(k+1)=y1(k+1);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION VIRTUAL 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% a) Configuracion de escena
scene=figure;  % Crear figura (Escena)
set(scene,'Color','white'); % Color del fondo de la escena
set(gca,'FontWeight','bold') ;% Negrilla en los ejes y etiquetas
sizeScreen=get(0,'ScreenSize'); % Retorna el tamaño de la pantalla del computador
set(scene,'position',sizeScreen); % Congigurar tamaño de la figura
camlight('headlight'); % Luz para la escena
axis equal; % Establece la relación de aspecto para que las unidades de datos sean las mismas en todas las direcciones.
grid on; % Mostrar líneas de cuadrícula en los ejes
box on; % Mostrar contorno de ejes
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); % Etiqueta de los eje
view([25 25]); % Orientacion de la figura
axis([-2 7 -2 7 0 2]); 
% b) Graficar robots en la posicion inicial
scale = 4;
MobileRobot_5;
H1=MobilePlot_4(x1(1),y1(1),phi(1),scale);hold on;
% c) Graficar Trayectorias
H2=plot3(hx(1),hy(1),0,'r','lineWidth',2);
% d) Bucle de simulacion de movimiento del robot
step=1; % pasos para simulacion
for k=1:step:N
   delete(H1);   
   delete(H2);
  
   H1=MobilePlot_4(x1(k),y1(k),phi(k),scale);
   H2=plot3(hx(1:k),hy(1:k),zeros(1,k),'r','lineWidth',2);
  
   pause(ts);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Graficas %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph=figure;  % Crear figura (Escena)
set(graph,'position',sizeScreen); % Congigurar tamaño de la figura
subplot(211)
plot(t,u,'b','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('m/s'),legend('u');
subplot(212)
plot(t,w,'r','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[rad/s]'),legend('w');
%grafica de x
figure; 
subplot(311)
plot(t, x1(1:N), 'g', 'LineWidth', 2), grid('on'), xlabel('Tiempo [s]'), ylabel('x [m]'), legend('x');

%grafica de y
subplot(312)
plot(t, y1(1:N), 'm', 'LineWidth', 2), grid('on'), xlabel('Tiempo [s]'), ylabel('y [m]'), legend('y');

%grafica de phi
subplot(313)
plot(t, phi(1:N), 'k', 'LineWidth', 2), grid('on'), xlabel('Tiempo [s]'), ylabel('\phi [rad]'), legend('\phi');

%Calculo de velocidades de ruedas 
r = 0.05; 
L = 0.17; 
wl = (u - (w*L)/2)/r;
wr = (u + (w*L)/2)/r;

figure; 
%grafica de wl
subplot(211)
plot(t, wl, 'b', 'LineWidth', 2), grid('on'), xlabel('Tiempo [s]'), ylabel('rad/s'), legend('wl');

%grafica de wr
subplot(212)
plot(t, wr, 'r', 'LineWidth', 2), grid('on'), xlabel('Tiempo [s]'), ylabel('rad/s'), legend('wr');