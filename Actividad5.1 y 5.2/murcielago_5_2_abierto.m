clear
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tf = 260;            % Tiempo de simulacion en segundos (s)
ts = 0.01;           % Tiempo de muestreo en segundos (s)
t = 0: ts: tf;       % Vector de tiempo
N = length(t);       % Muestras

%%%%%%%%%%%%%%%%%%%%%%%% CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%

x1 = zeros (1,N+1);  % Posición en el centro del eje que une las ruedas (eje x) en metros (m)
y1 = zeros (1,N+1);  % Posición en el centro del eje que une las ruedas (eje y) en metros (m)
phi = zeros(1, N+1); % Orientacion del robot en radianes (rad)

x1(1) = -27;  % Posicion inicial eje x
y1(1) = -13;  % Posicion inicial eje y
phi(1) = pi/2;   % Orientacion inicial del robot

%%%%%%%%%%%%%%%%%%%%%%%%%%%% PUNTO DE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%

hx = zeros(1, N+1);  % Posicion en el punto de control (eje x) en metros (m)
hy = zeros(1, N+1);  % Posicion en el punto de control (eje y) en metros (m)

a = 0.15; % Distancia al punto de control

hx(1) = x1(1) + a*cos(phi(1)); % Posicion en el punto de control del robot en el eje x
hy(1) = y1(1) + a*sin(phi(1)); % Posicion en el punto de control del robot en el eje y


%%%%%%%%%%%%%%%%%%%%%% VELOCIDADES DE REFERENCIA %%%%%%%%%%%%%%%%%%%%%%%%%%

u = zeros(1,N); % Velocidad lineal de referencia (m/s)
w = zeros(1,N); % Velocidad angular de referencia (rad/s)

% Puntos a seguir
puntos = [-27,-13; -26,-13; -24,-12; -21,-13; -18,-15; -17,-15; -15,-14; -12,-14; -10,-12; ...
          -9,-11; -8,-11; -7,-11; -5,-14; -3,-16; 0,-14; 6,-15; 8,-18; 9,-15; 11,-13; ... 
          13,-9; 15,-6; 15,-2; 16,1; 17,3; 19,5; 22,6; 23,9; 25,13; 26,16; ... 
          24,15; 22,15; 19,15; 15,15; 13,16; 13,12; ... 
          12,8; 10,4; 8,0; 7,-3; 5,-2; ... 
          2,3; 1,0; 0,0; -1,0; -3,1; -1,-5; -1,-7; -4,-7; -7,-7; -10,-6; -13,-4; ... 
          -15,-2; -19,1; -20,-1; -22,-5; -24,-8; -26,-11; -27,-13];
idx = 1;

%%%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k=1:N 
    
    % Ley de control para el seguimiento
    hxd = puntos(idx,1); hyd = puntos(idx,2);
    he = [hxd - hx(k); hyd - hy(k)];
    if sqrt(he(1)^2 + he(2)^2) < 0.2 && idx < size(puntos,1)
        idx = idx + 1;
    end
    J = [cos(phi(k)) -a*sin(phi(k)); sin(phi(k)) a*cos(phi(k))];
    qpRef = pinv(J)*[0.6 0; 0 0.6]*he;
    u(k) = qpRef(1); 
    w(k) = qpRef(2);
    
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
    hx(k+1)=x1(k+1) + a*cos(phi(k+1)); 
    hy(k+1)=y1(k+1) + a*sin(phi(k+1));

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

view([0 90]); % Orientacion de la figura
axis([-30 30 -20 20 0 2]); % Ingresar limites minimos y maximos en los ejes x y z [minX maxX minY maxY minZ maxZ]

% b) Graficar robots en la posicion inicial
scale = 0.5;
MobileRobot_5;
H1=MobilePlot_4(x1(1),y1(1),phi(1),scale);hold on;

% c) Graficar Trayectorias
H2=plot3(hx(1),hy(1),0,'r','lineWidth',2);
H3=plot3(puntos(:,1),puntos(:,2),zeros(size(puntos,1),1),'b.','markersize',10);

% d) Bucle de simulacion de movimiento del robot

step=15; % pasos para simulacion

for k=1:step:N

    delete(H1);    
    delete(H2);
    
    H1=MobilePlot_4(x1(k),y1(k),phi(k),scale);
    H2=plot3(hx(1:k),hy(1:k),zeros(1,k),'r','lineWidth',2);
    
    drawnow;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Graficas %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph=figure;  % Crear figura (Escena)
set(graph,'position',sizeScreen); % Congigurar tamaño de la figura
subplot(211)
plot(t,u,'b','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('m/s'),legend('u');
subplot(212)
plot(t,w,'r','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[rad/s]'),legend('w');