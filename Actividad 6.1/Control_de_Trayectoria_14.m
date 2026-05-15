%Limpieza de pantalla
clear all
close all
clc


%1 TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf=20;             % Tiempo de simulación en segundos (s)
ts=0.01;            % Tiempo de muestreo en segundos (s)
t=0:ts:tf;         % Vector de tiempo
N= length(t);      % Muestras




%2 CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Damos valores a nuestro punto inicial de posición y orientación
x1(1)=0;  %Posición inicial eje x
y1(1)=0;  %Posición inicial eje y
phi(1)=0; %Orientación inicial del robot 


%Igualamos el punto de control con las proyecciones X1 y Y1 por su
%coincidencia
hx(1)= x1(1);       % Posición del punto de control en el eje (X) metros (m)
hy(1)= y1(1);       % Posición del punto de control en el eje (Y) metros (m)


%3 TRAYECTORIA DESEADA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Variable para seleccionar trayectoria (a, b, c, d, e, f, g, h)
opcion = 'k';

switch opcion
    case 'a'%ganancia 10 tiempo 30 0.01 ts
        %a) x =2cos(0.2*t), y = 2sen(0.4*t)
        hxd=2*cos(0.2*t);
        hyd=2*sin(0.4*t);
        hxdp=-2*0.2*sin(0.2*t);
        hydp=2*0.4*cos(0.4*t);
    case 'b'%ganancia 10 tiempo 30 0.01ts
        %b) x =t−3sen(t), y = 4−3cos(t)
        hxd=t-3*sin(t);
        hyd=4-3*cos(t);
        hxdp=1-3*cos(t);
        hydp=3*sin(t);
    case 'c'%ganancia 20 tiempo 30 0.01 ts
        %c) x =3cos(t)−cos(3t), y = 4sin(3t)
        hxd=3*cos(t)-cos(3*t);
        hyd=4*sin(3*t);
        hxdp=-3*sin(t)+3*sin(3*t);
        hydp=12*cos(3*t);
    case 'd'%ganancia 20 ts de 0.01 tiempo 30
        %d) x = cos(t) + 1/2cos(7t) + 1/3sen(17t), y = sen(t) + 1/2sen(7t) + 1/3cos(17t)
        hxd=cos(t) + 0.5*cos(7*t) + (1/3)*sin(17*t);
        hyd=sin(t) + 0.5*sin(7*t) + (1/3)*cos(17*t);
        hxdp=-sin(t) - 3.5*sin(7*t) + (17/3)*cos(17*t);
        hydp=cos(t) + 3.5*cos(7*t) - (17/3)*sin(17*t);
    case 'e'%ganancia 10 tiempo 30 0.001ts
        %e) x =17cos(t)+7cos(17+7t), y = 17sen(t) −7sen(17+7t)
        hxd=17*cos(t)+7*cos(17+7*t);
        hyd=17*sin(t)-7*sin(17+7*t);
        hxdp=-17*sin(t)-49*sin(17+7*t);
        hydp=17*cos(t)-49*cos(17+7*t);
    case 'f'%ganancia 20 tiempo 30 0.001ts
        %f) x =2cos(t), y = 2sen(t)
        hxd=2*cos(t);
        hyd=2*sin(t);
        hxdp=-2*sin(t);
        hydp=2*cos(t);
    case 'g'%ganancia de 10 tiempo de 30 0.01 ts
        %g) x =5t−4sen(t), y = 5t−4cos(t)
        hxd=5*t-4*sin(t);
        hyd=5*t-4*cos(t);
        hxdp=5-4*cos(t);
        hydp=5+4*sin(t);
    case 'h' %20 ganancia, 30 tiempo 0.01ts
        %h) x =4cos(t)+cos(4t), y = 4sen(t) −sen(4t) 
        hxd=4*cos(t)+cos(4*t);
        hyd=4*sin(t)-sin(4*t);
        hxdp=-4*sin(t)-4*sin(4*t);
        hydp=4*cos(t)-4*cos(4*t);
    case 'i'
        %i) Hexágono 0.5 ganancia 0.05ts 30 tiempo
        a = mod(t, pi/3) - pi/6;
        r = cos(pi/6) ./ cos(a);
        dr = cos(pi/6) .* sin(a) ./ (cos(a).^2);
        hxd = r .* cos(t);
        hyd = r .* sin(t);
        hxdp = dr .* cos(t) - r .* sin(t);
        hydp = dr .* sin(t) + r .* cos(t);
    case 'j'
        %j) Flor de 9 pétalos ganancia 20 ts de 0.001 tiempo 30
        hxd = 50*sin(9*t).*cos(t);
        hyd = 50*sin(9*t).*sin(t);
        hxdp = 50*(9*cos(9*t).*cos(t) - sin(9*t).*sin(t));
        hydp = 50*(9*cos(9*t).*sin(t) + sin(9*t).*cos(t));
    case 'k'
        %k) Corazón ganancia 20 ts de 0.01 tiempo 30
        hxd = 16*sin(t).^3;
        hyd = 13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t);
        hxdp = 48*sin(t).^2 .* cos(t);
        hydp = -13*sin(t) + 10*sin(2*t) + 6*sin(3*t) + 4*sin(4*t);
end
%4 CONTROL, BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:N 
    %a)Errores de control (Aqui la posición deseada ya no es constante,
    % varia con el tiempo)
    hxe(k)=hxd(k)-hx(k);
    hye(k)=hyd(k)-hy(k);  
    %Matriz de error
    he= [hxe(k);hye(k)];
    %Magnitud del error de posición
    Error(k)= sqrt(hxe(k)^2 +hye(k)^2);
    %b)Matriz Jacobiana
    J=[cos(phi(k)) -sin(phi(k));... %Matriz de rotación en 2D
       sin(phi(k)) cos(phi(k))];
    %c)Matriz de Ganancias
    K=[20 0;...
       0 20];  
    %d)Velocidades deseadas
    hdp=[hxdp(k);hydp(k)];
    %e)Ley de Control:Agregamos las velocidades deseadas
    qpRef= pinv(J)*(hdp + K*he);
    v(k)= qpRef(1);   %Velocidad lineal de entrada al robot 
    w(k)= qpRef(2);   %Velocidad angular de entrada al robot 
%5 APLICACIÓN DE CONTROL AL ROBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Aplico la integral a la velocidad angular para obtener el angulo "phi" de la orientación
    phi(k+1)=phi(k)+w(k)*ts; % Integral numérica (método de Euler)          
   %%%%%%%%%%%%%%%%%%%%% MODELO CINEMATICO %%%%%%%%%%%%%%%%%%%%%%%%%   
    xp1=v(k)*cos(phi(k)); 
    yp1=v(k)*sin(phi(k));
    %Aplico la integral a la velocidad lineal para obtener las cordenadas
    %"x1" y "y1" de la posición
    x1(k+1)=x1(k)+ ts*xp1; % Integral numérica (método de Euler)
    y1(k+1)=y1(k)+ ts*yp1; % Integral numérica (método de Euler)
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
set(scene,'position',sizeScreen); % Configurar tamaño de la figura
camlight('headlight'); % Luz para la escena
axis equal; % Establece la relación de aspecto para que las unidades de datos sean las mismas en todas las direcciones.
grid on; % Mostrar líneas de cuadrícula en los ejes
box on; % Mostrar contorno de ejes
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); % Etiqueta de los eje
view([-0.1 90]); % Orientacion de la figura
% Ajuste dinámico de ejes según la trayectoria seleccionada
axis([min(hxd)-2 max(hxd)+2 min(hyd)-2 max(hyd)+2 0 1]); 
% b) Graficar robots en la posicion inicial
scale = 1;
MobileRobot_5;
H1=MobilePlot_4(x1(1),y1(1),phi(1),scale);hold on;
% c) Graficar Trayectorias
H2=plot3(hx(1),hy(1),0,'r','lineWidth',2);
H3=plot3(hxd,hyd,zeros(1,N),'g','lineWidth',2); %Grafico circulo en posición deseada
%H4=plot3(hx(1),hy(1),0,'go','lineWidth',2);%Grafico circulo en posición inicial
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
subplot(311)
plot(t,v,'b','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('m/s'),legend('Velocidad Lineal (v)');
subplot(312)
plot(t,w,'g','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[rad/s]'),legend('Velocidad Angular (w)');
subplot(313)
plot(t,Error,'r','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[metros]'),legend('Error de posición (m)');
