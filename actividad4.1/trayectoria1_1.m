clear
close all
clc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf = 5.5;             
ts = 0.00001;          
t = 0: ts: tf;       
N = length(t);       
%%%%%%%%%%%%%%%%%%%%%%%% CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1 = zeros(1,N+1);  
y1 = zeros(1,N+1);  
phi = zeros(1,N+1); 
x1(1) = 0;              
y1(1) = 0;              
phi(1) = 0;             
%%%%%%%%%%%%%%%%%%%%%%%%%%%% PUNTO DE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hx = zeros(1,N+1);  
hy = zeros(1,N+1);  
hx(1) = x1(1);
hy(1) = y1(1);
%%%%%%%%%%%%%%%%%%%%%% VELOCIDADES DE REFERENCIA %%%%%%%%%%%%%%%%%%%%%%%%%%
u = zeros(1,N); 
w = zeros(1,N); 
for k = 1:N
    
    tk = t(k);   
        dx = 1;
        dy = 4*tk*cos(tk^2);
        
        u(k) = sqrt(dx^2 + dy^2);
        w(k) = (4*cos(tk^2)-(8*(tk^2)*(sin(tk^2)))) / (dx^2 + dy^2);
       
    
end
%%%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:N 
    
    phi(k+1)=phi(k)+w(k)*ts;
    
    xp1=u(k)*cos(phi(k+1)); 
    yp1=u(k)*sin(phi(k+1));
    x1(k+1)=x1(k) + xp1*ts; 
    y1(k+1)=y1(k) + yp1*ts; 
    
    hx(k+1)=x1(k+1); 
    hy(k+1)=y1(k+1);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
scene=figure;
set(scene,'Color','white');
set(gca,'FontWeight','bold');
sizeScreen=get(0,'ScreenSize');
set(scene,'position',sizeScreen);
axis equal;
grid on;
box on;
xlabel('x(m)'); ylabel('y(m)'); 
axis([-1 7 -5 5]); 
scale = 1.5;             
MobileRobot_5;
H1=MobilePlot_4(x1(1),y1(1),phi(1),scale);hold on;
H2=plot(hx(1),hy(1),'r','lineWidth',2);
step=floor(0.011 / ts); 
for k=1:step:N
    delete(H1);    
    delete(H2);
    
    H1=MobilePlot_4(x1(k),y1(k),phi(k),scale);
    H2=plot(hx(1:k),hy(1:k),'r','lineWidth',2);
    
    pause(0.011); 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GRAFICAS %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph=figure;
set(graph,'position',sizeScreen);
subplot(211)
plot(t,u,'b','LineWidth',2),grid on
xlabel('Tiempo [s]'),ylabel('m/s'),legend('u')
subplot(212)
plot(t,w,'r','LineWidth',2),grid on
xlabel('Tiempo [s]'),ylabel('[rad/s]'),legend('w')
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