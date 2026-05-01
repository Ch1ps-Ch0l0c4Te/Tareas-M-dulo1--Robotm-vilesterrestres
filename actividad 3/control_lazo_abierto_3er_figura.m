clear 
close all
clc
tf = 13.9;
ts = 0.1;
t = 0: ts: tf;
N = length(t);
x1 = zeros (1,N+1); y1 = zeros (1,N+1); phi = zeros(1,N+1);
x1(1) = 0; y1(1) = 6; phi(1) = 0;
hx = zeros(1, N+1); hy = zeros(1, N+1);
hx(1) = x1(1); hy(1) = y1(1);

u = [ones(1,20) zeros(1,10) ones(1,15) zeros(1,10) ...];
w = [zeros(1,20) -pi/2*ones(1,10) zeros(1,15)-pi/2*ones(1,10) ...];

for k=1:N
    phi(k+1) = phi(k) + w(k)*ts;
    xp1 = u(k)*cos(phi(k+1));
    yp1 = u(k)*sin(phi(k+1));
    x1(k+1) = x1(k) + xp1*ts;
    y1(k+1) = y1(k) + yp1*ts;
end
for k=1:step:N
delete(H1); delete(H2);
H1 = MobilePlot_4(x1(k), y1(k), phi(k), scale);
H2 = plot3(hx(1:k), hy(1:k), zeros(1,k), 'r','lineWidth', 2);
pause(ts);
end
subplot(211)
plot(t, u, 'b', 'LineWidth', 2); title('Velocidad Lineal');
subplot(212)
plot(t, w, 'r', 'LineWidth', 2); title('VelocidadAngular');