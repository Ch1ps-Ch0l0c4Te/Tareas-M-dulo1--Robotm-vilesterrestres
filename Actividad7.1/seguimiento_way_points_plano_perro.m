%% CONFIGURACIÓN DEL ROBOT - EL PERRO (Coordenadas del Usuario)
R = 0.1; L = 0.5; dd = DifferentialDrive(R,L);
sampleTime = 0.05; 
tVec = 0:sampleTime:200; % Tiempo ampliado para cubrir todos los puntos y retrocesos

% Posición inicial en el primer punto de tu lista (4, -1)
initPose = [4; -1; 1.57]; 
pose = zeros(3,numel(tVec)); pose(:,1) = initPose;

%% WAYPOINTS (29 puntos exactos proporcionados)
waypoints = [
    4, -1;
    3, 1;
    3, 4;
    2, 5;
    -1, 5;
    -2, 7;
    -2, 8;
    -1, 8;
    0, 8;
    0, 9;
    2, 9.5;
    3, 10;
    2, 11;
    2, 9.5;
    3, 10;
    4, 10;
    4, 11;
    5, 8;
    4, 7;
    5, 8;
    4, 11;
    6, 7;
    7, 6;
    8, 5;
    9, 5;
    8, 5;
    3, 3;
    3, 4;
    7, 6
];

viz = Visualizer2D; viz.hasWaypoints = true;

%% CONFIGURACIÓN RÍGIDA DEL CONTROLADOR
controller = controllerPurePursuit;
controller.Waypoints = waypoints;

% Parámetros muy ajustados para respetar las idas y vueltas de tu lista
controller.LookaheadDistance = 0.15;       % Muy corto para no saltarse los puntos que están muy cerca
controller.DesiredLinearVelocity = 0.30;   % Velocidad constante
controller.MaxAngularVelocity = 10.0;      % Giro muy agresivo para dar la vuelta en los retrocesos

%% BUCLE DE SIMULACIÓN
close all; r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    [vRef,wRef] = controller(pose(:,idx-1));
    [wL,wR] = inverseKinematics(dd,vRef,wRef);
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w]; vel = bodyToWorld(velB,pose(:,idx-1));  
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    viz(pose(:,idx),waypoints); waitfor(r);
end