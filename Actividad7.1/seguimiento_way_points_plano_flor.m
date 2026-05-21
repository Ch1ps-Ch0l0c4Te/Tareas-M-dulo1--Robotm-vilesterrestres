R = 0.1; L = 0.5; dd = DifferentialDrive(R,L);
sampleTime = 0.05; tVec = 0:sampleTime:250;

initPose = [4; 1; 0]; 
pose = zeros(3,numel(tVec)); pose(:,1) = initPose;

waypoints = [
    4, 1; 2, 1; 0, 3; 2, 3; 4, 1; 6, 1; 8, 3; 6, 3; 
    4, 1; 4, 5; 2, 3; 2, 5; 0, 5; 2, 7; 0, 9; 2, 9; 
    2, 11; 4, 9; 6, 11; 6, 9; 8, 9; 6, 7; 8, 5; 6, 5; 
    6, 3; 4, 5; 4, 6; 3, 6; 3, 8; 5, 8; 5, 6; 4, 6
];

viz = Visualizer2D; viz.hasWaypoints = true;

controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.15;       
controller.DesiredLinearVelocity = 0.30;   
controller.MaxAngularVelocity = 10.0;      

close all; r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    [vRef,wRef] = controller(pose(:,idx-1));
    [wL,wR] = inverseKinematics(dd,vRef,wRef);
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w]; vel = bodyToWorld(velB,pose(:,idx-1));  
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    viz(pose(:,idx),waypoints); waitfor(r);
end