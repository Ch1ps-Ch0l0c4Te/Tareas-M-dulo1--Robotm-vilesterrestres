R = 0.1; L = 0.5; dd = DifferentialDrive(R,L);
sampleTime = 0.05; tVec = 0:sampleTime:200;

initPose = [4; 9; 0]; 
pose = zeros(3,numel(tVec)); pose(:,1) = initPose;

waypoints = [
    4, 9; 6, 11; 8, 11; 10, 9; 4, 9; 10, 9; 7, 5; 6, 6; 
    7, 5; 8, 5; 7, 5; 8, 6; 7, 7; 5, 7; 3, 5; 3, 3; 
    5, 1; 7, 1; 9, 3; 9, 5; 8, 6
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