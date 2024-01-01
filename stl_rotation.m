clear all;
clc;

model = stlread('khopnoimattrau.STL');

%make point and conectivity matrice from this model
points = model.Points;
cList = model.ConnectivityList;

%define the angle
thetax = deg2rad(90);
thetay = 0;
thetaz = 0;

%rotation matrix
Rx = [1 0 0; 0 cos(thetax) -sin(thetax); 0 sin(thetax) cos(thetax)];
Ry = [cos(thetay) 0 sin(thetay); 0 1 0; -sin(thetay) 0 cos(thetay)];
Rz = [cos(thetaz) -sin(thetaz) 0; sin(thetaz) cos(thetaz) 0; 0 0 1];

%rotate point matrix
pointRx = points*Rx;
pointR = points*Ry;
pointR = points*Rz;

%plot the origin object
figure
h1 = trimesh(model);

%plot the rotated object
figure
h2 = trimesh(cList, pointRx(:,1),pointRx(:,2),pointRx(:,3));   

