function [err,oks] = RadialDistortionInvariant3dEstimationFailureTest2
% a synthetic example to the singular point of 3D estimation which is invariant
% to radial distortion. 
% We will make three cameras which lookat vectors intersect. This is a
% singular point in which there is n+o solution. 
err=RadialDistortionInvariant3dEstimationTest()
return
debug = false;
%ideal points and cameras:
points3D = [rand(3,100) ; ones(1,100)];
points3D(1,:) = points3D(1,:) - 0.5;
points3D(2,:) = points3D(2,:) - 0.5;
points3D(3,:) = points3D(3,:) + 2;

K = eye(3);
cam0 = [eye(3) [0 ;0 ;0]];
a =  0.17;
cam1 = [1,      0,     0, 0;...
        0, cos(a),sin(a),-1;...
        0,-sin(a),cos(a), 0];
    
cam2 = [ cos(a),      0, sin(a),-1;...
         0     ,      1,      0, 0;...
        -sin(a),      0, cos(a), 0];


%projection of 3D points to all cameras
cam0pixelsIdeal = K * cam0 * points3D;
cam0pixelsIdeal = [cam0pixelsIdeal(1,:)./cam0pixelsIdeal(3,:) ; cam0pixelsIdeal(2,:)./cam0pixelsIdeal(3,:)];

cam1pixelsIdeal = K * cam1 * points3D;
cam1pixelsIdeal = [cam1pixelsIdeal(1,:)./cam1pixelsIdeal(3,:) ; cam1pixelsIdeal(2,:)./cam1pixelsIdeal(3,:)];

cam2pixelsIdeal = K * cam2 * points3D;
cam2pixelsIdeal = [cam2pixelsIdeal(1,:)./cam2pixelsIdeal(3,:) ; cam2pixelsIdeal(2,:)./cam2pixelsIdeal(3,:)];

writematrix(cam0);
writematrix(cam1);
writematrix(cam2);

writematrix(cam0pixelsIdeal);
writematrix(cam1pixelsIdeal);
writematrix(cam2pixelsIdeal);

writematrix(points3D);

%radial distortion:
polyCoefs = [0.5,-0.5,1.5];

cam0pixelDistorted  = ApplyDistortion(cam0pixelsIdeal,polyCoefs);
cam1pixelDistorted  = ApplyDistortion(cam1pixelsIdeal,polyCoefs);
cam2pixelDistorted  = ApplyDistortion(cam2pixelsIdeal,polyCoefs);
writematrix(cam0pixelDistorted)
if(debug)
   figure(4);
   subplot(2,2,1);
   plot3(points3D(1,:),points3D(2,:),points3D(3,:),'.b');
   hold on;
   cam0Loc = -cam0(:,1:3)'*cam0(:,4);
   cam0Dir = -cam0(:,1:3)'*(cam0(:,4)+[0;0;1]);
   plot3([cam0Loc(1,:) cam0Dir(1,:)],[cam0Loc(2,:) cam0Dir(2,:)],[cam0Loc(3,:) cam0Dir(3,:)],'.-g');
   
   cam1Loc = -cam1(:,1:3)'*cam1(:,4);
   cam1Dir = -cam1(:,1:3)'*(cam1(:,4)+[0;0;1]);
   plot3([cam1Loc(1,:) cam1Dir(1,:)],[cam1Loc(2,:) cam1Dir(2,:)],[cam1Loc(3,:) cam1Dir(3,:)],'.-g');
   
   cam2Loc = -cam2(:,1:3)'*cam2(:,4);
   cam2Dir = -cam2(:,1:3)'*(cam2(:,4)+[0;0;1]);
   plot3([cam2Loc(1,:) cam2Dir(1,:)],[cam2Loc(2,:) cam2Dir(2,:)],[cam2Loc(3,:) cam2Dir(3,:)],'.-g');
     
   axis equal;
   hold off;
   title('3d points and cameras');
   
   subplot(2,2,2);
   plot(cam0pixelsIdeal(1,:),cam0pixelsIdeal(2,:),'b.');
   hold on;
   plot(cam0pixelDistorted(1,:),cam0pixelDistorted(2,:),'b.');
   for i=1:size(cam0pixelsIdeal,2)
       plot([cam0pixelsIdeal(1,i) cam0pixelDistorted(1,i)],[cam0pixelsIdeal(2,i) cam0pixelDistorted(2,i)],'g-');       
   end
   axis([-0.8 0.8 -0.8 0.8]);   
   hold off;
   title('radial distortion cam 0');
      
   subplot(2,2,3);
   plot(cam1pixelsIdeal(1,:),cam1pixelsIdeal(2,:),'b.');
   hold on;
   plot(cam1pixelDistorted(1,:),cam1pixelDistorted(2,:),'b.');
   for i=1:size(cam1pixelsIdeal,2)
       plot([cam1pixelsIdeal(1,i) cam1pixelDistorted(1,i)],[cam1pixelsIdeal(2,i) cam1pixelDistorted(2,i)],'g-');       
   end
   axis([-0.8 0.8 -0.8 0.8]);   
   hold off;
   title('radial distortion cam 1');
   
   subplot(2,2,4);
   plot(cam2pixelsIdeal(1,:),cam2pixelsIdeal(2,:),'b.');
   hold on;
   plot(cam2pixelDistorted(1,:),cam2pixelDistorted(2,:),'b.');
   for i=1:size(cam2pixelsIdeal,2)
       plot([cam2pixelsIdeal(1,i) cam2pixelDistorted(1,i)],[cam2pixelsIdeal(2,i) cam2pixelDistorted(2,i)],'g-');       
   end
   axis([-0.8 0.8 -0.8 0.8]);   
   hold off;
   title('radial distortion cam 2');
   
     
end

estimatedPoints = zeros(size(points3D));
ok = zeros(size(points3D,2),1);
for i = 1:size(cam2pixelsIdeal,2)
    [estimatedPoints(:,i),ok(i)] = RadialDistortionInvariant3dEstimation(cam0,cam1,cam2,cam0pixelDistorted(:,i),cam1pixelDistorted(:,i),cam2pixelDistorted(:,i));
end
writematrix(estimatedPoints)
err = norm(estimatedPoints - points3D);
oks = sum(ok);
if debug
    subplot(2,2,1);
    hold on;
    plot3(estimatedPoints(1,:),estimatedPoints(2,:),estimatedPoints(3,:),'.r');
    hold off;
    title (['3d estimation failure. Error: ' num2str(err)]);
         
end


function distortedPixels = ApplyDistortion(idealPixels,polyCoefs)
    cam0pixelsRadiusIdeal = (idealPixels(1,:).^2 + idealPixels(2,:).^2 ).^0.5;
    cam0pixelsRadiusDistorted = polyCoefs(1)*cam0pixelsRadiusIdeal.^5 + polyCoefs(2)*cam0pixelsRadiusIdeal.^3 + polyCoefs(3)*cam0pixelsRadiusIdeal;
    cam0RadiusRatio = cam0pixelsRadiusDistorted./cam0pixelsRadiusIdeal;
    distortedPixels = [idealPixels(1,:).*cam0RadiusRatio  ;  idealPixels(2,:).*cam0RadiusRatio];

function err = RadialDistortionInvariant3dEstimationTest
% a synthetic example to the principle of 3D estimation which is invariant
% to radial distortion. We will make distorted projection, and reconstruct
% the 3D points regardless to the radial distortion.
debug = true;
%ideal points and cameras:
points3D = [rand(3,100) ; ones(1,100)];
points3D(1,:) = points3D(1,:) - 0.5;
points3D(2,:) = points3D(2,:) - 0.5;
points3D(3,:) = points3D(3,:) + 2;
writematrix(points3D)
K = eye(3);
cam0 = [eye(3) [0;0;0]];
cam1 = GetRandomCamera([-1;0;0]);
cam2 = GetRandomCamera([0;-1;0]);
writematrix(cam0)
writematrix(cam1)
writematrix(cam2)
%projection of 3D points to all cameras
cam0pixelsIdeal = K * cam0 * points3D;
cam0pixelsIdeal = [cam0pixelsIdeal(1,:)./cam0pixelsIdeal(3,:) ; cam0pixelsIdeal(2,:)./cam0pixelsIdeal(3,:)];

cam1pixelsIdeal = K * cam1 * points3D;
cam1pixelsIdeal = [cam1pixelsIdeal(1,:)./cam1pixelsIdeal(3,:) ; cam1pixelsIdeal(2,:)./cam1pixelsIdeal(3,:)];

cam2pixelsIdeal = K * cam2 * points3D;
cam2pixelsIdeal = [cam2pixelsIdeal(1,:)./cam2pixelsIdeal(3,:) ; cam2pixelsIdeal(2,:)./cam2pixelsIdeal(3,:)];
writematrix(cam0pixelsIdeal)
writematrix(cam1pixelsIdeal)
writematrix(cam2pixelsIdeal)
%radial distortion:
polyCoefs = [0.5,-0.5,1.5];

cam0pixelDistorted  = ApplyDistortion(cam0pixelsIdeal,polyCoefs);
cam1pixelDistorted  = ApplyDistortion(cam1pixelsIdeal,polyCoefs);
cam2pixelDistorted  = ApplyDistortion(cam2pixelsIdeal,polyCoefs);

if(debug)
   figure(1);
   subplot(2,2,1);
   plot3(points3D(1,:),points3D(2,:),points3D(3,:),'.b');
   hold on;
   cam0Loc = -cam0(:,1:3)'*cam0(:,4);
   cam0Dir = -cam0(:,1:3)'*(cam0(:,4)+[0;0;1]);
   plot3([cam0Loc(1,:) cam0Dir(1,:)],[cam0Loc(2,:) cam0Dir(2,:)],[cam0Loc(3,:) cam0Dir(3,:)],'.-g');
   axis vis3d;
   
   cam1Loc = -cam1(:,1:3)'*cam1(:,4);
   cam1Dir = -cam1(:,1:3)'*(cam1(:,4)+[0;0;1]);
   plot3([cam1Loc(1,:) cam1Dir(1,:)],[cam1Loc(2,:) cam1Dir(2,:)],[cam1Loc(3,:) cam1Dir(3,:)],'.-g');
   
   cam2Loc = -cam2(:,1:3)'*cam2(:,4);
   cam2Dir = -cam2(:,1:3)'*(cam2(:,4)+[0;0;1]);
   plot3([cam2Loc(1,:) cam2Dir(1,:)],[cam2Loc(2,:) cam2Dir(2,:)],[cam2Loc(3,:) cam2Dir(3,:)],'.-g');
     
   axis equal;
   hold off;
   
   subplot(2,2,2);
   plot(cam0pixelsIdeal(1,:),cam0pixelsIdeal(2,:),'b.');
   hold on;
   plot(cam0pixelDistorted(1,:),cam0pixelDistorted(2,:),'b.');
   for i=1:size(cam0pixelsIdeal,2)
       plot([cam0pixelsIdeal(1,i) cam0pixelDistorted(1,i)],[cam0pixelsIdeal(2,i) cam0pixelDistorted(2,i)],'g-');       
   end
   axis([-0.8 0.8 -0.8 0.8]);   
   hold off;
   title('radial distortion cam 0');
      
   subplot(2,2,3);
   plot(cam1pixelsIdeal(1,:),cam1pixelsIdeal(2,:),'b.');
   hold on;
   plot(cam1pixelDistorted(1,:),cam1pixelDistorted(2,:),'b.');
   for i=1:size(cam1pixelsIdeal,2)
       plot([cam1pixelsIdeal(1,i) cam1pixelDistorted(1,i)],[cam1pixelsIdeal(2,i) cam1pixelDistorted(2,i)],'g-');       
   end
   axis([-0.8 0.8 -0.8 0.8]);   
   hold off;
   title('radial distortion cam 1');
   
   subplot(2,2,4);
   plot(cam2pixelsIdeal(1,:),cam2pixelsIdeal(2,:),'b.');
   hold on;
   plot(cam2pixelDistorted(1,:),cam2pixelDistorted(2,:),'b.');
   for i=1:size(cam2pixelsIdeal,2)
       plot([cam2pixelsIdeal(1,i) cam2pixelDistorted(1,i)],[cam2pixelsIdeal(2,i) cam2pixelDistorted(2,i)],'g-');       
   end
   axis([-0.8 0.8 -0.8 0.8]);   
   hold off;
   title('radial distortion cam 2');
   
     
end

estimatedPoints = zeros(size(points3D));
for i = 1:size(cam2pixelsIdeal,2)
    estimatedPoints(:,i) = RadialDistortionInvariant3dEstimation(cam0,cam1,cam2,cam0pixelDistorted(:,i),cam1pixelDistorted(:,i),cam2pixelDistorted(:,i));
end
writematrix(estimatedPoints)
err = norm(estimatedPoints - points3D);

if(debug)
    figure(1);
    subplot(2,2,1);    
    title(['3d points and cameras. Estimation error: ' num2str(err)]);
end



function cam = GetRandomCamera(T)
R1 = rand(3) + eye(3);
[u s v] = svd(R1);
R1 = v'*u;
if(det(R1)<0)
    R1 =-R1;
end
cam = [R1 T];




function [points3d] = calc_points_3d(points1,points2,points3)
[points1, points2, points3] = make_of_from_box_corners();
size = [640, 480];
points1 = tl2cen(points1, size);
points2 = tl2cen(points2, size);
points3 = tl2cen(points3, size);

cam1 = [-0.7152 1.4076 1.0733 -0.09384593045608725 -2.2928236739615913 2.6677269190232535];
cam2 = [-0.4469 1.0960 1.0644 -0.10819746217318274, -2.278048850768324, 2.3112982598988383];
cam3 = [-0.2742 0.8198 1.0402 -0.11214622521033156, -2.261327353083634, 2.107934302777922];

cam_mat1 = calc_cam_mat(cam1);
cam_mat2 = calc_cam_mat(cam2);
cam_mat3 = calc_cam_mat(cam3);


[res,success] = RadialDistortionInvariant3dEstimation(cam_mat1,cam_mat2,cam_mat3,points1,points2,points3);




function points = tl2cen(points, size)
    hsz = size/2;
    points(:,1) = points(:,1) - hsz(1)
    points(:,2) = hsz(2) - points(:,2)
    
function cam_mat = calc_cam_mat(cam_params)
az = cam_params(6)
ro = cam_params(5)
pi = cam_params(4)

az_mat = [cos(az) -sin(az) 0  ; sin(az) cos(az) 0 ; 0 0 0]
ro_mat = [cos(ro) 0 -sin(ro) ; 0 0 0; sin(ro) 0 cos(ro) ]
pi_mat = [0 0 0 ; 0 cos(pi) -sin(pi); 0 sin(pi) cos(pi)]

R = ro_mat*pi_mat*az_mat
t = cam_params(1:3)
cam_mat = [R t']



[res,success] = RadialDistortionInvariant3dEstimationMultiview(cams,camsPixelDistorted)
%this function takes 3 pixels of the same 3D point and the 3 camera
%matrices, and calculates the 3d point location, even under severe radial distortion. 
% success - false if two cameras are identical or if all 3 cameras center
% ray is looking at the same 3D point or are parallel. 
nCams = size(cams,3);
planes = zeros(nCams,4);
success = true;
for i = 1:nCams
    [planes(i,:),ok] = GetPlaneFromPixel(cams(:,:,i),camsPixelDistorted(:,i));    
    success = success && ok;
end
[res, ok4] = Get3planesIntersectionMultiview(planes);
success = success && ok4;

function [plane, ok] = GetPlaneFromPixel(cam,pixelDistorted)
p0 = [ 0; 0; 0 ]; % camera center;
p1 = [ 0; 0; 1 ]; %centerOfProjection plane;
p2 = [ pixelDistorted(1); pixelDistorted(2); 1 ];

R = cam(:,1:3);
T = cam(:,4);
p0 = R' * (p0 - T); 
p1 = R' * (p1 - T); 
p2 = R' * (p2 - T);
A = [p0' 1;p1' 1;p2' 1];% 3x4
[~, s, v] = svd(A);
ok = s(3,3) > 1e-5;
plane = v(:,end);

function [point3D, ok] = Get3planesIntersectionMultiview(planes)
[~, s, v] = svd(planes);
ok = s(3,3) > 1e-5;
point3D = v(:,end);
point3D = point3D/point3D(4);


function [res,success] = RadialDistortionInvariant3dEstimation(cam0,cam1,cam2,cam0pixelDistorted,cam1pixelDistorted,cam2pixelDistorted)
%this function takes 3 pixels of the same 3D point and the 3 camera
%matrices, and calculates the 3d point location, even under severe radial distortion. 
% success - false if two cameras are identical or if all 3 cameras center
% ray is looking at the same 3D point or are parallel. 
[plane0, ok1] = GetPlaneFromPixel(cam0,cam0pixelDistorted);
[plane1, ok2] = GetPlaneFromPixel(cam1,cam1pixelDistorted);
[plane2, ok3] = GetPlaneFromPixel(cam2,cam2pixelDistorted);
[res, ok4] = Get3planesIntersection(plane0,plane1,plane2);
success = ok1 & ok2 & ok3 & ok4;


function [point3D, ok] = Get3planesIntersection(plane0,plane1,plane2)
A = [plane0';plane1';plane2'];% 3x4
[~, s, v] = svd(A);
ok = s(3,3) > 1e-5;
point3D = v(:,end);

point3D = point3D/point3D(4);



function  [corners1, corners2, corners3] = make_of_from_box_corners()
debug=0;
show_images()
corners1 = [235,53;497,32;612,99;305,149;236,231;289,337;531,267];
corners2 = [213,83;434,27;563,78;366,177;219,271;327,369;495,243];
corners3 = [260,93;421,15;560,57;453,171;255,285;397,363;499,221];

face1 = [1,2,3,4,1];
face2 = [1,4,6,5,1];
face3 = [4,3,7,6,4];

if(debug)
    subplot(2,2,1)
    hold('on')
    plot(corners1(face1,1),corners1(face1,2), '-g');
    plot(corners1(face2,1),corners1(face2,2), '-g');
    plot(corners1(face3,1),corners1(face3,2), '-g');
    
    subplot(2,2,2)
    hold('on')
    plot(corners2(face1,1),corners2(face1,2), '-g');
    plot(corners2(face2,1),corners2(face2,2), '-g');
    plot(corners2(face3,1),corners2(face3,2), '-g');
    
    subplot(2,2,3)
    hold('on')
    plot(corners3(face1,1),corners3(face1,2), '-g');
    plot(corners3(face2,1),corners3(face2,2), '-g');
    plot(corners3(face3,1),corners3(face3,2), '-g');
end


    
    
function [res,success] = RadialDistortionInvariant3dEstimationMultiview(cams,camsPixelDistorted)
%this function takes 3 pixels of the same 3D point and the 3 camera
%matrices, and calculates the 3d point location, even under severe radial distortion. 
% success - false if two cameras are identical or if all 3 cameras center
% ray is looking at the same 3D point or are parallel. 
nCams = size(cams,3);
planes = zeros(nCams,4);
success = true;
for i = 1:nCams
    [planes(i,:),ok] = GetPlaneFromPixel(cams(:,:,i),camsPixelDistorted(:,i));    
    success = success && ok;
end
[res, ok4] = Get3planesIntersectionMultiview(planes);
success = success && ok4;






