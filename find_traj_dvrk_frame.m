function traj_dvrk = find_traj_dvrk_frame(traj_XY,corners,depth)


x1  = 1000*corners(:,1);
x2  = 1000*corners(:,2);
x3  = 1000*corners(:,3);
x4  = 1000*corners(:,4);

vx = x1-x2;
vytmp = x3-x2;
vz = cross(vx,vytmp);

vx=vx/norm(vx);

vz=vz/norm(vz);
vy=cross(vz,vx);

vy=vy/norm(vy);
t = x2(:);

R = [vx(:),vy(:),vz(:)];
% T = [R;[0 0 0],[t;1]];

traj_dvrk = R*[traj_XY;-depth*ones(1,size(traj_XY,2))]+t;

traj_dvrk = traj_dvrk/1000;
end