function DrawPathPlan(DATA)
%%  Long Wang, 2014/9/19
%   This func will draw the path plan given the path data
if size(DATA,1)==4
    X = DATA(2:4,:);
elseif size(DATA,1)==3
    X = DATA(1:3,:);
end
figure;
hold on;
axis equal;
view(45,45);
scatter3(X(1,:),X(2,:),X(3,:),8,'m');
scatter3(X(1,1),X(2,1),X(3,1),80,'r','filled');
scatter3(X(1,end),X(2,end),X(3,end),80,'g','filled');
plot3(X(1,:),X(2,:),X(3,:),'b');
draw_coordinate_system([10;10;10],eye(3),zeros(3,1),'rgb');
figure
hold on;
axis equal;
view(45,45);
scatter3(X(1,:),X(2,:),X(3,:),8,'m');
scatter3(X(1,1),X(2,1),X(3,1),80,'r','filled');
scatter3(X(1,end),X(2,end),X(3,end),80,'g','filled');
plot3(X(1,:),X(2,:),X(3,:),'b');

end

