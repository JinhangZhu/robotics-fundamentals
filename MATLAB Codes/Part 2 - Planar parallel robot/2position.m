%% base coordinate
R=290*sqrt(3); % side lenth of the triangle base.(mm)
r=130;%the distance from the vertex to the center of the platform
%coordinate of three vertex of the base
PB1=[0,0];
PB2=[R,0];
PB3=[R/2,sqrt(3)*R/2];

%% link lenth (mm)
%lenth of first arm
SA=170;
SB=170;
SC=170;
%length of second arm
L=130;

%%
%input the cartesian coordinate
xc=input('please input the x coordinate of the center:');
yc=input('please input the y coordinate of the center:');
a=input('please input the orientation of the center:');
    
phi_1=a+pi/6;
phi_2=a+5*pi/6;
phi_3=a+3*pi/2;
     
%coordinate of three vertexs of the platform
pp1=[xc-r*cos(phi_1),yc-r*sin(phi_1)]; 
pp2=[xc+r*cos(phi_2-pi),yc+r*sin(phi_2-pi)];
pp3=[(xc-r*cos(2*pi-phi_3)),yc+r*sin(2*pi-phi_3)];
    
%parameter for theta1
c1=atan((yc-r*sin(phi_1))/(xc-r*cos(phi_1)));
d1=acos((SA^2-L^2+norm(pp1-PB1)*norm(pp1-PB1))/(2*SA*norm(pp1-PB1)));
theta1=c1+d1;
    
%parameter for theta2
c2=atan2(pp2(2)-PB2(2),pp2(1)-PB2(1));
d2=acos((SB^2-L^2+norm(pp2-PB2)*norm(pp2-PB2))/(2*SB*norm(pp2-PB2)));
theta2=c2+d2;
    
%parameter for theta3
c3=atan2(pp3(2)-PB3(2),pp3(1)-PB3(1));
d3=acos((SC^2-L^2+norm(pp3-PB3)*norm(pp3-PB3))/(2*SC*norm(pp3-PB3)));
theta3=c3+d3;
    
%two position of M
M1=[SA*cos(theta1),SA*sin(theta1)];
M2=[R+SB*cos(theta2),SB*sin(theta2)];
M3=[R/2+SC*cos(theta3),sqrt(3)*R/2+SC*sin(theta3)];
    
%computed length for the second arm
L1=norm(M1-pp1);
L2=norm(M2-pp2);
L3=norm(M3-pp3);
    
if(L1-130<1e-5&&L2-130<1e-5&&L3-130<1e-5)    
%plot the base
Base=[PB1;PB2;PB3;PB1];
plot(Base(:,1),Base(:,2),'linewidth',3);
axis([-100 550 -100 500]);
hold on

%plot the platform
Platform=[pp1;pp2;pp3;pp1];
plot(Platform(:,1),Platform(:,2),'g-','linewidth',3);
    
%plot three arms
arm1=[PB1;M1;pp1];
arm2=[PB2;M2;pp2];
arm3=[PB3;M3;pp3];
plot(arm1(:,1),arm1(:,2),'k-^','linewidth',3);
plot(arm2(:,1),arm2(:,2),'r-^','linewidth',3);
plot(arm3(:,1),arm3(:,2),'y-^','linewidth',3);
else
    disp("the center and orientation is out of the workspace of the robot");
end
hold off
