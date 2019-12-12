%% this is to generate the workspace of the parallel robots
R=290*sqrt(3); % side lenth of the triangle base.(mm)
r=130; 
PB1=[0,0];
PB2=[R,0];
PB3=[R/2,sqrt(3)*R/2];

base=[PB1;PB2;PB3;PB1];
plot(base(:,1),base(:,2));
axis([-20,inf,-20,inf])
hold on

%% link lenth (mm)
S=170;
L=130;

a=input("please input an angle: ");
phi_1=a+pi/6;
phi_2=a+5*pi/6;
phi_3=a+3*pi/2;
%% traversal of xc,yc
xc = 0:2:506;
yc = 0:2:449;
for i=1:size(xc,2)
    for j=1:size(yc,2)
pp1=[xc(i)-r*cos(phi_1),yc(j)-r*sin(phi_1)];%note: 
pp2=[xc(i)+r*cos(a-pi/6),yc(j)+r*sin(a-pi/6)];
pp3=[(xc(i)-r*cos(pi/2-a)),yc(j)+r*sin(pi/2-a)];

d1=norm(pp1-PB1);
d2=norm(pp2-PB2);
d3=norm(pp3-PB3);
 if (d1<(S+L)&&d1>(S-L)&&d2<(S+L)&&d2>(S-L)&&d3<(S+L)&&d3>(S-L))
     plot(xc(i),yc(j),'r.')
 end
    end
end
hold off
     
