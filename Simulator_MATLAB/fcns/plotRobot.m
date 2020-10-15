function h = plotRobot(X,p)

params = p.params;
ang = 0:0.1:2*pi;
R_mat = 1.2*params(3);

q = X(1:4);
p0 = [0 0 0]';
p1 = fcn_p1(q,params);
p2_aux = fcn_p2_aux(q,params);
p2 = fcn_p2(q,params);
p3 = fcn_p3(q,params);
p_toe = fcn_p_toe(q,params);
HTM2 = fcn_T02(X(1:4),p.params);                   % Hip box HTM
HTM3 = fcn_T03(X(1:4),p.params);                   % Shank box HTM

p_weight = p1-(p2_aux - p1) / norm(p2_aux - p1)*0.25;

chain = [p0 p1 p_weight p2_aux p2 p3 p_toe];

plot3(chain(1,:),chain(2,:),chain(3,:),'k','LineWidth',4);
hold on
plot3(chain(1,:),chain(2,:),chain(3,:),'.r','LineWidth',4);
fill3(R_mat*cos(ang),R_mat*sin(ang),0*ang,0.5*[1 1 1])
DrawCube([.144 .096 .048],[0 -0.043 0],[0.5 0.5 0.8],HTM2);
DrawCube([.1 .1 .1],[-0.823 -0.048 0],0.2*[1 1 1],HTM2); %Counterweight
DrawCube([.043 .043 .136],[0 0 0.048],[0.5 0.5 0.8],HTM3);
axis equal
grid on
xlabel('X_0 [m]')
ylabel('Y_0 [m]')
zlabel('Z_0 [m]')

h = gcf;

end
