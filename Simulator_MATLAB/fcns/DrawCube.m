function h = DrawCube(size,offset,color,HTM)

W = size(1);
H = size(2);
L = size(3);

P1 = [offset'; 0] + [W/2 H/2 L/2 1]';
P2 = [offset'; 0] + [W/2 -H/2 L/2 1]';
P3 = [offset'; 0] + [-W/2 -H/2 L/2 1]';
P4 = [offset'; 0] + [-W/2 H/2 L/2 1]';
P5 = [offset'; 0] + [W/2 H/2 -L/2 1]';
P6 = [offset'; 0] + [W/2 -H/2 -L/2 1]';
P7 = [offset'; 0] + [-W/2 -H/2 -L/2 1]';
P8 = [offset'; 0] + [-W/2 H/2 -L/2 1]';

Pts = HTM*[P1 P2 P3 P4];
fill3(Pts(1,:),Pts(2,:),Pts(3,:),color);
hold on

Pts = HTM*[P3 P2 P6 P7];
fill3(Pts(1,:),Pts(2,:),Pts(3,:),color);
Pts = HTM*[P4 P1 P5 P8];
fill3(Pts(1,:),Pts(2,:),Pts(3,:),color);
Pts = HTM*[P4 P3 P7 P8];
fill3(Pts(1,:),Pts(2,:),Pts(3,:),color);
Pts = HTM*[P2 P1 P5 P6];
fill3(Pts(1,:),Pts(2,:),Pts(3,:),color);
Pts = HTM*[P5 P6 P7 P8];
fill3(Pts(1,:),Pts(2,:),Pts(3,:),color);

h = gcf;


end