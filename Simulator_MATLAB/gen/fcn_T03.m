function [T03] = fcn_T03(q,p)

T03 = zeros(4,4);

  T03(1,1)=cos(q(1))*cos(q(2));
  T03(1,2)=cos(q(1))*sin(q(2))*sin(q(3)) - cos(q(3))*sin(q(1));
  T03(1,3)=sin(q(1))*sin(q(3)) + cos(q(1))*cos(q(3))*sin(q(2));
  T03(1,4)=p(3)*cos(q(1))*cos(q(2)) - p(4)*sin(q(1)) - p(5)*sin(q(1))*sin(q(3)) - p(5)*cos(q(1))*...
         cos(q(3))*sin(q(2));
  T03(2,1)=cos(q(2))*sin(q(1));
  T03(2,2)=cos(q(1))*cos(q(3)) + sin(q(1))*sin(q(2))*sin(q(3));
  T03(2,3)=cos(q(3))*sin(q(1))*sin(q(2)) - cos(q(1))*sin(q(3));
  T03(2,4)=p(4)*cos(q(1)) + p(3)*cos(q(2))*sin(q(1)) + p(5)*cos(q(1))*sin(q(3)) - p(5)*cos(q(3))*...
         sin(q(1))*sin(q(2));
  T03(3,1)=-sin(q(2));
  T03(3,2)=cos(q(2))*sin(q(3));
  T03(3,3)=cos(q(2))*cos(q(3));
  T03(3,4)=p(2) - p(3)*sin(q(2)) - p(5)*cos(q(2))*cos(q(3));
  T03(4,1)=0;
  T03(4,2)=0;
  T03(4,3)=0;
  T03(4,4)=1;

 