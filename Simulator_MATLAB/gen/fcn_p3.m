function [p3] = fcn_p3(q,p)

p3 = zeros(3,1);

  p3(1,1)=p(3)*cos(q(1))*cos(q(2)) - p(4)*sin(q(1)) - p(5)*sin(q(1))*sin(q(3)) - p(5)*cos(q(1))*...
         cos(q(3))*sin(q(2));
  p3(2,1)=p(4)*cos(q(1)) + p(3)*cos(q(2))*sin(q(1)) + p(5)*cos(q(1))*sin(q(3)) - p(5)*cos(q(3))*...
         sin(q(1))*sin(q(2));
  p3(3,1)=p(2) - p(3)*sin(q(2)) - p(5)*cos(q(2))*cos(q(3));

 