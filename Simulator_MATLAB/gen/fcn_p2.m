function [p2] = fcn_p2(q,p)

p2 = zeros(3,1);

  p2(1,1)=p(3)*cos(q(1))*cos(q(2)) - p(4)*sin(q(1));
  p2(2,1)=p(4)*cos(q(1)) + p(3)*cos(q(2))*sin(q(1));
  p2(3,1)=p(2) - p(3)*sin(q(2));

 