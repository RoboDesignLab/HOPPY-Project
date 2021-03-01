function [p2_aux] = fcn_p2_aux(q,p)

p2_aux = zeros(3,1);

  p2_aux(1,1)=p(3)*cos(q(1))*cos(q(2));
  p2_aux(2,1)=p(3)*cos(q(2))*sin(q(1));
  p2_aux(3,1)=p(2) - p(3)*sin(q(2));

 