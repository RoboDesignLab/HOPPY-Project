function [p_h2f_b] = fcn_p_h2f_b(q,p)

p_h2f_b = zeros(3,1);

  p_h2f_b(1,1)=0;
  p_h2f_b(2,1)=sin(q(3) + q(4))*(p(6)^2 + p(7)^2)^(1/2) + p(5)*sin(q(3));
  p_h2f_b(3,1)=- cos(q(3) + q(4))*(p(6)^2 + p(7)^2)^(1/2) - p(5)*cos(q(3));

 