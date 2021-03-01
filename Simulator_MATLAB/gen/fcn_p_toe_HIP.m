function [p_toe_HIP] = fcn_p_toe_HIP(q,p)

p_toe_HIP = zeros(2,1);

  p_toe_HIP(1,1)=p(5)*sin(q(3)) + cos(q(3))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) + cos(q(4))*sin(q(3))*...
         (p(6)^2 + p(7)^2)^(1/2);
  p_toe_HIP(2,1)=sin(q(3))*sin(q(4))*(p(6)^2 + p(7)^2)^(1/2) - cos(q(3))*cos(q(4))*(p(6)^2 +...
          p(7)^2)^(1/2) - p(5)*cos(q(3));

 