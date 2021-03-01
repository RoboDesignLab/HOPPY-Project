function [Be] = fcn_Be(q,p)

Be = zeros(4,2);

  Be(1,1)=0;
  Be(1,2)=0;
  Be(2,1)=0;
  Be(2,2)=0;
  Be(3,1)=1;
  Be(3,2)=0;
  Be(4,1)=0;
  Be(4,2)=1;

 