% Function to evaluate second derivative of bezier polynomials
% Inputs: Alpha - Bezeir coefficients (alpha_0 ... alpha_M)
%         s - s parameter. Range [0 1]
% Outputs: b = sum(k=0 to m-2)[ (alpha_k+2 -2*alpha_k+1 + alpha_k) * M!/(k!(M-k-2)!) s^k (1-s)^(M-k-2)]
%factorial(M)/(factorial(k)*factorial(M-k-2))
function b = polyval_bz_dd(alpha, s)
    b = zeros(size(s)) ;
    M = size(alpha,2)-1 ;  % length(alpha) = M+1
    switch M
        case 3
            c = [6 6];
        case 4
            c = [12 24 12];
        case 5
            c = [20 60 60 20];
        case 6
            c = [30 120 180 120 30];
        case 7
            c = [42 210 420 420 210 42];
        case 8
            c = [56 336 840 1120 840 336 56];
        case 9
            c = [72 504 1512 2520 2520 1512 504 72];    
        case 10
            c = [90 720 2520 5040 6300 5040 2520 720 90];
    end
    for k = 0:M-2
        b = b + (alpha(:,k+3)-2*alpha(:,k+2)+alpha(:,k+1)) .* c(k+1) .* s.^k .* (1-s).^(M-k-2) ;
    end
end