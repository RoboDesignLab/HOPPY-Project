% Function to evaluate first derivative of bezier polynomials
% Inputs: Alpha - Bezeir coefficients (alpha_0 ... alpha_M)
%         s - s parameter. Range [0 1]
% Outputs: b = sum(k=0 to m-1)[ (alpha_k+1 - alpha_k) * M!/(k!(M-k-1)!) s^k (1-s)^(M-k-1)]
%factorial(M)/(factorial(k)*factorial(M-k-1))
function b = polyval_bz_d(alpha, s)
    b = zeros(size(s)) ;
    M = size(alpha,2)-1 ;  % length(alpha) = M+1
    switch M
        case 3
            c = [3 6 3];
        case 4 
            c = [4 12 12 4];
        case 5
            c = [5 20 30 20 5];
        case 6
            c = [6 30 60 60 30 6];
        case 7
            c = [7 42 105 140 105 42 7];
        case 8
            c = [8 56 168 280 280 168 56 8];
        case 9
            c = [9 72 252 504 630 504 252 72 9];
        case 10
            c = [10 90 360 840 1260 1260 840 360 90 10];
    end    
    for k = 0:M-1
        b = b + (alpha(:,k+2)-alpha(:,k+1)) .* c(k+1) .* s.^k .* (1-s).^(M-k-1) ;
    end
end