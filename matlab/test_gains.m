L1 = 0.3;
M  = 0.2;
L2 = 0.8;
V  = 1; 

R2_f = @(alpha) abs(L1/tan(alpha));
R3_f = @(R2) sqrt(R2^2 + M^2 - L2^2);
beta_f = @(alpha,R2,R3) atan2(M,R2) + atan2(L2,R3);

A_f = @(alpha,beta,psi) (V/L1)*M*(tan(alpha)/L2)*(sin(beta) + cot(psi)*cos(beta));
B_f = @(alpha,beta,psi) (V/L1)*(1/cos(alpha)^2)*(1 - (M/L2)*(cos(beta) - sin(beta)/tan(psi)));


% alpha = 0.000;
% alphas = 0.01:0.01:0.2;
alphas = -0.2:0.01:-0.01;
all_K = zeros(1,length(alphas));

for i = 1:length(alphas)
    alpha = alphas(i);

%     alpha = 0.2
    R2 = R2_f(alpha);
    R3 = R3_f(R2);
    psi = atan2(M,R2);
    
    beta = beta_f(alpha,R2,R3);
    if beta > pi
        beta = -(2*pi - beta);
    elseif beta < -pi
        beta = 2*pi + beta;
    end
    beta = beta*(alpha/abs(alpha));

    
    A = A_f(alpha,beta,psi);
    B = B_f(alpha,beta,psi);
    
    
    Q = 10;
    R = 1;
      
    K  = lqr(A,B,Q,R);
    all_K(i) = K;
end
plot(alphas,all_K)