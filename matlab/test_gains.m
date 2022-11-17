L1 = 0.3;
M  = 0.2;
L2 = 0.8;
V  = 1; 

phi  = @(alpha,R1) atan2(M/abs(R1))*(alpha/abs(alpha))*-1; 
R1_f = @(alpha) L1/tan(alpha);
R2_F = @(phi)   M/sin(phi);



R2_f = @(alpha) abs(L1/tan(alpha));
R3_f = @(R2) sqrt(R2^2 + M^2 - L2^2);
beta_f = @(alpha,R2,R3) atan2(M,R2) + atan2(L2,R3);

A_fp = @(alpha,beta,psi) -(V/L1)*M*(tan(alpha)/L2)*(cos(beta-psi)/sin(psi));
B_fp = @(alpha,beta,psi)  (V/L1)*(1/cos(alpha)^2)*(1 - (M/L2)*(sin(beta - psi)/sin(psi)));

A_fn = @(alpha,beta,psi) -(V/L1)*M*(tan(alpha)/L2)*(cos(-beta-psi)/sin(psi));
B_fn = @(alpha,beta,psi) -(V/L1)*(1/cos(alpha)^2)*(1 - (M/L2)*(sin(-beta - psi)/sin(psi)));
% (sin(abs(beta) - abs(psi))/sin(abs(psi)))
alpha = 0.000;
alphas = -0.36:0.01:0.36;
% alphas = 0.01:0.01:0.2;
% alphas = -0.2:0.01:-0.01;
all_K = zeros(1,length(alphas));
% 
for i = 1:length(alphas)
    alpha = alphas(i);
    if alpha == 0
        continue
    end
    

%     alpha = -0.1
    R2  = R2_f(alpha);

    R3  = R3_f(R2);
    psi = abs(atan2(M,R2));
    
    beta = beta_f(alpha,R2,R3);
    if beta > pi
        beta = -(2*pi - beta);
    elseif beta < -pi
        beta = 2*pi + beta;
    end
    beta = beta*(alpha/abs(alpha))

    if alpha < 0
        A = A_fn(alpha,beta,psi)
        B = B_fn(alpha,beta,psi)
    else
        A = A_fp(alpha,beta,psi)
        B = B_fp(alpha,beta,psi)
    end

%     Q = 0.001;
%     R = 0.001;
      
%     K  = lqr(A,B,Q,R);
    all_K(i) = B;
end
plot(alphas,all_K)