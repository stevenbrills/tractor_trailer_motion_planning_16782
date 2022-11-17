L1 = 0.3;
M  = 0.2;
L2 = 0.8;
V  = 1; 

R2_f = @(alpha) abs(L1/tan(alpha));
R3_f = @(R2) sqrt(R2^2 + M^2 - L2^2);
beta_f = @(alpha,R2,R3) pi - atan2(M,R2) - atan2(L2,R3);

alphas = -0.3695:0.001:0.3695;
% alphas = 0.01:0.01:0.2;
% alphas = -0.2:0.01:-0.01;
all_K = zeros(1,length(alphas));
% 
for i = 1:length(alphas)
    alpha = alphas(i);
    if alpha == 0
%         al = [];
        all_K(i) = K;
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
    beta = beta*(alpha/abs(alpha));

%     if alpha < 0
%         A = A_fn(alpha,beta,psi)
%         B = B_fn(alpha,beta,psi)
%     else
    A = get_A(V,L1,L2,M,alpha,beta);
    B = get_B(V,L1,L2,M,alpha,beta);

    Q = 0.001;
    R = 0.001;
      
    K  = lqr(A,B,Q,R);
    all_K(i) = K;
end
plot(alphas,all_K)