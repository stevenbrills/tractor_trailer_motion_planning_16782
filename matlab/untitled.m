
syms V L1 L2 M alpha beta psi

R1  = L1/tan(alpha);
psi = -atan2(M,abs(R1))*(alpha/abs(alpha));
R2  = M/sin(psi);


% beta = pi - (atan2(M,R1) + asin(L2,R2)); 
R3   = L2/sin(psi-beta);

w1 = V/R1;
w2 = (V/R1)*(R2/R3);
beta_dot = (w2 - w1); 

A = diff(beta_dot,beta);
B = diff(beta_dot,alpha);

% A = A;
% B  = 1/(L1*cos(alpha)^2);
% B = (tan(alpha)^2 + 1)/L1;
% B = (M*sin(psi-beta))/(L2*sin(psi));
% B = ((tan(alpha)^2 + 1)/L1)*(((M*sin(psi-beta))/(L2*sin(psi))) - 1);
% B = (1/(L1*cos(alpha)^2))*((M*sin(psi-beta))/(L2*sin(psi)) - 1);
% B = B;


matlabFunction(A,'File','get_A','Vars',{V L1 L2 M alpha beta});
matlabFunction(B,'File','get_B','Vars',{V L1 L2 M alpha beta});


