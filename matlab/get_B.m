function B = get_B(V,L1,L2,M,alpha,beta)
%get_B
%    B = get_B(V,L1,L2,M,ALPHA,BETA)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    12-Nov-2022 16:24:27

t2 = imag(M);
t3 = real(M);
t4 = abs(L1);
t5 = abs(alpha);
t6 = sign(alpha);
t7 = tan(alpha);
t11 = 1.0./L1;
t12 = 1.0./L2;
t16 = M.*1i;
t8 = abs(t7);
t9 = sign(t7);
t10 = t7.^2;
t13 = t3.^2;
t14 = 1.0./t5;
t15 = t14.^2;
t17 = t10+1.0;
t18 = 1.0./t8;
t19 = t18.^2;
t20 = t4.*t18;
t21 = -t20;
t23 = t16+t20;
t22 = t2+t21;
t24 = angle(t23);
t25 = t22.^2;
t27 = t14.*t24;
t32 = alpha.*t6.*t15.*t24;
t26 = t13+t25;
t28 = alpha.*t27;
t35 = -t32;
t29 = sin(t28);
t30 = 1.0./t26;
t31 = beta+t28;
t33 = sin(t31);
t34 = 1.0./t29;
t36 = alpha.*t3.*t4.*t9.*t14.*t17.*t19.*t30;
t37 = t27+t35+t36;
B = -V.*t11.*t17+M.*V.*t11.*t12.*t17.*t33.*t34+M.*V.*t7.*t11.*t12.*t34.*t37.*cos(t31)-M.*V.*t7.*t11.*t12.*t33.*t34.^2.*t37.*cos(t28);
