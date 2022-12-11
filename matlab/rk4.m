function xnext = rk4(f, x, h)

    xnext = zero(x);
    
    k1 = f(x);
    k2 = f(x + 0.5*h*k1);
    k3 = f(x + 0.5*h*k2);
    k4 = f(x + h*k3);
    
    xnext = x + (1/6)*(k1 + 2*k2 + 2*k3 + k4)*h;
    
end