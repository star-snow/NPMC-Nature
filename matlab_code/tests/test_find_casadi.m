import casadi.*

x = MX.sym('x',4,1);

x(2,1) = 2;
found = find(x);