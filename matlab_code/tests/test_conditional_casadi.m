import casadi.*
e_index = SX.sym('index');

x = SX.sym('x');

v = [x^2;x^3];
v_cell = vertsplit(v);

default = 0;
c = conditional(e_index,v_cell,default,false); 
% short_circuit true: not supported


% Test 
f = Function('f',{e_index,x},{c});


f(0,3) % 3^2
f(1,3) % 3^3
f(0.5,3) % default
f(-1,3) % default
f(2,3) % default