function q = Csolve(q0, T)

[q, Cf] = fsolve(@(q)CC(q,T), q0);%, options);
disp(Cf);

q = mod(q,2*pi);
q(q>pi) = q(q>pi)-2*pi;
q(q<-pi) = q(q<-pi)+2*pi;
end

function C = CC(q, T)

b = 0;
l1x= 0.024645e3+0.055695e3-0.02e3; l1y=-0.25e3; l1z=0.118588e3+0.011038e3;
l2a=0.073e3;
l2b=0.245e3;
l3=0.102e3;
l4a=0.069e3;
l4b=0.26242e3-0.015e3;
l5=0.1e3;
l6a=0.01e3;
l6b=0.2707e3;
l7=0.16e3;
lEE=0.15e3;

r11 = T(1,1); r12 = T(1,2); r13 = T(1,3); t1 = T(1,4);
r21 = T(2,1); r22 = T(2,2); r23 = T(2,3); t2 = T(2,4);
r31 = T(3,1); r32 = T(3,2); r33 = T(3,3); t3 = T(3,4);

Cons;

end