function qp=qprod(q,p) %quaternion product
a=q(1);
v=reshape(q(2:4),3,1);
x=p(1);
u=reshape(p(2:4),3,1);

qp = [a*x-v'*u, (a*u+x*v)' + cross(v,u)'];