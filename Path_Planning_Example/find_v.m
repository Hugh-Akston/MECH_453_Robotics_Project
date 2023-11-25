%% Author Information
% Written By: Dr. Eric Markvicka
% Modified By: Luke Freyhof
% Date: November 25, 2023

function [outputArg1,outputArg2] = find_v(l1,l2,t1_val,t2_val)

syms t1 t2 l1 l2 'real'

c = @cos; 
s = @sin; 

% FK
x = l1*c(t1) + l2*c(t1+t2);
y = l1*s(t1) + l2*s(t1+t2);

%% analytic Jacobian
J = [diff(x,t1) diff(x,t2);
     diff(y,t1) diff(y,t2)];

%%
detJ = real(simplify(det(J)));

Jval = subs(J,[l2 l3],[1 1]);
eqn1 = real(simplify(det(Jval))) == 0;

S_t1 = simplify(solve(eqn1, t1,'Real',true))
S_t2 = simplify(solve(eqn1, t2,'Real',true))
S_t3 = simplify(solve(eqn1, t3,'Real',true))

%%
eqn2 = detJ == 0;

S_t2 = simplify(solve(eqn2, t2,'Real',true))
S_t3 = simplify(solve(eqn2, t3,'Real',true))


end