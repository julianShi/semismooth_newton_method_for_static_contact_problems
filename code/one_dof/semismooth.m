% semismooth Newton method for one dof contact problem. 
% Yulin 6 April 2016

% set up the system coefficients
k=1;	% stiffness 
f=0;	% exteranal force
GAP=-1;	% the initial gap
c=1;	% arbitrary positive constant
% -- the nonlinear system equation
func=@(u,lambda) [k*u-lambda-f;
	lambda+max(0,c*(u-GAP)-lambda)]
funcX=@(x) func(x(1),x(2))
% -- solve with semismooth Newton method
x=zeros(2,1);	% initial guess
x=fsolve(funcX,x);	% solve
u=x(1)
lambda=x(2)
