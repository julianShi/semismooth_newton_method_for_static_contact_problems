% semismooth Newton method for one dof contact problem. 
% Yulin 6 April 2016

%% -- set up the system coefficients
k=1;	% stiffness 
f=0;	% exteranal force
GAP=-1;	% the initial gap
epsilon=1;	% penalty coefficient
varepsilon=1e-2;	% tolerance

%% -- solve the nonlinear system equation
gPlus=@(u) max(0,u-GAP);	% penaltration
u=0;	% initial guess
for il=1:100
	func=@(u) k*u+epsilon*gPlus(u)-f;	% residual
	% -- solve with semismooth Newton method
	u=fsolve(func,u)	% solve
	% -- check convergence
	if(norm(gPlus(u)) < varepsilon)
		break
	end
	% -- update penalty coefficient
	epsilon=2*epsilon
end
lambda=-epsilon*gPlus(u)
