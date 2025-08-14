using ControlSystemsBase
using LinearAlgebra # For identity matrix I
using Plots

# Create system
Ts      = 0.1
A       = [1 Ts; 0 1]
B       = [0; 1]
C       = [1 0]
sys     = ss(A,B,C,0,Ts)

# Design controller
Q       = I # Weighting matrix for state
R       = I # Weighting matrix for input
L       = lqr(Discrete,A,B,Q,R) # lqr(sys,Q,R) can also be used

# Simulation
u(x,t)  = -L*x .+ 1.5(t>=2.5) # Form control law (u is a function of t and x), a constant input disturbance is affecting the system from t≧2.5
t       = 0:Ts:5              # Time vector
x0      = [1,0]               # Initial condition
res = lsim(sys,u,t,x0=x0)
plot(res, lab=["Position" "Velocity"], ploty=false, plotx=true, layout=1, sp=1)

#  save_docs_plot("lqrplot.svg"); # hide