module MPCJulia
using ControlSystems
using LinearAlgebra
using StaticArrays
using JuMP
using Plots
using Ipopt


#define the dynamics of the quadrotor 

function continious_dynamics(x::Vector{Float64}, u::Vector{Float64})

    #differentiation of the dynamics to define the physics of the system, how the system "behaves" and "evolves" over time,

    x_dot = u[1] * cos(x[3]) # velocity*direction of x position
    y_dot = u[1] * sin(x[3]) #velocity*direction of y position
    yaw_dot = u[2]           #angular velocity


    zeta_dot = [x_dot; y_dot; yaw_dot] #differentiation of the velocity is basically accelaration,which is omega
    #B = @SMatrix [] 
    return zeta_dot
end

#For this module, we are keeping it nonlinear

function discrete_dynamics(x::Vector{Float64}, u::Vector{Float64}, Ts::Float64)
    """
    We are using the forward Eular for the discritization:
        x[n+1] = x[n] + Ts * f(x[n],u[n])
    """
    zeta_dot = continious_dynamics(x, u)
    zeta_next = x + Ts * zeta_dot
    return zeta_next
end





#Definition of the structure of the MPC controller  
mutable struct NonlinearMPCController
    # System parameters
    Ts::Float64          # sampling time
    N::Int              # prediction horizon
    nx::Int             # number of states
    nu::Int             # number of inputs

    # Cost matrices for the objective function
    Q::Matrix{Float64}   # state cost
    R::Matrix{Float64}   # control cost


    # Constraints
    u_min::Vector{Float64}  # input lower bounds
    u_max::Vector{Float64}  # input upper bounds


    # Solver
    model::Model
end

function referencetrajectory(x_target, N, Ts)
    x_ref = zeros(3, N + 1)
    u_ref = zeros(2, N)

    ref_velocity = 0.2
    for i in 1:(N+1)
        x_ref[:, i] = x_target .+ ref_velocity * Ts

        #println("time $i = $x_ref\n")
        x_target = x_ref[:, i]
    end

    x_axis = range(0, 10, length=100)
    y_axis = x_ref
    plot(x_axis, y_axis[:, 1])
    return x_ref

end


#Here, we are designing the MPC controller based on JuMP
#As we are using the IpOpt solver which uses the modeling language JuMP; 
#we are defining the parameters and variables as per JuMP syntax for the convention of the solver
#function NonlinearMPCController(x_ref,x_initial,N, Ts)
function NonlinearMPCController()
    N=10
    Ts=0.1
    nx, nu = 3, 2 #number of states, number of control inputs

    Q = Diagonal([1.0, 1.0, 1.0])      # position and yaw angle weight 
    R = Diagonal([0.01, 0.01])           # control penalties


    # Default constraints bounds for control
    u_min = [-2.0, -π / 2]  # [min velocity, min angular velocity]
    u_max = [2.0, π / 2]    # [max velocity, max angular velocity]
    x_initial = [0.0, 0.0, 0.0]
    x_target = [5.0, 3.0, pi/4]  
    x_ref = referencetrajectory(x_target,N,Ts)
    #Initial_state and control

    #First, we need to initialize the solver model that we are using.
    #We will be using it when we are defining the variables and constraints
    #to point which solver model we are using.
    model = Model(Ipopt.Optimizer)
    #set_optimizer_attribute(model, "print_level", 0)
    set_optimizer_attribute(model, "max_iter", 20)

    #Define the variables in the JuMP formulation

    @variable(model, x[1:nx, 1:N+1]) # states (columns) x timesteps(rows); N+1 because we are always predicting the next time step
    @variable(model, u[1:nu, 1:N])


    #The initial state of the robot is being sent form the main function where we are defining the initial position
    # and the target position that the robot needs to go to

    @constraint(model, x[:,1] .== x_initial)

    #Note: there cannot be any space after constraints!!!

    #We need to declare all the input as a constraint, which is basically a JuMP convention;
    #As we already have the discritized verison of the dynamics, we need to put it in a loop
    #and treat it as a constraint with the variables that we have in the controller function
    for n in 1:N
        @constraint(model, x[1,n+1] == x[1,n] +Ts * u[1,n] * cos(x[3,n]) )  #In the dynamics discritization, we are basically calculating the next state
        @constraint(model, x[2,n+1] == x[2,n] +Ts * u[1,n] * sin(x[3,n]) )
        @constraint(model, x[3,n+1] == x[3,n] +Ts * u[2,n])
        @constraint(model, u_min[1] <= u[1,n] <= u_max[1])
        @constraint(model, u_min[2] <= u[2,n] <= u_max[2])
    end
    


    #x0 = [0.0, 0.0, 0.0] #Initial state for the robot 




    #Objective function : total cost(x,u) = statecost(x) + controlcost(u)
    state_cost = 0.0
    control_cost = 0.0
    total_cost = 0.0

    # #Constraints
    # for i in 1:N
    #     u_constrained[i] = clamp(u[i], u_min[i], u_max[i])
    # end

    #We have the difference between the reference and the current state as a L2 norm squared 
    #We have taken the diagonal weighted matrix for the state and control cost, which makes it easier to track the error for 
    #the different parts of the state(x_pos,y_pos,yaw)

    for k in 1:N

        for i in 1:nx
            state_cost += Q[i,i] * (x_ref[i,k] - x[i,k])^2  
        end
        for i in 1:nu
            control_cost += R[i,i] * (u[i+1] - u[i])^2
        end

        total_cost += state_cost + control_cost
    end

    #Objective function
    @objective(model,Min,total_cost)
    optimize!(model)
    solution_summary(model)

    println("""
    termination_status = $(termination_status(model))
    primal_status      = $(primal_status(model))
    objective_value    = $(objective_value(model))
    """)
    assert_is_solved_and_feasible(model)

    return

    #use the IPOPT solver to optimize

end




#Generate reference trajectory which has a constant target


# function plotting(x_ref, Ts, N)
#     x_pos = x_ref[1, :]
#     y_pos = x_ref[2, :]
#     yaw = x_ref[3, :]
#     time = 0:Ts:N*Ts


#     plot(time, x_pos, linewidth=2, color=:blue, label="x(t)")
#     plot!(time, y_pos, linewidth=2, color=:red, label="y(t)")
#     plot!(time, yaw, linewidth=2, color=:green, label="yaw(t)")


#     xlabel!("Time(s)")
#     ylabel!("State")
#     title!("Reference Trajectory")




# end
#Writing down a simple solver to see how controller works with optimization
function TestFunction()

    println("Examples!!!!")
    Ts = 0.1
    N = 10

    #Initial parameters



    x_initial = [0.0, 0.0, 0.0] # Initial x_pos, y_pos and yaw 

    x_target = [5.0, 3.0, pi/4]    #The end position for the 


    x_ref = referencetrajectory(x_target,N,Ts)
    
    optimization = NonlinearMPCController(x_ref,x_initial,N, Ts)
    println("Optimization result = $optimization")


end



#state vector - [position x, position y , yaw angle]

# x = [0, 1.5, 2.5, 3.5]
# y = [0, 0.5, 3.5, 4.5]
# yaw = [30, 20, 10, 25]

# #Control input - angular velocity
# v = [1.1, 5.9, 4.5, 7.9] #constant linear velocity
# omega = [1.0, 2.5, 3.2, 4.5] #constant angular velocity



#zeta =@SMatrix [x ; y ; yaw]

#We need to discretize the dynamics from the continious Time
#Ts = 0.1  #sample time
#N = 10 #Optimization Horizon

end


#Testing out how the solver with a really simple problem 
