module MPCJulia
using ControlSystems
using LinearAlgebra
using StaticArrays



    #define the dynamics of the quadrotor 

    function continious_dynamics(x::Vector{Float64},u::Vector{Float64})

        #differentiation of the dynamics to define the physics of the system, how the system "behaves" and "evolves" over time,
        
        x_dot = u[1]*cos(x[3]) # velocity*direction of x position
        y_dot = u[1]*sin(x[3]) #velocity*direction of y position
        yaw_dot = u[2]           #angular velocity


        zeta_dot = [x_dot ; y_dot ; yaw_dot] #differentiation of the velocity is basically accelaration,which is omega
        #B = @SMatrix [] 
        return zeta_dot 
    end

    #For this module, we are keeping it nonlinear

    function discrete_dynamics(x::Vector{Float64}, u::Vector{Float64}, Ts::Float64)
        """
        We are using the forward Eular for the discritization:
            x[n+1] = x[n] + Ts * f(x[n],u[n])
        """
        zeta_dot = continious_dynamics(x,u)
        zeta_next = x + Ts * zeta_dot
        return zeta_next
    end



    # Forward Simulation is the technique which is giving it MPC formulation
    function ForwardSimulation()

        
    end

    mutable struct NonlinearMPCController
    # System parameters
    Ts::Float64          # sampling time
    N::Int              # prediction horizon
    nx::Int             # number of states
    nu::Int             # number of inputs
    
    # Cost matrices
    Q::Matrix{Float64}   # state cost
    R::Matrix{Float64}   # control cost
    
    
    # Constraints
    u_min::Vector{Float64}  # input lower bounds
    u_max::Vector{Float64}  # input upper bounds
 
    
    # Solver
    #model::Model
    end

    #Definition of the structure of the MPC controller    
    function NonlinearMPCController(x_ref, x, u, Ts)
        Ts = 0.1
        N=10
        nx, nu = 3,2 #number of states, number of control inputs

        Q = Diagonal([1.0, 1.0, 1.0])      # position and yaw angle weight 
        R = Diagonal([0.1, 0.1])           # control penalties
    
    
        # Default constraints bounds
        u_min = [-2.0, -π/2]  # [min velocity, min angular velocity]
        u_max = [2.0, π/2]    # [max velocity, max angular velocity]
        

        #Initial_state and control

        x0 = [0.0, 0.0, 0.0 ] #Initial state for the robot 

        #Objective function : total cost(x,u) = statecost(x) + controlcost(u)
        state_cost = 0.0
        control_cost = 0.0
        total_cost = 0.0

        #Constraints
        for i in 1:length(u)
            u_constrained[i] = clamp(u[i], u_min[i], u_max[i]) 
        end

        for k in 1:N

            for i in 1:nx
                state_cost+=  Q*transpose!(x_ref[i]-x[i])  #trans
            end
            for i in 1:nu
                control_cost+=  R * norm(u_constrained[i+1] - u_constrained[i]) 
            end

            total_cost += state_cost +control_cost
        end


        #Constraints



        #use the IPOPT solver to optimize
        
    end

    #Generate reference trajectory
    function referencetrajectory(x_initial::Vector{Float64}, N::Int, Ts::Int)
        x_ref = zeros(3,N+1)
        u_ref = zeros(2,N)

        ref_velocity = 2
        for i in 1:N
            x_ref[1, i] = x_initial[1]+ ref_velocity * (N-1) * Ts
            x_ref[2, i] = x_initial[2]+ ref_velocity * (N-1) * Ts
            x_ref[3, i] = x_initial[3]   
        end
        return x_ref
 
    end


    #Writing down a simple solver to see how controller works with optimization
    function TestFunction()
        Ts = 0.1
        N = 10

        #Initial parameters

        x_initial = [0.0, 0.0, 0.0] # Initial x_pos, y_pos and yaw 

        x_goal = [5.0, 3.0, 30]    #The end position for the 









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
