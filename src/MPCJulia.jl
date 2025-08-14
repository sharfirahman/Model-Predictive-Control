module MPCJulia
using ControlSystems
using LinearAlgebra
using StaticArrays



    #define the dynamics of the quadrotor 

    function continious_dynamics(x::Vector{Float64},u::Vector{Float64})

        #differentiation of the dynamics to define the physics of the system, how the system "behaves" and "evolves" over time,

        x.x_dot = u[1]*cos(yaw[1]) # velocity*direction of x position
        x.y_dot = u[1]*sin(yaw[1]) #velocity*direction of y position
        x.yaw_dot = u[2]           #angular velocity


        zeta_dot =@SMatrix [x_dot ; y_dot ; yaw_dot] #differentiation of the velocity is basically accelaration,which is omega
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
    model::Model
    end

    #Definition of the structure of the MPC controller    
    function NonlinearMPCController(x_ref, x, u, Ts)
        Ts = 0.1
        N=10
        nx, nu = 3,2 #number of states, number of control inputs

        Q = Diagonal([1.0, 1.0, 1.0])      # position and yaw angle weight 
        R = Diagonal([0.1, 0.1])           # input penalties
    
    
        # Default constraints bounds
        u_min = [-2.0, -π/2]  # [min velocity, min angular velocity]
        u_max = [2.0, π/2]    # [max velocity, max angular velocity]


        #Objective
        state_cost = 0.0
        control_cost = 0.0
        total_cost = 0.0



        for k in 1:N

            for i in 1:nx
                state_cost+=  Q*(x_ref[i]-x[i])  #trans
            end
            for i in 1:nu
                control_cost+=  R * (u[i+1] - u[i]) 
            end

            total_cost += state_cost +control_cost
        end
        #use the IPOPT solver to optimize
        
    end


    function referencetrajectory()
        
    end

    


    #state vector - [position x, position y , yaw angle]

    # x = [0, 1.5, 2.5, 3.5]
    # y = [0, 0.5, 3.5, 4.5]
    # yaw = [30, 20, 10, 25]

    # #Control input - angular velocity
    # v = [1.1, 5.9, 4.5, 7.9] #constant linear velocity
    # omega = [1.0, 2.5, 3.2, 4.5] #constant angular velocity



    zeta =@SMatrix [x ; y ; yaw]

    #We need to discretize the dynamics from the continious Time
    Ts = 0.1  #sample time
    N = 10 #Optimization Horizon
    
end


#Testing out how the solver with a really simple problem 
