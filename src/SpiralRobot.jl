module CircularRobot


using LinearAlgebra
using Plots
using JuMP
using Ipopt

struct CircularMPC
    N::Int     #Finite Time Horizon
    Ts:: Float64    #Time step
    u_min::Float64  #minimum velocity
    u_max::Float64  #maximum velocity
    Q::Matrix       #state cost matrix
    R::Matrix       #control cost matrix
    Qf::Matrix      #terminal cost matrix
    decay_weight::Int
end

# function discrete_dynamics(x, u, Ts)
#     """
#     We are using the forward Eular for the discritization:
#         x[n+1] = x[n] + Ts * f(x[n],u[n])
#     """
#         x_next = zeros(2)
#         x_next[1] = x[1] + Ts *u
# end


function spiral_trajectory(x0, x_target,params )

    N= params.N
    states = 2
    control = 2


    model = Model(Ipopt.Optimizer)
    set_optimizer_attribute(model, "print_level", 0)
    set_optimizer_attribute(model, "max_iter", 300)
    set_optimizer_attribute(model, "tol", 1e-6)

    # Decision variables
    @variable(model, x[1:states, 1:N+1])  # States
    @variable(model, params.u_min <= u[1:control, 1:N] <= params.u_max)  # Controls

    println("x0: $x0")

    #Start position as the initial condition
    @constraint(model, x[:,1] .== x0)

    #As we are using the Ipopt, we are formulating the discrete dynamics as a constraint
    #Forward eular as Constraints

    for n in 1:N
        @constraint(model, x[1, n+1] == x[1,n]+ u[1,n] * params.Ts)
        @constraint(model, x[2, n+1] == x[2,n]+ u[1,n]* params.Ts)
    end
   
    for k in 1:N
        #objective function
        objective =0.0


        # Here we are using the LQR 


        #State cost
        error = x[:,k] - x_target #Here we are calculating the error based on how far it is from the traget point, as we did if we had a reference trajectory
        objective  += error' * params.Q * error


        #Control Cost
        objective += u[:,k]' *params.R *u[:,k]



        #Step 1: we need to make sure we are keeping the spiral. Here position and velocity are perpendicular. but for our robot to get into the middle of the spiral, we need to keep up with the momentum

        momentum = (((x[1,k]-x_target[1])*u[2,k]) - ((x[2,k]-x_target[2])*u[1,k]))
        #momentum = cross(x[:,k],u[:,k])
        objective -= 2.0*momentum                                                      
        #The reason it is negative is because it works as a reward function. As we are in a MPC formulation, our goal is to minimize the cost. We want the momentum value to be higher as it proceeds
        #towards the target, but the cost of our objective needs to be lower


        #Step 2: need to make sure that the radius decay is happening as the robot approaches the middle of the spiral(0,0)


        if k<N
            current_distance = sqrt((x[1,k]-x_target[1])^2 - (x[2,k]-x_target[2])^2)
            next_distance = sqrt((x[1,k+1]-x_target[1])^2 - (x[2,k+1]-x_target[2])^2)
            objective += params.decay_weight*(next_distance - 0.9 * current_distance)^2
        end

    end

    # Terminal cost
    terminal_error = x[:, N+1] - x_target
    objective += terminal_error' * params.Qf * terminal_error

    @objective(model, Min, objective)

    optimize!(model)


    x_val = value.(x)
    u_val = value.(u)
    status = termination_status(model)
    obj_val = objective_value(model)

    println("x_val: ($x_val")
    #println("u_val: ($u_val")

    return x_val,u_val,status,obj_val
    
end


function simulate_trajectory(x0, x_target, params)
    x_current = copy(x0)
    trajectory = [copy(x_current)]
    controls = Vector{Float64}[]
    
    max_steps = 100
    goal_tolerance = 0.0
    
    
    println("Starting Spiral MPC with Ipopt solver...")

    for step in 1:20

        
        x_pred,u_pred,status,objective_value = spiral_trajectory(x_current,x_target,params)

                
        # if status != MOI.LOCALLY_SOLVED && status != MOI.OPTIMAL
        #     println("Warning: Status = $status at step $step")
        # end


        #Make the control sequence that you want to Apply
                # Apply first control
            u_applied = u_pred[:, 1]
            push!(controls, u_applied)
            
            # Simulate system
            x_current = x_current + u_applied * params.Ts
            push!(trajectory, copy(x_current))
            
            # Print progress
            if step % 10 == 0 || step == 1
                r = norm(x_current - x_target)
                theta = atan(x_current[2] - x_target[2], x_current[1] - x_target[1])
                println("Step $step: r = $(round(r, digits=3)), θ = $(round(rad2deg(theta), digits=1))°")
            end
            
            # Check convergence
            if norm(x_current - x_target) < goal_tolerance
                println("─" * 50)
                println("✓ Goal reached at step $step!")
                break
            end

        end        
        return trajectory,controls
    
end

# 

function plot_spiral_results(trajectory, controls, params, x_goal)
    """
    Create visualization plots for spiral trajectory
    """
    

    
    # Extract data
    x_vals = [s[1] for s in trajectory]
    y_vals = [s[2] for s in trajectory]
    
    # Main trajectory plot
    p1 = plot(x_vals, y_vals,
              label="Spiral path",
              xlabel="X (m)",
              ylabel="Y (m)",
              title="Spiral MPC Trajectory)",
              lw=3,
              color=:blue,
              aspect_ratio=:equal,
              grid=true)
    
    # Add start and goal
    scatter!(p1, [trajectory[1][1]], [trajectory[1][2]], 
             label="Start", 
             color=:green, 
             markersize=10)
    scatter!(p1, [x_goal[1]], [x_goal[2]], 
             label="Goal", 
             color=:red, 
             markersize=10)
    
    # # Add spiral reference circles
    # for r in [0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4]
    #     circle_theta = range(0, 2π, length=100)
    #     plot!(p1, x_goal[1] .+ r * cos.(circle_theta), 
    #           x_goal[2] .+ r * sin.(circle_theta),
    #           color=:gray,
    #           alpha=0.2,
    #           linestyle=:dash,
    #           label="")
    # end
    

        plot!(p1, size=(800, 400))
end




function main()
parameters = CircularMPC(
    10,
    0.5,
    0.2,
    1.0,
    diagm([5.0, 5.0]),       
    diagm([0.01, 0.01]), 
    diagm([100.0, 100.0]),
    10.0
)

#Initial and Goal position
x0 = [1.0,1.0]
x_goal = [0.0,0.0]

println("\n" * "="^50)
println("Spiral MPC Robot Navigation with Ipopt")
println("="^50)
println("Initial position: $x0")
println("Goal position: $x_goal")
println("Initial radius: $(round(norm(x0 - x_goal), digits=3)) m")
println()

trajectory,control = simulate_trajectory(x0,x_goal,parameters)



# Create plots
plot_spiral_results(trajectory, control, parameters, x_goal)

# Print summary
println("\n" * "="^50)
println("Simulation Summary")
println("="^50)
println("Final position: $(trajectory[end])")
println("Final error: $(round(norm(trajectory[end] - x_goal), digits=4)) m")
println("Total steps: $(length(trajectory) - 1)")
println("Total time: $(round((length(trajectory) - 1) * parameters.Ts, digits=1)) s")




    
end

    
end