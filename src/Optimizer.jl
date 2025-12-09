using JuMP
using Ipopt
using LinearAlgebra
using Plots

# MPC Parameters
struct MPCParams
    N::Int          # Prediction horizon
    dt::Float64     # Time step
    Q::Matrix       # State cost matrix
    R::Matrix       # Control cost matrix
    Qf::Matrix      # Terminal cost matrix
    v_max::Float64  # Max velocity constraint
end

function solve_mpc_ipopt(x0::Vector{Float64}, x_goal::Vector{Float64}, params::MPCParams)
    """
    Solve MPC optimization problem using Ipopt
    State: [x, y]
    Control: [vx, vy]
    Dynamics: x_k+1 = x_k + u_k * dt (Forward Euler)
    """
    
    N = params.N
    n_states = 2
    n_controls = 2
    
    # Create optimization model with Ipopt
    model = Model(Ipopt.Optimizer)
    set_optimizer_attribute(model, "print_level", 0)  # Reduce output verbosity
    set_optimizer_attribute(model, "max_iter", 200)
    set_optimizer_attribute(model, "tol", 1e-6)
    
    # Decision variables
    @variable(model, x[1:n_states, 1:N+1])  # States
    @variable(model, -params.v_max <= u[1:n_controls, 1:N] <= params.v_max)  # Controls with box constraints
    
    # Initial condition constraint
    @constraint(model, x[:, 1] .== x0)
    
    # Dynamics constraints (Forward Euler)
    for k in 1:N
        @constraint(model, x[1, k+1] == x[1, k] + u[1, k] * params.dt)
        @constraint(model, x[2, k+1] == x[2, k] + u[2, k] * params.dt)
    end
    
    # Optional: Add velocity magnitude constraint (circle constraint)
    # This creates a more interesting spiral behavior
    for k in 1:N
        @constraint(model, u[1, k]^2 + u[2, k]^2 <= params.v_max^2)
    end
    
    # Objective function
    obj = 0.0
    
    # Stage costs
    for k in 1:N
        # State cost (distance to goal)
        state_error = x[:, k] - x_goal
        obj += state_error' * params.Q * state_error
        
        # Control cost
        obj += u[:, k]' * params.R * u[:, k]
    end
    
    # Terminal cost
    terminal_error = x[:, N+1] - x_goal
    obj += terminal_error' * params.Qf * terminal_error
    
    @objective(model, Min, obj)
    
    # Solve
    optimize!(model)
    
    # Extract solution
    x_sol = value.(x)
    u_sol = value.(u)
    
    # Get optimization status
    status = termination_status(model)
    obj_val = objective_value(model)
    
    return x_sol, u_sol, status, obj_val
end

function simulate_mpc_closed_loop(x0::Vector{Float64}, x_goal::Vector{Float64}, params::MPCParams)
    """
    Run MPC in closed-loop (receding horizon)
    """
    
    x_current = copy(x0)
    trajectory = [copy(x_current)]
    controls = Vector{Float64}[]
    
    max_steps = 100
    goal_tolerance = 0.05
    
    println("Starting MPC with Ipopt solver...")
    println("─" ^ 50)
    
    for step in 1:max_steps
        # Solve MPC from current state
        x_pred, u_pred, status, obj_val = solve_mpc_ipopt(x_current, x_goal, params)
        
        if status != MOI.LOCALLY_SOLVED && status != MOI.OPTIMAL
            println("Warning: Optimization not optimal at step $step. Status: $status")
        end
        
        # Apply first control (receding horizon)
        u_applied = u_pred[:, 1]
        push!(controls, u_applied)
        
        # Simulate system with applied control (Forward Euler)
        x_current = x_current + u_applied * params.dt
        push!(trajectory, copy(x_current))
        
        # Print progress
        if step % 10 == 0 || step == 1
            dist = norm(x_current - x_goal)
            println("Step $step: Position = ($(round(x_current[1], digits=3)), $(round(x_current[2], digits=3))), Distance to goal = $(round(dist, digits=3))")
        end
        
        # Check convergence
        if norm(x_current - x_goal) < goal_tolerance
            println("─" ^ 50)
             println("✓ Goal reached at step $step")
            break
        end
    end
    
    return trajectory, controls
end

function plot_results(trajectory, controls, params)
    """
    Create visualization plots
    """
    
    # Extract data
    x_vals = [s[1] for s in trajectory]
    y_vals = [s[2] for s in trajectory]
    
    # Create trajectory plot
    p1 = plot(x_vals, y_vals,
              label="MPC trajectory",
              xlabel="X (m)",
              ylabel="Y (m)",
              title="Robot Path (Ipopt solver)",
              lw=3,
              color=:blue,
              aspect_ratio=:equal,
              grid=true,
              legend=:topright)
    
    # Add start and goal
    scatter!(p1, [1.0], [1.0], 
             label="Start", 
             color=:green, 
             markersize=10)
    scatter!(p1, [0.0], [0.0], 
             label="Goal", 
             color=:red, 
             markersize=10)
    
    # Add direction arrows
    n_arrows = min(10, length(trajectory) - 1)
    if n_arrows > 0
        arrow_idx = round.(Int, range(1, length(trajectory)-1, length=n_arrows))
        for i in arrow_idx
            dx = x_vals[i+1] - x_vals[i]
            dy = y_vals[i+1] - y_vals[i]
            quiver!(p1, [x_vals[i]], [y_vals[i]], 
                   quiver=([dx*0.3], [dy*0.3]),
                   color=:blue,
                   alpha=0.5,
                   label="")
        end
    end
    
    if !isempty(controls)
        # Control inputs plot
        vx_vals = [u[1] for u in controls]
        vy_vals = [u[2] for u in controls]
        v_magnitude = [norm(u) for u in controls]
        time_steps = (0:length(controls)-1) * params.dt
        
        p2 = plot(time_steps, vx_vals,
                  label="vx",
                  xlabel="Time (s)",
                  ylabel="Velocity (m/s)",
                  title="Control Inputs",
                  lw=2,
                  grid=true)
        plot!(p2, time_steps, vy_vals, label="vy", lw=2)
        plot!(p2, time_steps, v_magnitude, 
              label="|v|", 
              lw=2, 
              color=:green,
              linestyle=:dash)
        hline!(p2, [params.v_max, -params.v_max], 
               label="v_max", 
               color=:red, 
               linestyle=:dot)
        
        # Phase plot
        p3 = plot(vx_vals, vy_vals,
                  xlabel="vx (m/s)",
                  ylabel="vy (m/s)",
                  title="Velocity Phase Space",
                  label="Velocity trajectory",
                  lw=2,
                  color=:purple,
                  aspect_ratio=:equal,
                  grid=true)
        
        # Add constraint circle
        theta = range(0, 2π, length=100)
        plot!(p3, params.v_max * cos.(theta), params.v_max * sin.(theta),
              label="Velocity constraint",
              color=:red,
              linestyle=:dash)
        
        # Distance to goal over time
        distances = [norm(s - [0.0, 0.0]) for s in trajectory]
        p4 = plot((0:length(distances)-1) * params.dt, distances,
                  xlabel="Time (s)",
                  ylabel="Distance (m)",
                  title="Convergence",
                  label="Distance to goal",
                  lw=3,
                  color=:orange,
                  grid=true)
        hline!(p4, [0.05], 
               label="Goal tolerance", 
               color=:green, 
               linestyle=:dash)
        
        # Combine all plots
        plot(p1, p2, p3, p4, layout=(2, 2), size=(1000, 800))
    else
        display(p1)
    end
end

# Main execution
function main()
    # MPC parameters
    params = MPCParams(
        20,                        # N: prediction horizon
        0.1,                       # dt: time step
        diagm([10.0, 10.0]),      # Q: state cost
        diagm([0.1, 0.1]),        # R: control cost
        diagm([100.0, 100.0]),    # Qf: terminal cost
        1.5                        # v_max: maximum velocity
    )
    
    # Initial and goal positions
    x0 = [1.0, 1.0]
    x_goal = [0.0, 0.0]
    
    println("\n" * "="^50)
    println("MPC Robot Navigation with Ipopt")
    println("="^50)
    println("Initial position: $x0")
    println("Goal position: $x_goal")
    println("Prediction horizon: $(params.N)")
    println("Time step: $(params.dt) s")
    println("Max velocity: $(params.v_max) m/s")
    println()
    
    # Run closed-loop MPC simulation
    trajectory, controls = simulate_mpc_closed_loop(x0, x_goal, params)
    
    # Print summary
    println("\n" * "="^50)
    println("Simulation Summary")
    println("="^50)
    println("Final position: $(trajectory[end])")
    println("Final error: $(round(norm(trajectory[end] - x_goal), digits=4)) m")
    println("Total steps: $(length(trajectory) - 1)")
    println("Total time: $(round((length(trajectory) - 1) * params.dt, digits=1)) s")
    
    # if !isempty(controls)
    #     avg_speed = mean([norm(u) for u in controls]) 
    #     max_speed = maximum([norm(u) for u in controls])
    #     println("Average speed: $(round(avg_speed, digits=2)) m/s")
    #     println("Maximum speed: $(round(max_speed, digits=2)) m/s")
    # end
    
    # Create plots
    plot_results(trajectory, controls, params)
end

# # For package installation (run once)
# function install_packages()
#     using Pkg
#     Pkg.add("JuMP")
#     Pkg.add("Ipopt")
#     Pkg.add("LinearAlgebra")
#     Pkg.add("Plots")
# end

# Run the main simulation
main()