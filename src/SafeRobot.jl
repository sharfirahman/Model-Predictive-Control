module SafeRobot

using LinearAlgebra
using Ipopt
using JuMP 
using Plots


struct RobotParameters
    N::Int          #Finite Time Horizon
    Ts:: Float64    #Time step
    u_min::Vector{Float64}  # input lower bounds
    u_max::Vector{Float64}  # input upper bounds
    Dist::Float64   #Safety distance from the actor
    target_position::Vector{Float64}
 end


 function RobotDynamics(state,control,dt)
    
    x,y = state
    vx,vy = control
    return [x+vx*dt,y+vy*dt]
 end

 function SafeTrajectory(x_current,u_current,RobotParameters)

    
    
    N= RobotParameters.N
    states = 2
    control = 2
    radius = 1.0
    cx,cy = 0.0,0.0
    


    model = Model(Ipopt.Optimizer)
    set_optimizer_attribute(model, "print_level", 0)
    set_optimizer_attribute(model, "max_iter", 300)
    set_optimizer_attribute(model, "tol", 1e-6)

    #Decision_Variables
    @variable(model, x[1:states,1:N+1])
    @variable(model, -RobotParameters.u_min[i]<=u[i=1:control,1:N]<=RobotParameters.u_max[i])

    #Variable Constraints



    #Initial condition is sent from the current state from the main
    @constraint(model, x[:,1] == x_current)
    #@constraint(model, u[:,1] == u_current)
   # @constraint(model, -RobotParameters.u_min[1] <= u[1:N] <= RobotParameters.u_max[1])
   # @constraint(model, -RobotParameters.u_min[2] <= u[2:N] <= RobotParameters.u_max[2])
    #@constraint(model, RobotParameters.Dist == 0.5)  #safety constraints


    #System Dynamics
    for k in 1:N
        @constraint(model, x[1,k+1] == x[1,k] + RobotParameters.Ts*u[1,k])  #with vel_x
        @constraint(model, x[2,k+1] == x[2,k] + RobotParameters.Ts*u[2,k])  #with vel_y
    
        
    end

    # for k in 2:N

        #velocity constraint

        #@constraint(model,u[1,k] == u[1,k-1]*pi/2)

        #Circle constraint
        #@constraint(model, x[1,k]^2 + x[2,k]^2 == radius^2)

    # end 


    #Safety constraint

    #target_position = RobotParameters.target_position
    # minimum_distance = RobotParameters.Dist * 0.7 #we want at least 70% of the distance

    # for k in 1:N 
    #     @constraint(model,((x[1,k]-target_position[1])^2 - (x[1,k]-target_position[1])^2) >= minimum_distance^2 ) #Calculating Euclidean distance
        
    # end

    #Calculating the cost for the MPC formulation

    #circle center to rotate around
    

    cost = 0.0

    for k in 1:N

        #Robot Distance from the center
        error_position_x = x[1,k]-cx
        error_position_y = x[2,k]-cy

        #Perpendicular Velocity
        circular_velocity = u[1,k]*error_position_x + (u[2,k]*error_position_y)
        cost += circular_velocity^2

        #approaching and staying on the circle cost
        distance_from_circle = error_position_x^2 + error_position_y^2
        distance_cost = (distance_from_circle - radius)^2 #circular cost calculation
        #distance_cost = 
        cost +=  distance_cost


        #control cost
        control_cost =u[1,k]^2 + u[2,k]^2
        cost += control_cost

        #For clockwise motion
        clockwise_motion = (u[1,k]*error_position_y-u[2,k]*error_position_x) + pi/2
        cost -= clockwise_motion
        

    #     #here we are calulating the motion cost around the circle
    # #     if distance_from_circle == radius
    #         #position_angle =  atan(error_position_y,error_position_x)
    #         position_angle = atan(x[2,k],x[1,k]) +pi/2
            

    #         desired_velocity_angle = atan(error_distance_cost,RobotParameters.Ts)  #this basically tells the robot which way to go

    #         angle_error = desired_velocity_angle - position_angle
    #         #angle_error = atan(sin(angle_error), cos(angle_error))
    #         angle_cost = 50*angle_error^2

    #         cost += angle_cost
    # #    end

    #     cost += distance_cost  
    end



       """
        if phase == "straight"
            distance_error = distance_cost -RobotParameters.Dist
            cost += distance_error^2
        
        elseif phase == "circle"
            #maintaining the distance
            distance_error = distance_cost -RobotParameters.Dist
            cost += distance_error^2

            if distance_error > 1e-6
                direction_x = current_position_x / distance_cost
                direction_y = current_position_y / distance_cost
                rotation_x = - direction_y
                rotation_y = direction_x


                #we need a speed that will keep the robot in the circular Trajectory
                desired_vel = 1.5 #We are assuming this will keep the robot in the circular trajectory
                desired_velocity_x = rotation_x * desired_vel
                desired_velocity_y = rotation_y * desired_vel

                #This is where we are calculating the control error, which keeps the robot in a speed to keep into the circle
                velocity_error_x = u[1,k] - desired_velocity_x
                velocity_error_y = u[2,k] - desired_velocity_y

                cost += velocity_error_x^2 +velocity_error_y^2

                #Circle constraint, it keep the velocity with direction the same and the robot in the circle trajectory

                cost+= 0.5 * (u[1,k]*direction_x + u[2,k]*direction_y)^2

            end

            

        end
        """



        



    

    #Terminal Cost

    # terminal_position_x = x[1,N] - target_position[1]
    # terminal_position_y = x[1,N] - target_position[2]
    # terminal_position = sqrt(terminal_position_x^2 + terminal_position_y^2)
    # terminal_error = terminal_position - RobotParameters.Dist
    # cost += terminal_error^2

    @objective(model,Min,cost)



    optimize!(model)

        # if termination_status(model) == MOI.LOCALLY_SOLVED || termination_status(model) == MOI.OPTIMAL
        # pos_opt = value.(x)
        # vel_opt = value.(u)
        # return pos_opt, vel_opt, true
        # else
        #     println("MPC solver failed: ", termination_status(model))
        #     return nothing, nothing, false
        #  end

    solution_summary(model)

    println("""
    termination_status = $(termination_status(model))
    primal_status      = $(primal_status(model))
    objective_value    = $(objective_value(model))
    """)
    assert_is_solved_and_feasible(model)
    pos_opt = value.(x)
    vel_opt = value.(u)
    println("pos_opt: $pos_opt")
    println("u_opt: $vel_opt")

    return pos_opt, vel_opt
 end
    
    # function determine_phase(current_pos, target_pos, safe_distance, time_in_circle)
    #     """Determine whether robot should be approaching or circling"""
    #     distance_to_target = norm(current_pos[1:2] - target_pos[1:2])  
        
    #     if distance_to_target > safe_distance * 1.15  # 15% buffer for approach
    #         return "approach"
    #     elseif time_in_circle < 25.0  # Circle for 25 seconds
    #         return "circle"
    #     else
    #         return "finished"
    #     end
    # end




function main()

    params = RobotParameters(
    10,          #Finite Time Horizon
    0.1,    #Time step
    [2.0,2.0],  #minimum velocity
    [5.0,5.0],  #maximum velocity
    2.0,  #Safety distance from the actor
    [2.5,2.5] #target position
    )

    current_position = [2.0,3.5]
    current_velocity = [0.0,0.0]
    trajectory = Vector{Vector{Float64}}()
    trajectory_1 = Float64[]
    trajectory_2 = Float64[]
    
    target_position = [5.0,5.0]
    safe_distance = 2.0
    time_vec = Float64[]
    
    # Simulation parameters
    sim_time = 30.0
    max_sim_time = 10.0
    t = 0.0
    
    #current_phase = "approach"


    # println("Starting reference-free aerial robot simulation...")
    # println("Initial position: $current_position")
    # println("Target position: $(params.target_position)")
    # println("Safe distance: $(safe_distance)")
    # println("Using reference-free MPC formulation")

    # while sim_time < max_sim_time

    #     #Determine current phase
    #     which_phase = determine_phase(current_position,params.target_position,params.safe_distance,time_in_circle)

    #     if which_phase == "finished"
    #         println("Circle done!!")
    #         break
    #     end

    #     if which_phase != current_phase
    #         println("Phase transition: $current_phase -> $which_phase at time $(round(sim_time, digits=2))s")
    #         if which_phase == "circle"
    #             time_in_circle = 0.0
    #         end
    #         current_phase = which_phase
    #     end
        
    # end

    #Time to solve the MPC formulation!!

    # phase_time = (current_phase == "circle") ? time_in_circle : 0.0
    #println("Current_position $current_position")

    
    #pos_opt, vel_opt = SafeTrajectory(current_position,current_velocity,params)

    #for t in 0:params.Ts:params.N
    while t<sim_time

        pos_opt, vel_opt = SafeTrajectory(current_position,current_velocity,params)
        

        #applied_velocity = vel_opt[:,1]

            # current_position = current_position + applied_velocity*params.Ts
            # current_velocity = applied_velocity

        current_position = RobotDynamics(current_position,vel_opt,params.Ts)


        push!(trajectory_1,current_position[1,end])
        push!(trajectory_2,current_position[2,end])
        push!(time_vec,t)
        t +=params.Ts
   
      
        
        

        

       
        
    end
    
    println("x_points: $trajectory_1\n")
    println("x_points: $trajectory_2\n")
   
    # println("time: $time_vec")
    # point = trajectory[N]
    # x_point = point[1]
    # y

    #x_points = getindex.(current_position, 1)
    #y_points = getindex.(current_position, 2)

    #println("x_points: $x_points")
    
    plt = plot(
        # #trajectory_1, 
        # pos_opt[1,:],
        # pos_opt[2,:],
        # #x_points,
        # #y_points,
        # linewidth=2,
        # legend =nothing,
        title ="Circular Trajectory",
        xlabel = "x_points",
        ylabel = "y_points",
        #zlabel = "time",
        aspect_ratio=:equal
        )

    plot!(plt,
        trajectory_1,
        trajectory_2,
        #time_vec,
        color=:blue,
        label="Robot Trajectory"
    )

    #scatter!(plt, [trajectory_1[1],trajectory_2[1]], color=:red,markersize=8,label="")
    
    
    display(plt)
    
end
end