using Plots
gr()  # Use GR backend instead of unicodeplots

function animate_functions()
    x_data = Float64[]
    sin_data = Float64[]
    cos_data = Float64[]
    
    anim = Animation()
    
    for x in range(0, stop = 2π, length = 50)
        push!(x_data, x)
        push!(sin_data, sin(x))
        push!(cos_data, cos(x))
        
        p = plot(x_data, sin_data, 
                color = :blue,
                linewidth = 2,
                xlims = (0, 2π), 
                ylims = (-1.2, 1.2),
                title = "Sin and Cos Animation",
                xlabel = "x", 
                ylabel = "y",
                legend = false)
        
        plot!(x_data, cos_data, 
              color = :red,
              linewidth = 2)
        
        frame(anim)
    end
    
    return anim
end

anim = animate_functions()
gif(anim, "trig_functions.gif", fps = 10)
