using JuMP, Ipopt, Test

# model = Model(Ipopt.Optimizer)
# set_silent(model)

# @variables(model,begin
# x[i=1:3]
# u[i=1:2]
# t[i=1:10]
# end)
# function square_test(x,t)
    
#         sqrt((x[1])^2-(x[1])^2)
    
# end

# @objective(model,Min, square_test(4,2))

# function my_callback(
#     alg
# )
    
# end

model = Model(Ipopt.Optimizer)
#set_silent(model)
@variable(model, x >= 1)
@objective(model, Min, x + 0.5)
x_vals = Float64[]
function my_callback(
   alg_mod::Cint,
   iter_count::Cint,
   obj_value::Float64,
   inf_pr::Float64,
   inf_du::Float64,
   mu::Float64,
   d_norm::Float64,
   regularization_size::Float64,
   alpha_du::Float64,
   alpha_pr::Float64,
   ls_trials::Cint,
)
   push!(x_vals, callback_value(model, x))
  
   @test isapprox(obj_value, 1.0 * x_vals[end] + 0.5, atol = 1e-1)
   # return `true` to keep going, or `false` to terminate the optimization.
   return iter_count < 1
end
MOI.set(model, Ipopt.CallbackFunction(), set_optimizer_attribute(model, "max_iter", 1))
println("this======",Ipopt.CallbackFunction())
optimize!(model)
# @test MOI.get(model, MOI.TerminationStatus()) == MOI.INTERRUPTED
# @test length(x_vals) == 2
