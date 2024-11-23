using KerbTk
using Test
using Aqua
using JET

@testset "KerbTk.jl" begin
    @testset "Code quality (Aqua.jl)" begin
        Aqua.test_all(KerbTk)
    end
    @testset "Code linting (JET.jl)" begin
        JET.test_package(KerbTk; target_defined_modules = true)
    end
    # Write your tests here.
end
