module FFSTypes

using MsgPack: MsgPack
using StaticArrays: SVector

using ..Arenas: Arena, AbstractId
using ..Math: H1

struct ResourceId <: AbstractId
    inner::UInt64
end

Base.convert(::Type{ResourceId}, x::UInt64) = ResourceId(x)
Base.convert(::Type{UInt64}, x::ResourceId) = x.inner
MsgPack.msgpack_type(::Type{ResourceId}) = MsgPack.StructType()

mutable struct Resource
    free::Bool
    max_amount::Float64
    amount::Float64
    density::Float64
    residual::Float64
    enabled::Bool
    name::String
end

MsgPack.msgpack_type(::Type{Resource}) = MsgPack.StructType()

@enum FlowMode begin
    NoFlow
    AllVessel
    StagePriorityFlow
    StackPrioritySearch
    AllVesselBalance
    StagePriorityFlowBalance
    StageStackFlow
    StageStackFlowBalance
    Null
end

MsgPack.msgpack_type(::Type{FlowMode}) = MsgPack.IntegerType()
MsgPack.to_msgpack(::MsgPack.IntegerType, x::FlowMode) = Int32(x)
MsgPack.from_msgpack(::Type{FlowMode}, x::Int32) = FlowMode(x)
Base.isless(x::FlowMode, y::Int32) = x < FlowMode(y)
Base.isgreater(x::FlowMode, y::Int32) = x > FlowMode(y)
Base.isequal(x::FlowMode, y::Int32) = x == FlowMode(y)
Base.convert(::Type{FlowMode}, x::Integer) = FlowMode(x)

mutable struct Propellant
    ignore_for_isp::Bool
    ratio::Float64
    flow_mode::FlowMode
    density::Float64
end

MsgPack.msgpack_type(::Type{Propellant}) = MsgPack.StructType()

mutable struct FuelStats
    start_mass::Float64
    start_time::Float64
    end_mass::Float64

    thrust::Float64
    isp::Float64
    spool_up_time::Float64
    delta_time::Float64

    max_rcs_deltav::Float64
    min_rcs_deltav::Float64
    deltav::Float64

    rcs_thrust::Float64
    rcs_delta_time::Float64
    rcs_mass::Float64
    rcs_isp::Float64

    rcs_start_tmr::Float64
    rcs_end_tmr::Float64
end

MsgPack.msgpack_type(::Type{FuelStats}) = MsgPack.StructType()

function FuelStats()::FuelStats
    return FuelStats(
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    )
end

mutable struct Conditions
    atm_pressure::Float64
    atm_density::Float64
    mach_number::Float64
    main_throttle::Float64
end

MsgPack.msgpack_type(::Type{Conditions}) = MsgPack.StructType()

struct SimPartId <: AbstractId
    inner::UInt64
end

Base.convert(::Type{SimPartId}, x::UInt64) = SimPartId(x)
Base.convert(::Type{UInt64}, x::SimPartId) = x.inner
MsgPack.msgpack_type(::Type{SimPartId}) = MsgPack.StructType()

mutable struct Engine
    propellants::Dict{ResourceId,Propellant}
    propellant_flow_modes::Dict{ResourceId,FlowMode}
    resource_consumptions::Dict{ResourceId,Float64}
    thrust_transform_multipliers::Vector{Float64}
    thrust_direction_vectors::Vector{SVector{3,Float64}}

    is_operational::Bool
    flow_multiplier::Float64
    thrust_current::SVector{3,Float64}
    thrust_max::SVector{3,Float64}
    thrust_min::SVector{3,Float64}
    mass_flow_rate::Float64
    isp::Float64
    g::Float64
    max_fuel_flow::Float64
    max_thrust::Float64
    min_fuel_flow::Float64
    min_thrust::Float64
    mult_isp::Float64
    clamp::Float64
    flow_mult_cap::Float64
    flow_mult_cap_sharpness::Float64
    throttle_locked::Bool
    throttle_limiter::Float64
    atm_change_flow::Bool
    use_atm_curve::Bool
    use_atm_curve_isp::Bool
    use_throttle_isp_curve::Bool
    use_vel_curve::Bool
    use_vel_curve_isp::Bool
    module_residuals::Float64
    module_spoolup_time::Float64
    no_propellants::Bool
    is_unrestartable_dead_engine::Bool
    is_module_engines_rf::Bool

    throttle_isp_curve::H1
    throttle_isp_curve_atm_strength::H1
    vel_curve::H1
    vel_curve_isp::H1
    atm_curve::H1
    atm_curve_isp::H1
    atmosphere_curve::H1

    is_sepratron::Bool
    part::SimPartId
end

MsgPack.msgpack_type(::Type{Engine}) = MsgPack.StructType()

end
