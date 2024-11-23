"""
Fuel flow simulation, as adapted from Lamont Granquist's MechJeb
code. The MechJebLib code underlying this simulation is licensed
individually from the rest of the project under a public domain
license.
"""
module FFS

using MsgPack: MsgPack
using StaticArrays: SVector
using LinearAlgebra: norm

using ..Arenas: Arena, AbstractId
using ..Math: evaluate, h1_add!
using ..Vessels: TrackedId, VesselId
using ..FFSTypes:
    ResourceId,
    Resource,
    SimPartId,
    Engine,
    Conditions,
    FuelStats,
    NoFlow,
    AllVessel,
    StagePriorityFlow,
    StackPrioritySearch,
    AllVesselBalance,
    StagePriorityFlowBalance,
    StageStackFlow,
    StageStackFlowBalance,
    Null

mutable struct SimPart
    tracked_id::TrackedId
    on_vessel::VesselId
    crossfeed_part_set::Vector{SimPartId}
    resources::Dict{ResourceId,Resource}
    resource_drains::Dict{ResourceId,Float64}

    resource_priority::Int32
    resource_request_remaining_threshold::Float64

    mass::Float64
    dry_mass::Float64
    crew_mass::Float64
    modules_current_mass::Float64
    disabled_resource_mass::Float64

    is_launch_clamp::Bool
end

MsgPack.msgpack_type(::Type{SimPart}) = MsgPack.StructType()

mutable struct SimVessel
    parts::Arena{SimPartId,SimPart}
    active_engines::Vector{Engine}
    #active_rcs::Vector{RCS}
    mass::Float64
    thrust_current::SVector{3,Float64}
    #rcs_thrust::Float64
    thrust_magnitude::Float64
    thrust_no_cos_loss::Float64
    spoolup_current::Float64
    conditions::Conditions
end

MsgPack.msgpack_type(::Type{SimVessel}) = MsgPack.StructType()

mutable struct FuelFlowSimulation
    segments::Vector{FuelStats}
    current_segment::FuelStats
    time::Float64
    dv_linear_thrust::Bool
    parts_with_resource_drains::Set{SimPartId}
    sources::Vector{SimPartId}
end

MsgPack.msgpack_type(::Type{FuelFlowSimulation}) = MsgPack.StructType()

residual_threshold(resource::Resource)::Float64 = resource.residual * resource.max_amount

function drain(resource::Resource, resource_drain::Float64)
    resource.amount -= resource_drain
    if resource.amount < 0.0
        resource.amount = 0.0
    end
    return nothing
end

function max_accel(stats::FuelStats)::Float64
    if stats.end_mass > 0.0
        stats.thrust / stats.end_mass
    else
        0.0
    end
end

function resource_mass(stats::FuelStats)::Float64
    return stats.start_mass - stats.end_mass
end

const G0::Float64 = 9.80665

rcs_start_twr(stats::FuelStats, gee_asl::Float64)::Float64 =
    stats.rcs_start_tmr / (G0 * gee_asl)

rcs_max_twr(stats::FuelStats, gee_asl::Float64)::Float64 =
    stats.rcs_end_tmr / (G0 * gee_asl)

function start_twr(stats::FuelStats, gee_asl::Float64)::Float64
    if stats.start_mass > 0.0
        stats.thrust / (G0 * gee_asl * stats.start_time)
    else
        0.0
    end
end

max_twr(stats::FuelStats, gee_asl::Float64)::Float64 = max_accel(stats) / (G0 * gee_asl)

function update_mass(self::SimVessel)
    self.mass = 0.0

    for (_, part) in self.parts
        update_mass(part)
        self.mass += part.mass
    end
end

function update_active_engines(self::SimVessel)
    active_engines = self.active_engines
    self.active_engines = Engine[]
    for engine in active_engines
        if engine.mass_flow_rate <= 0.0 || engine.is_unrestartable_dead_engine
            continue
        end
        update_engine_status(engine, self)
        if !engine.is_operational
            continue
        end
        push!(self.active_engines, engine)
    end

    return compute_thrust_and_spoolup(self)
end

function compute_thrust_and_spoolup(self::SimVessel)
    self.thrust_current = SVector{3}(0.0, 0.0, 0.0)
    self.thrust_magnitude = 0.0
    self.thrust_no_cos_loss = 0.0
    self.spoolup_current = 0.0

    for engine in self.active_engines
        if !engine.is_operational
            continue
        end

        self.spoolup_current += norm(engine.thrust_current) * engine.module_spoolup_time

        update(engine, self.conditions)
        self.thrust_current += engine.thrust_current
        self.thrust_no_cos_loss += norm(engine.thrust_current)
    end

    self.thrust_magnitude = norm(self.thrust_current)
    return self.spoolup_current /= norm(self.thrust_current)
end

function update_engine_stats(self::SimVessel)
    for engine in self.active_engines
        update(engine, self.conditions)
    end
end

function update_mass(self::SimPart)
    if self.is_launch_clamp
        self.mass = 0.0
        return nothing
    end

    self.mass =
        self.dry_mass +
        self.crew_mass +
        self.disabled_resource_mass +
        self.modules_current_mass
    for resource in values(self.resources)
        self.mass += resource.amount * resource.density
    end
end

function apply_drains(self::SimPart, dt::Float64)
    for (id, drain_) in self.resource_drains
        if haskey(self.resources, id)
            drain(self.resources[id], drain_ * dt)
        end
    end
end

function update_residuals(self::SimPart, residual::Float64, res::ResourceId)
    if haskey(self.resources, res)
        self.resources[res].residual = max(self.resources[res].residual, residual)
    end
end

function clear_residuals(self::SimPart)
    for (_, resource) in self.resources
        resource.residual = 0.0
    end
end

function add_drain(self::SimPart, res::ResourceId, consumption::Float64)
    if haskey(self.resource_drains, res)
        self.resource_drains[res] += consumption
    else
        self.resource_drains[res] = consumption
    end
end

function max_time(self::SimPart)::Float64
    max_time = floatmax(Float64)

    for (res, resource) in self.resources
        if resource.free || resource.amount <= self.resource_request_remaining_threshold
            continue
        end

        if haskey(self.resource_drains, res)
            resource_drain = self.resource_drains[res]
            dt = (resource.amount - residual_threshold(resource)) / resource_drain

            max_time = min(max_time, dt)
        end
    end

    return max_time
end

function clear_resource_drains(self::SimPart)
    self.resource_drains = Dict()
    return nothing
end

lerp(x::Float64, y::Float64, t::Float64)::Float64 = x + t * (y - x)

function update_engine_status(self::Engine, vessel::SimVessel)
    self.is_operational = can_draw_resources(self, vessel)
    return nothing
end

function can_draw_resources(self::Engine, vessel::SimVessel)::Bool
    if self.no_propellants
        return false
    end

    for prop in keys(self.resource_consumptions)
        fm = self.propellant_flow_modes[prop]
        if fm == NoFlow
            if !part_has_resource(self, vessel, self.part, prop)
                return false
            end
        elseif fm == AllVessel ||
            fm == AllVesselBalance ||
            fm == StagePriorityFlow ||
            fm == StagePriorityFlowBalance
            if !parts_have_resource(self, vessel, keys(vessel.parts), prop)
                return false
            end
        elseif fm == StackPrioritySearch ||
            fm == StageStackFlow ||
            fm == StageStackFlowBalance
            if !parts_have_resource(
                self, vessel, vessel.parts[self.part].crossfeed_part_set, prop
            )
                return false
            end
        else
            return false
        end
    end

    return true
end

function part_has_resource(
    self::Engine, vessel::SimVessel, part::SimPartId, res::ResourceId
)::Bool
    if haskey(vessel.parts[part].resources, res)
        resource = vessel.parts[part].resources[res]
        return resource.amount >
               resource.max_amount * self.module_residuals +
               vessel.parts[part].resource_request_remaining_threshold
    else
        return false
    end
end

function parts_have_resource(
    self::Engine, vessel::SimVessel, parts::AbstractVector{SimPartId}, res::ResourceId
)::Bool
    for part in parts
        if part_has_resource(self, vessel, part, res)
            return true
        end
    end
    return false
end

function update(self::Engine, conditions::Conditions)
    self.isp = isp_at_conditions(self, conditions)
    self.flow_multiplier = flow_multiplier_at_conditions(self, conditions)
    self.mass_flow_rate = flow_rate_at_conditions(self, conditions)
    refresh_thrust(self)
    set_consumption_rates(self)
    return nothing
end

function flow_rate_at_conditions(self::Engine, conditions::Conditions)::Float64
    min_fuel_flow = self.min_fuel_flow
    max_fuel_flow = self.max_fuel_flow

    if min_fuel_flow == 0.0 && self.min_thrust > 0.0
        min_fuel_flow = self.min_thrust / (evaluate(self.atmosphere_curve, 0.0) * self.g)
    end
    if max_fuel_flow == 0.0 && self.max_thrust > 0.0
        max_fuel_flow = self.max_thrust / (evaluate(self.atmosphere_curve, 0.0) * self.g)
    end

    return lerp(
        min_fuel_flow,
        max_fuel_flow,
        conditions.main_throttle * 0.01 * self.throttle_limiter,
    ) * self.flow_multiplier
end

function refresh_thrust(self::Engine)
    self.thrust_current = SVector{3}(0.0, 0.0, 0.0)
    self.thrust_max = SVector{3}(0.0, 0.0, 0.0)
    self.thrust_min = SVector{3}(0.0, 0.0, 0.0)

    thrust_limiter = self.throttle_limiter / 100.0
    max_thrust =
        self.max_fuel_flow * self.flow_multiplier * self.isp * self.g * self.mult_isp
    min_thrust =
        self.min_fuel_flow * self.flow_multiplier * self.isp * self.g * self.mult_isp

    e_max_thrust = min_thrust + (max_thrust - min_thrust) * thrust_limiter
    e_min_thrust = if self.throttle_locked
        e_max_thrust
    else
        self.min_thrust
    end
    e_current_thrust = self.mass_flow_rate * self.isp * self.g * self.mult_isp

    for (i, thrust_direction_vector) in enumerate(self.thrust_direction_vectors)
        thrust_transform_multiplier = self.thrust_transform_multipliers[i]
        t_current_thrust = e_current_thrust * thrust_transform_multiplier

        self.thrust_current += t_current_thrust * thrust_direction_vector
        self.thrust_max +=
            e_max_thrust * thrust_direction_vector * thrust_transform_multiplier
        self.thrust_min +=
            e_min_thrust * thrust_direction_vector * thrust_transform_multiplier
    end
end

function flow_multiplier_at_conditions(self::Engine, conditions::Conditions)::Float64
    flow_multiplier = 1.0

    if self.atm_change_flow
        if self.use_atm_curve
            flow_multiplier = evaluate(self.atm_curve, conditions.atm_density * 40.0 / 49.0)
        else
            flow_multiplier = conditions.atm_density * 40.0 / 49.0
        end
    end

    if self.use_vel_curve
        flow_multiplier *= evaluate(self.vel_curve, conditions.mach_number)
    end

    if flow_multiplier > self.flow_mult_cap
        excess = flow_multiplier - self.flow_mult_cap
        flow_multiplier =
            self.flow_mult_cap +
            excess / (self.flow_mult_cap_sharpness + excess / self.flow_mult_cap)
    end

    if flow_multiplier < self.clamp && self.clamp < 1.0
        flow_multiplier = self.clamp
    end

    return flow_multiplier
end

function isp_at_conditions(self::Engine, conditions::Conditions)::Float64
    isp = evaluate(self.atmosphere_curve, conditions.atm_pressure)
    if self.use_throttle_isp_curve
        isp *= lerp(
            1.0,
            evaluate(self.throttle_isp_curve, conditions.main_throttle),
            evaluate(self.throttle_isp_curve_atm_strength, conditions.atm_pressure),
        )
    end
    if self.use_atm_curve_isp
        isp *= evaluate(self.atm_curve_isp, conditions.atm_density * 40.0 / 49.0)
    end
    if self.use_vel_curve_isp
        isp *= evaluate(self.vel_curve_isp, conditions.mach_number)
    end
    return isp
end

function set_consumption_rates(self::Engine)
    self.resource_consumptions = Dict()
    self.propellant_flow_modes = Dict()

    total_density = 0.0

    for (id, propellant) in self.propellants
        density = propellant.density

        if density <= 0.0
            continue
        end

        self.propellant_flow_modes[id] = propellant.flow_mode

        if propellant.ignore_for_isp
            continue
        end

        total_density += propellant.ratio * density
    end

    volume_flow_rate = self.mass_flow_rate / total_density

    for (id, propellant) in self.propellants
        density = propellant.density

        prop_volume_rate = propellant.ratio * volume_flow_rate

        if density <= 0.0
            continue
        end

        if haskey(self.resource_consumptions, id)
            self.resource_consumptions[id] += prop_volume_rate
        else
            self.resource_consumptions[id] = prop_volume_rate
        end
    end
end

function run(
    self::FuelFlowSimulation,
    vessel::SimVessel,
    max_deltav::Union{Float64,Nothing},
    resume::Bool,
)
    if resume
        pop!(self.segments)
    else
        self.time = 0.0
        self.segments = Vector[]

        vessel.conditions.main_throttle = 1.0
    end

    (
        () -> begin
            if !resume
                update_mass(vessel)
                update_engine_stats(vessel)
                update_active_engines(vessel)

                get_next_segment(self, vessel)
                # compute_rcs_min_values(self, vessel)

                update_resource_drains_and_residuals(self, vessel)
            end

            current_thrust = vessel.thrust_magnitude

            dt = 0.01
            dv_achieved = 0.0
            dv_segment = 0.0
            for i in 1:100_000_000
                if isempty(vessel.active_engines) ||
                    all(x -> x.is_sepratron, vessel.active_engines) ||
                    (!isnothing(max_deltav) && (dv_achieved + dv_segment) >= max_deltav) ||
                    dt <= eps(Float64)
                    return nothing
                end

                exhvel = 0.0

                if abs(vessel.thrust_magnitude - current_thrust) > 1e-12
                    clear_residuals(self, vessel)
                    # compute_rcs_max_values(self, vessel)
                    finish_segment(self, vessel)
                    dv_achieved += self.current_segment.deltav
                    dv_segment = 0.0
                    get_next_segment(self, vessel)
                    current_thrust = vessel.thrust_magnitude
                end

                if isnothing(max_deltav)
                    dt = minimum_time_step(self, vessel)
                else
                    mass_flow_rate = sum(
                        map(x -> 1000.0 * x.mass_flow_rate, vessel.active_engines)
                    )
                    exhvel = (current_thrust * 1000.0) / mass_flow_rate
                    alpha = exp(-(max_deltav - (dv_achieved + dv_segment)) / exhvel)
                    bt =
                        (1000.0 * vessel.mass * exhvel) / (1000.0 * current_thrust) *
                        (1.0 - alpha)
                    dt = min(bt, minimum_time_step(self, vessel))
                end

                self.time += dt
                apply_resource_drains(self, vessel, dt)

                update_mass(vessel)
                dv_segment =
                    exhvel * log(self.current_segment.start_mass / vessel.mass)

                update_engine_stats(vessel)
                update_active_engines(vessel)
                update_resource_drains_and_residuals(self, vessel)
            end

            error("OOPS")
        end
    )()

    if isnothing(max_deltav)
        clear_residuals(self, vessel)
    end
    # compute_rcs_max_values(self, vessel)
    finish_segment(self, vessel)
    if isnothing(max_deltav)
        self.parts_with_resource_drains = Set()
    end

    return nothing
end

function clear_residuals(self::FuelFlowSimulation, vessel::SimVessel)
    for part in self.parts_with_resource_drains
        clear_residuals(vessel.parts[part])
    end
end

function apply_resource_drains(self::FuelFlowSimulation, vessel::SimVessel, dt::Float64)
    for part in self.parts_with_resource_drains
        apply_drains(vessel.parts[part], dt)
    end
end

function update_resource_drains_and_residuals(self::FuelFlowSimulation, vessel::SimVessel)
    for part in self.parts_with_resource_drains
        clear_resource_drains(vessel.parts[part])
        clear_residuals(vessel.parts[part])
    end

    self.parts_with_resource_drains = Set()

    for e in vessel.active_engines
        for (res, mode) in e.propellant_flow_modes
            if mode == NoFlow
                update_resource_drains_and_residuals_in_part(
                    self,
                    vessel,
                    e.part,
                    e.resource_consumption[res],
                    res,
                    e.module_residuals,
                )
            elseif mode == AllVessel || mode == AllVesselBalance
                update_resource_drains_and_residuals_in_parts(
                    self,
                    vessel,
                    keys(vessel.parts),
                    e.resource_consumption[res],
                    res,
                    false,
                    e.module_residuals,
                )
            elseif mode == StagePriorityFlow || mode == StagePriorityFlowBalance
                update_resource_drains_and_residuals_in_parts(
                    self,
                    vessel,
                    keys(vessel.parts),
                    e.resource_consumptions[res],
                    res,
                    true,
                    e.module_residuals,
                )
            elseif mode == StageStackFlow ||
                mode == StageStackFlowBalance ||
                mode == StackPrioritySearch
                update_resource_drains_and_residuals_in_parts(
                    self,
                    vessel,
                    vessel.parts[e.part].crossfeed_part_set,
                    e.resource_consumptions[res],
                    res,
                    true,
                    e.module_residuals,
                )
            end
        end
    end
end

function update_resource_drains_and_residuals_in_parts(
    self::FuelFlowSimulation,
    vessel::SimVessel,
    parts::AbstractVector{SimPartId},
    resource_consumption::Float64,
    res::ResourceId,
    use_priority::Bool,
    residual::Float64,
)
    max_priority = typemin(Int32)

    self.sources = SimPartId[]

    for p in parts
        if haskey(vessel.parts[p].resources, res)
            resource = vessel.parts[p].resources[res]
            if resource.free ||
                resource.amount <= (
                residual * resource.max_amount +
                vessel.parts[p].resource_request_remaining_threshold
            )
                continue
            end

            if use_priority
                if vessel.parts[p].resource_priority < max_priority
                    continue
                end

                if vessel.parts[p].resource_priority > max_priority
                    self.sources = SimPartId[]
                    max_priority = vessel.parts[p].resource_priority
                end
            end
            push!(self.sources, p)
        end
    end

    for source in self.sources
        update_resource_drains_and_residuals_in_part(
            self, vessel, source, resource_consumption / length(self.sources), res, residual
        )
    end
end

function update_resource_drains_and_residuals_in_part(
    self::FuelFlowSimulation,
    vessel::SimVessel,
    p::SimPartId,
    resource_consumption::Float64,
    res::ResourceId,
    residual::Float64,
)
    push!(self.parts_with_resource_drains, p)
    add_drain(vessel.parts[p], res, resource_consumption)
    return update_residuals(vessel.parts[p], residual, res)
end

function minimum_time_step(self::FuelFlowSimulation, vessel::SimVessel)::Float64
    max_time = resource_max_time(self, vessel)

    return 0.0 <= max_time < floatmax(Float64) ? max_time : 0.0
end

function resource_max_time(self::FuelFlowSimulation, vessel::SimVessel)::Float64
    max_time1 = floatmax(Float64)

    for part in self.parts_with_resource_drains
        max_time1 = min(max_time(vessel.parts[part]), max_time1)
    end

    return max_time1
end

function get_next_segment(self::FuelFlowSimulation, vessel::SimVessel)
    self.current_segment = FuelStats(
        vessel.mass,
        self.time,
        0.0,
        self.dv_linear_thrust ? vessel.thrust_magnitude : vessel.thrust_no_cos_loss,
        0.0,
        vessel.spoolup_current,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )

    return nothing
end

function finish_segment(self::FuelFlowSimulation, vessel::SimVessel)
    start_mass = self.current_segment.start_mass
    thrust = self.current_segment.thrust
    end_mass = vessel.mass
    delta_time = self.time - self.current_segment.start_time
    delta_v = if start_mass > end_mass
        thrust * delta_time / (start_mass - end_mass) * log(start_mass / end_mass)
    else
        0.0
    end
    isp = if start_mass > end_mass
        delta_v / (G0 * log(start_mass / end_mass))
    else
        0.0
    end

    self.current_segment.delta_time = delta_time
    self.current_segment.end_mass = end_mass
    self.current_segment.deltav = delta_v
    self.current_segment.isp = isp
    return push!(self.segments, deepcopy(self.current_segment))
end

end
