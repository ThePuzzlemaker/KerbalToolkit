module Vessels

using MsgPack: MsgPack

import ..KRPCTypes
using ..Arenas: Arena, AbstractId
using ..StateVectors: StateVector
using ..Time: UT
using ..FFSTypes: ResourceId, Resource, Engine

struct VesselClassId <: AbstractId
    inner::UInt64
end

Base.convert(::Type{UInt64}, x::VesselClassId) = x.inner
Base.convert(::Type{VesselClassId}, x::UInt64) = VesselClassId(x)
MsgPack.msgpack_type(::Type{VesselClassId}) = MsgPack.StructType()

struct VesselId <: AbstractId
    inner::UInt64
end

Base.convert(::Type{UInt64}, x::VesselId) = x.inner
Base.convert(::Type{VesselId}, x::UInt64) = VesselId(x)
MsgPack.msgpack_type(::Type{VesselId}) = MsgPack.StructType()

struct PartId <: AbstractId
    inner::UInt64
end

Base.convert(::Type{UInt64}, x::PartId) = x.inner
Base.convert(::Type{PartId}, x::UInt64) = PartId(x)
MsgPack.msgpack_type(::Type{PartId}) = MsgPack.StructType()

struct PersistentId <: AbstractId
    inner::UInt64
end

Base.convert(::Type{UInt64}, x::PersistentId) = x.inner
Base.convert(::Type{PersistentId}, x::UInt64) = PersistentId(x)
MsgPack.msgpack_type(::Type{PersistentId}) = MsgPack.StructType()

struct TrackedId <: AbstractId
    inner::UInt64
end

Base.convert(::Type{UInt64}, x::TrackedId) = x.inner
Base.convert(::Type{TrackedId}, x::UInt64) = TrackedId(x)
MsgPack.msgpack_type(::Type{TrackedId}) = MsgPack.StructType()

@enum Attachment begin
    RadialAttach
    AxialAttach
    NoAttach
end

MsgPack.msgpack_type(::Type{Attachment}) = MsgPack.IntegerType()
MsgPack.to_msgpack(::MsgPack.IntegerType, x::Attachment) = Int32(x)
MsgPack.from_msgpack(::Type{Attachment}, x::Int32) = Attachment(x)
Base.isless(x::Attachment, y::Int32) = x < Attachment(y)
Base.isgreater(x::Attachment, y::Int32) = x > Attachment(y)
Base.isequal(x::Attachment, y::Int32) = x == Attachment(y)
Base.convert(::Type{Attachment}, x::Integer) = Attachment(x)

@enum ModifierChangeWhen begin
    Fixed
    Staged
    Constantly
end

MsgPack.msgpack_type(::Type{ModifierChangeWhen}) = MsgPack.IntegerType()
MsgPack.to_msgpack(::MsgPack.IntegerType, x::ModifierChangeWhen) = Int32(x)
MsgPack.from_msgpack(::Type{ModifierChangeWhen}, x::Int32) = ModifierChangeWhen(x)
Base.isless(x::ModifierChangeWhen, y::Int32) = x < ModifierChangeWhen(y)
Base.isgreater(x::ModifierChangeWhen, y::Int32) = x > ModifierChangeWhen(y)
Base.isequal(x::ModifierChangeWhen, y::Int32) = x == ModifierChangeWhen(y)
Base.convert(::Type{ModifierChangeWhen}, x::Integer) = ModifierChangeWhen(x)

mutable struct MassModifier
    current_mass::Float64
    staged_mass::Float64
    unstaged_mass::Float64
    changes_when::ModifierChangeWhen
    module_name::String
end

MsgPack.msgpack_type(::Type{MassModifier}) = MsgPack.StructType()

struct PFDecoupler end

MsgPack.msgpack_type(::Type{PFDecoupler}) = MsgPack.StructType()

struct Decoupler
    is_omni_decoupler::Bool
    attached_part::Union{PartId,Nothing}
end

MsgPack.msgpack_type(::Type{Decoupler}) = MsgPack.StructType()

struct Decouplers
    top::Union{Decoupler, Nothing}
    bot::Union{Decoupler, Nothing}
    pf::Union{PFDecoupler, Nothing}
end

MsgPack.msgpack_type(::Type{Decouplers}) = MsgPack.StructType()

mutable struct Part
    tracked_id::TrackedId
    parent::Union{PartId,Nothing}
    children::Vector{PartId}
    name::String
    title::String
    tag::String
    decouplers::Decouplers
    attachment::Attachment
    crossfeed_part_set::Vector{PartId}
    resources::Dict{ResourceId,Resource}
    resource_priority::Int32
    resource_request_remaining_threshold::Float64
    mass::Float64
    dry_mass::Float64
    crew_mass::Float64
    disabled_resource_mass::Float64
    is_launch_clamp::Bool
    engines::Vector{Engine}
    mass_modifiers::Vector{MassModifier}
end

MsgPack.msgpack_type(::Type{Part}) = MsgPack.StructType()

Part()::Part = Part(
    TrackedId(0),
    nothing,
    PartId[],
    "",
    "",
    "",
    Decouplers(nothing, nothing, nothing),
    NoAttach,
    PartId[],
    Dict(),
    0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    false,
    Engine[],
    MassModifier[],
)

mutable struct VesselClass
    name::String
    description::String
    shortcode::String
    parts::Arena{PartId,Part}
    tracked_id_map::Dict{TrackedId,PartId}
    root::Union{PartId,Nothing}
end

MsgPack.msgpack_type(::Type{VesselClass}) = MsgPack.StructType()

mutable struct Vessel
    name::String
    description::String
    link::Union{KRPCTypes.Vessel,Nothing}
    class::Union{VesselClassId,Nothing}
    resources::Dict{Tuple{PartId,ResourceId},Resource}
    tracked_persistent_id_map::Dict{TrackedId,PersistentId}
    svs::Dict{String,StateVector}
    get_base::UT
end

MsgPack.msgpack_type(::Type{Vessel}) = MsgPack.StructType()

function decoupled_vessels(
    vessel::VesselClass,
    fired_decouplers::AbstractVector{PartId},
    ro_decouplers::AbstractDict{PartId,Tuple{Bool,Bool}},
)::Vector{Set{PartId}}
    parts = Set(keys(vessel.parts))
    worklist = [parts]
    vessels = Vector{Set{PartId}}()
    visited_decouplers = Set{PartId}()
    fired_decouplers = vcat(fired_decouplers, keys(ro_decouplers))

    while !isempty(worklist)
        subvessel = pop!(worklist)
        found_decoupler = false
        for decoupler in fired_decouplers
            if !(decoupler in visited_decouplers) && decoupler in subvessel
                found_decoupler = true
                push!(visited_decouplers, decoupler)

                d = vessel.parts[decoupler].decouplers
                if isnothing(d.bot) && !isnothing(d.top)
                    d = d.top
                    explosive_node = Set{PartId}()
                    secondary_node = Set{PartId}()

                    if !isnothing(d.attached_part)
                        traverse_until(
                            vessel, d.attached_part, visited_decouplers, explosive_node
                        )
                    end
                    if !isnothing(vessel.parts[decoupler].parent)
                        parent = vessel.parts[decoupler].parent
                        if parent != d.attached_part
                            traverse_until(
                                vessel, parent, visited_decouplers, explosive_node
                            )
                        end
                    end

                    if d.is_omni_decoupler
                        for child in vessel.parts[decoupler].children
                            if child == d.attached_part
                                continue
                            end

                            if vessel.parts[child].attachment == RadialAttach
                                set = Set()
                                traverse_until(vessel, child, visited_decouplers, set)
                                push!(worklist, set)
                            else
                                traverse_until(
                                    vessel, child, visited_decouplers, secondary_node
                                )
                            end
                        end
                        push!(worklist, Set{PartId}([decoupler]))
                    else
                        for child in vessel.parts[decoupler].children
                            if child == d.attached_part
                                continue
                            end

                            traverse_until(
                                vessel, child, visited_decouplers, secondary_node
                            )
                        end
                        push!(secondary_node, decoupler)
                    end

                    push!(worklist, explosive_node, secondary_node)
                elseif !isnothing(d.bot)
                    top = d.top
                    bot = d.bot
                    (top_en, bot_en) = get(ro_decouplers, decoupler, (false, false))

                    explosive_node = Set{PartId}()
                    secondary_node = Set{PartId}()

                    omni = false
                    attached_part = if top_en && bot_en
                        omni = true
                        top.attached_part
                    elseif top_en && !bot_en
                        top.attached_part
                    elseif !top_en && bot_en
                        bot.attached_part
                    else
                        nothing
                    end

                    if !isnothing(attached_part)
                        traverse_until(
                            vessel, attached_part, visited_decouplers, explosive_node
                        )
                    end
                    if !isnothing(vessel.parts[decoupler].parent)
                        parent = vessel.parts[decoupler].parent
                        if parent != attached_part
                            traverse_until(
                                vessel, parent, visited_decouplers, secondary_node
                            )
                        end
                    end

                    if omni
                        for child in vessel.parts[decoupler].children
                            if child == attached_part
                                continue
                            end

                            if vessel.parts[child].attachment == RadialAttach
                                set = Set{PartId}()
                                traverse_until(vessel, child, visited_decouplers, set)
                                push!(worklist, set)
                            else
                                traverse_until(
                                    vessel, child, visited_decouplers, secondary_node
                                )
                            end
                        end
                        push!(worklist, Set{PartId}([decoupler]))
                    else
                        for child in vessel.parts[decoupler].children
                            if child == attached_part
                                continue
                            end

                            traverse_until(
                                vessel, child, visited_decouplers, secondary_node
                            )
                        end
                        push!(secondary_node, decoupler)
                    end

                    push!(worklist, explosive_node, secondary_node)
                elseif !isnothing(d.pf)
                    # Decouple the fairing pieces, i.e. do nothing
                else
                    error("Unreachable")
                end
            end
        end

        if !found_decoupler && !isempty(subvessel)
            push!(vessels, subvessel)
        end
    end

    return vessels
end

function traverse_until(
    vessel::VesselClass,
    target::PartId,
    visited::AbstractSet{PartId},
    set::AbstractSet{PartId},
)
    worklist = [target]
    while !isempty(worklist)
        target = pop!(worklist)
        if target in set
            continue
        end

        if !(target in visited)
            push!(set, target)
        end

        for child in vessel.parts[target].children
            if !(child in visited)
                push!(worklist, child)
            end
        end

        if !isnothing(vessel.parts[target].parent)
            parent = vessel.parts[target].parent
            if !(parent in visited)
                push!(worklist, parent)
            end
        end
    end

    return nothing
end

end
