module KRPC

using ..KRPCTypes

module Schema
    include("KRPC/Schema.jl")
end

include("KRPC/Encode.jl")

using Sockets: Sockets, TCPSocket
using ProtoBuf: encode, decode, ProtoEncoder, ProtoDecoder
using ProtoBuf.Codecs: _encode
using StaticArrays: SVector

using .Encode: encode_value, decode_value
using ..Time: UT
using ..Orbits: Orbits
using ..Bodies: Bodies
using ..Vessels: ModifierChangeWhen
using ..FFSTypes: FlowMode

export Schema, Client, connect, raw_call, procedure_call

struct Client
    rpc::TCPSocket
end

"""
    connect(name::String, host::String, rpc_port::Integer)

Connect to the kRPC server at `host` on port `rpc_port` with the
client name `name`.
"""
function connect(name::String, host::String, rpc_port::Integer)
    rpc = Sockets.connect(host, rpc_port)

    request = Schema.ConnectionRequest(name, [])
    buf1 = IOBuffer()
    encode(ProtoEncoder(buf1), request)
    buf2 = IOBuffer()
    buf1 = take!(buf1)
    _encode(buf2, length(buf1))
    write(rpc, take!(buf2))
    write(rpc, buf1)
    flush(rpc)

    data = UInt8[]
    size = 0
    while true
        try
            push!(data, read(rpc, UInt8))
            size = decode(ProtoDecoder(IOBuffer(data)), Int32)
            break
        catch e
            if e isa EOFError
                continue
            else
                rethrow(e)
            end
        end
    end

    buf = IOBuffer(read(rpc, size))
    resp = decode(ProtoDecoder(buf), Schema.ConnectionResponse)
    if resp.status != 0
        error("Failed to connect to kRPC: $(resp.message)")
    end

    return Client(rpc)
end

function Base.close(client::Client)
    return close(client.rpc)
end

function raw_call(client::Client, request::Schema.Request)::Schema.Response
    buf1 = IOBuffer()
    encode(ProtoEncoder(buf1), request)
    buf2 = IOBuffer()
    buf1 = take!(buf1)
    _encode(buf2, length(buf1))
    write(client.rpc, take!(buf2))
    write(client.rpc, buf1)
    flush(client.rpc)

    data = UInt8[]
    size = 0
    while true
        try
            push!(data, read(client.rpc, UInt8))
            size = decode(ProtoDecoder(IOBuffer(data)), Int32)
            break
        catch e
            if e isa EOFError
                continue
            else
                rethrow(e)
            end
        end
    end

    buf = IOBuffer(read(client.rpc, size))
    response = decode(ProtoDecoder(buf), Schema.Response)

    !isnothing(response.error) && error(repr(response.error))

    for res in response.results
        !isnothing(res.error) && error(repr(res.error))
    end

    return response
end

function procedure_call(
    client::Client,
    service::String,
    procedure::String,
    arguments::Vector{Schema.Argument},
    ::Type{V},
)::V where {V}
    call = Schema.ProcedureCall(service, procedure, 0, 0, arguments)
    request = Schema.Request([call])
    response = raw_call(client, request)
    @assert length(response.results) == 1
    return decode_value(response.results[1].value, V)
end

function get_status(client::Client)
    return procedure_call(client, "KRPC", "GetStatus", Schema.Argument[], Schema.Status)
end

function get_ut(client::Client)
    return UT(procedure_call(client, "SpaceCenter", "get_UT", Schema.Argument[], Float64))
end

function get_bodies(client::Client)
    return procedure_call(
        client, "SpaceCenter", "get_Bodies", Schema.Argument[], Dict{String,CelestialBody}
    )
end

function get_name(client::Client, b::CelestialBody)
    return procedure_call(
        client,
        "SpaceCenter",
        "CelestialBody_get_Name",
        [Schema.Argument(0, encode_value(b))],
        String,
    )
end

function get_satellites(client::Client, b::CelestialBody)
    return procedure_call(
        client,
        "SpaceCenter",
        "CelestialBody_get_Satellites",
        [Schema.Argument(0, encode_value(b))],
        Vector{CelestialBody},
    )
end

function get_gravitational_parameter(client::Client, b::CelestialBody)
    return procedure_call(
        client,
        "SpaceCenter",
        "CelestialBody_get_GravitationalParameter",
        [Schema.Argument(0, encode_value(b))],
        Float64,
    )
end

function get_equatorial_radius(client::Client, b::CelestialBody)
    return procedure_call(
        client,
        "SpaceCenter",
        "CelestialBody_get_EquatorialRadius",
        [Schema.Argument(0, encode_value(b))],
        Float64,
    )
end

function get_rotational_period(client::Client, b::CelestialBody)
    return procedure_call(
        client,
        "SpaceCenter",
        "CelestialBody_get_RotationalPeriod",
        [Schema.Argument(0, encode_value(b))],
        Float64,
    )
end

function get_initial_rotation(client::Client, b::CelestialBody)
    return procedure_call(
        client,
        "SpaceCenter",
        "CelestialBody_get_InitialRotation",
        [Schema.Argument(0, encode_value(b))],
        Float64,
    )
end

function get_is_star(client::Client, b::CelestialBody)
    return procedure_call(
        client,
        "SpaceCenter",
        "CelestialBody_get_IsStar",
        [Schema.Argument(0, encode_value(b))],
        Bool,
    )
end

function get_sphere_of_influence(client::Client, b::CelestialBody)
    return procedure_call(
        client,
        "SpaceCenter",
        "CelestialBody_get_SphereOfInfluence",
        [Schema.Argument(0, encode_value(b))],
        Float64,
    )
end

function get_orbit(client::Client, b::CelestialBody)
    return procedure_call(
        client,
        "SpaceCenter",
        "CelestialBody_get_Orbit",
        [Schema.Argument(0, encode_value(b))],
        Union{Orbit,Nothing},
    )
end

function get_body(client::Client, o::Orbit)
    return procedure_call(
        client,
        "SpaceCenter",
        "Orbit_get_Body",
        [Schema.Argument(0, encode_value(o))],
        CelestialBody,
    )
end

function get_ephemerides(client::Client, o::Orbit, epoch::UT)
    args = [Schema.Argument(0, encode_value(o))]
    sma = Schema.ProcedureCall("SpaceCenter", "Orbit_get_SemiMajorAxis", 0, 0, args)
    ecc = Schema.ProcedureCall("SpaceCenter", "Orbit_get_Eccentricity", 0, 0, args)
    inc = Schema.ProcedureCall("SpaceCenter", "Orbit_get_Inclination", 0, 0, args)
    lan = Schema.ProcedureCall(
        "SpaceCenter", "Orbit_get_LongitudeOfAscendingNode", 0, 0, args
    )
    argpe = Schema.ProcedureCall("SpaceCenter", "Orbit_get_ArgumentOfPeriapsis", 0, 0, args)
    ta = Schema.ProcedureCall(
        "SpaceCenter",
        "Orbit_TrueAnomalyAtUT",
        0,
        0,
        [args[1], Schema.Argument(1, encode_value(epoch.value))],
    )
    req = Schema.Request([sma, ecc, inc, lan, argpe, ta])
    response = raw_call(client, req)
    sma = decode_value(response.results[1].value, Float64) / 1000.0
    ecc = decode_value(response.results[2].value, Float64)
    inc = decode_value(response.results[3].value, Float64)
    lan = decode_value(response.results[4].value, Float64)
    argpe = decode_value(response.results[5].value, Float64)
    ta = decode_value(response.results[6].value, Float64)
    return Orbits.Orbit(sma * (1 - ecc^2), ecc, inc, lan, argpe, ta, epoch)
end

function get_non_rotating_reference_frame(client::Client, b::CelestialBody)
    return procedure_call(
        client,
        "SpaceCenter",
        "CelestialBody_get_NonRotatingReferenceFrame",
        [Schema.Argument(0, encode_value(b))],
        ReferenceFrame,
    )
end

function get_angular_velocity(client::Client, b::CelestialBody, rf::ReferenceFrame)
    (x, y, z) = procedure_call(
        client,
        "SpaceCenter",
        "CelestialBody_AngularVelocity",
        [Schema.Argument(0, encode_value(b)), Schema.Argument(1, encode_value(rf))],
        Tuple{Float64,Float64,Float64},
    )
    return SVector{3}(x, y, z)
end

function Bodies.SolarSystem(client::Client)
    bodies = get_bodies(client)
    system = Bodies.SolarSystem(Dict())
    for (name, body) in bodies
        mu = get_gravitational_parameter(client, body) / (1000.0^3)
        radius = get_equatorial_radius(client, body) / 1000.0
        rotperiod = get_rotational_period(client, body)
        rotini = get_initial_rotation(client, body)
        satellites = map(get_satellites(client, body)) do (body)
            get_name(client, body)
        end
        orbit = get_orbit(client, body)
        (ephem, parent) = if isnothing(orbit)
            (Orbits.Orbit(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, UT(0.0)), nothing)
        else
            ephem = get_ephemerides(client, orbit, UT(0.0))
            parent = get_name(client, get_body(client, orbit))
            (ephem, parent)
        end

        rf = get_non_rotating_reference_frame(client, body)

        body = Bodies.Body(
            mu,
            radius,
            ephem,
            rotperiod,
            rotini,
            satellites,
            parent,
            name,
            get_is_star(client, body),
            get_sphere_of_influence(client, body) / 1000.0,
            get_angular_velocity(client, body, rf),
        )
        system.bodies[name] = body
    end
    return system
end

function get_vessels(client::Client)::Vector{Vessel}
    return procedure_call(
        client, "SpaceCenter", "get_Vessels", Schema.Argument[], Vector{Vessel}
    )
end

function get_active_vessel(client::Client)::Union{Vessel,Nothing}
    return procedure_call(
        client, "SpaceCenter", "get_Activeessel", Schema.Argument[], Union{Vessel,Nothing}
    )
end

function get_editor(client::Client)::Editor
    return procedure_call(client, "SpaceCenter", "get_Editor", Schema.Argument[], Editor)
end

function get_current_ship(client::Client, editor::Editor)::Union{EditorShip,Nothing}
    return procedure_call(
        client,
        "SpaceCenter",
        "Editor_get_CurrentShip",
        [Schema.Argument(0, encode_value(editor))],
        Union{EditorShip,Nothing},
    )
end

function get_parts(client::Client, vessel::Vessel)::Parts
    return procedure_call(
        client,
        "SpaceCenter",
        "Vessel_get_Parts",
        [Schema.Argument(0, encode_value(vessel))],
        Parts,
    )
end

function get_orbit(client::Client, vessel::Vessel)::Orbit
    return procedure_call(
        client,
        "SpaceCenter",
        "Vessel_get_Orbit",
        [Schema.Argument(0, encode_value(vessel))],
        Orbit,
    )
end

function get_name(client::Client, vessel::Vessel)::String
    return procedure_call(
        client,
        "SpaceCenter",
        "Vessel_get_Name",
        [Schema.Argument(0, encode_value(vessel))],
        String,
    )
end

function get_met_base(client::Client, vessel::Vessel)::UT
    return UT(
        procedure_call(
            client,
            "KerbTk",
            "VesselMETBase",
            [Schema.Argument(0, encode_value(vessel))],
            Float64,
        ),
    )
end

function get_state_vector(
    client::Client, vessel::Vessel, rf::ReferenceFrame
)::Tuple{String,SVector{3,Float64},SVector{3,Float64},UT}
    args = vec[
        Schema.Argument(0, encode_value(vessel)), Schema.Argument(1, encode_value(rf))
    ]
    soi = Schema.ProcedureCall(
        "KerbTk", "VesselSOIBodyName", 0, 0, [Schema.Argument(0, encode_value(vessel))]
    )
    pos = Schema.ProcedureCall("SpaceCenter", "Vessel_Position", 0, 0, args)
    vel = Schema.ProcedureCall("SpaceCenter", "Vessel_Velocity", 0, 0, args)
    ut = Schema.ProcedureCall("SpaceCenter", "get_UT", 0, 0, Schema.Argument[])
    req = Schema.Request([soi, pos, vel, ut])
    response = raw_call(client, req)
    soi = decode_value(response.results[1].value, String)
    pos = decode_value(response.results[2].value, Tuple{Float64,Float64,Float64})
    vel = decode_value(response.results[3].value, Tuple{Float64,Float64,Float64})
    ut = UT(decode_value(response.results[4].value, UT))
    return (
        soi,
        SVector{3}(pos[1] / 1000.0, pos[2] / 1000.0, pos[3] / 1000.0),
        SVector{3}(vel[1] / 1000.0, vel[2] / 1000.0, vel[3] / 1000.0),
        ut,
    )
end

function get_parts(client::Client, es::EditorShip)::EditorParts
    return procedure_call(
        client,
        "SpaceCenter",
        "EditorShip_get_Parts",
        [Schema.Argument(0, encode_value(es))],
        EditorParts,
    )
end

function get_ship_name(client::Client, es::EditorShip)::String
    return procedure_call(
        client,
        "SpaceCenter",
        "EditorShip_get_ShipName",
        [Schema.Argument(0, encode_value(es))],
        String,
    )
end

function get_ship_description(client::Client, es::EditorShip)::String
    return procedure_call(
        client,
        "SpaceCenter",
        "EditorShip_get_ShipDescription",
        [Schema.Argument(0, encode_value(es))],
        String,
    )
end

function get_all(client::Client, ep::EditorParts)::Vector{Part}
    return procedure_call(
        client,
        "SpaceCenter",
        "EditorParts_get_All",
        [Schema.Argument(0, encode_value(ep))],
        Vector{Part},
    )
end

function get_all(client::Client, p::Parts)::Vector{Part}
    return procedure_call(
        client,
        "SpaceCenter",
        "Parts_get_All",
        [Schema.Argument(0, encode_value(p))],
        Vector{Part},
    )
end

function get_name(client::Client, p::Part)::String
    return procedure_call(
        client,
        "SpaceCenter",
        "Part_get_Name",
        [Schema.Argument(0, encode_value(p))],
        String,
    )
end

function get_persistent_id(client::Client, p::Part)::UInt32
    return procedure_call(
        client, "KerbTk", "PartPersistentID", [Schema.Argument(0, encode_value(p))], UInt32
    )
end

function start_tracking(client::Client, p::Part, id::UInt32)
    procedure_call(
        client,
        "KerbTk",
        "PartStartTracking",
        [Schema.Argument(0, encode_value(p)), Schema.Argument(1, encode_value(id))],
        Tuple{},
    )
    return nothing
end

function get_tracked_id(client::Client, p::Part)::UInt32
    return procedure_call(
        client, "KerbTk", "PartTrackedID", [Schema.Argument(0, encode_value(p))], UInt32
    )
end

function get_orig_vessel_id(client::Client, p::Part)::UInt32
    return procedure_call(
        client, "KerbTk", "PartOrigVesselID", [Schema.Argument(0, encode_value(p))], UInt32
    )
end

function get_tracked_vessel_id(client::Client, p::Part)::UInt32
    return procedure_call(
        client,
        "KerbTk",
        "PartTrackedVesselID",
        [Schema.Argument(0, encode_value(p))],
        UInt32,
    )
end

function get_title(client::Client, p::Part)::String
    return procedure_call(
        client,
        "SpaceCenter",
        "Part_get_Title",
        [Schema.Argument(0, encode_value(p))],
        String,
    )
end

function get_tag(client::Client, p::Part)::String
    return procedure_call(
        client, "SpaceCenter", "Part_get_Tag", [Schema.Argument(0, encode_value(p))], String
    )
end

function get_parent(client::Client, p::Part)::Union{Part,Nothing}
    return procedure_call(
        client,
        "SpaceCenter",
        "Part_get_Parent",
        [Schema.Argument(0, encode_value(p))],
        Union{Part,Nothing},
    )
end

function get_children(client::Client, p::Part)::Vector{Part}
    return procedure_call(
        client,
        "SpaceCenter",
        "Part_get_Children",
        [Schema.Argument(0, encode_value(p))],
        Vector{Part},
    )
end

function get_decoupler(client::Client, p::Part)::Union{Decoupler,Nothing}
    return procedure_call(
        client,
        "SpaceCenter",
        "Part_get_Decoupler",
        [Schema.Argument(0, encode_value(p))],
        Union{Decoupler,Nothing},
    )
end

function get_ro_decoupler(client::Client, p::Part)::Union{RODecoupler,Nothing}
    return procedure_call(
        client,
        "KerbTk",
        "RODecouplerOnPart",
        [Schema.Argument(0, encode_value(p))],
        Union{RODecoupler,Nothing},
    )
end

function get_pf_decoupler(client::Client, p::Part)::Union{PFDecoupler,Nothing}
    return procedure_call(
        client,
        "KerbTk",
        "PFDecouplerOnPart",
        [Schema.Argument(0, encode_value(p))],
        Union{RODecoupler,Nothing},
    )
end

function get_axially_attached(client::Client, p::Part)::Bool
    return procedure_call(
        client,
        "SpaceCenter",
        "Part_get_AxiallyAttached",
        [Schema.Argument(0, encode_value(p))],
        Bool,
    )
end

function get_radially_attached(client::Client, p::Part)::Bool
    return procedure_call(
        client,
        "SpaceCenter",
        "Part_get_RadiallyAttached",
        [Schema.Argument(0, encode_value(p))],
        Bool,
    )
end

function get_crossfeed_part_set(client::Client, p::Part)::Vector{Part}
    return procedure_call(
        client,
        "KerbTk",
        "CrossfeedPartSet",
        [Schema.Argument(0, encode_value(p))],
        Vector{Part},
    )
end

function get_resource_priority(client::Client, p::Part)::Int32
    return procedure_call(
        client, "KerbTk", "ResourcePriority", [Schema.Argument(0, encode_value(p))], Int32
    )
end

function get_resource_request_remaining_threshold(client::Client, p::Part)::Float64
    return procedure_call(
        client,
        "KerbTk",
        "ResourceRequestRemainingThreshold",
        [Schema.Argument(0, encode_value(p))],
        Float64,
    )
end

function get_part_masses(client::Client, p::Part)::Tuple{Float64,Float64,Float64}
    return procedure_call(
        client,
        "KerbTk",
        "PartMasses",
        [Schema.Argument(0, encode_value(p))],
        Tuple{Float64,Float64,Float64},
    )
end

function get_resources(client::Client, p::Part)::Resources
    return procedure_call(
        client,
        "SpaceCenter",
        "Part_get_Resources",
        [Schema.Argument(0, encode_value(p))],
        Resources,
    )
end

function get_launch_clamp(client::Client, p::Part)::Union{LaunchClamp,Nothing}
    return procedure_call(
        client,
        "SpaceCenter",
        "Part_get_LaunchClamp",
        [Schema.Argument(0, encode_value(p))],
        Union{LaunchClamp,Nothing},
    )
end

function get_engine_params_f64(client::Client, p::Part)::Dict{String,Dict{String,Float64}}
    return procedure_call(
        client,
        "KerbTk",
        "EngineParametersF64",
        [Schema.Argument(0, encode_value(p))],
        Dict{String,Dict{String,Float64}},
    )
end

function get_engine_params_bool(client::Client, p::Part)::Dict{String,Dict{String,Bool}}
    return procedure_call(
        client,
        "KerbTk",
        "EngineParametersBool",
        [Schema.Argument(0, encode_value(p))],
        Dict{String,Dict{String,Bool}},
    )
end

function get_engine_params_curve(
    client::Client, p::Part
)::Dict{String,Dict{String,Vector{Tuple{Float64,Float64,Float64,Float64}}}}
    return procedure_call(
        client,
        "KerbTk",
        "EngineParametersCurve",
        [Schema.Argument(0, encode_value(p))],
        Dict{String,Dict{String,Vector{Tuple{Float64,Float64,Float64,Float64}}}},
    )
end

function get_engine_thrust_transform_multipliers(
    client::Client, p::Part
)::Dict{String,Vector{Float32}}
    return procedure_call(
        client,
        "KerbTk",
        "EngineThrustTransformMultipliers",
        [Schema.Argument(0, encode_value(p))],
        Dict{String,Vector{Float32}},
    )
end

function get_engine_thrust_transforms(
    client::Client, p::Part
)::Dict{String,Vector{Tuple{Float64,Float64,Float64}}}
    return procedure_call(
        client,
        "KerbTk",
        "EngineThrustTransforms",
        [Schema.Argument(0, encode_value(p))],
        Dict{String,Vector{Tuple{Float64,Float64,Float64}}},
    )
end

function get_engine_propellants(
    client::Client, p::Part
)::Dict{String,Vector{Tuple{Int32,Bool,Float32,Int32,Float32}}}
    return procedure_call(
        client,
        "KerbTk",
        "EnginePropellants",
        [Schema.Argument(0, encode_value(p))],
        Dict{String,Vector{Tuple{Int32,Bool,Float32,Int32,Float32}}},
    )
end

function get_part_mass_modifiers(client::Client, p::Part)::Vector{PartModule}
    return procedure_call(
        client,
        "KerbTk",
        "PartMassModifiers",
        [Schema.Argument(0, encode_value(p))],
        Vector{PartModule},
    )
end

function get_name(client::Client, m::PartModule)::String
    return procedure_call(
        client,
        "SpaceCenter",
        "Module_get_Name",
        [Schema.Argument(0, encode_value(m))],
        String,
    )
end

function get_module_mass_change_when(client::Client, m::PartModule)::ModifierChangeWhen
    x = procedure_call(
        client,
        "KerbTk",
        "GetModuleMassChangeWhen",
        [Schema.Argument(0, encode_value(m))],
        Int32,
    )
    if x > 2
        x = 0
    end
    return ModifierChangeWhen(x)
end

@enum StagingSituation begin
    Current
    Unstaged
    Staged
end

function get_module_mass(
    client::Client, m::PartModule, default_mass::Float32, situation::StagingSituation
)::Float32
    return procedure_call(
        client,
        "KerbTk",
        "GetModuleMass",
        [
            Schema.Argument(0, encode_value(m)),
            Schema.Argument(1, encode_value(default_mass)),
            Schema.Argument(2, encode_value(Int32(situation))),
        ],
        Float32,
    )
end

function get_all(client::Client, r::Resources)::Vector{Resource}
    return procedure_call(
        client,
        "SpaceCenter",
        "Resources_get_All",
        [Schema.Argument(0, encode_value(r))],
        Vector{Resource},
    )
end

function get_density(client::Client, r::Resource)::Float32
    return procedure_call(
        client,
        "SpaceCenter",
        "Resource_get_Density",
        [Schema.Argument(0, encode_value(r))],
        Float32,
    ) / 1000.0
end

function get_name(client::Client, r::Resource)::String
    return procedure_call(
        client,
        "SpaceCenter",
        "Resource_get_Name",
        [Schema.Argument(0, encode_value(r))],
        String,
    )
end

function get_max_amount(client::Client, r::Resource)::Float32
    return procedure_call(
        client,
        "SpaceCenter",
        "Resource_get_Max",
        [Schema.Argument(0, encode_value(r))],
        Float32,
    )
end

function get_amount(client::Client, r::Resource)::Float32
    return procedure_call(
        client,
        "SpaceCenter",
        "Resource_get_Amount",
        [Schema.Argument(0, encode_value(r))],
        Float32,
    )
end

function get_enabled(client::Client, r::Resource)::Bool
    return procedure_call(
        client,
        "SpaceCenter",
        "Resource_get_Enabled",
        [Schema.Argument(0, encode_value(r))],
        Bool,
    )
end

function get_flow_mode(client::Client, r::Resource)::FlowMode
    return FlowMode(
        procedure_call(
            client,
            "KerbTk",
            "ResourceFlowMode",
            [Schema.Argument(0, encode_value(r))],
            Int32,
        ),
    )
end

function get_id(client::Client, r::Resource)::Int32
    return procedure_call(
        client, "KerbTk", "ResourceId", [Schema.Argument(0, encode_value(r))], Int32
    )
end

function get_is_omni_decoupler(client::Client, d::Decoupler)::Bool
    return procedure_call(
        client,
        "SpaceCenter",
        "Decoupler_get_IsOmniDecoupler",
        [Schema.Argument(0, encode_value(d))],
        Bool,
    )
end

function get_attached_part(client::Client, d::Decoupler)::Union{Part,Nothing}
    return procedure_call(
        client,
        "SpaceCenter",
        "Decoupler_get_AttachedPart",
        [Schema.Argument(0, encode_value(d))],
        Union{Part,Nothing},
    )
end

function get_top_decoupler(client::Client, d::RODecoupler)::Union{Decoupler,Nothing}
    return procedure_call(
        client,
        "KerbTk",
        "RODecoupler_get_TopDecoupler",
        [Schema.Argument(0, encode_value(d))],
        Union{Decoupler,Nothing},
    )
end

function get_bottom_decoupler(client::Client, d::RODecoupler)::Union{Decoupler,Nothing}
    return procedure_call(
        client,
        "KerbTk",
        "RODecoupler_get_BottomDecoupler",
        [Schema.Argument(0, encode_value(d))],
        Union{Decoupler,Nothing},
    )
end

using ..Arenas: Arena
using ..Vessels:
    Vessels, PartId, TrackedId, RadialAttach, AxialAttach, NoAttach, Engine, MassModifier
using ..FFSTypes: Propellant, ResourceId, SimPartId
using ..Math: Math, H1, h1_add!

function load_parts_from_editor(
    client::Client
)::Tuple{Arena{PartId,Vessels.Part},Union{PartId,Nothing},Dict{TrackedId,PartId}}
    map = Dict{Part,PartId}()
    parts = Arena{PartId,Vessels.Part}()
    tracked_id_map = Dict{TrackedId,PartId}()

    editor = get_editor(client)
    current_ship = get_current_ship(client, editor)
    isnothing(current_ship) && error("No ship in editor.")

    root = nothing
    for part in get_all(client, get_parts(client, current_ship))
        new_root = load_part(client, part, map, parts, tracked_id_map)
        if !isnothing(new_root)
            root = new_root
        end
    end

    return (parts, root, tracked_id_map)
end

function load_parts_from_flight(
    client::Client
)::Tuple{Arena{PartId,Vessels.Part},Union{PartId,Nothing},Dict{TrackedId,PartId}}
    map = Dict{Part,PartId}()
    parts = Arena{PartId,Vessels.Part}()
    tracked_id_map = Dict{TrackedId,PartId}()

    active_vessel = get_active_vessel(client)
    isnothing(active_vessel) && error("No flight in progress.")

    root = nothing
    for part in get_all(client, get_parts(client, active_vessel))
        new_root = load_part(client, part, map, parts, tracked_id_map)
        if !isnothing(new_root)
            root = new_root
        end
    end

    return (parts, root, tracked_id_map)
end

function load_part(
    client::Client,
    part::Part,
    d::Dict{Part,PartId},
    parts::Arena{PartId,Vessels.Part},
    tracked_id_map::Dict{TrackedId,PartId},
)::Union{PartId,Nothing}
    root = nothing

    name = get_name(client, part)
    title = get_title(client, part)
    tag = get_tag(client, part)

    pfdecoupler = get_pf_decoupler(client, part)
    rodecoupler = get_ro_decoupler(client, part)
    decoupler = get_decoupler(client, part)
    decouplers = if !isnothing(pfdecoupler)
        Vessels.Decouplers(nothing, nothing, Vessels.PFDecoupler())
    elseif !isnothing(rodecoupler)
        top = get_top_decoupler(client, rodecoupler)
        bot = get_bottom_decoupler(client, rodecoupler)
        top = begin
            is_omni_decoupler = get_is_omni_decoupler(client, top)
            attached_part = get_attached_part(client, top)
            attached_part = if !isnothing(attached_part)
                get_or_insert(d, part) do
                    push!(parts, Vessels.Part())
                end
            else
                nothing
            end
            Vessels.Decoupler(is_omni_decoupler, attached_part)
        end
        bot = begin
            is_omni_decoupler = get_is_omni_decoupler(client, bot)
            attached_part = get_attached_part(client, bot)
            attached_part = if !isnothing(attached_part)
                get_or_insert(d, part) do
                    push!(parts, Vessels.Part())
                end
            else
                nothing
            end
            Vessels.Decoupler(is_omni_decoupler, attached_part)
        end
        Vessels.Decouplers(top, bot, nothing)
    elseif !isnothing(decoupler)
        is_omni_decoupler = get_is_omni_decoupler(client, decoupler)
        attached_part = get_attached_part(client, decoupler)
        attached_part = if !isnothing(attached_part)
            get_or_insert(d, part) do
                push!(parts, Vessels.Part())
            end
        else
            nothing
        end
        Vessels.Decouplers(Vessels.Decoupler(is_omni_decoupler, attached_part), nothing, nothing)
    else
        Vessels.Decouplers(nothing, nothing, nothing)
    end

    children = map(get_children(client, part)) do (child)
        get_or_insert(d, child) do
            push!(parts, Vessels.Part())
        end
    end
    parent = get_parent(client, part)
    parent = if !isnothing(parent)
        get_or_insert(d, parent) do
            push!(parts, Vessels.Part())
        end
    else
        nothing
    end

    attachment = if get_radially_attached(client, part)
        RadialAttach
    elseif get_axially_attached(client, part)
        AxialAttach
    else
        NoAttach
    end

    crossfeed_part_set = map(get_crossfeed_part_set(client, part)) do (part)
        get_or_insert(d, part) do
            push!(parts, Vessels.Part())
        end
    end

    (mass, dry_mass, crew_mass) = get_part_masses(client, part)

    params_curve = get_engine_params_curve(client, part)
    params_bool = get_engine_params_bool(client, part)
    params_f64 = get_engine_params_f64(client, part)
    ttm = get_engine_thrust_transform_multipliers(client, part)
    ttv = get_engine_thrust_transforms(client, part)
    prop = get_engine_propellants(client, part)
    engines = Engine[]

    # TODO: Engine names for display
    for engine_id in keys(params_curve)
        propellants = Dict(
            map(prop[engine_id]) do (id, ignore_for_isp, ratio, flow_mode, density)
                (
                    ResourceId(id % UInt64),
                    Propellant(ignore_for_isp, ratio, FlowMode(flow_mode), density),
                )
            end,
        )
        params_f64 = params_f64[engine_id]
        params_bool = params_bool[engine_id]
        params_curve = params_curve[engine_id]

        engine = Engine(
            propellants,
            Dict{ResourceId,FlowMode}(),
            Dict{ResourceId,Float64}(),
            map(x -> Float64(x), ttm[engine_id]),
            map(x -> SVector{3}(x[1], x[2], x[3]), ttv[engine_id]),
            true,
            1.0,
            SVector{3}(0.0, 0.0, 0.0),
            SVector{3}(0.0, 0.0, 0.0),
            SVector{3}(0.0, 0.0, 0.0),
            0.0,
            0.0,
            params_f64["g"],
            params_f64["maxFuelFlow"],
            params_f64["maxThrust"],
            params_f64["minFuelFlow"],
            params_f64["minThrust"],
            params_f64["multIsp"],
            params_f64["clamp"],
            params_f64["flowMultCap"],
            params_f64["flowMultCapSharpness"],
            params_bool["throttleLocked"],
            params_f64["throttleLimiter"],
            params_bool["atmChangeFlow"],
            params_bool["useAtmCurve"],
            params_bool["useAtmCurveIsp"],
            params_bool["useThrottleIspCurve"],
            params_bool["useVelCurve"],
            params_bool["useVelCurveIsp"],
            params_f64["moduleResiduals"],
            params_f64["moduleSpoolupTime"],
            false,
            false,
            params_bool["isModuleEnginesRf"],
            H1(params_curve["throttleIspCurve"]),
            H1(params_curve["throttleIspCurveAtmStrength"]),
            H1(params_curve["velCurve"]),
            H1(params_curve["velCurveIsp"]),
            H1(params_curve["atmCurve"]),
            H1(params_curve["atmCurveIsp"]),
            H1(params_curve["atmosphereCurve"]),
            false,
            SimPartId(typemax(UInt64)),
        )
        push!(engines, engine)
    end

    resources = get_resources(client, part)
    resources = get_all(client, resources)
    part_resources = Dict{ResourceId,Vessels.Resource}()
    disabled_resource_mass = 0.0
    for resource in resources
        id = ResourceId(get_id(client, resource) % UInt64)
        density = get_density(client, resource)
        amount = get_amount(client, resource)
        part_resources[id] = Vessels.Resource(
            density <= eps(Float32),
            Float64(get_max_amount(client, resource)),
            Float64(amount),
            Float64(density),
            0.0,
            get_enabled(client, resource),
            get_name(client, resource),
        )

        if !part_resources[id].enabled
            disabled_resource_mass += Float64(amount) * Float64(density)
        end
    end

    mass_modifiers = map(get_part_mass_modifiers(client, part)) do (x)
        changes_when = get_module_mass_change_when(client, x)
        current_mass = get_module_mass(client, x, Float32(dry_mass), Current)
        staged_mass = get_module_mass(client, x, Float32(dry_mass), Staged)
        unstaged_mass = get_module_mass(client, x, Float32(dry_mass), Unstaged)
        MassModifier(
            current_mass, staged_mass, unstaged_mass, changes_when, get_name(client, x)
        )
    end

    tracked_id = TrackedId(get_tracked_id(client, part))

    part1 = Vessels.Part(
        tracked_id,
        parent,
        children,
        name,
        title,
        tag,
        decouplers,
        attachment,
        crossfeed_part_set,
        part_resources,
        get_resource_priority(client, part),
        get_resource_request_remaining_threshold(client, part),
        mass,
        dry_mass,
        crew_mass,
        disabled_resource_mass,
        !isnothing(get_launch_clamp(client, part)),
        engines,
        mass_modifiers,
    )

    if haskey(d, part)
        id = d[part]
        if isnothing(parent)
            root = id
        end
        parts[id] = part1
        tracked_id_map[tracked_id] = id
    else
        id = push!(parts, part1)
        if isnothing(parent)
            root = id
        end
        d[part] = id
        tracked_id_map[tracked_id] = id
    end

    return root
end

function get_or_insert(f::Function, d::AbstractDict{K,V}, k::K) where {K,V}
    if haskey(d, k)
        return d[k]
    else
        d[k] = f()
    end
end

function Math.H1(v::Vector{Tuple{Float64,Float64,Float64,Float64}})::H1
    h = H1()
    for (t, x, i, o) in v
        h1_add!(h, t, x, i, o)
    end
    return h
end

end
