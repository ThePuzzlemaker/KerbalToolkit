module KRPCTypes

export AbstractKRPCValue,
    CelestialBody,
    Orbit,
    ReferenceFrame,
    Vessel,
    Editor,
    EditorShip,
    Parts,
    EditorParts,
    Part,
    Decoupler,
    RODecoupler,
    PFDecoupler,
    Resources,
    PartModule,
    LaunchClamp,
    Resource

abstract type AbstractKRPCValue end

struct CelestialBody <: AbstractKRPCValue
    inner::UInt64
end

struct Orbit <: AbstractKRPCValue
    inner::UInt64
end

struct ReferenceFrame <: AbstractKRPCValue
    inner::UInt64
end

struct Vessel <: AbstractKRPCValue
    inner::UInt64
end

struct Editor <: AbstractKRPCValue
    inner::UInt64
end

struct EditorShip <: AbstractKRPCValue
    inner::UInt64
end

struct Parts <: AbstractKRPCValue
    inner::UInt64
end

struct EditorParts <: AbstractKRPCValue
    inner::UInt64
end

struct Part <: AbstractKRPCValue
    inner::UInt64
end

struct Decoupler <: AbstractKRPCValue
    inner::UInt64
end

struct RODecoupler <: AbstractKRPCValue
    inner::UInt64
end

struct PFDecoupler <: AbstractKRPCValue
    inner::UInt64
end

struct Resources <: AbstractKRPCValue
    inner::UInt64
end

struct PartModule <: AbstractKRPCValue
    inner::UInt64
end

struct LaunchClamp <: AbstractKRPCValue
    inner::UInt64
end

struct Resource <: AbstractKRPCValue
    inner::UInt64
end

end
