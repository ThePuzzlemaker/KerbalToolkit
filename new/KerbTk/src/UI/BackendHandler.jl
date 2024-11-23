module BackendHandler

using MsgPack: MsgPack
using DataStructures: Deque

import ...KRPC
using ...Vessels: VesselClass
using ...Bodies: SolarSystem

export Backend, tx, tx_loopback, backend

mutable struct Backend
    reqc::Channel{Tuple{UInt64,Symbol,Vararg{Any}}}
    resc::Channel{Tuple{UInt64,Symbol,Vararg{Any}}}
    txc::UInt64
    txq::Dict{UInt64,UInt16}
    stq::Deque{Function}

    function Backend()
        return new(
            Channel{Tuple{UInt64,Symbol,Vararg{Any}}}(2),
            Channel{Tuple{UInt64,Symbol,Vararg{Any}}}(2),
            0,
            Dict(),
            Deque{Function}(),
        )
    end
end

function tx(backend::Backend, src::UInt16, proc::Symbol, args...)
    put!(backend.reqc, (backend.txc, proc, args...))
    backend.txq[backend.txc] = src
    backend.txc += 1
    return nothing
end

function tx_loopback(backend::Backend, src::UInt16, tag::Symbol, args...)
    put!(backend.resc, (backend.txc, tag, args...))
    backend.txq[backend.txc] = src
    backend.txc += 1
    return nothing
end

function backend(backend::Backend)
    try
        reqc = backend.reqc
        resc = backend.resc

        client = nothing
        while true
            (txi, proc, args...) = take!(reqc)
            if proc == :RPCConnect
                (host, port) = args
                try
                    if !isnothing(client)
                        close(client)
                    end
                    client = KRPC.connect("KerbTk", host, parse(UInt16, port))
                    status = KRPC.get_status(client)
                    put!(resc, (txi, :Connected, status.version))
                catch e
                    put!(resc, (txi, :Failure, e))
                end
            elseif proc == :RPCDisconnect
                if !isnothing(client)
                    close(client)
                end
                client = nothing
                put!(resc, (txi, :Disconnected))
            elseif proc == :LoadSystem
                try
                    put!(resc, (txi, :LoadedSystem, SolarSystem(client)))
                catch e
                    put!(resc, (txi, :Failure, e))
                end
            elseif proc == :LoadVesselPartsFromEditor
                try
                    parts = KRPC.load_parts_from_editor(client)
                    put!(resc, (txi, :LoadedVesselClass, parts...))
                catch e
                    put!(resc, (txi, :Failure, e))
                end
            else
                error("Invalid backend procedure")
            end
            yield()
        end
    catch e
        if !(e isa InterruptException)
            @warn "Backend error: $e"
            rethrow(e)
        end
    end
end

end
