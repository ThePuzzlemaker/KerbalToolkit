module Support

export OpaqueLock, ReadLock, WriteLock

abstract type OpaqueLock end

mutable struct ReadLock{T,Opaque} <: Base.AbstractLock
    _inner::Ptr{Opaque}
    _lock::Ptr{OpaqueLock}
    _parent::Any
    _we_own_lock::Bool
end

function Base.getindex(lck::ReadLock{T,Opaque}) where {T,Opaque}
    if islocked(lck) && lck._we_own_lock
        return T(lck._inner, lck, true)
    else
        error("Attempted to access ReadLock which was not locked")
    end
end

function Base.show(io::IO, ::ReadLock{T,Opaque}) where {T,Opaque}
    # if !islocked(lck)
    #     lock(lck)
    #     try
    #         x = T(lck._inner, lck, true)
    #         write(io, "KerbTk.Support.ReadLock($(repr(x)))")
    #     finally
    #         unlock(lck)
    #     end
    # elseif lck._we_own_lock
    #     x = T(lck._inner, lck, true)
    #     write(io, "KerbTk.Support.ReadLock($(repr(x)))")
    # else
    write(io, "KerbTk.Support.ReadLock(<locked>)")
    # end
end

function Base.deepcopy_internal(::ReadLock{T,Opaque}, ::IdDict) where {T,Opaque}
    error("Not implemented")
end

global ktk_locks_is_locked_read = nothing
global ktk_locks_lock_read = nothing
global ktk_locks_try_lock_read = nothing
global ktk_locks_unlock_read = nothing

function Base.islocked(lck::ReadLock{T,Opaque}) where {T,Opaque}
    ccall(ktk_locks_is_locked_read, Bool, (Ptr{OpaqueLock},), lck._lock)
end

function Base.trylock(lck::ReadLock{T,Opaque}) where {T,Opaque}
    res = ccall(ktk_locks_try_lock_read, Bool, (Ptr{OpaqueLock},), lck._lock)
    if res
        lck._we_own_lock = true
    end
    res
end

function Base.lock(lck::ReadLock{T,Opaque}) where {T,Opaque}
    ccall(ktk_locks_lock_read, Nothing, (Ptr{OpaqueLock},), lck._lock)
    lck._we_own_lock = true
    nothing
end

function Base.unlock(lck::ReadLock{T,Opaque}) where {T,Opaque}
    ccall(ktk_locks_unlock_read, Nothing, (Ptr{OpaqueLock},), lck._lock)
    lck._we_own_lock = false
    nothing
end

mutable struct WriteLock{T,Opaque} <: Base.AbstractLock
    _inner::Ptr{Opaque}
    _lock::Ptr{OpaqueLock}
    _parent::Any
    _we_own_lock::Bool
end

function Base.getindex(lck::WriteLock{T,Opaque}) where {T,Opaque}
    if islocked(lck) && lck._we_own_lock
        return T(lck._inner, lck, false)
    else
        error("Attempted to access ReadLock which was not locked")
    end
end

function Base.show(io::IO, ::WriteLock{T,Opaque}) where {T,Opaque}
    # if !islocked(lck)
    #     lock(lck)
    #     try
    #         x = T(lck._inner, lck, false)
    #         write(io, "KerbTk.Support.WriteLock($(repr(x)))")
    #     finally
    #         unlock(lck)
    #     end
    # elseif lck._we_own_lock
    #     x = T(lck._inner, lck, false)
    #     write(io, "KerbTk.Support.WriteLock($(repr(x)))")
    # else
    write(io, "KerbTk.Support.WriteLock(<locked>)")
    # end
end

function Base.deepcopy_internal(::WriteLock{T,Opaque}, ::IdDict) where {T,Opaque}
    error("Not implemented")
end

global ktk_locks_is_locked_write = nothing
global ktk_locks_lock_write = nothing
global ktk_locks_try_lock_write = nothing
global ktk_locks_unlock_write = nothing

function Base.islocked(lck::WriteLock{T,Opaque}) where {T,Opaque}
    ccall(ktk_locks_is_locked_write, Bool, (Ptr{OpaqueLock},), lck._lock)
end

function Base.trylock(lck::WriteLock{T,Opaque}) where {T,Opaque}
    res = ccall(ktk_locks_try_lock_write, Bool, (Ptr{OpaqueLock},), lck._lock)
    if res
        lck._we_own_lock = true
    end
    res
end

function Base.lock(lck::WriteLock{T,Opaque}) where {T,Opaque}
    ccall(ktk_locks_lock_write, Nothing, (Ptr{OpaqueLock},), lck._lock)
    lck._we_own_lock = true
    nothing
end

function Base.unlock(lck::WriteLock{T,Opaque}) where {T,Opaque}
    ccall(ktk_locks_unlock_write, Nothing, (Ptr{OpaqueLock},), lck._lock)
    lck._we_own_lock = false
    nothing
end

end # KerbTk.Support
