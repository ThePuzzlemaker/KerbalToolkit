module Support

global ktk_support_take_error = nothing

function take_error()::Union{String,Nothing}
    ccall(ktk_support_take_error, Any, ())
end

macro check(code)
    quote
        __final = $(esc(code))
        __res = take_error()
        if isnothing(__res)
            __final
        else
            error(__res)
        end
    end
end

end # KerbTk.Support
