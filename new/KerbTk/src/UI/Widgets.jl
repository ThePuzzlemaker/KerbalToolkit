module Widgets

using ..UI: UiCfg, UI

import CImGui as ig
using CImGui.lib
using CImGui.CSyntax: @c

export Spinner

mutable struct InputTextCallbackUserData
    str::Ref{Vector{UInt8}}
    chain_cb::ImGuiInputTextCallback
    chain_data::Ptr{Nothing}
end

function ptroffset(ptr::Ptr{T}, ty::Type{T}, name::Symbol, ::Type{U})::Ptr{U} where {T,U}
    for i in 1:fieldcount(ty)
        if fieldname(ty, i) == name
            return convert(Ptr{U}, ptr + fieldoffset(ty, i))
        end
    end
    return nothing
end

function input_text_callback(data_ptr::Ptr{ImGuiInputTextCallbackData})::Cint
    user_data = convert(Ptr{InputTextCallbackUserData}, unsafe_load(data_ptr).UserData)
    user_data = unsafe_load(user_data)
    if unsafe_load(data_ptr).EventFlag == ImGuiInputTextFlags_CallbackResize
        @assert pointer(user_data.str[]) == unsafe_load(data_ptr).Buf
        resize!(user_data.str[], unsafe_load(data_ptr).BufTextLen + 1)
        user_data.str[][end] = 0x00
        unsafe_store!(
            ptroffset(data_ptr, ImGuiInputTextCallbackData, :Buf, Ptr{UInt8}),
            pointer(user_data.str[]),
        )
    elseif user_data.chain_cb != C_NULL
        unsafe_store!(
            ptroffset(data_ptr, ImGuiInputTextCallbackData, :UserData, Ptr{Nothing}),
            user_data.chain_data,
        )
        return ccall(
            user_data.chain_cb, Int32, (Ptr{ImGuiInputTextCallbackData},), data_ptr
        )
    end
    return 0
end

function nullstring(v::Vector{UInt8})
    # Find first zero
    zeropos = 0
    @inbounds for i in eachindex(v)
        iszero(v[i]) && (zeropos = i; break)
    end
    iszero(zeropos) && error("Not null-terminated")
    GC.@preserve v unsafe_string(pointer(v), zeropos - 1)
end

function ig.InputText(
    label::String,
    str::Ref{String},
    flags::Union{ImGuiInputTextFlags_,Integer},
    callback::Union{Ptr{Nothing},Base.CFunction},
    user_data::Ptr{Nothing},
)
    @assert (flags & ImGuiInputTextFlags_CallbackResize) == 0
    flags = flags | ImGuiInputTextFlags_CallbackResize

    strdata = Ref(Vector{UInt8}(str[]))
    push!(strdata[], 0x0)
    data = Ref(InputTextCallbackUserData(strdata, callback, user_data))
    res = GC.@preserve strdata ig.InputText(
        label,
        Base.cconvert(Ptr{UInt8}, strdata[]),
        length(strdata[]),
        flags,
        @cfunction(input_text_callback, Int32, (Ptr{ImGuiInputTextCallbackData},)),
        data,
    )
    str[] = nullstring(strdata[])
    return res
end

function ig.InputTextWithHint(
    label::String,
    hint::String,
    str::Ref{String},
    flags::Union{ImGuiInputTextFlags_,Integer},
    callback::Union{Ptr{Nothing},Base.CFunction},
    user_data::Ptr{Nothing},
)
    @assert (flags & ImGuiInputTextFlags_CallbackResize) == 0
    flags = flags | ImGuiInputTextFlags_CallbackResize

    strdata = Ref(Vector{UInt8}(str[]))
    push!(strdata[], 0x0)
    data = Ref(InputTextCallbackUserData(strdata, callback, user_data))
    res = GC.@preserve strdata ig.InputTextWithHint(
        label,
        hint,
        Base.cconvert(Ptr{UInt8}, strdata[]),
        length(strdata[]),
        flags,
        @cfunction(input_text_callback, Int32, (Ptr{ImGuiInputTextCallbackData},)),
        data,
    )
    str[] = nullstring(strdata[])
    return res
end

function ig.InputTextMultiline(
    label::String,
    str::Ref{String},
    size::Union{ImVec2,Tuple{T,T} where T},
    flags::Union{ImGuiInputTextFlags_,Integer},
    callback::Union{Ptr{Nothing},Base.CFunction},
    user_data::Ptr{Nothing},
)
    @assert (flags & ImGuiInputTextFlags_CallbackResize) == 0
    flags = flags | ImGuiInputTextFlags_CallbackResize

    strdata = Ref(Vector{UInt8}(str[]))
    push!(strdata[], 0x0)
    data = Ref(InputTextCallbackUserData(strdata, callback, user_data))
    res = GC.@preserve strdata ig.InputTextMultiline(
        label,
        Base.cconvert(Ptr{UInt8}, strdata[]),
        length(strdata[]),
        size,
        flags,
        @cfunction(input_text_callback, Int32, (Ptr{ImGuiInputTextCallbackData},)),
        data,
    )
    str[] = nullstring(strdata[])
    return res
end

function Spinner(ui::UiCfg)
    size = ui.sf * 20.0
    color = ig.IM_COL32_WHITE
    radius = (size / 2.0) - ui.sf * 3.5
    time = ig.GetTime()
    start_angle = 2π * time
    end_angle = start_angle + deg2rad(240) * sin(time)
    p = ig.GetCursorScreenPos()
    dist = sqrt((radius / 2)^2 + (radius / 2)^2)
    center = [p.x + dist + ui.sf * 2.0, p.y + dist + ui.sf * 6.0]
    drawlist = ig.GetWindowDrawList()
    ig.PathArcTo(drawlist, (center[1], center[2]), radius, start_angle, end_angle)
    return ig.PathStroke(drawlist, color, ImDrawFlags_None, 3.0 * ui.sf)
end

function lerp(a::Real, b::Real, t::Real)
    return (one(t) - t) * a + t * b
end

using ...Time: UT, GET, days, hours, minutes, seconds, millis
using Printf: @sprintf

@enum TimeInputKind begin
    InputUT
    InputGET
end

@enum TimeDisplayKind begin
    DisplayDhms
    DisplaySec
end

function with_duration(kind::TimeInputKind, dur::Float64)::Union{UT,GET}
    return kind == InputUT ? UT(dur) : GET(dur)
end

struct TimeInput
    id::String
    buf::Ref{String}
    parsed::Ref{Union{UT,GET,Nothing}}
    desired_width::Union{Float64,Nothing}
    kind::Ref{TimeInputKind}
    disp::Ref{TimeDisplayKind}
    interactive::Bool
    allow_neg::Bool
end

struct TimeDisplayBtn
    disp::Ref{TimeDisplayKind}
end

function UI.show_ui(btn::TimeDisplayBtn, ::UiCfg)
    dirty = false
    ig.BeginGroup()
    if ig.Button("\UE492")
        btn.disp[] = btn.disp[] == DisplayDhms ? DisplaySec : DisplayDhms
        dirty = true
    end
    ig.SetItemTooltip("Toggle Time Units")
    ig.EndGroup()
    return dirty
end

function parse_dhms_duration(s::String, allow_neg::Bool)
    posregex = r"(?:([0-9]+(?:\.[0-9]+)?)d)?\s?(?:([0-9]+(?:\.[0-9]+)?)h)?\s?(?:([0-9]+(?:\.[0-9]+)?)m(?:in)?)?\s?(?:([0-9]+(?:\.[0-9]+)?)s)?"
    negregex = r"(-?)(?:([0-9]+(?:\.[0-9]+)?)d)?\s?(?:([0-9]+(?:\.[0-9]+)?)h)?\s?(?:([0-9]+(?:\.[0-9]+)?)m(?:in)?)?\s?(?:([0-9]+(?:\.[0-9]+)?)s)?"

    if isempty(s)
        return nothing
    end

    (neg, d, h, m, s) = if allow_neg
        m = match(negregex, s)
        (isnothing(m) || m.match != s) && return nothing
        (m[1] == "-" ? -1.0 : 1.0, m[2], m[3], m[4], m[5])
    else
        m = match(posregex, s)
        (isnothing(m) || m.match != s) && return nothing
        (1.0, m[1], m[2], m[3], m[4])
    end

    if isnothing(d) && isnothing(h) && isnothing(m) && isnothing(s)
        return nothing
    end

    try
        d = isnothing(d) ? 0.0 : parse(Float64, d)
        h = isnothing(h) ? 0.0 : parse(Float64, h)
        m = isnothing(m) ? 0.0 : parse(Float64, m)
        s = isnothing(s) ? 0.0 : parse(Float64, s)
    catch _
        return nothing
    end

    return neg * (d * 60 * 60 * 24 + h * 60 * 60 + m * 60 + s)
end

function parse_dhms_time(s::String, allow_neg::Bool)
    posregex = r"([0-9]+):([0-9]+):([0-9]+)(?::([0-9]+))?(?:\.([0-9]+))?"
    negregex = r"(-?)([0-9]+):([0-9]+):([0-9]+)(?::([0-9]+))?(?:\.([0-9]+))?"

    if isempty(s)
        return nothing
    end

    (neg, n1, n2, n3, n4, millis) = if allow_neg
        m = match(negregex, s)
        (isnothing(m) || m.match != s) && return nothing
        (m[1] == "-" ? -1.0 : 1.0, m[2], m[3], m[4], m[5], m[6])
    else
        m = match(posregex, s)
        (isnothing(m) || m.match != s) && return nothing
        (1.0, m[1], m[2], m[3], m[4], m[5])
    end

    if isnothing(n1) && isnothing(n2) && isnothing(n3)
        return nothing
    end

    (d, h, m, s) = if !isnothing(n4)
        (n1, n2, n3, n4)
    else
        ("0.0", n1, n2, n3)
    end

    millismag = 0.0
    try
        d = isnothing(d) ? 0.0 : parse(Float64, d)
        h = isnothing(h) ? 0.0 : parse(Float64, h)
        m = isnothing(m) ? 0.0 : parse(Float64, m)
        s = isnothing(s) ? 0.0 : parse(Float64, s)
        millismag = isnothing(millis) ? 0.0 : length(millis)
        millis = isnothing(millis) ? 0.0 : parse(Float64, millis)
    catch _
        return nothing
    end

    return neg * (millis / 10^millismag + s + m * 60 + h * 60 * 60 + d * 60 * 60 * 24)
end

function parse_sec_time(input::String, allow_neg::Bool)
    if isempty(input)
        return nothing
    end

    neg = 1.0
    if startswith(input, '-') && allow_neg
        input = input[2:end]
        neg = -1.0
    elseif startswith(input, '-') && !allow_neg
        return nothing
    end

    input = split(input, '.')

    if length(input) > 2
        return nothing
    end

    millismag = 0.0
    (sec, millis) = try
        sec = parse(UInt64, input[1])
        millismag = length(input) == 2 ? length(input[2]) : 0.0
        millis = length(input) == 2 ? parse(UInt64, input[2]) : 0.0
        (sec, millis)
    catch _
        return nothing
    end

    return neg * millis / 10^millismag + sec
end

function UI.show_ui(inp::TimeInput, ui::UiCfg)
    ig.BeginGroup()

    @assert inp.interactive "TODO"

    val = nothing
    if (val = parse_dhms_duration(inp.buf[], inp.allow_neg); !isnothing(val)) ||
        (val = parse_dhms_time(inp.buf[], inp.allow_neg); !isnothing(val)) ||
        (val = parse_sec_time(inp.buf[], inp.allow_neg); !isnothing(val))
        inp.parsed[] = with_duration(inp.kind[], val)
    else
        inp.parsed[] = nothing
    end

    passive = !isempty(inp.buf[])
    if isnothing(inp.parsed[])
        ig.PushStyleColor(ImGuiCol_FrameBgActive, (1.0, 0.0, 0.0, 1.0))
        if passive
            ig.PushStyleColor(ImGuiCol_FrameBg, (1.0, 0.0, 0.0, 1.0))
        end
    end

    if !isnothing(inp.desired_width)
        ig.PushItemWidth(inp.desired_width)
    else
        width = max(
            ui.sf * 128.0,
            ig.CalcTextSize(inp.buf[]).x + (unsafe_load(ig.GetStyle().FramePadding.x) * 4),
        )
        ig.PushItemWidth(width)
    end
    ig.InputText(inp.id, inp.buf, 0, C_NULL, C_NULL)
    ig.PopItemWidth()
    focus = ig.IsItemFocused()
    done = ig.IsItemDeactivatedAfterEdit()

    if isnothing(inp.parsed[])
        ig.PopStyleVar(ImGuiStyleVar_FrameBorderSize)
        if passive
            ig.PopStyleColor()
        end
        ig.PopStyleColor()
    end

    if (done || !focus) && !isnothing(inp.parsed[])
        t = inp.parsed[]
        if inp.disp[] == DisplayDhms
            d = abs(days(t))
            h = abs(hours(t))
            m = abs(minutes(t))
            s = abs(seconds(t))
            ms = abs(millis(t))
            n = convert(Float64, t) < 0.0 ? "-" : ""
            inp.buf[] = @sprintf("%s%03i:%02i:%02i:%02i.%03i", n, d, h, m, s, ms)
        else
            inp.buf[] = @sprintf("%.03f", t.value)
        end
    end

    ig.EndGroup()

    return true
end

end
