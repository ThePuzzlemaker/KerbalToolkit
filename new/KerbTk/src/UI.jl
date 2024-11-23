module UI

import ..KRPC
using ..Missions: Mission
using ..Bodies: SolarSystem

import CImGui as ig
using ModernGL: ModernGL
using GLFW: GLFW
using NativeFileDialog: NativeFileDialog
using MsgPack: MsgPack

using CImGui.lib
using CImGui.CSyntax: @c
using Match: @match
using DataStructures: Deque
using Base.Threads: @spawn
using Base: Lockable

macro Window(app, title, expr)
    quote
        ig.PushFont($(esc(app)).ui.fonts.title)
        if ig.Begin($(esc(title)))
            ig.PopFont()
            $(esc(expr))
            ig.End()
        else
            ig.PopFont()
        end
    end
end

macro Display(ui, dis, title, pre, expr)
    quote
        if $(esc(dis))[]
            $(esc(pre))
            ig.PushFont($(esc(ui)).fonts.title)
            if ig.Begin($(esc(title)), $(esc(dis)))
                ig.PopFont()
                $(esc(expr))
                ig.End()
            else
                ig.PopFont()
            end
        end
    end
end

module DisplaySelect
    const SYSCFG = UInt16(0)
    const KRPC = UInt16(1)

    const CLASSES = UInt16(300)
end

struct Fonts
    regular::Ptr{ImFont}
    bold::Ptr{ImFont}
    mono::Ptr{ImFont}
    title::Ptr{ImFont}
    heading::Ptr{ImFont}
end

struct Displays
    krpc::Ref{Bool}
    syscfg::Ref{Bool}
    classes::Ref{Bool}

    Displays() = new(Ref(false), Ref(false), Ref(false))
end

mutable struct Menu
    id::String
    selector::String

    Menu() = new("Menu", '\0'^4)
end

struct UiCfg
    fonts::Fonts
    sf::Float64
end

abstract type AbstractState end

function handle_rx end
function show_display end
function show_ui end

include("UI/BackendHandler.jl")
include("UI/Widgets.jl")
include("UI/Utils.jl")
include("UI/Vessels.jl")

using .BackendHandler
using .Widgets
using .Utils
using .Vessels

function handle_rx(::Any, ::UiCfg, ::Any, ::Displays, ::Backend, ::Lockable{Mission})
    return error("Fallback handle_rx used")
end
function show_display(::UiCfg, ::Any, ::Displays, ::Backend, ::Lockable{Mission})
    return error("Fallback show_display used")
end
function show_ui(::Any, ::UiCfg)
    return error("Fallback show_ui used")
end

mutable struct State <: AbstractState
    dis::Displays
    menu::Menu
    krpc::Krpc
    syscfg::SysCfg
    classes::Classes

    State() = new(Displays(), Menu(), Krpc(), SysCfg(), Classes())
end

struct App
    mission::Lockable{Mission}
    ui::UiCfg
    state::State
    backend::Backend

    function App(mission::Lockable{Mission}, ui::UiCfg, state::State)
        return new(mission, ui, state, Backend())
    end
end

text_filter_decimal(data::Ptr{ImGuiInputTextCallbackData})::Cint =
    ('0' <= Char(unsafe_load(data).EventChar) <= '9') ? 0 : 1

function (@main)(::Vector{String})
    ig.set_backend(:GlfwOpenGL3)
    ctx = ig.CreateContext()

    # TODO: oops
    sf = 1.4

    io = ig.GetIO()
    io.ConfigFlags = unsafe_load(io.ConfigFlags) | ImGuiConfigFlags_DockingEnable

    fontcfg = ig.ImFontConfig()
    fontcfg.MergeMode = true
    fontcfg.GlyphMinAdvanceX = floor(sf * 16.0)
    fontcfg.GlyphOffset = ImVec2(0.0, sf * 2.5)

    ranges = ig.ImVector_ImWchar_create()
    builder = ig.ImFontGlyphRangesBuilder()
    ig.AddText(builder, "\UE256\UEAF4\UE492\UE34C\UE4A6")
    ig.BuildRanges(builder, ranges)

    regular = ig.AddFontFromFileTTF(
        unsafe_load(io.Fonts),
        joinpath(@__DIR__, "assets", "Inter", "Inter-Regular.ttf"),
        floor(sf * 16.0),
    )
    ig.AddFontFromFileTTF(
        unsafe_load(io.Fonts),
        joinpath(@__DIR__, "assets", "Phosphor.ttf"),
        floor(sf * 16.0),
        fontcfg,
        unsafe_load(ranges).Data,
    )

    bold = ig.AddFontFromFileTTF(
        unsafe_load(io.Fonts),
        joinpath(@__DIR__, "assets", "Inter", "Inter-Bold.ttf"),
        floor(sf * 16.0),
    )
    title = ig.AddFontFromFileTTF(
        unsafe_load(io.Fonts),
        joinpath(@__DIR__, "assets", "Inter", "Inter-ExtraBold.ttf"),
        floor(sf * 22.0),
    )
    mono = ig.AddFontFromFileTTF(
        unsafe_load(io.Fonts),
        joinpath(@__DIR__, "assets", "IBMPlexMono-Regular.ttf"),
        floor(sf * 16.0),
    )
    heading = ig.AddFontFromFileTTF(
        unsafe_load(io.Fonts),
        joinpath(@__DIR__, "assets", "Inter", "Inter-Regular.ttf"),
        floor(sf * 22.0),
    )
    ig.Build(unsafe_load(io.Fonts))

    ImFontGlyphRangesBuilder_destroy(builder)
    ImVector_ImWchar_destroy(ranges)
    ig.ImFontConfig_destroy(fontcfg)

    fonts = Fonts(regular, bold, mono, title, heading)

    mission = Lockable(Mission())

    app = App(mission, UiCfg(fonts, sf), State())

    backend_task = @spawn backend(app.backend)

    style = ig.GetStyle()
    style.FrameRounding = 2.5
    style.WindowRounding = 5.0
    style.WindowTitleAlign = ImVec2(0.5, 0.5)
    ig.ScaleAllSizes(style, sf)

    ig.render(ctx; window_title="KerbalToolkit") do
        while isready(app.backend.resc)
            (txi, tag, res...) = take!(app.backend.resc)
            sel = app.backend.txq[txi]
            delete!(app.backend.txq, txi)

            if sel == DisplaySelect.SYSCFG
                handle_rx(
                    (tag, res...),
                    app.ui,
                    app.state.syscfg,
                    app.state.dis,
                    app.backend,
                    app.mission,
                )
            elseif sel == DisplaySelect.KRPC
                handle_rx(
                    (tag, res...),
                    app.ui,
                    app.state.krpc,
                    app.state.dis,
                    app.backend,
                    app.mission,
                )
            elseif sel == DisplaySelect.CLASSES
                handle_rx(
                    (tag, res...),
                    app.ui,
                    app.state.classes,
                    app.state.dis,
                    app.backend,
                    app.mission,
                )
            end
        end

        ig.DockSpaceOverViewport(0, ig.GetMainViewport())
        ig.SetNextWindowSize((0.0, 0.0), ImGuiCond_Always)
        @Window app "Menu" begin
            if ig.Button("\UE256 Load Mission")
                try
                    file = NativeFileDialog.pick_file()
                    if !isempty(file)
                        newmission = MsgPack.unpack(read(file), Mission)
                        @lock mission begin
                            mission[].system = newmission.system
                        end
                    end
                catch e
                    @warn "Failed to load mission: $e"
                end
            end
            ig.SameLine()
            if ig.Button("\UEAF4 Save Mission")
                try
                    file = NativeFileDialog.save_file()
                    if !isempty(file)
                        data = @lock mission MsgPack.pack(mission[])
                        @assert write(file, data) == length(data)
                    end
                catch e
                    @warn "Failed to save mission: $e"
                end
            end

            ig.AlignTextToFramePadding()
            ig.Text("Display Select")

            ig.SameLine()
            ig.PushFont(app.ui.fonts.mono)
            ig.PushItemWidth(4.5 * ig.CalcTextSize("0").x)
            @c ig.InputText(
                "##DisplaySelect",
                &app.state.menu.selector,
                ImGuiInputTextFlags_CallbackCharFilter,
                @cfunction(text_filter_decimal, Cint, (Ptr{ImGuiInputTextCallbackData},)),
                C_NULL,
            )
            done = ig.IsItemDeactivatedAfterEdit()
            ig.PopItemWidth()
            ig.PopFont()

            ig.SameLine()
            clicked = ig.Button("Open")
            if (done && ig.IsKeyDown(ImGuiKey_Enter)) || clicked
                selector = swapfield!(app.state.menu, :selector, "")
                open_window(app, selector)
            end

            ig.Separator()

            openall = false
            closeall = false
            if ig.Button("Expand All")
                openall = true
            end
            ig.SameLine()
            if ig.Button("Collapse All")
                closeall = true
            end

            (openall || closeall) && ig.SetNextItemOpen(openall || !closeall)
            if ig.TreeNode("00XX: Config and Utilities")
                ig.Checkbox("0000: System Configuration", app.state.dis.syscfg)
                ig.Checkbox("0001: kRPC Configuration", app.state.dis.krpc)
                ig.TreePop()
            end
            (openall || closeall) && ig.SetNextItemOpen(openall || !closeall)
            if ig.TreeNode("01XX: Mission Plan Table")
                ig.TreePop()
            end
            (openall || closeall) && ig.SetNextItemOpen(openall || !closeall)
            if ig.TreeNode("02XX: State Vectors")
                ig.TreePop()
            end
            (openall || closeall) && ig.SetNextItemOpen(openall || !closeall)
            if ig.TreeNode("03XX: Vessels and Classes")
                ig.Checkbox("0300: Vessel Classes", app.state.dis.classes)
                ig.TreePop()
            end
            (openall || closeall) && ig.SetNextItemOpen(openall || !closeall)
            if ig.TreeNode("04XX: Maneuver Targeting")
                ig.TreePop()
            end
        end

        show_display(app.ui, app.state.syscfg, app.state.dis, app.backend, app.mission)
        show_display(app.ui, app.state.krpc, app.state.dis, app.backend, app.mission)
        show_display(app.ui, app.state.classes, app.state.dis, app.backend, app.mission)
    end
    @spawn Base.throwto(backend_task, InterruptException())
    return nothing
end

function open_window(app::App, selector::String)
    try
        selector = parse(UInt16, selector)
        if selector == DisplaySelect.SYSCFG
            app.state.dis.syscfg[] = true
        elseif selector == DisplaySelect.KRPC
            app.state.dis.krpc[] = true
        elseif selector == DisplaySelect.CLASSES
            app.state.dis.classes[] = true
        else
            true
        end
    catch _
    end
end

end
