module Utils

import CImGui as ig
using CImGui.lib
using CImGui.CSyntax: @c
using Base: Lockable

using ..UI: UI, UiCfg, Displays, @Display, DisplaySelect
using ..Widgets: Spinner
using ..BackendHandler: Backend, tx
using ...Bodies: Body
using ...Missions: Mission

using ..UI: show_ui
using ..UI.Widgets:
    TimeInput,
    TimeInputKind,
    InputUT,
    InputGET,
    TimeDisplayKind,
    DisplayDhms,
    DisplaySec,
    TimeDisplayBtn
import ...Time

export Krpc, SysCfg

mutable struct Krpc
    id::String
    ip::String
    port::String
    status::String
    loading::Bool
    inp::TimeInput
    btn::TimeDisplayBtn

    function Krpc()
        disp = Ref(DisplayDhms)
        return new(
            "Krpc",
            "127.0.0.1",
            "50000",
            "Status: Not Connected",
            false,
            TimeInput(
                "##TimeInput",
                Ref(""),
                Ref{Union{Time.UT,Time.GET,Nothing}}(nothing),
                nothing,
                Ref(InputUT),
                disp,
                true,
                false,
            ),
            TimeDisplayBtn(disp),
        )
    end
end

function UI.handle_rx(
    res::Any, ::UiCfg, state::Krpc, ::Displays, ::Backend, ::Lockable{Mission}
)
    if res isa Exception
        rethrow(res) # TODO
    end

    (tag, res...) = res
    if tag == :Failure
        state.status = "Status: Connection failed:\nError: $(res[1])"
        @error "kRPC connection failed:\n$(res[1])"
        state.loading = false
    elseif tag == :Connected
        state.status = "Status: Connection successful\nkRPC version: v$(res[1])"
        @info "kRPC connection succesful, kRPC version: v$(res[1])"
        state.loading = false
    elseif tag == :Disconnected
        state.status = "Status: Not Connected"
        @info "kRPC disconnected"
        state.loading = false
    else
        error("unreachable")
    end

    return nothing
end

function UI.show_display(
    ui::UiCfg, state::Krpc, dis::Displays, backend::Backend, ::Lockable{Mission}
)
    @Display ui dis.krpc "kRPC Configuration" begin
        ig.SetNextWindowSize((0.0, 0.0), ImGuiCond_Always)
    end begin
        ig.AlignTextToFramePadding()
        ig.Text("Host")

        ig.SameLine()
        ig.PushItemWidth(ui.sf * 128.0)
        @c ig.InputText("##KRPCHost", &state.ip, 0, C_NULL, C_NULL)
        ig.PopItemWidth()

        ig.SameLine()
        ig.Text("RPC Port")

        ig.SameLine()
        ig.PushItemWidth(ui.sf * 48.0)
        @c ig.InputText("##KRPCPort", &state.port, 0, C_NULL, C_NULL)
        ig.PopItemWidth()

        if ig.Button("Connect")
            tx(backend, DisplaySelect.KRPC, :RPCConnect, state.ip, state.port)
            state.loading = true
        end
        ig.SameLine()
        if ig.Button("Disconnect")
            tx(backend, DisplaySelect.KRPC, :RPCDisconnect)
            state.loading = true
        end
        if state.loading
            ig.BeginGroup()
            ig.SameLine()
            Spinner(ui)
            ig.EndGroup()
        end

        ig.Text(state.status)

        show_ui(state.inp, ui)
        ig.SameLine()
        show_ui(state.btn, ui)
    end
end

mutable struct SysCfg
    id::String
    loading::Bool

    SysCfg() = new("SysCfg", false)
end

function UI.handle_rx(
    res::Any, ::UiCfg, state::SysCfg, ::Displays, ::Backend, mission::Lockable{Mission}
)
    if res isa Exception
        rethrow(res) # TODO
    end

    (tag, res...) = res
    if tag == :Failure
        @error "Loading bodies failed: \n$(res[1])"
        state.loading = false
    elseif tag == :LoadedSystem
        @lock mission mission[].system = res[1]
        state.loading = false
    else
        error("unreachable")
    end

    return nothing
end

function UI.show_display(
    ui::UiCfg, state::SysCfg, dis::Displays, backend::Backend, mission::Lockable{Mission}
)
    @Display ui dis.syscfg "System Configuration" begin
        ig.SetNextWindowSize((ui.sf * 256.0, ui.sf * 512.0), ImGuiCond_FirstUseEver)
    end begin
        if ig.Button("Load bodies from kRPC")
            tx(backend, DisplaySelect.SYSCFG, :LoadSystem)
            state.loading = true
        end
        if state.loading
            ig.BeginGroup()
            ig.SameLine()
            Spinner(ui)
            ig.EndGroup()
        end

        n_bodies = @lock mission length(mission[].system.bodies)
        noun = n_bodies != 1 ? "bodies" : "body"
        ig.Text("$n_bodies $noun loaded")

        open = nothing
        if ig.Button("Expand All")
            open = :open
        end
        ig.SameLine()
        if ig.Button("Collapse All")
            open = :close
        end
        @lock mission begin
            for (star, body) in filter((x) -> x[2].is_star, mission[].system.bodies)
                _show_body(mission[].system.bodies, star, body, open)
            end
        end
    end
end

function _show_body(
    bodies::Dict{String,Body}, name::String, body::Body, open::Union{Symbol,Nothing}
)
    if isempty(body.satellites)
        ig.BulletText(name)
    else
        !isnothing(open) && ig.SetNextItemOpen(open == :open, ImGuiCond_Always)
        if ig.TreeNode(name)
            for (name, body) in map(x -> (x, bodies[x]), body.satellites)
                _show_body(bodies, name, body, open)
            end
            ig.TreePop()
        end
    end
end

end
