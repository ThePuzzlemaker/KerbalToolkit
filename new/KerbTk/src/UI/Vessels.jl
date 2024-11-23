module Vessels

import CImGui as ig
using CImGui.lib
using CImGui.CSyntax: @c
using Base: Lockable

using ..UI: UI, UiCfg, Displays, @Display, DisplaySelect
using ..BackendHandler: Backend, tx
using ...Vessels: VesselClassId, PartId, VesselClass, Part, Decoupler
using ...Missions: Mission
using ...Arenas: Arena, retain!
using ..Widgets: Spinner

export Classes

@enum SubvesselOption begin
    KeepSubvessel
    DiscardSubvessel
end

mutable struct Classes
    id::String
    search::String
    current_class::Union{VesselClassId,Nothing}
    renaming::Bool
    just_clicked_rename::Bool
    classes_filtered::Vector{VesselClassId}
    force_refilter::Bool
    loading::Bool
    checkboxes::Dict{PartId,Bool}
    fairings::Dict{PartId,Bool}
    rocheckboxes::Dict{PartId,Tuple{Bool,Bool}}
    subvessels::Vector{Set{PartId}}
    subvessel_options::Vector{SubvesselOption}
    subvessel_names::Vector{String}

    function Classes()
        return new(
            "VesselClasses",
            "",
            nothing,
            false,
            false,
            VesselClassId[],
            false,
            false,
            Dict(),
            Dict(),
            Dict(),
            Vector{Set{PartId}}(),
            SubvesselOption[],
            String[],
        )
    end
end

function UI.handle_rx(
    res::Any, ::UiCfg, state::Classes, ::Displays, ::Backend, mission::Lockable{Mission}
)
    if res isa Exception
        rethrow(res)
    end

    state.loading = false
    tag = res[1]
    if tag == :LoadedVesselClass
        @lock mission begin
            (_, parts, root, tracked_id_map) = res
            if !isnothing(state.current_class)
                class = mission[].classes[state.current_class]
                class.parts = parts
                class.root = root
                class.tracked_id_map = tracked_id_map
            end
        end
    elseif tag == :Failure
        @error res[2]
    else
        error("Unreachable")
    end

    return nothing
end

function UI.show_display(
    ui::UiCfg, state::Classes, dis::Displays, backend::Backend, mission::Lockable{Mission}
)
    @Display ui dis.classes "Vessel Classes" begin
        ig.SetNextWindowSize((ui.sf * 512.0, ui.sf * 512.0), ImGuiCond_Always)
    end @lock mission begin
        if (ig.BeginChild(
            "##SearchBox",
            (ig.GetContentRegionAvail().x / 3.0, 0.0),
            ImGuiChildFlags_Borders | ImGuiChildFlags_NavFlattened,
        ))
            search_box(state, ui, mission)
        end
        ig.EndChild()
        ig.SameLine()

        if ig.BeginChild(
            "##Parts", (ig.GetContentRegionAvail().x, 0.0f0), ImGuiChildFlags_NavFlattened
        )
            ig.Spacing()
            if !state.renaming
                ig.Spacing()
            end

            if !isnothing(state.current_class)
                if state.renaming
                    ig.SetNextItemWidth(ig.GetContentRegionAvail().x)
                    name = deepcopy(mission[].classes[state.current_class].name)

                    ig.PushFont(ui.fonts.heading)
                    changed = @c ig.InputText("##Rename", &name, 0, C_NULL, C_NULL)
                    ig.PopFont()
                    lost_focus = ig.IsItemDeactivatedAfterEdit()

                    if state.just_clicked_rename
                        state.just_clicked_rename = false
                        ig.ActivateItemByID(ig.GetItemID())
                    end

                    if changed
                        mission[].classes[state.current_class].name = name
                        state.force_refilter = true
                    end

                    if lost_focus && ig.IsKeyDown(ImGuiKey_Enter)
                        state.renaming = false
                    end
                else
                    ig.PushFont(ui.fonts.heading)
                    ig.Text(mission[].classes[state.current_class].name)
                    ig.PopFont()
                end

                if state.renaming &&
                    (res = ig.Button("\UEAF4"); ig.SetItemTooltip("Save"); res)
                    state.renaming = false
                elseif !state.renaming &&
                    (res = ig.Button("\UE34C"); ig.SetItemTooltip("Rename"); res)
                    state.renaming = true
                    state.just_clicked_rename = true
                    state.force_refilter = true
                end

                ig.SameLine()
                if (res = ig.Button("\UE4A6"); ig.SetItemTooltip("Delete"); res)
                    state.renaming = false
                    pos = nothing
                    for (i, x) in enumerate(state.classes_filtered)
                        if x == state.current_class
                            pos = i
                        end
                    end
                    retain!(x -> x[1] != state.current_class, mission[].classes)
                    deleteat!(state.classes_filtered, pos)
                    state.current_class = get(state.classes_filtered, pos, nothing)
                    if isnothing(state.current_class)
                        state.current_class = get(state.classes_filtered, pos - 1, nothing)
                    end
                    state.force_refilter = true
                end

                if (
                    res = ig.Button("Load from Editor");
                    ig.SetItemTooltip(
                        "Open the craft in the editor (VAB/SPH) then press this button to load.",
                    );
                    res
                )
                    tx(backend, DisplaySelect.CLASSES, :LoadVesselPartsFromEditor)
                    state.loading = true
                end
                ig.SameLine()
                if (
                    res = ig.Button("Load from Flight");
                    ig.SetItemTooltip(
                        "Switch to the craft in the flight scene then press this button to load.",
                    );
                    res
                )
                    tx(backend, DisplaySelect.CLASSES, :LoadVesselClassFromFlight)
                    state.loading = true
                end

                if state.loading
                    ig.BeginGroup()
                    ig.SameLine()
                    Spinner(ui)
                    ig.EndGroup()
                end

                ig.AlignTextToFramePadding()
                ig.Text("Description")
                description = mission[].classes[state.current_class].description
                changed = @c ig.InputTextMultiline(
                    "##Description",
                    &description,
                    (ig.GetContentRegionAvail().x, 0.0f0),
                    0,
                    C_NULL,
                    C_NULL,
                )
                if changed
                    mission[].classes[state.current_class].description = description
                end

                ig.PushFont(ui.fonts.heading)
                ig.Text("Decouplers")
                ig.PopFont()
                ig.TextWrapped(
                    "Select one or more decouplers and press \"Calculate Separation\" to create subvessels.",
                )

                v = filter(
                    x -> (!isnothing(x[2].decouplers.top) || !isnothing(x[2].decouplers.pf) || !isempty(x[2].mass_modifiers)),
                    collect(mission[].classes[state.current_class].parts),
                )
                sort!(v, by=x->(x[2].title, x[2].tag))
                for (partid, part) in v
                    ig.Spacing()
                    if isempty(strip(part.tag))
                        ig.TextWrapped("$(part.title)")
                    else
                        ig.TextWrapped("$(part.title) (tag: \"$(part.tag)\"")
                    end
                    ig.Indent()

                    if (
                        any(part.mass_modifiers) do x
                            x.module_name == "ModuleProceduralFairing" &&
                                isapprox(x.current_mass, x.unstaged_mass)
                        end
                    )
                        fairing = Ref(get(state.fairings, partid, false))
                        ig.Checkbox("Fairing##$partid-fairing", fairing)
                        state.fairings[partid] = fairing[]
                    end

                    if !isnothing(part.decouplers.top) && isnothing(part.decouplers.bot)
                        checkbox = Ref(get(state.checkboxes, partid, false))
                        ig.Checkbox("Decoupler##$partid-decoupler", checkbox)
                        state.checkboxes[partid] = checkbox[]
                    elseif !isnothing(part.decouplers.bot)
                        (top, bot) = get(state.rocheckboxes, partid, (false, false))
                        top = Ref(top)
                        bot = Ref(bot)
                        ig.Checkbox("Top Decoupler##$partid-decouplertop", top)
                        ig.Checkbox("Bottom Decoupler##$partid-decoupelrbot", bot)
                        state.rocheckboxes[partid] = (top[], bot[])
                    end
                    ig.Unindent()
                end
            else
                ig.PushFont(ui.fonts.heading)
                ig.Text("No class selected")
                ig.PopFont()
            end
        end

        ig.EndChild()
    end
end

# N.B. mission must be locked
function search_box(state::Classes, ::UiCfg, mission::Lockable{Mission})
    ig.SetNextItemWidth(ig.GetContentRegionAvail().x)
    changed = @c ig.InputTextWithHint(
        "##ClassesSearchbox", "Search or create", &state.search, 0, C_NULL, C_NULL
    )
    lost_focus = ig.IsItemDeactivatedAfterEdit()
    if changed || state.force_refilter
        state.force_refilter = false
        refilter(state, mission)
    end

    already_exists = nothing
    for (id, class) in mission[].classes
        if strip(class.name) == strip(state.search)
            already_exists = id
        end
    end

    if isnothing(already_exists) &&
       (lost_focus && ig.IsKeyDown(ImGuiKey_Enter) && !isempty(strip(state.search))) || (
        isnothing(already_exists) &&
        !isempty(strip(state.search)) &&
        ig.Button("Create \"$(strip(state.search))\"")
    )
        class_id = push!(
            mission[].classes,
            VesselClass(strip(state.search), "", "", Arena{PartId,Part}(), Dict(), nothing),
        )
        state.current_class = class_id
        state.search = ""
        refilter(state, mission)
    end

    ig.Spacing()

    for class in state.classes_filtered
        checked = state.current_class == class
        if ig.Selectable("$(strip(mission[].classes[class].name))##$(class)", Ref(checked))
            state.checkboxes = Dict()
            state.rocheckboxes = Dict()
            state.current_class = class
        end
    end
end

# N.B. mission must be locked
function refilter(state::Classes, mission::Lockable{Mission})
    v = collect(mission[].classes)
    sort!(v; by=x -> x[2].name)
    filter!(v) do x
        isempty(state.search) || startswith(strip(x[2].name), strip(state.search))
    end
    return state.classes_filtered = map(x -> x[1], v)
end

end
