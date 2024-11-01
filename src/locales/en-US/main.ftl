title = KerbalToolkit

error-start-failed      = Application start failed
error-krpc-conn         = kRPC Connection failed:
    {$error}
error-krpc-noconn       = kRPC not connected.
error-krpc-svbody       = State vector body was not loaded.
erorr-krpc-noflight     = No flight in progress.
error-krpc-noeditor     = No ship in editor.
error-tli-nosoln        = No solution for TLI found with the given parameters.
error-tli-general       = Could not calculate TLI maneuver.
error-no-vessel         = No vessel selected.
error-no-sv-in-slot     = No state vector in slot.
error-body-no-load      = Body was not loaded.
error-mpt-no-init       = MPT was not initialized for the selected vessel.
error-mpt-noclass       = The selected vessel did not have an associated vessel class.
error-no-anchor-time    = No MPT anchor time was specified.
error-mpt-uniqueness    = Candidate maneuver `{$candidate}` conflicts with executed maneuver `{$executed}`.
error-tlmcc-no-targets  = TLMCC nodal targeting parameters were not initialized.
error-tlmcc-nosoln      = No solution for MCC found with the given parameters.
error-calc-general      = Calculation failed.
error-no-mnv-time       = Maneuver time was not specified.
error-mnv-negative-time = Specified maneuver time was before anchor time.

expand-all   = Expand All
collapse-all = Collapse All
connect      = Connect
disconnect   = Disconnect

time-ut       = UT (s)
time-ut-dhms  = UT (d:hh:mm:ss.ms)
time-get-dhms = GET (d:hh:mm:ss.ms)

syscfg-title         = System Configuration
syscfg-load-krpc     = Load Bodies from kRPC
syscfg-bodies-loaded = {$bodies ->
    [one] {$bodies} body loaded
    *[other] {$bodies} bodies loaded
}

krpc-status-noconn    = Status: Not Connected
krpc-title            = kRPC Configuration
krpc-host             = Host
krpc-rpc-port         = RPC Port
krpc-stream-port      = Stream Port
krpc-status-error     = Status: Connection failed:
    Error: {$error}
krpc-status-success   = Status: Connection successful
    kRPC version: v{$version}
krpc-log-success      = kRPC connection successful, kRPC version: v{$version}
krpc-log-disconnected = kRPC disconnected

classes-title          = Vessel Classes
classes-searchbox-hint = Search or create
classes-create         = Create "{$class}"
classes-subvessel      = Subvessel {$n}
classes-keep           = Keep
classes-discard        = Discard
classes-cancel         = Cancel
classes-finish         = Finish
classes-save           = Save
classes-rename         = Rename
classes-delete         = Delete
classes-load-editor    = Load from Editor
classes-load-flight    = Load from Flight
classes-description    = Description
classes-shortcode      = Shortcode
classes-decouplers     = Decouplers
classes-calcsep        = Calculate Separation
classes-no-class       = No Class Selected
classes-engines        = Engines

part-tag = {$part} (tag: "{$tag}")

classes-part                      = {$part}
classes-part-tag                  = {$part} (tag: "{$tag}")
classes-part-staged-fairing-tag   = {$part} (fairing staged, tag: "{$tag}")
classes-part-unstaged-fairing-tag = {$part} (fairing not staged, tag: "{$tag}")
classes-part-staged-fairing       = {$part} (fairing staged)
classes-part-unstaged-fairing     = {$part} (fairing not staged)

classes-decoupler-fairing-tag = {$part} (fairing, tag: "{$tag}")
classes-decoupler-fairing     = {$part} (fairing)
classes-decoupler-tag         = {$part} (tag: "{$tag}")
classes-decoupler-top         = Top
classes-decoupler-bottom      = Bottom - {$part}
classes-decoupler-bottom-tag  = Bottom - {$part} (tag: "{$tag}")

classes-calcsep-default-name = {$class} {$n}

classes-calcsep-explainer     = Select one or more decouplers and press "Calculate Separation" to create subvessels.

classes-shortcode-explainer-1 = A short code (ideally 1-2 characters, max 5 characters) used to identify this vessel in docking and separation configurations.
    For example, Command Module as C, Service Module as S, and Lunar Module as L would allow docked configurations such as "C+S+L".
classes-shortcode-explainer-2 = Must be unique.

classes-load-editor-explainer = Open the craft in the editor (VAB/SPH) then press this button to load.
classes-load-flight-explainer = Switch to the craft in the flight scene then press this button to load.

vessels-title          = Vessels
vessels-searchbox-hint = Search or create
vessels-create         = Create "{$vessel}"
vessels-save           = Save
vessels-rename         = Rename
vessels-delete         = Delete
vessels-description    = Description
vessels-class          = Class
vessels-no-class       = N/A
vessels-link           = Link to KSP Vessel
vessels-link-explainer = In order to operate with telemetry, this vessel must be linked to its KSP counterpart. Select one vessel from the following list to link to it.
vessels-refresh-list   = Refresh List
vessels-no-vessel      = No Vessel Selected
vessels-get-base       = GET Time Base
vessels-link-utilities = Link Utilities
vessels-link-getbase   = Load GET Time Base
vessels-link-resources = Load Resources
vessels-link-ids       = Load Part IDs
vessels-resources      = Resources
vessels-edit-get-base  = Change GET Time Base

vessels-error-no-link  = Vessel was not linked to a KSP vessel
vessels-error-no-class = Vessel did not have a vessel class

vc-title           = Vector Comparison
vc-no-vessel       = N/A
vc-comparison-time = Comparison Time
vc-vec             = V{$n}
vc-calculate       = Calculate
vc-tag             = Tag

vps-title     = Vector Panel Summary
vps-vessel    = Vessel
vps-no-vessel = N/A
vps-slot      = Slot
vps-load-ksp  = Load from KSP
vps-no-sv     = No state vector in slot.

vps-error-no-vessel   = No vessel selected.
vps-error-no-link     = Vessel was not linked to a KSP vessel
vps-error-empty-name  = State vector slot name cannot be empty

mpt-title       = Mission Plan Table
mpt-geti        = GETI
mpt-delta-t     = ΔT
mpt-delta-v     = ΔV
mpt-delta-v-rem = ΔVREM
mpt-ha          = Ha
mpt-hp          = Hp
mpt-code        = Code
mpt-vessel      = MPT Vessel
mpt-av-slot     = Anchor Vector Slot
mpt-status      = MPT Status:
mpt-init        = Initialize
mpt-reinit      = Reinitialize
mpt-delete      = Remove Maneuver

mpt-status-active        = Active
mpt-status-active-noinit = Partially Active (vessel definition not initialized)
mpt-status-partial-av    = Partially Active (anchor vector is based after one or more maneuvers)
mpt-status-missing-av    = Inactive (missing or invalid anchor vector)
mpt-status-no-vessel     = Inactive (no vessel)

logs-title = Logs

tliproc-title        = TLI Processor
tliproc-parking-sv   = Pre-TLI SV
tliproc-central-body = Central Body:
tliproc-moon-body    = Moon
tliproc-no-moon      = N/A
tliproc-calc         = Calculate
tliproc-to           = to
tliproc-flight-time  = Flight Time
tliproc-coast-time   = Coast Time
tliproc-periapse     = Periapsis Altitude
tliproc-opt-periapse = Optimize Periapsis?
tliproc-iter-fac     = Iteration Factor
tliproc-iter-max     = Iteration Maximum
tliproc-code         = Code
tliproc-geti         = GETI
tliproc-dv-prograde  = Prograde
tliproc-dv-normal    = Normal
tliproc-dv-radial    = Radial
tliproc-dv-total     = ΔV Total
tliproc-allow-retro  = Allow Retrograde?
tliproc-hohmann-tof  = Hohmann Flight Time:
tliproc-delete       = Delete Maneuver

tlmcc-title        = TLMCC Processor
tlmcc-sv           = MCC SV
tlmcc-central-body = Central Body:
tlmcc-moon-body    = Moon
tlmcc-calc         = Calculate
tlmcc-opt          = Targeting Option
tlmcc-opt-1        = 1: Nodal Targeting
tlmcc-opt-2        = 2: Fixed-Time LOI Ellipse
tlmcc-opt-3        = 3: Variable-Time LOI Ellipse
tlmcc-nt-refmnv    = Reference Maneuver
tlmcc-nt-nomnv     = N/A (Nodal targets not defined)


transfer-to-mpt = Transfer to MPT
transfer-to-tlmcc = Transfer targets to TLMCC Processor

vector-select-slot = Slot
vector-select-time = Anchor Time

mpt-trfr-title       = MPT Maneuver Transfer
mpt-trfr-no-vessel   = N/A
mpt-trfr-vessel      = Vessel
mpt-trfr-code        = Code
mpt-trfr-geti        = GETI
mpt-trfr-dv-prograde = ΔV Prograde
mpt-trfr-dv-normal   = ΔV Normal
mpt-trfr-dv-radial   = ΔV Radial
mpt-trfr-dv-total    = ΔV Total
mpt-trfr-mnvinfo     = Maneuver Info
mpt-trfr-vessel-cfg  = Vessel Configuration
mpt-trfr-calc-fuel   = Calculate Fuel Consumption
mpt-trfr-dvrem       = ΔV Remaining (m/s)
mpt-trfr-startmass   = Start Mass (tons)
mpt-trfr-endmass     = End Mass (tons)
mpt-trfr-burntime    = Burn Time (s)
mpt-trfr-trfr        = Transfer to MPT
mpt-trfr-maninp      = Manual Input
mpt-trfr-maninp-conf = Confirm Maneuver

mpt-sep-title       = Vessel Separation
mpt-sep-get         = GET
mpt-sep-vessel-name = New Vessel Name
mpt-sep-class       = New Class
mpt-sep-trfr        = Add to MPT

mpt-trfr-error-no-geti = Maneuver GETI was not specified.

time-display-toggle = Toggle Time Units

time-utils-title  = Time Utilities
time-utils-vessel = Reference Vessel
time-utils-ut     = UT
time-utils-get    = GET
time-utils-t1     = Time 1
time-utils-t2     = Time 2
time-utils-t3     = Result

gpm-title        = General-Purpose Maneuver
gpm-sv           = Maneuver SV
gpm-geti         = GETI
gpm-mode         = Mode
gpm-mode-circ    = Circularize
gpm-mode-apo     = Change apoapsis
gpm-mode-peri    = Change periapsis

altitude = Altitude

gpm-optimum      = At the optimum time
gpm-circ-when    = When
gpm-circ-deltat  = After a specific ΔT
gpm-circ-alt     = At a specific altitude
gpm-circ-pe      = At the next periapsis
gpm-circ-ap      = At the next apoapsis
gpm-circ-fixtime = At a specific time


menu-title             = Menu
menu-load-mission      = Load Mission
menu-save-mission      = Save Mission
menu-display-select    = Display Select
menu-organize-windows  = Organize Windows
menu-close-all-windows = Close All Windows
menu-open              = Open

menu-display-config = 00XX: Config and Utilities
menu-display-syscfg = 0000: System Configuration
menu-display-krpc   = 0001: kRPC Configuration
menu-display-logs   = 0002: Logs
menu-display-time   = 0003: Time Utilities

menu-display-mpt      = 01XX: Mission Plan Table
menu-display-open-mpt = 0100: Mission Plan Table
menu-display-mpt-trfr = 0101: Transfer Maneuver
menu-display-mpt-sep  = 0102: Vessel Separation

menu-display-sv      = 02XX: State Vectors
menu-display-sv-comp = 0200: Comparison
menu-display-sv-vps  = 0201: Vector Panel Summary

menu-display-vesselsclasses = 03XX: Vessels and Classes
menu-display-classes        = 0300: Vessel Classes
menu-display-vessels        = 0301: Vessels

menu-display-target     = 04XX: Maneuver Targeting
menu-display-tliproc    = 0400: TLI Processor
menu-display-tlmcc      = 0401: TLMCC Processor
menu-display-gpm        = 0402: General-Purpose Maneuver