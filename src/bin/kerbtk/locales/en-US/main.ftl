title = KerbalToolkit

error-start-failed = Application start failed
error-krpc-conn = kRPC Connection failed:
    {$error}

expand-all = Expand All
collapse-all = Collapse All
connect = Connect
disconnect = Disconnect

time-ut = UT (s)
time-ut-dhms = UT (d:hh:mm:ss.ms)
time-get-dhms = GET (d:hh:mm:ss.ms)

syscfg-title = System Configuration
syscfg-load-krpc = Load Bodies from kRPC
syscfg-bodies-loaded = {$bodies ->
    [one] {$bodies} body loaded
    *[other] {$bodies} bodies loaded
}

krpc-status-noconn = Status: Not Connected
krpc-title = kRPC Configuration
krpc-host = Host
krpc-rpc-port = RPC Port
krpc-stream-port = Stream Port
krpc-status-error = Status: Connection failed:
    Error: {$error}
krpc-status-success = Status: Connection successful
    kRPC version: v{$version}
krpc-log-success = kRPC connection successful, kRPC version: v{$version}
krpc-log-disconnected = kRPC disconnected

classes-calcsep-explainer = Select one or more decouplers and press "Calculate Separation" to create subvessels.
classes-shortcode-explainer = A short code (ideally 1-2 characters, max 5 characters) used to identify this vessel in docking and separation configurations.
    For example, Command Module as C, Service Module as S, and Lunar Module as L would allow docked configurations such as "C+S+L".
classes-load-editor-explainer = Open the craft in the editor (VAB/SPH) then press this button to load.
classes-load-flight-explainer = Switch to the craft in the flight scene then press this button to load.

menu-title = Menu
menu-load-mission = Load Mission
menu-save-mission = Save Mission
menu-display-select = Display Select
menu-organize-windows = Organize Windows
menu-close-all-windows = Close All Windows
menu-open = Open

menu-display-config = 00XX: Configuration
menu-display-syscfg = 0000: System Configuration
menu-display-krpc   = 0001: kRPC Configuration
menu-display-logs   = 0002: Logs

menu-display-mpt      = 01XX: Mission Plan Table
menu-display-open-mpt = 0100: Open MPT

menu-display-sv      = 02XX: State Vectors
menu-display-sv-comp = 0200: Comparison
menu-display-sv-vps  = 0201: Vector Panel Summary

menu-display-vesselsclasses = 03XX: Vessels and Classes
menu-display-classes        = 0300: Vessel Classes

