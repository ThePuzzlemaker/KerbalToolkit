"""
Several useful enumerations are consolidated here.
"""
module Enums

using Match

export Apsis, Apoapsis, Periapsis, OrbitalNode, Ascending, Descending

"""
An apsis of an orbit, one of the extremal points of an orbit

- `Apoapsis`: the highest point of an orbit
- `Periapsis`: the lowest point of an orbit
"""
@enum Apsis begin
    Apoapsis
    Periapsis
end

"""
An orbital node, the intersections between an orbit and its
orbited body's equatorial plane

- `Ascending` node: the node which moves north through the plane at
  intersection
- `Descending` node: the node which moves south through the plane at
  intersection
"""
@enum OrbitalNode begin
    Ascending
    Descending
end

"""
  ~(apsis::Apsis)

Return the opposite apsis (i.e., apoapsis => periapsis; periapsis =>
apoapsis)
"""
function Base.:~(apsis::Apsis)
    @match apsis begin
        $Apoapsis => Periapsis
        $Periapsis => Apoapsis
    end
end

"""
  ~(node::OrbitalNode)

Return the opposite orbital node (i.e., ascending => descending; descending =>
ascending)
"""
function Base.:~(node::OrbitalNode)
    @match node begin
        $Ascending => Descending
        $Descending => Ascending
    end
end

end
