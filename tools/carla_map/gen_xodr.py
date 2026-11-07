#!/usr/bin/env python3

"""
gen_xodr.py -- configuration-as-code generator for the Artery CARLA/SUMO map.

The *same* generated OpenDRIVE file is

  * loaded into CARLA at runtime through ``World::GenerateOpenDriveWorld()``
    (``CarlaScenarioManager`` parameter ``opendriveFile``), and
  * converted to a SUMO network with
    ``netconvert --opendrive-file intersection.xodr --offset.disable-normalization=true``

so both simulators run the identical intersection.  Determinism: the output is
byte-identical across runs (fixed ordering, fixed date, no random numbers) so
the generated artifacts can be committed and diffed.

Constraints honoured:
  * Python standard library only (no pip in the container).
  * No YAML/JSON map definition -- all dimensions are plain Python constants /
    dataclasses below, reviewable and unit-testable.

One sidecar artifact is written next to the .xodr (plain text, trivially
parseable from C++ without a JSON library):

  <out>.lanepoints  "road lane s x y z hdg_deg"  (lane centrelines, 1 m steps;
                    input of the static CARLA-vs-generator geometry check)

Coordinate conventions
----------------------
OpenDRIVE / SUMO: right handed, x east, y north, heading in radians
counter-clockwise from +x.  SUMO keeps these coordinates exactly when
netconvert runs with --offset.disable-normalization=true.
CARLA: left handed;  x_carla = x, y_carla = -y, yaw_carla = -hdg (degrees).
SUMO/TraCI angles: degrees clockwise from north;  psi_math = 90 - a_sumo.
"""

import argparse
import math
import sys
from dataclasses import dataclass, field
from xml.etree import ElementTree as ET

# ---------------------------------------------------------------------------
# PLACEHOLDER dimensions -- replace with the user-supplied dimension table.
#
# Symmetric four-way intersection, right hand traffic:
#   * every arm: 1 inbound + 1 outbound driving lane of 3.5 m
#   * 0.5 m shoulder + 2.0 m sidewalk on both sides
#   * 18 m long arms (from the junction centre), junction inscribed radius 8 m
#     (short on purpose: vehicles at speed reach the junction within ~1 s of
#     simulated traffic time)
#   * through / left / right legal from every arm, no U-turns
#   * 50 km/h on the arms, 20 km/h on the turning connectors
# ---------------------------------------------------------------------------

# A local transverse mercator projection centred on the intersection keeps the
# coordinates small (CARLA/UE4 is single precision and degrades beyond a few
# 10 km from the origin).  Any projection works as long as the *same* string
# ends up in <geoReference> -- netconvert reads it from there, so GNSS, SUMO
# and CARLA agree on Earth coordinates.  A UTM projection
# ("+proj=utm +zone=32 +datum=WGS84 +units=m +no_defs") is fine too if the
# false easting/northing are (approximately) at the site.
PROJECTION = "+proj=tmerc +lat_0=55.6761 +lon_0=12.5683 +k=1 +x_0=0 +y_0=0 " \
             "+datum=WGS84 +units=m +no_defs"

ORIGIN = (0.0, 0.0)  # xodr (and, with normalization disabled, SUMO) coordinates
# of the junction centre, in projection metres.


@dataclass(frozen=True)
class TurnSet:
    through: bool = True
    left: bool = True
    right: bool = True
    uturn: bool = False


@dataclass
class Approach:
    name: str
    bearing_deg: float            # compass bearing of the arm, seen from the centre
    lanes_in: int = 1
    lanes_out: int = 1
    lane_width: float = 3.5
    approach_length: float = 18.0  # junction centre -> arm end
    speed_kmh: float = 50.0
    shoulder_width: float = 0.5
    sidewalk_width: float = 2.0
    kerb_height: float = 0.15
    turns: TurnSet = TurnSet()
    # stop line offset per manoeuvre [m] in front of the junction boundary
    stops: dict = field(default_factory=lambda:
                        {"through": 2.0, "left": 2.0, "right": 2.0})


@dataclass
class Intersection:
    name: str = "artery_intersection"
    projection: str = PROJECTION
    origin: tuple = ORIGIN
    junction_radius: float = 8.0          # inscribed radius of the junction
    connector_speed_kmh: float = 20.0
    country: str = "OpenDRIVE"            # signal country key used by netconvert
    approaches: list = field(default_factory=list)


def arm(name, bearing, turns=TurnSet(), **kw):
    return Approach(name=name, bearing_deg=bearing, turns=turns, **kw)


# PLACEHOLDER dimensions
INTERSECTION = Intersection(
    approaches=[
        arm("north", 0.0),
        arm("east", 90.0),
        arm("south", 180.0),
        arm("west", 270.0),
    ],
)

# ---------------------------------------------------------------------------
# geometry helpers
# ---------------------------------------------------------------------------

EPS = 1e-9


def wrap_pi(a):
    """Normalise an angle to (-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def unit(hdg):
    return (math.cos(hdg), math.sin(hdg))


def left_normal(hdg):
    return (-math.sin(hdg), math.cos(hdg))


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def add(a, b, k=1.0):
    return (a[0] + k * b[0], a[1] + k * b[1])


def f(v):
    """Deterministic float formatting."""
    return f"{v:.9g}"


class Line:
    def __init__(self, x0, y0, hdg, length):
        self.x0, self.y0, self.hdg, self.length = x0, y0, wrap_pi(hdg), length

    @property
    def curvature(self):
        return 0.0

    def position(self, s):
        d = unit(self.hdg)
        return (self.x0 + s * d[0], self.y0 + s * d[1])

    def heading(self, s):
        return self.hdg

    def to_xml(self, parent, s0):
        g = ET.SubElement(parent, "geometry", s=f(s0), x=f(self.x0), y=f(self.y0),
                          hdg=f(self.hdg), length=f(self.length))
        ET.SubElement(g, "line")


class Arc:
    def __init__(self, x0, y0, hdg, length, curvature):
        self.x0, self.y0, self.hdg = x0, y0, wrap_pi(hdg)
        self.length = length
        self.curvature = curvature

    @property
    def center(self):
        r = 1.0 / self.curvature
        n = left_normal(self.hdg)
        return (self.x0 + r * n[0], self.y0 + r * n[1])

    def position(self, s):
        c = self.center
        v = (self.x0 - c[0], self.y0 - c[1])
        a = self.curvature * s
        ca, sa = math.cos(a), math.sin(a)
        return (c[0] + ca * v[0] - sa * v[1], c[1] + sa * v[0] + ca * v[1])

    def heading(self, s):
        return wrap_pi(self.hdg + self.curvature * s)

    def to_xml(self, parent, s0):
        g = ET.SubElement(parent, "geometry", s=f(s0), x=f(self.x0), y=f(self.y0),
                          hdg=f(self.hdg), length=f(self.length))
        ET.SubElement(g, "arc", curvature=f(self.curvature))


def lane_center_t(lanes, lane_id):
    """Centre t offset of `lane_id`; `lanes` = [(id, width)] from centre outwards."""
    acc = 0.0
    for lid, w in lanes:
        if lid == lane_id:
            return acc + (-w / 2.0 if lid < 0 else w / 2.0)
        acc += -w if lid < 0 else w
    raise KeyError(lane_id)


def lane_point(geom, s, t):
    """Point at distance s / lateral offset t (exact for line and arc)."""
    if isinstance(geom, Arc):
        c = geom.center
        r = 1.0 / geom.curvature
        p = geom.position(s)
        v = (p[0] - c[0], p[1] - c[1])
        scale = (r + t) / r
        return (c[0] + v[0] * scale, c[1] + v[1] * scale)
    return add(geom.position(s), left_normal(geom.heading(s)), t)


def line_circle_intersection(p0, d, radius, origin=(0.0, 0.0)):
    """Roots r of |p0 + r*d - origin| = radius (d unit)."""
    q = add(p0, origin, -1.0)
    b = dot(q, d)
    c = dot(q, q) - radius * radius
    disc = b * b - c
    if disc < 0:
        return []
    sq = math.sqrt(disc)
    return [-b - sq, -b + sq]


def fit_connecting_arc(start_pt, start_hdg, end_pt, end_hdg):
    """Arc touching both points with the given tangents; returns (arc, error).

    error is the terminal position mismatch; 0 for an exactly tangent
    construction (the symmetric case).  If no exact arc exists, a fallback arc
    tangent at the start and through the end point is returned with a non-zero
    error, so --verify flags it.
    """
    delta = wrap_pi(end_hdg - start_hdg)
    if abs(delta) < 1e-6:
        return None, None
    side = 1.0 if delta > 0 else -1.0
    ns = left_normal(start_hdg) if side > 0 else tuple(-x for x in left_normal(start_hdg))
    ne = left_normal(end_hdg) if side > 0 else tuple(-x for x in left_normal(end_hdg))
    det = cross(ns, ne)
    if abs(det) > 1e-9:
        # start_pt + a*ns = end_pt + b*ne
        q = add(end_pt, start_pt, -1.0)
        a = cross(q, ne) / det
        b = cross(q, ns) / det
        if a > 1e-3 and b > 1e-3 and abs(a - b) < max(1e-3, 1e-3 * max(a, b)):
            r = 0.5 * (a + b)
            arc = Arc(start_pt[0], start_pt[1], start_hdg, abs(delta) * r, side / r)
            err = math.dist(arc.position(arc.length), end_pt)
            herr = abs(wrap_pi(arc.heading(arc.length) - end_hdg))
            return arc, max(err, herr)

    # fallback: centre on the perpendicular at start and on the perpendicular
    # bisector of start/end -> tangent at start, passes through end.
    mid = add(start_pt, end_pt, 0.5)
    chord = add(end_pt, start_pt, -1.0)
    pl = math.hypot(*chord)
    if pl < 1e-6:
        return None, None
    nrm = (-chord[1] / pl, chord[0] / pl)
    det = cross(ns, nrm)
    if abs(det) < 1e-9:
        return None, None
    a = cross(add(mid, start_pt, -1.0), nrm) / det
    if a <= 0:
        side = -side
        a = -a
    arc = Arc(start_pt[0], start_pt[1], start_hdg, abs(delta) * a, side / a)
    return arc, math.dist(arc.position(arc.length), end_pt)


# ---------------------------------------------------------------------------
# model
# ---------------------------------------------------------------------------

@dataclass
class LaneSpec:
    id: int
    lane_type: str
    width: float


@dataclass
class RoadModel:
    rid: int
    name: str
    geom: object
    lanes: list                     # LaneSpec, ids -1,-2,... from the centre line out
    lane_offset: float = 0.0        # shifts all lanes left (moves lane 0 off the axis)
    junction: int = -1
    predecessor: tuple = None       # (element_type, element_id, contact_point)
    successor: tuple = None
    speed_kmh: float = 0.0
    signals: list = field(default_factory=list)   # (s, t, zOffset, hOffset)
    stop_mark_s: float = None
    kind: str = "arm"               # arm | connector

    def length(self):
        return self.geom.length

    def driving_lanes(self):
        return [(l.id, l.width) for l in self.lanes if l.lane_type == "driving"]

    def side(self):
        return [(l.id, l.width) for l in self.lanes if l.id < 0]

    def lane_center(self, s, lane_id):
        """(x, y), heading of the lane centre at distance s."""
        t = lane_center_t(self.side(), lane_id) + self.lane_offset
        return lane_point(self.geom, s, t), self.geom.heading(s)


class OpenDriveBuilder:
    def __init__(self, data):
        self.data = data
        self.roads = []
        self.junction = {"id": 1, "name": "junction_1", "connections": [], "priority": []}
        self._next_road_id = 10
        self._problems = []

    def new_id(self):
        rid = self._next_road_id
        self._next_road_id += 1
        return rid

    def warn(self, msg):
        self._problems.append(msg)

    # -- assembly -----------------------------------------------------------

    def build(self):
        R = self.data.junction_radius
        arms = []
        for ap in self.data.approaches:
            bearing = math.radians(ap.bearing_deg)
            u = (math.sin(bearing), math.cos(bearing))   # outward along the arm
            hdg_out = math.atan2(u[1], u[0])
            arms.append((ap, u, hdg_out))

        for ap, u, hdg_out in arms:
            lanes_in = [LaneSpec(-1 - i, "driving", ap.lane_width)
                        for i in range(ap.lanes_in)]
            lanes_in += [LaneSpec(-ap.lanes_in - 1, "shoulder", ap.shoulder_width),
                         LaneSpec(-ap.lanes_in - 2, "sidewalk", ap.sidewalk_width)]
            lanes_out = [LaneSpec(-1 - i, "driving", ap.lane_width)
                         for i in range(ap.lanes_out)]
            lanes_out += [LaneSpec(-ap.lanes_out - 1, "shoulder", ap.shoulder_width),
                          LaneSpec(-ap.lanes_out - 2, "sidewalk", ap.sidewalk_width)]

            far = add(self.data.origin, u, ap.approach_length)
            near = add(self.data.origin, u, R)

            # inbound: reference line on the arm axis, from the far end towards
            # the junction; driving lanes occupy the right half (RHT)
            in_geom = Line(far[0], far[1], hdg_out + math.pi,
                           ap.approach_length - R)
            inbound = RoadModel(self.new_id(), f"in_{ap.name}", in_geom, lanes_in,
                                speed_kmh=ap.speed_kmh, kind="arm")
            inbound.successor = ("junction", str(self.junction["id"]), "end")
            stop = min(ap.stops.values()) if ap.stops else 2.0
            # No <signal> elements on purpose: CARLA would spawn traffic
            # light props for them (with no sensible way to place the poles
            # off the roadway), while SUMO does not need them at all -- it
            # infers its own traffic lights via netconvert --tls.guess-
            # signals.  The stop line below is a roadMark and stays.
            inbound.stop_mark_s = inbound.length() - stop
            self.roads.append(inbound)

            out_geom = Line(near[0], near[1], hdg_out, ap.approach_length - R)
            outbound = RoadModel(self.new_id(), f"out_{ap.name}", out_geom, lanes_out,
                                 speed_kmh=ap.speed_kmh, kind="arm")
            outbound.predecessor = ("junction", str(self.junction["id"]), "start")
            self.roads.append(outbound)

            ap._in, ap._out, ap._u, ap._hdg_out = inbound, outbound, u, hdg_out

        conn_id = 0
        for ap in self.data.approaches:
            for target in self.data.approaches:
                if target is ap:
                    continue
                delta = wrap_pi(target._hdg_out - ap._in.geom.hdg)
                if abs(delta) < 1e-6:                       # through
                    if ap.turns.through:
                        # netconvert supports exactly ONE connecting road per
                        # (incoming, outgoing) pair -- all laneLinks go into it
                        n_conn = min(len(ap._in.driving_lanes()),
                                     len(target._out.driving_lanes()))
                        lane_links = [(-(i + 1), -(i + 1)) for i in range(n_conn)]
                        self._connector(ap, target, "through", lane_links, -1,
                                        conn_id)
                        conn_id += 1
                elif abs(abs(delta) - math.pi) < 1e-6:      # U-turn
                    if ap.turns.uturn:
                        self._connector(ap, target, "uturn", [(-1, -1)], -1, conn_id)
                        conn_id += 1
                elif delta < 0:                             # right turn
                    if ap.turns.right:
                        # right turns hug the rightmost (outer) lane
                        self._connector(ap, target, "right", [(-1, -1)],
                                        -len(ap._in.driving_lanes()), conn_id)
                        conn_id += 1
                else:                                       # left turn
                    if ap.turns.left:
                        # left turns start from the leftmost (inner) lane
                        self._connector(ap, target, "left", [(-1, -1)], -1, conn_id)
                        conn_id += 1

        for ap in self.data.approaches:
            self.junction["priority"].append(ap._in.rid)

    def _connector(self, ap, target, maneuver, lane_links, geom_in_lane, conn_id):
        """One connecting road per (incoming, outgoing) pair.

        `lane_links` is the full [(from, to)] list of the junction connection;
        `geom_in_lane` is the driving lane whose centre line the reference
        geometry follows (right turns: rightmost lane, left turns: leftmost).
        """
        inc, outc = ap._in, target._out
        R = self.data.junction_radius
        in_lane, out_lane = lane_links[0]

        p_in0 = lane_point(inc.geom, 0.0, lane_center_t(inc.side(), geom_in_lane))
        p_out0 = lane_point(outc.geom, 0.0, lane_center_t(outc.side(), out_lane))
        d_in, d_out = unit(inc.geom.hdg), unit(outc.geom.hdg)

        # the lane centrelines cross the junction circle at S and E
        rs = line_circle_intersection(p_in0, d_in, R, self.data.origin)
        re = line_circle_intersection(p_out0, d_out, R, self.data.origin)
        if not rs or not re:
            self.warn(f"{maneuver} {inc.name}->{outc.name}: lane centre misses "
                      f"the junction circle")
            return
        S = add(p_in0, d_in, max(rs, key=lambda r: dot(add(p_in0, d_in, r), ap._u)))
        E = add(p_out0, d_out, max(re, key=lambda r: dot(add(p_out0, d_out, r),
                                                          target._u)))
        alpha, omega = inc.geom.hdg, outc.geom.hdg
        delta = wrap_pi(omega - alpha)

        if abs(delta) < 1e-6:
            lateral = abs(cross(add(E, S, -1.0), d_in))
            if lateral > 0.05:
                self.warn(f"{maneuver} {inc.name}->{outc.name}: not straight "
                          f"({lateral:.3f} m lateral offset)")
            geom = Line(S[0], S[1], alpha, math.dist(S, E))
        else:
            geom, err = fit_connecting_arc(S, alpha, E, omega)
            if geom is None:
                self.warn(f"{maneuver} {inc.name}->{outc.name}: no arc found")
                return
            if err > 0.01:
                self.warn(f"connecting road {inc.name}->{outc.name} ({maneuver}) "
                          f"tangent error {err:.3f} m")

        n_lanes = len(lane_links)
        lanes = [LaneSpec(-(i + 1), "driving", ap.lane_width) for i in range(n_lanes)]
        road = RoadModel(self.new_id(),
                         f"conn_{maneuver}_{ap.name}_to_{target.name}",
                         geom, lanes,
                         # lane 1 centre on the reference line
                         lane_offset=ap.lane_width / 2.0,
                         junction=self.junction["id"],
                         predecessor=("road", str(inc.rid), "end"),
                         successor=("road", str(outc.rid), "start"),
                         speed_kmh=self.data.connector_speed_kmh,
                         kind="connector")
        self.roads.append(road)
        self.junction["connections"].append({
            "id": conn_id,
            "incoming": inc.rid,
            "connecting": road.rid,
            "contact": "start",
            "lane_links": lane_links,
            "geom_from_lane": geom_in_lane,
            "maneuver": maneuver,
        })

    # -- verification -------------------------------------------------------

    def verify(self):
        problems = list(self._problems)
        ids = [r.rid for r in self.roads]
        if len(ids) != len(set(ids)):
            problems.append("duplicate road ids")
        by_id = {r.rid: r for r in self.roads}
        jid = str(self.junction["id"])

        for r in self.roads:
            if any(l.width <= 0 for l in r.lanes):
                problems.append(f"{r.name}: non-positive lane width")
            if not r.driving_lanes():
                problems.append(f"{r.name}: no driving lane")
            for role in ("predecessor", "successor"):
                link = getattr(r, role)
                if link is None:
                    if r.kind == "connector":
                        problems.append(f"connector {r.name} has no {role}")
                    continue
                et, eid, _cp = link
                if et == "junction":
                    if eid != jid:
                        problems.append(f"{r.name}: unknown junction {eid}")
                    if r.kind == "connector":
                        problems.append(f"connector {r.name} links to a junction")
                else:
                    other = by_id.get(int(eid))
                    if other is None:
                        problems.append(f"{r.name}: {role} -> unknown road {eid}")
                    elif r.kind == "arm":
                        problems.append(f"arm road {r.name} links to road {eid}")

        conn_ids = [c["id"] for c in self.junction["connections"]]
        if len(conn_ids) != len(set(conn_ids)):
            problems.append("duplicate junction connection ids")
        for c in self.junction["connections"]:
            cr = by_id.get(c["connecting"])
            ir = by_id.get(c["incoming"])
            if cr is None or ir is None or cr.kind != "connector":
                problems.append(f"junction connection {c['id']}: bad road reference")
                continue
            if cr.predecessor != ("road", str(ir.rid), "end"):
                problems.append(f"connecting road {cr.name} does not link back to "
                                f"incoming road {ir.rid}")
            outc = by_id.get(int(cr.successor[1])) if cr.successor else None
            if outc is None:
                problems.append(f"connecting road {cr.name}: no valid successor")
                continue
            inc_ids = {l.id for l in ir.lanes}
            conn_ids_ = {l.id for l in cr.lanes}
            out_ids = {l.id for l in outc.lanes}
            for frm, to in c["lane_links"]:
                if frm not in inc_ids or to not in conn_ids_:
                    problems.append(f"laneLink {frm}->{to} invalid on connection "
                                    f"{c['id']}")
            # every legal manoeuvre links all incoming driving lanes?  At least
            # require that each connection links >=1 lane and all to lane -1.
            for frm, to in c["lane_links"]:
                if to not in out_ids:
                    problems.append(f"laneLink to {to} not on {outc.name}")
            # geometric continuity: the connector reference line follows the
            # centre line of geom_from_lane; connector lane -1's centre sits on
            # it (lane_offset = w/2)
            frm = c.get("geom_from_lane", c["lane_links"][0][0])
            sp, _ = ir.lane_center(ir.length(), frm)
            start, _ = cr.lane_center(0.0, -1)
            if math.dist(start, sp) > 0.6:
                problems.append(f"{cr.name}: {math.dist(start, sp):.2f} m gap to "
                                f"incoming lane centre")
            end, _ = cr.lane_center(cr.length(), -1)
            estart, _ = outc.lane_center(0.0, c["lane_links"][0][1])
            if math.dist(end, estart) > 0.6:
                problems.append(f"{cr.name}: {math.dist(end, estart):.2f} m gap to "
                                f"outgoing lane centre")
        return problems

    # -- serialisation ------------------------------------------------------

    def to_xml(self):
        root = ET.Element("OpenDRIVE")
        roads = sorted(self.roads, key=lambda r: r.rid)
        xs, ys = [], []
        for r in roads:
            for s in (0.0, r.length()):
                p = r.geom.position(s)
                xs.append(p[0])
                ys.append(p[1])
        hdr = ET.SubElement(root, "header", revMajor="1", revMinor="4",
                            name=self.data.name, version="1",
                            date="2024-01-01T00:00:00",
                            north=f(max(ys)), south=f(min(ys)),
                            east=f(max(xs)), west=f(min(xs)),
                            vendor="artery")
        geo = ET.SubElement(hdr, "geoReference")
        geo.text = self.data.projection

        for r in roads:
            e = ET.SubElement(root, "road", name=r.name, length=f(r.length()),
                              id=str(r.rid), junction=str(r.junction))
            link = ET.SubElement(e, "link")
            for role, target in (("predecessor", r.predecessor),
                                 ("successor", r.successor)):
                if target is not None:
                    ET.SubElement(link, role, elementType=target[0],
                                  elementId=target[1], contactPoint=target[2])
            typ = ET.SubElement(e, "type", s="0", type="town")
            if r.speed_kmh > 0:
                ET.SubElement(typ, "speed", sOffset="0", max=str(int(r.speed_kmh)),
                              unit="km/h")
            plan = ET.SubElement(e, "planView")
            r.geom.to_xml(plan, 0.0)
            elev = ET.SubElement(e, "elevationProfile")
            ET.SubElement(elev, "elevation", s="0", a="0", b="0", c="0", d="0")
            lanes = ET.SubElement(e, "lanes")
            if abs(r.lane_offset) > EPS:
                ET.SubElement(lanes, "laneOffset", s="0", a=f(r.lane_offset),
                              b="0", c="0", d="0")
            section = ET.SubElement(lanes, "laneSection", s="0")
            center = ET.SubElement(section, "center")
            cl = ET.SubElement(center, "lane", id="0", type="none", level="true")
            ET.SubElement(cl, "roadMark", sOffset="0", type="solid", color="white",
                          width="0.12", laneChange="none")
            right = ET.SubElement(section, "right")
            for l in r.lanes:
                le = ET.SubElement(right, "lane", id=str(l.id), type=l.lane_type,
                                   level="false")
                ET.SubElement(le, "width", sOffset="0", a=f(l.width), b="0", c="0",
                              d="0")
                marks = {"driving": ("solid", "white"),
                         "shoulder": ("curb", "gray"),
                         "sidewalk": ("curb", "gray")}
                mk = ET.SubElement(le, "roadMark", sOffset="0", type=marks[l.lane_type][0],
                                   color=marks[l.lane_type][1], laneChange="none")
                if l.lane_type == "shoulder":
                    mk.set("width", "0.15")
                if l.id == -1:
                    # keep the incoming road clear up to the stop line
                    mk.set("laneChange", "none")
                    if r.stop_mark_s is not None and r.stop_mark_s < r.length():
                        ET.SubElement(le, "roadMark", sOffset=f(r.stop_mark_s),
                                      type="strikethrough", color="white",
                                      width="0.4", laneChange="none")
                if r.speed_kmh > 0 and l.lane_type == "driving":
                    ET.SubElement(le, "speed", sOffset="0",
                                  max=str(int(r.speed_kmh)), unit="km/h")
            sigs = ET.SubElement(e, "signals")
            for i, (s, t, z, h) in enumerate(r.signals):
                ET.SubElement(sigs, "signal", name=f"{r.name}_tl",
                              id=str(r.rid * 100 + i), s=f(s), t=f(t),
                              zOffset=f(z), hOffset=f(h), orientation="+",
                              dynamic="yes", country=self.data.country,
                              type="1000001", subtype="-1", value="-1",
                              text="", height="0.6", width="0.6")
            ET.SubElement(e, "objects")

        j = ET.SubElement(root, "junction", id=str(self.junction["id"]),
                          name=self.junction["name"])
        for c in sorted(self.junction["connections"], key=lambda c: c["id"]):
            ce = ET.SubElement(j, "connection", id=str(c["id"]),
                               incomingRoad=str(c["incoming"]),
                               connectingRoad=str(c["connecting"]),
                               contactPoint=c["contact"])
            for frm, to in c["lane_links"]:
                lle = ET.SubElement(ce, "laneLink", to=str(to))
                lle.set("from", str(frm))  # 'from' is a Python keyword
        for rid in self.junction["priority"]:
            ET.SubElement(j, "priority", road=str(rid), access="yes")
        return root

    # -- sidecars -----------------------------------------------------------

    def lane_points(self, step=1.0):
        pts = []
        for r in self.roads:
            for lid, _w in r.driving_lanes():
                s = 0.0
                while s <= r.length() + 1e-6:
                    (x, y), hdg = r.lane_center(min(s, r.length()), lid)
                    pts.append((r.rid, lid, min(s, r.length()), x, y, 0.0,
                                math.degrees(hdg)))
                    s += step
        return pts


def indent(elem, level=0, spaces=2):
    pad = "\n" + level * spaces * " "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = pad + spaces * " "
        for child in elem:
            indent(child, level + 1, spaces)
        if not child.tail or not child.tail.strip():
            child.tail = pad
    elif level and (not elem.tail or not elem.tail.strip()):
        elem.tail = pad
    if level == 0:
        elem.tail = "\n"


def build_intersection(data=INTERSECTION):
    builder = OpenDriveBuilder(data)
    builder.build()
    return builder


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Generate the Artery CARLA/SUMO intersection OpenDRIVE")
    ap.add_argument("--out", default="intersection.xodr", help="output .xodr path")
    ap.add_argument("--verify", action="store_true", help="run the self checks")
    ap.add_argument("--no-sidecars", action="store_true")
    args = ap.parse_args(argv)

    builder = build_intersection()
    problems = builder.verify()

    if args.verify:
        print(f"roads:              {len(builder.roads)}")
        print(f"  arm roads:        {sum(1 for r in builder.roads if r.kind == 'arm')}")
        print(f"  connecting roads: {sum(1 for r in builder.roads if r.kind == 'connector')}")
        print(f"  driving lanes:    {sum(len(r.driving_lanes()) for r in builder.roads)}")
        print(f"  junction links:   {len(builder.junction['connections'])}")
        print(f"  signals:          {sum(len(r.signals) for r in builder.roads)}")
        if problems:
            print("PROBLEMS:")
            for p in problems:
                print("  -", p)
            return 1
        print("self check: OK")

    tree = builder.to_xml()
    indent(tree)
    xml = '<?xml version="1.0" encoding="UTF-8"?>\n' + ET.tostring(
        tree, encoding="unicode")
    with open(args.out, "w") as out:
        out.write(xml)
    print(f"wrote {args.out} ({len(xml)} bytes)")

    if not args.no_sidecars:
        lp = args.out + ".lanepoints"
        with open(lp, "w") as out:
            out.write("# road lane s x y z hdg_deg\n")
            for rid, lid, s, x, y, z, hdg in builder.lane_points():
                out.write(f"{rid} {lid} {s:.2f} {x:.4f} {y:.4f} {z:.4f} {hdg:.4f}\n")
        print(f"wrote {lp}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
