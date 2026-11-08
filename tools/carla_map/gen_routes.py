#!/usr/bin/env python3

"""
gen_routes.py -- derive a deterministic routes file from the generated
OpenDRIVE model and the SUMO network converted from it.

Movements are taken from the *authoritative* source, the <junction>
connections of the .xodr (so exactly the legal manoeuvres -- no U-turns
unless the map defines them).  Each OpenDRIVE arm road is matched with its
SUMO edge by endpoint coordinates; this is exact because netconvert ran with
--offset.disable-normalization=true (both worlds share the same coordinates).

Standard library only; deterministic output (sorted ids, no RNG).
"""

import argparse
import math
import xml.etree.ElementTree as ET


def road_endpoints(road):
    """Start and end point of a road's plan view (single geometry segment)."""
    plan = road.find("planView")
    geoms = plan.findall("geometry")
    assert len(geoms) == 1, "gen_routes expects single-segment arm roads"
    g = geoms[0]
    x0, y0, hdg, length = (float(g.get(k)) for k in ("x", "y", "hdg", "length"))
    arc = g.find("arc")
    if arc is None:
        return (x0, y0), (x0 + length * math.cos(hdg), y0 + length * math.sin(hdg))
    k = float(arc.get("curvature"))
    r = 1.0 / k
    cx, cy = x0 - r * math.sin(hdg), y0 + r * math.cos(hdg)
    a = k * length
    dx, dy = x0 - cx, y0 - cy
    return (x0, y0), (cx + math.cos(a) * dx - math.sin(a) * dy,
                      cy + math.sin(a) * dx + math.cos(a) * dy)


def xodr_movements(xodr):
    """[(incoming_arm_road_id, outgoing_arm_road_id)] from the junction."""
    root = ET.parse(xodr).getroot()
    roads = {int(r.get("id")): r for r in root.findall("road")}
    moves = set()
    for junction in root.findall("junction"):
        for conn in junction.findall("connection"):
            incoming = int(conn.get("incomingRoad"))
            connector = roads[int(conn.get("connectingRoad"))]
            succ = connector.find("link/successor")
            if succ is None or succ.get("elementType") != "road":
                continue
            moves.add((incoming, int(succ.get("elementId"))))
    return sorted(moves)


def parse_shape(shape):
    first, last = shape.split()[0], shape.split()[-1]
    return tuple(float(v) for v in first.split(",")), \
        tuple(float(v) for v in last.split(","))


def sumo_edges(net):
    root = ET.parse(net).getroot()
    edges = []
    for e in root.findall("edge"):
        if e.get("function") == "internal" or e.get("id").startswith(":"):
            continue
        shape = e.get("shape")
        if not shape:
            continue
        start, end = parse_shape(shape)
        edges.append({"id": e.get("id"), "from": e.get("from"), "to": e.get("to"),
                      "start": start, "end": end})
    return edges


def junction_nodes(edges):
    """Nodes that have both incoming and outgoing edges."""
    to_nodes = {e["to"] for e in edges}
    from_nodes = {e["from"] for e in edges}
    return to_nodes & from_nodes


def match(edges, point, as_entry, nodes):
    """Edge starting at `point` at a junction node (netconvert rounds shapes
    to 2 decimals, hence the tolerance)."""
    found = [e["id"] for e in edges
             if e["start"] is not None and math.dist(e["start"], point) < 0.25
             and ((as_entry and e["to"] in nodes) or (not as_entry and e["from"] in nodes))]
    return sorted(set(found))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--xodr", required=True)
    ap.add_argument("--net", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--per-movement", type=int, default=2)
    ap.add_argument("--only", action="append", metavar="ENTRY:EXIT",
                    help="restrict to the given movement(s), repeatable; "
                         "default: all movements of the junction")
    ap.add_argument("--depart-step", type=float, default=0.4,
                    help="departure spacing [s] within a movement")
    # vehicles are present from t=0 (no warm-up gap before the scene exists)
    ap.add_argument("--depart-start", type=float, default=0.0)
    ap.add_argument("--max-speed", type=float, default=13.89)
    args = ap.parse_args()

    edges = sumo_edges(args.net)
    nodes = junction_nodes(edges)
    roads = {int(r.get("id")): r for r in ET.parse(args.xodr).getroot().findall("road")}
    endpoints = {rid: road_endpoints(road) for rid, road in roads.items()}
    only = set()
    for spec in args.only or []:
        entry, exit_ = spec.split(":")
        only.add((entry, exit_))
    routes = []
    for incoming, outgoing in xodr_movements(args.xodr):
        in_start = endpoints[incoming][0]
        out_start = endpoints[outgoing][0]
        entry = match(edges, in_start, True, nodes)
        exit_ = match(edges, out_start, False, nodes)
        if len(entry) != 1 or len(exit_) != 1:
            raise SystemExit(f"cannot map roads {incoming}->{outgoing} to unique "
                             f"SUMO edges ({entry} -> {exit_})")
        if only and (entry[0], exit_[0]) not in only:
            continue
        routes.append((entry[0], exit_[0]))
    if only and not routes:
        raise SystemExit(f"none of the --only movements {sorted(only)} exists "
                         f"at the junction")

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        "<!-- generated by tools/carla_map/gen_routes.py; do not edit -->",
        "<routes>",
        f'  <vType id="car" length="4.50" width="1.80" minGap="2.50" maxSpeed="'
        f'{args.max_speed:.2f}" accel="2.60" decel="4.50" sigma="0.00"/>',
    ]
    vehicles = []
    for entry, exit_ in routes:
        for i in range(args.per_movement):
            depart = args.depart_start + i * args.depart_step
            vid = f"v_{entry}_to_{exit_}_{i}".replace("-", "e")
            vehicles.append((depart, vid, entry, exit_))
    # SUMO requires routes sorted by departure time
    vehicles.sort()
    for depart, vid, entry, exit_ in vehicles:
        # explicit route edges; SUMO inserts the internal lanes of the
        # junction automatically
        # departSpeed="max": spawn rolling at the posted speed instead of
        # creeping up from 0 -- with the short arms this makes the vehicles
        # actually reach the junction within the (short) simulation
        lines.append(f'  <vehicle id="{vid}" type="car" depart="{depart:.2f}" '
                     f'departLane="best" departSpeed="max">')
        lines.append(f'    <route edges="{entry} {exit_}"/>')
        lines.append("  </vehicle>")
    count = len(vehicles)
    lines.append("</routes>")
    with open(args.out, "w") as out:
        out.write("\n".join(lines) + "\n")
    print(f"wrote {args.out} ({len(routes)} movements, {count} vehicles)")


if __name__ == "__main__":
    main()
