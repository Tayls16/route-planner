import time
import requests
from typing import List, Tuple, Optional
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

# ==============================
# CONFIG
# ==============================
OSRM_URL = "http://localhost:5000"           # your local OSRM server
USER_AGENT = "RoutePlannerTaylah/1.0"        # required by Nominatim
GEOCODE_SLEEP = 1.0                           # be polite to Nominatim (1 req/sec)

# Route mode: "roundtrip" | "open" | "fixed_end"
ROUTE_MODE = "open"                           # <- set this
FIXED_END_NAME: Optional[str] = None          # e.g., "Victoria University, Footscray Park" if ROUTE_MODE="fixed_end"

# Your stops (addresses OR "lat,lng")
STOPS = [
    "70 Ricky Way, Epping VIC",               # index 0 = start (depot)
    "Melbourne Museum",
    "RMIT University, Melbourne",
    "Victoria University, Footscray Park",
]

START_IDX = 0                                 # which stop to start from (usually 0)

# ==============================
# Geocoding with Nominatim
# ==============================
def parse_coord(s: str):
    try:
        if "," in s:
            lat, lng = s.split(",", 1)
            return float(lat.strip()), float(lng.strip())
    except Exception:
        pass
    return None

def geocode_one(q: str) -> Tuple[float, float]:
    c = parse_coord(q)
    if c:
        return c
    url = "https://nominatim.openstreetmap.org/search"
    params = {"q": q, "format": "json", "limit": 1, "countrycodes": "au"}
    headers = {"User-Agent": USER_AGENT}
    r = requests.get(url, params=params, headers=headers, timeout=20)
    r.raise_for_status()
    data = r.json()
    if not data:
        raise ValueError(f"Geocoding failed: {q}")
    lat = float(data[0]["lat"])
    lon = float(data[0]["lon"])
    time.sleep(GEOCODE_SLEEP)
    return (lat, lon)

def geocode_all(stops: List[str]) -> List[Tuple[float, float]]:
    return [geocode_one(s) for s in stops]

# ==============================
# OSRM helpers
# ==============================
def to_lnglat(coords: List[Tuple[float, float]]) -> str:
    return ";".join(f"{lng:.6f},{lat:.6f}" for (lat, lng) in coords)

def osrm_table_durations(coords: List[Tuple[float, float]]) -> List[List[int]]:
    url = f"{OSRM_URL}/table/v1/driving/{to_lnglat(coords)}"
    params = {"annotations": "duration"}
    r = requests.get(url, params=params, timeout=60)
    r.raise_for_status()
    js = r.json()
    if not js.get("durations"):
        raise RuntimeError("OSRM /table returned no durations")
    return [[int(round(x if x is not None else 10**9)) for x in row] for row in js["durations"]]

# ==============================
# TSP / Route building
# ==============================
def solve_roundtrip(time_matrix: List[List[int]], start_idx: int, search_seconds: int = 20) -> List[int]:
    n = len(time_matrix)
    manager = pywrapcp.RoutingIndexManager(n, 1, start_idx)
    routing = pywrapcp.RoutingModel(manager)

    def transit(i, j):
        return time_matrix[manager.IndexToNode(i)][manager.IndexToNode(j)]

    cb = routing.RegisterTransitCallback(transit)
    routing.SetArcCostEvaluatorOfAllVehicles(cb)

    p = pywrapcp.DefaultRoutingSearchParameters()
    p.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    p.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    p.time_limit.seconds = search_seconds
    sol = routing.SolveWithParameters(p)
    if not sol:
        raise RuntimeError("No solution found")

    order = []
    idx = routing.Start(0)
    while not routing.IsEnd(idx):
        order.append(manager.IndexToNode(idx))
        idx = sol.Value(routing.NextVar(idx))
    order.append(manager.IndexToNode(idx))  # back to start
    return order

def to_path_from_cycle(cycle: List[int], tm: List[List[int]], start_idx: int) -> List[int]:
    """
    Convert a roundtrip cycle into an open path by dropping the single heaviest leg.
    Returns reordered indices starting at start_idx and ending at the best end.
    """
    # Find edge with maximum cost in the cycle
    max_w = -1
    max_pos = -1
    for i in range(len(cycle) - 1):
        a, b = cycle[i], cycle[i+1]
        w = tm[a][b]
        if w > max_w:
            max_w, max_pos = w, i
    # Build path by skipping that edge
    path_nodes = cycle[max_pos+1:-1] + cycle[:max_pos+1]
    # Rotate so it starts at start_idx if present
    if start_idx in path_nodes:
        k = path_nodes.index(start_idx)
        path_nodes = path_nodes[k:] + path_nodes[:k]
    return path_nodes

def solve_open(time_matrix: List[List[int]], start_idx: int) -> List[int]:
    # Get a strong roundtrip then drop the worst edge to make it open (good and simple)
    cycle = solve_roundtrip(time_matrix, start_idx, search_seconds=30)
    return to_path_from_cycle(cycle, time_matrix, start_idx)

def solve_fixed_end(time_matrix: List[List[int]], start_idx: int, end_idx: int) -> List[int]:
    """Try roundtrip, convert to open, then rotate to end at end_idx with minimal extra cost."""
    path = solve_open(time_matrix, start_idx)
    # rotate so that it ends at end_idx
    if end_idx in path:
        # Ensure path starts at start_idx
        if path[0] != start_idx and start_idx in path:
            k = path.index(start_idx)
            path = path[k:] + path[:k]
        # Now rotate so last node is end_idx
        if path[-1] != end_idx:
            k = path.index(end_idx)
            path = path[:k+1] + path[k+1:]
            # move any tail to the front if needed (already fine as end==last)
            if path[-1] != end_idx:
                path = path[:k+1]
    return path

# ==============================
# Utils
# ==============================
def route_time(tm: List[List[int]], order: List[int], roundtrip: bool) -> int:
    total = 0
    for a, b in zip(order, order[1:]):
        total += tm[a][b]
    if roundtrip:
        total += tm[order[-1]][order[0]]
    return total

def fmt_hm(sec: int) -> str:
    h = sec // 3600
    m = (sec % 3600) // 60
    return f"{h}h {m}m" if h else f"{m}m"

# ==============================
# MAIN
# ==============================
def main():
    print("Geocoding…")
    coords = geocode_all(STOPS)
    for s, (lat, lng) in zip(STOPS, coords):
        print(f" - {s} -> {lat:.6f},{lng:.6f}")

    print("\nBuilding time matrix with OSRM…")
    tm = osrm_table_durations(coords)

    print("\nSolving best order…")
    if ROUTE_MODE == "roundtrip":
        order = solve_roundtrip(tm, START_IDX, search_seconds=30)[:-1]  # drop the explicit return for display
        total = route_time(tm, order, roundtrip=True)
        label = "start → … → start"
    elif ROUTE_MODE == "open":
        order = solve_open(tm, START_IDX)
        total = route_time(tm, order, roundtrip=False)
        label = "start → … (no return)"
    elif ROUTE_MODE == "fixed_end":
        if FIXED_END_NAME is None or FIXED_END_NAME not in STOPS:
            raise ValueError("Set FIXED_END_NAME to one of your STOPS when ROUTE_MODE='fixed_end'")
        end_idx = STOPS.index(FIXED_END_NAME)
        order = solve_fixed_end(tm, START_IDX, end_idx)
        total = route_time(tm, order, roundtrip=False)
        label = f"start → … → {FIXED_END_NAME}"
    else:
        raise ValueError("ROUTE_MODE must be 'roundtrip', 'open', or 'fixed_end'")

    print(f"\nBest visiting order ({label}):")
    for i in order:
        print(" -", STOPS[i])

    print(f"\nEstimated drive time (average speeds): {fmt_hm(total)}")

if __name__ == "__main__":
    main()
