import sys, time, json, webbrowser, pathlib, datetime
from typing import List, Tuple, Optional
import requests
import pandas as pd

from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QTextEdit, QComboBox, QLineEdit, QMessageBox, QListWidget, QListWidgetItem,
    QFileDialog, QInputDialog
)
from PyQt6.QtCore import Qt

# ==============================
# CONFIG
# ==============================
OSRM_URL = "http://localhost:5000"     # Your local OSRM server
USER_AGENT = "RoutePlannerTaylah/1.0"  # Required by Nominatim
GEOCODE_SLEEP = 1.0                    # Be polite to Nominatim (1 req/sec)

# ---------- OR-Tools ----------
from ortools.constraint_solver import pywrapcp, routing_enums_pb2


# ==============================
# Core helpers
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

def solve_roundtrip(time_matrix: List[List[int]], start_idx: int, search_seconds: int = 30) -> List[int]:
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
    max_w = -1
    max_pos = -1
    for i in range(len(cycle) - 1):
        a, b = cycle[i], cycle[i+1]
        w = tm[a][b]
        if w > max_w:
            max_w, max_pos = w, i
    path_nodes = cycle[max_pos+1:-1] + cycle[:max_pos+1]
    if start_idx in path_nodes:
        k = path_nodes.index(start_idx)
        path_nodes = path_nodes[k:] + path_nodes[:k]
    return path_nodes

def solve_open(time_matrix: List[List[int]], start_idx: int) -> List[int]:
    cycle = solve_roundtrip(time_matrix, start_idx, search_seconds=30)
    return to_path_from_cycle(cycle, time_matrix, start_idx)

def solve_fixed_end(time_matrix: List[List[int]], start_idx: int, end_idx: int) -> List[int]:
    path = solve_open(time_matrix, start_idx)
    if end_idx in path:
        if path[0] != start_idx and start_idx in path:
            k = path.index(start_idx)
            path = path[k:] + path[:k]
        if path[-1] != end_idx:
            k = path.index(end_idx)
            path = path[:k+1] + path[k+1:]
            if path[-1] != end_idx:
                path = path[:k+1]
    return path

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

def write_leaflet_map(filepath: pathlib.Path, coords: List[Tuple[float, float]], order: List[int], title: str):
    ordered_coords = [coords[i] for i in order]
    markers = [{"idx": i, "lat": lat, "lng": lng} for i, (lat, lng) in zip(order, ordered_coords)]
    geojson = {
        "type": "LineString",
        "coordinates": [[c[1], c[0]] for c in ordered_coords]  # lng,lat
    }
    html = f"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8"/>
<title>{title}</title>
<meta name="viewport" content="width=device-width,initial-scale=1.0"/>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>html,body,#map{{height:100%;margin:0;}}</style>
</head>
<body>
<div id="map"></div>
<script>
var map = L.map('map');
L.tileLayer('https://tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
  attribution:'© OpenStreetMap'
}}).addTo(map);
var line = {json.dumps(geojson)};
var layer = L.geoJSON(line).addTo(map);
map.fitBounds(layer.getBounds().pad(0.2));
var markers = {json.dumps(markers)};
markers.forEach(function(m,step){{
  L.marker([m.lat,m.lng]).addTo(map).bindPopup("Step " + (step+1));
}});
</script>
</body>
</html>"""
    filepath.write_text(html, encoding="utf-8")


# ==============================
# GUI
# ==============================
class RouteApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Route Planner (OSRM + OR-Tools)")
        self.resize(880, 680)

        layout = QVBoxLayout(self)

        # Addresses list + controls
        topRow = QHBoxLayout()
        lbl = QLabel("Stops (one per line). First line is the START:")
        topRow.addWidget(lbl)

        self.btnLoadExcel = QPushButton("Load from Excel/CSV")
        self.btnLoadExcel.clicked.connect(self.on_load_excel)
        topRow.addWidget(self.btnLoadExcel)

        layout.addLayout(topRow)

        self.txtStops = QTextEdit()
        self.txtStops.setPlaceholderText("70 Ricky Way, Epping VIC\nMelbourne Museum\nRMIT University, Melbourne\nVictoria University, Footscray Park")
        layout.addWidget(self.txtStops)

        # Mode row
        row = QHBoxLayout()
        row.addWidget(QLabel("Route mode:"))
        self.cmbMode = QComboBox()
        self.cmbMode.addItems(["open (start → …)", "roundtrip (start → … → start)", "fixed end (start → … → end)"])
        row.addWidget(self.cmbMode, 1)

        row.addWidget(QLabel("Fixed end (only if 'fixed end' mode):"))
        self.edEnd = QLineEdit()
        self.edEnd.setPlaceholderText("Type exact end stop text as in the list")
        row.addWidget(self.edEnd, 2)

        layout.addLayout(row)

        # Buttons
        btnRow = QHBoxLayout()
        self.btnSolve = QPushButton("Optimize Route")
        self.btnSolve.clicked.connect(self.on_solve)
        btnRow.addWidget(self.btnSolve)

        self.btnMap = QPushButton("Open Map")
        self.btnMap.clicked.connect(self.on_open_map)
        self.btnMap.setEnabled(False)
        btnRow.addWidget(self.btnMap)

        self.btnExport = QPushButton("Export to Excel")
        self.btnExport.clicked.connect(self.on_export_excel)
        self.btnExport.setEnabled(False)
        btnRow.addWidget(self.btnExport)

        layout.addLayout(btnRow)

        # Results
        layout.addWidget(QLabel("Result:"))
        self.listResult = QListWidget()
        layout.addWidget(self.listResult)

        self.status = QLabel("")
        self.status.setAlignment(Qt.AlignmentFlag.AlignLeft)
        layout.addWidget(self.status)

        # Data holders (for export)
        self._coords: List[Tuple[float, float]] = []
        self._order: List[int] = []
        self._stops: List[str] = []
        self._tm: Optional[List[List[int]]] = None
        self._mode_label: str = ""
        self._roundtrip: bool = False
        self._total_secs: int = 0
        self._coords_from_excel: Optional[List[Tuple[float, float]]] = None  # if provided

    # ---------- Excel/CSV import ----------
    def on_load_excel(self):
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Open Excel/CSV with stops",
            "",
            "Excel/CSV (*.xlsx *.xls *.csv)"
        )
        if not path:
            return

        try:
            if path.lower().endswith(".csv"):
                df = pd.read_csv(path)
            else:
                df = pd.read_excel(path)  # requires openpyxl installed
        except Exception as e:
            self.show_error(f"Failed to open file:\n{e}")
            return

        if df.empty:
            self.show_error("The file appears to be empty.")
            return

        # Try to auto-detect columns
        cols = [c.strip() for c in df.columns.astype(str)]
        lower = [c.lower() for c in cols]

        addr_candidates = []
        for want in ["address", "stop", "location", "site", "addr"]:
            if want in lower:
                addr_candidates.append(cols[lower.index(want)])

        lat_col = None
        lng_col = None
        for want in ["lat", "latitude"]:
            if want in lower:
                lat_col = cols[lower.index(want)]
                break
        for want in ["lng", "lon", "long", "longitude"]:
            if want in lower:
                lng_col = cols[lower.index(want)]
                break

        # Let user choose address column if not clear
        addr_col = None
        if addr_candidates:
            addr_col = addr_candidates[0]
        else:
            ok = False
            addr_col, ok = QInputDialog.getItem(
                self,
                "Select address column",
                "Pick the column that contains addresses (or cancel if using lat/lng):",
                cols, 0, False
            )
            if not ok:
                addr_col = None

        stops = []
        coords_from_excel = []

        if lat_col and lng_col:
            # Use provided coordinates (skip geocoding)
            for _, row in df.iterrows():
                try:
                    lat = float(row[lat_col])
                    lng = float(row[lng_col])
                    coords_from_excel.append((lat, lng))
                    if addr_col and pd.notna(row.get(addr_col, "")):
                        stops.append(str(row[addr_col]))
                    else:
                        stops.append(f"{lat:.6f},{lng:.6f}")
                except Exception:
                    continue
        elif addr_col:
            # Only addresses provided
            for v in df[addr_col].dropna().astype(str).tolist():
                s = v.strip()
                if s:
                    stops.append(s)
        else:
            self.show_error("Could not find an address column or lat/lng columns.")
            return

        if len(stops) < 2:
            self.show_error("Need at least two stops.")
            return

        # Fill the text box; first row becomes start
        self.txtStops.setPlainText("\n".join(stops))
        self._coords_from_excel = coords_from_excel if coords_from_excel else None
        QMessageBox.information(self, "Loaded", f"Loaded {len(stops)} stops from:\n{path}")

    # ---------- Core GUI actions ----------
    def show_error(self, msg: str):
        QMessageBox.critical(self, "Error", msg)

    def parse_stops(self) -> List[str]:
        raw = self.txtStops.toPlainText().strip().splitlines()
        stops = [s.strip() for s in raw if s.strip()]
        if len(stops) < 2:
            raise ValueError("Please enter at least 2 stops (one per line).")
        return stops

    def on_solve(self):
        try:
            stops = self.parse_stops()
        except Exception as e:
            self.show_error(str(e))
            return

        self.listResult.clear()
        self.status.setText("Preparing coordinates…")
        QApplication.processEvents()

        # Use coords from Excel if we have them and they match count; else geocode
        coords: List[Tuple[float, float]]
        try:
            if self._coords_from_excel and len(self._coords_from_excel) == len(stops):
                coords = self._coords_from_excel
            else:
                self.status.setText("Geocoding (OSM Nominatim)…")
                QApplication.processEvents()
                coords = geocode_all(stops)
        except Exception as e:
            self.show_error(f"Coordinate lookup error:\n{e}")
            return

        self.status.setText("Building time matrix with OSRM…")
        QApplication.processEvents()
        try:
            tm = osrm_table_durations(coords)
        except Exception as e:
            self.show_error(f"OSRM error:\n{e}")
            return

        mode = self.cmbMode.currentIndex()  # 0=open, 1=roundtrip, 2=fixed
        start_idx = 0

        self.status.setText("Optimising…")
        QApplication.processEvents()
        try:
            if mode == 0:  # open
                order = solve_open(tm, start_idx)
                total = route_time(tm, order, roundtrip=False)
                label = "start → … (no return)"
                roundtrip = False
            elif mode == 1:  # roundtrip
                cyc = solve_roundtrip(tm, start_idx, search_seconds=30)
                order = cyc[:-1]  # display without duplicate start
                total = route_time(tm, order, roundtrip=True)
                label = "start → … → start"
                roundtrip = True
            else:  # fixed end
                end_name = self.edEnd.text().strip()
                if not end_name:
                    raise ValueError("Enter the fixed end stop text exactly as in the list.")
                if end_name not in stops:
                    raise ValueError("Fixed end must match one of the stops.")
                end_idx = stops.index(end_name)
                order = solve_fixed_end(tm, start_idx, end_idx)
                total = route_time(tm, order, roundtrip=False)
                label = f"start → … → {end_name}"
                roundtrip = False
        except Exception as e:
            self.show_error(f"Solver error:\n{e}")
            return

        # Display results
        self._coords, self._order = coords, order
        self._stops, self._tm = stops, tm
        self._mode_label, self._roundtrip, self._total_secs = label, roundtrip, total

        self.listResult.addItem(QListWidgetItem(f"Best order ({label}):"))
        for i in order:
            self.listResult.addItem(QListWidgetItem(f" - {stops[i]}"))
        self.listResult.addItem(QListWidgetItem(""))
        self.listResult.addItem(QListWidgetItem(f"Estimated drive time (avg speeds): {fmt_hm(total)}"))
        self.status.setText("Done.")
        self.btnMap.setEnabled(True)
        self.btnExport.setEnabled(True)

    def on_open_map(self):
        if not self._coords or not self._order:
            self.show_error("No route yet. Click Optimize Route first.")
            return
        out = pathlib.Path.cwd() / "map.html"
        write_leaflet_map(out, self._coords, self._order, "Optimised Route")
        webbrowser.open(out.as_uri())

    def on_export_excel(self):
        if not (self._coords and self._order and self._tm and self._stops):
            self.show_error("No route to export. Click Optimize Route first.")
            return

        order = self._order[:]
        roundtrip = self._roundtrip
        tm = self._tm
        stops = self._stops
        coords = self._coords

        export_pairs = list(zip(order, order[1:]))
        if roundtrip:
            export_pairs.append((order[-1], order[0]))

        rows = []
        cumulative = 0
        step_num = 1
        for a, b in export_pairs:
            leg_secs = tm[a][b]
            cumulative += leg_secs
            rows.append({
                "Step": step_num,
                "From": stops[a],
                "To": stops[b],
                "To_Lat": f"{coords[b][0]:.6f}",
                "To_Lng": f"{coords[b][1]:.6f}",
                "Leg_Duration_s": leg_secs,
                "Leg_Duration_hm": fmt_hm(leg_secs),
                "Cumulative_hm": fmt_hm(cumulative)
            })
            step_num += 1

        df = pd.DataFrame(rows)

        summary = pd.DataFrame([{
            "Route Mode": self._mode_label,
            "Stops": len(order),
            "Estimated Total (avg speeds)": fmt_hm(self._total_secs),
            "Exported At": datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
        }])

        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = pathlib.Path.cwd() / f"route_export_{ts}.xlsx"
        with pd.ExcelWriter(out_path, engine="openpyxl") as writer:
            summary.to_excel(writer, sheet_name="Summary", index=False)
            df.to_excel(writer, sheet_name="Legs", index=False)

        QMessageBox.information(self, "Export complete", f"Saved to:\n{out_path}")
        # webbrowser.open(out_path.as_uri())


def main():
    app = QApplication(sys.argv)
    w = RouteApp()
    w.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()

