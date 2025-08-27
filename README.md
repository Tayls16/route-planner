# Route Planner (OSRM + OR-Tools)

A desktop app that finds the best order to visit multiple stops using [OSRM](http://project-osrm.org/) and [Google OR-Tools](https://developers.google.com/optimization).  

🚀 Features:
- Optimises delivery routes
- Works offline with a local OSRM server
- Simple desktop GUI with PyQt
- Exports routes to an interactive Leaflet map

## Requirements
- Python 3.12+
- Docker Desktop
- OSRM map extract (e.g. [Geofabrik](https://download.geofabrik.de/))

## Setup
```bash
git clone https://github.com/Tayls16/route-planner.git
cd route-planner
pip install -r requirements.txt
python app.py
