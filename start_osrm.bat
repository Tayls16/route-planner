@echo off
cd /d "C:\OSRM"
docker run -t -i -p 5000:5000 -v "%cd%":/data osrm/osrm-backend osrm-routed --algorithm mld /data/australia-latest.osrm

