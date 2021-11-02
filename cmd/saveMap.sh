#!/bin/bash

cd ..


echo "Saving map $HOME/"LIO-SAM_Maps/GlobalMap" and converting to georeferenced las file" 

rosservice call /lio_sam/save_map "resolution: 0.05
destination: '/LIO-SAM_Maps'"

python FillJson.py $HOME/"LIO-SAM_Maps/GlobalMap.pcd"

pdal pipeline ConvertLasToPcdWithGeoRef.json

echo "Las info: "

pdal info $HOME/"LIO-SAM_Maps/GlobalMap.las"

