#!/bin/bash

cd ..

echo "Enter save folder name "
read folderName


echo "Saving maps to $HOME/"LIO-SAM_Maps/$folderName/" and converting to georeferenced las file" 

rosservice call /lio_sam/save_map "resolution: 0.0
destination: '/LIO-SAM_Maps/$folderName/'"

python FillJson.py $HOME/"LIO-SAM_Maps/$folderName/GlobalMap.pcd"

echo "Converting to Las file.."

pdal pipeline $HOME/"LIO-SAM_Maps/$folderName/ConvertLasToPcdWithGeoRef.json"

echo "Done."

echo "Las info: "

pdal info $HOME/"LIO-SAM_Maps/$folderName/GlobalMap.las"

