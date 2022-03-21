#------------------------------------------------------------
#  -- Project   : graphSLAM
#  ----------------------------------------------------------
#  -- File        : FillJson.py
#  -- Author      : Oscar Johansson 
#  -- Description : EXECUTABLE SCRIPT
#  -- Purpose     : .json creation for use in pdal pipeline
#  -- Reference   :
#------------------------------------------------------------
#  -- Include files.
#  ----------------------------------------------------------

import json
import sys
import os


## Takes the utm file outputted by graph slam and fills a 
## pdal pipeline .json file for adding the spatial reference system.

utmFile = ""
pcdFile = ""
for tags in sys.argv:
    if(tags.endswith(".utm") or tags.endswith(".UTM")):
        utmFile = tags
    if(tags.endswith(".pcd") or tags.endswith(".PCD")):
        pcdFile = tags

if(utmFile == ""):
    if(pcdFile == ""):
        utmFile = input("Enter utm file name: ")
    else:
        utmFile = pcdFile + ".utm"

if(utmFile.endswith(".UTM")):
    utmFile.replace(".UTM", ".utm")

with open(utmFile, 'r') as file:
    print("Opened file: ", utmFile)
    utmOrigin = file.readline().split()
file.close()
   
if(len(utmOrigin) != 3):
    print("Error in utm file: Wrong number of fields")
    sys.exit(-1)

print("Map origin: ", utmOrigin)

x0 = float(utmOrigin[0])
y0 = float(utmOrigin[1])
z0 = float(utmOrigin[2])

in_srs = "+proj=tmerc +lat_0=0 +lon_0=15 +ellps=GRS80 +units=m +no_defs"
out_srs = "+proj=tmerc +lat_0=0 +lon_0=15 +ellps=GRS80 +x_0={} +y_0={} +units=m +no_defs +k_0=0.9996".format(x0, y0)


cloudFile = utmFile.replace(".utm", "")

jsonList = []
jsonList.append({"filename": cloudFile})
jsonList.append({"type":"filters.reprojection", 
                "in_srs":in_srs,
                "out_srs":out_srs})
jsonList.append({"type":"writers.las",
                "a_srs":"EPSG:3006",
                "filename":cloudFile.replace(".pcd", ".las")})

jsonFile = os.path.dirname(pcdFile) + "/ConvertLasToPcdWithGeoRef.json"
print("saving pipeline file to ", jsonFile)

with open(jsonFile, "w+") as json_file:
    json.dump(jsonList, json_file)

json_file.close()
