# -*- coding: utf-8 -*-
"""
Created on Sat Jun  9 15:47:29 2018

@author: annaf
"""
#!/usr/bin/python3

import xml.etree.ElementTree as ET
import json
kmlFileName = 'input.kml'
jsonFileName = 'output.json'

tree = ET.parse(kmlFileName)
root = tree.getroot()
# get the list of waypoints
waypoints = []
for point in root.iter('{http://www.opengis.net/kml/2.2}Point'):
    coordinate = point.find('{http://www.opengis.net/kml/2.2}coordinates')
    coordinateString = coordinate.text
    splitCoordinates = coordinateString.split(',')
    waypoint = [float(splitCoordinates[0]), float(splitCoordinates[1])]
    waypoints.append(waypoint)
# create a dictionary
waypointNumber = 1
routeDict = {}
for waypoint in waypoints:
    waypointDict = {"is_checkpoint" : 0, "latitude" : waypoint[1],
                    "longitude" : waypoint[0], "declination" : 6,
                    "radius" : 15, "stay_time" : 0, "harvested" : 0}
    routeDict[str(waypointNumber)] = waypointDict
    waypointNumber = waypointNumber + 1
# write as json file
fp = open(jsonFileName, 'w')
json.dump(routeDict, fp, indent=4)
