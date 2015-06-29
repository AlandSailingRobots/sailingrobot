#!/usr/bin/env python3.4

import sys
import csv
import sqlite3


def readWaypointsFromCSVFileWithRadius(filename, radius):
	with open(filename, newline='') as csvfile:
		waypointList = []
		waypointReader = csv.reader(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_NONNUMERIC)
		for row in waypointReader:
			waypointWithRadius = [row[0], row[1], radius]
			waypointList.append(waypointWithRadius)
		return waypointList 

def getLatitudeKey(item):
	return item[0]

def getLongitudeKey(item):
	return item[1]

def sortWaypointListToSouth(waypointList):
	return sorted(waypointList, key=getLatitudeKey, reverse=True)

def sortWaypointListToNorth(waypointList):
	return sorted(waypointList, key=getLatitudeKey, reverse=False)	

def sortWaypointListToWest(waypointList):
	return sorted(waypointList, key=getLongitudeKey, reverse=True)

def sortWaypointListToEast(waypointList):
	return sorted(waypointList, key=getLongitudeKey, reverse=False)


def setWaypointsInDB(waypointList):
	conn = sqlite3.connect('asr.db')
	id = 1
	for row in waypointList:
		lat = row[0]
		lon = row[1]
		radius = row[2]
		conn.execute("INSERT INTO waypoints (id,lat,lon,radius) VALUES (?,?,?,?)", [id, lat, lon, radius])
		id = id+1
	conn.commit()
	conn.close()

def getTriangle1WaypointList(waypointList, rounds, reverse):
 	resultingWaypointList = []
 	for i in range(rounds):	
 		resultingWaypointList.append(waypointList[0])
 		if reverse:
 			resultingWaypointList.append(waypointList[2]) 
 			resultingWaypointList.append(waypointList[1])
 		else:
 			resultingWaypointList.append(waypointList[1])
 			resultingWaypointList.append(waypointList[2]) 
 	resultingWaypointList.append(waypointList[0])
 	return resultingWaypointList


firstWaypointList = readWaypointsFromCSVFileWithRadius(
	'waypointsPart1.csv', 25)
southWaypointList = sortWaypointListToSouth(firstWaypointList)
secondWaypointList = readWaypointsFromCSVFileWithRadius(
	'waypointsPart2.csv', 25)
westWaypointList = sortWaypointListToWest(secondWaypointList)
#first triangle - northwest, southwest, east
lagneskarTriangleWaypoints1=[["60.069941261", "19.902311946", "50"], 
["60.079282318", "19.889880245", "50"],["60.069941261", "19.879058051", "50"]]
triangleWaypointList = getTriangle1WaypointList(
	lagneskarTriangleWaypoints1, 3, True)
eastWaypointList = sortWaypointListToEast(secondWaypointList)
northWaypointList = sortWaypointListToNorth(firstWaypointList)
waypointList = southWaypointList + westWaypointList + \
triangleWaypointList + eastWaypointList + northWaypointList
setWaypointsInDB(waypointList)

