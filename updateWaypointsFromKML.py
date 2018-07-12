import sqlite3
import sys
import xml.etree.ElementTree as ET



#Defult is triangel outside MSC
if len(sys.argv) > 1:
    filepath = str(sys.argv[1])
else:
    filepath = 'MSC.kml'


id = 0
is_checkpoint = 0
declination = 6
radius = 16
stay_time = 0
harvested = 0

wps = []
id = 0

#Steps trought tree to find all placemarks
def iterTree(element):
    for child in element:
        #print(child.tag)
        if "Point" in child.tag:
            wps.append(parsePoint(child))
        iterTree(child)



#Gets coords and name from placemark
def parsePoint(point):
    global id
    id += 1
    lat = ""
    long = ""
    for child in point:
        if "coordinates" in child.tag:
            cord = child.text
            lat = cord.split()[0].split(',')[1]
            long = cord.split()[0].split(',')[0]

    return (id, is_checkpoint, lat,long, declination, radius, stay_time, harvested)



def main():

    tree = ET.parse(filepath)
    root = tree.getroot()

    conn = sqlite3.connect('asr.db')
    db = conn.cursor()

    iterTree(root)
    insertstr = "INSERT INTO current_Mission (id, is_checkpoint, latitude, longitude, declination, radius, stay_time, harvested) VALUES "

    db.execute('DELETE FROM current_Mission')
    for wp in wps:
        print(insertstr+ str(wp))
        db.execute(insertstr+ str(wp))
    conn.commit()
    db.close()

main()