#!/usr/bin/env python3.4

# Creates xml from db table scanning_measurements

import sqlite3
import xml.etree.ElementTree as ET
from xml.dom import minidom
from xml.dom import expatbuilder

def add(parent, element, text):
	"add SubElement"
	sub_element = ET.SubElement(parent, element)
	sub_element.text = text


print ("\nGenerating xml...")

# open db and file
db = sqlite3.connect('asr.db')
xml = open('area_scanning.xml', 'wb')
# clear file
xml.truncate()

# ----------------
# create xml
# ----------------

area_scanning = ET.Element("AreaScanning",
	{"Teamid":"",
	 "xmlns:gml":"http://www.opengis.net/gml",
	 "xmlns:smil20":"http://www.w3.org/2001/SMIL20/",
	 "xmlns:smil20lang":"http://www.w3.org/2001/SMIL20/Language",
	 "xmlns:xlink":"http://www.w3.org/1999/xlink",
	 "xmlns:xml":"http://www.w3.org/XML/1998/namespace",
	 "xmlns:xsi":"http://www.w3.org/2001/XMLSchema-instance",
	 "xsi:noNamespaceSchemaLocation":"AreaScanningResult.xsd"
	 })

add(area_scanning, "TeamName", "Team Name")


for row in db.execute('SELECT * FROM scanning_measurements'):
	section = ET.SubElement(area_scanning, "Section")

	add(section, "sectioni", str(row[2]))
	add(section, "sectionj", str(row[3]))

	dateTime = row[4] + "Z"
	
	dateTime = dateTime.replace(" ","T")
	add(section, "dateTime", dateTime)

	sub_element = ET.SubElement(section, "gml:pos",
		{"srsDimension":"2",
		 "srsName":"urn:ogc:def:crs:EPSG:6.6:4326",
		 })
	sub_element.text = str(row[5]) + " " + str(row[6])

	add(section, "airtemp", str(row[7]))


# make it pretty
rough_string = ET.tostring(area_scanning, 'UTF-8')
reparsed = minidom.parseString(rough_string)
pretty = reparsed.toprettyxml(indent="\t",encoding="UTF-8")

# write to file and close everything
xml.write(pretty)

xml.close()
db.close()


print ("DONE\n")