#!/usr/bin/env python3.4

import sqlite3

print ("\nGenerating xml...")

teamname = "Team name"

db = sqlite3.connect('asr.db')
xml = open('area_scanning.xml', 'w')
xml.truncate()

xml.write('\
<?xml version="1.0" encoding="UTF-8"?>\n\
<AreaScanning Teamid="" xmlns:gml="http://www.opengis.net/gml"\n\
	xmlns:smil20="http://www.w3.org/2001/SMIL20/"\n\
	xmlns:smil20lang="http://www.w3.org/2001/SMIL20/Language"\n\
	xmlns:xlink="http://www.w3.org/1999/xlink"\n\
	xmlns:xml="http://www.w3.org/XML/1998/namespace"\n\
	xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"\n\
	xsi:noNamespaceSchemaLocation="AreaScanningResult.xsd">\n\n\
	<TeamName>')
xml.write(teamname)
xml.write('</TeamName>\n\n')

for row in db.execute('SELECT * FROM scanning_measurements'):
	xml.write('\t<Section>\n')
	xml.write('\t\t<scanId>{}</scanId>\n'.format(row[0]))
	xml.write('\t\t<dateTime>{}</dateTime>\n'.format(row[1]))
	xml.write('\t\t<waypointId>{}</waypointId>\n'.format(row[2]))
	xml.write('\t\t<latitude>{}</latitude>\n'.format(row[3]))
	xml.write('\t\t<longitude>{}</longitude>\n'.format(row[4]))
	xml.write('\t\t<airtemp>{}</airtemp>\n'.format(row[5]))
	xml.write('\t</Section>\n\n')

xml.write('</AreaScanning>')

xml.close()
db.close()

print ("DONE\n")