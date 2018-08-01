#!/usr/bin/python3

# Python script to update/add waypoints to the database with data from a json
# Can run without argument or with a filename as argument

import json
import sqlite3
import sys
from collections import OrderedDict

if len(sys.argv) > 1:
    filepath = str(sys.argv[1])
else:
    filepath = 'Mission/Eckero_To_Grisslehamn.json'

conn = sqlite3.connect('asr.db')
db = conn.cursor()
try:
    waypoints = json.load(open(filepath), object_pairs_hook=OrderedDict)
except FileNotFoundError:
    sys.exit('Error to open the file.\nPlease enter in argument a filepath in the folder Mission')

db.execute('DELETE FROM currentMission')
for wp in waypoints:
    keystr = 'id'
    valstr = str(wp)
    for key, value in waypoints[wp].items():
        value = str(value)
        keystr = keystr + ', ' + key
        valstr = valstr + ', ' + value
    db.execute('INSERT INTO currentMission (' + keystr +
               ') VALUES (' + valstr + ')')
conn.commit()
db.close()

print('Successful insertion of the mission : ' + filepath)
