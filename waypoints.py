import json
import sqlite3
from collections import OrderedDict

conn = sqlite3.connect('asr.db')
db = conn.cursor()
path = 'json/'
filename = 'Eckero_To_Grisslehamn.json'
waypoints = json.load(open(path + filename), object_pairs_hook=OrderedDict)

db.execute('DELETE FROM waypoints')
for wp in waypoints:
    keystr = 'id'
    valstr = str(wp)
    for key, value in waypoints[wp].items():
        value = str(value)
        keystr = keystr + ', ' + key
        valstr = valstr + ', ' + value
    db.execute('INSERT INTO waypoints (' + keystr +
               ') VALUES (' + valstr + ')')
conn.commit()
db.close()
