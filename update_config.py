import json
import sqlite3

cfg = json.load(open('configuration.json'))
conn = sqlite3.connect('asr.db')
db = conn.cursor()

for table in cfg:
    data = cfg[table]
    setstr = ''
    for key, value in cfg[table].items():
        if isinstance(value, str):
            value = '"' + value + '"'
        else:
            value = str(value)
        if setstr == '':
            setstr = key + ' = ' + value
        else:
            setstr = setstr + ', ' + key + ' = ' + value
    db.execute('UPDATE ' + str(table) + ' SET '
               + setstr + ' WHERE ID = 1;')
conn.commit()
db.close()
