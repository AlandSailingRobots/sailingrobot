import json
import sqlite3

cfg = json.load(open('configuration.json'))
conn = sqlite3.connect('asr.db')
db = conn.cursor()

for table in cfg:
    data = cfg[table]
    setstr = ''
    keystr = ''
    valstr = ''
    for key, value in cfg[table].items():
        if isinstance(value, str):
            value = '"' + value + '"'
        else:
            value = str(value)
        if (setstr == ''):
            setstr = key + ' = ' + value
            keystr = key
            valstr = value
        else:
            setstr = setstr + ', ' + key + ' = ' + value
            keystr = keystr + ', ' + key
            valstr = valstr + ', ' + value
    db.execute('SELECT count(*) FROM ' + str(table) + ';')
    count = db.fetchone()[0]
    if count == 0:
        db.execute('INSERT INTO ' + str(table) + ' (' + keystr +
                   ') VALUES (' + valstr + ');')
    else:
        db.execute('UPDATE ' + str(table) + ' SET '
                   + setstr + ' WHERE ID = 1;')
conn.commit()
db.close()
