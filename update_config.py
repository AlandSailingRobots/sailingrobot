#!/usr/bin/python3

# Updates the configuration in the json to the database
# Can run without argument for using standard file
# Or specify the file by passing it as a argument

import json
import sqlite3
import sys

if len(sys.argv) > 1:
    if str(sys.argv[1]) == 'ASPire':
        filename = 'config_ASPire.json'
    elif str(sys.argv[1]) == 'Janet':
        filename = 'config_Janet.json'
    elif str(sys.argv[1]) == 'Velvet':
        filename = 'config_Velvet.json'
    else :
        filename = str(sys.argv[1])
else:
    filename = 'config_ASPire.json'



print(filename)
try:
    cfg = json.load(open(filename))
except FileNotFoundError:
    sys.exit('Error to open the file.\nPlease enter in argument either \'ASPire\', \'Janet\', \'Velvet\' or the filepath.')

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
    try:
        db.execute('SELECT count(*) FROM ' + str(table) + ';')
    except sqlite3.OperationalError:
        sys.exit('Error to retrieve the tables.\nCheck if the selected file \''+filename+'\' correspond to the current DataBase configuration')
    count = db.fetchone()[0]
    if count == 0:
        db.execute('INSERT INTO ' + str(table) + ' (' + keystr +
                   ') VALUES (' + valstr + ');')
    else:
        db.execute('UPDATE ' + str(table) + ' SET ' +
                   setstr + ' WHERE ID = 1;')
conn.commit()
db.close()
