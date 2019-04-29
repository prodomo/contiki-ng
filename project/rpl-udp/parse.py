import serial
from collections import OrderedDict
import MySQLdb
from datetime import datetime, timedelta

ser = serial.Serial('/dev/ttyACM0', 115200)

key_map = OrderedDict({'DATA_LEN':0, 'NODE_ID':1, 'SEQNO':2, 'STATUSCATEGORY':3, 'STATUS':4})

def upload_to_DB(data):
  try:
    db = MySQLdb.connect("127.0.0.1","root","sakimaru","ITRI_TIC" )
    cursor = db.cursor()
     

    insert_sql = "INSERT INTO RowData(seqno, statusCategory, status, mac) VALUES({0}, {1}, {2}, '{3}')".format(int(data[key_map['SEQNO']]), int(data[key_map['STATUSCATEGORY']]), int(data[key_map['STATUS']]), data[key_map['NODE_ID']])

    update_sql = "REPLACE INTO current(seqno, statusCategory, status, mac) VALUES({0}, {1}, {2}, '{3}')".format(int(data[key_map['SEQNO']]), int(data[key_map['STATUSCATEGORY']]), int(data[key_map['STATUS']]), data[key_map['NODE_ID']])
    try:
        # Execute the SQL command
      # print (insert_sql)
      cursor.execute(insert_sql)
      db.commit()
      print update_sql
      cursor.execute(update_sql)
      db.commit()

      print 'insert DB success'
    except:
          # Rollback in case there is any error
          #db.rollback()
      print 'insert DB failed, do not rollback'
  except:
      print'connect failed'
      

while True:
    data = ser.readline()
    if data:
      print(data)
      try:
        split_data = data.split(' ',5)
        print 'split_data={0}, s0={1}, s3={2}'.format(split_data, split_data[key_map['DATA_LEN']], split_data[key_map['NODE_ID']])
        upload_to_DB(split_data)
      except:
        pass
