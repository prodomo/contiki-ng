import serial
from collections import OrderedDict
import MySQLdb
from datetime import datetime, timedelta

ser = serial.Serial('/dev/ttyUSB0', 115200)

key_map = OrderedDict({'DATA_LEN':0, 'NODE_ID':1, 'HOPS':2, 'SEQNO':3, 'BEST_NEIGHBOR':4, 'BEST_NEIGHBOR_ETX':5,
 'RTMETRIC':6, 'NUM_NEIGHBORS':7, 'RSSI':8 ,'TEMP':9, 'EXT_T':10, 'INT_T':11 ,'BATTERY':12})

def upload_to_DB(data):
  try:
    #db = MySQLdb.connect("127.0.0.1","root","sakimaru","ITRI_OpenWSN" )
    db = MySQLdb.connect("http://carbooky.dsmynas:59506","root","sakimaru","ITRI_OpenWSN" )
    cursor = db.cursor()
    sql= "CREATE TABLE IF NOT EXISTS itri_MOEA_current_sensor ( `sn` INT(11), `position` VARCHAR(80), `mac_addr` VARCHAR(45), `led_status` VARCHAR(45), `pyranometer` INT(11), `int_temperature` FLOAT, `ext_temperature` FLOAT, `battery_volt` FLOAT, datetime DATETIME, `ID` INT(11), PRIMARY KEY(`mac_addr`))"

    try:
        cursor.execute(sql)
        db.commit()
    except:
        #db.rollback()
        print '1.connect SQL failed !!'

    

      #mySql cmd
    ext_value = 0.0
    ext_value += float(data[key_map['EXT_T']]) /100
    int_value = 0.0
    int_value += float(data[key_map['INT_T']]) /1000

    print "ext_value: {0}, {1}".format(int(data[key_map['EXT_T']]), ext_value)
    print "ext_value: {0}, {1}".format(int(data[key_map['INT_T']]), int_value)


      #add by Nancy
    etx = 0.0
    etx += int(data[key_map['BEST_NEIGHBOR_ETX']])
    pdr = 0.0
    pdr += 1.0/((etx/float(data[key_map['HOPS']]))/64)
    rssi = 0.0
    rssi += float(data[key_map['RSSI']])-65535-1

    insert_sql = "INSERT INTO itri_MOEA_sensor(sn, mac_addr, ext_temperature, pyranometer, datetime, int_temperature, battery_volt) \
    VALUES({0}, '{1}', {2}, {3}, '{4}', {5}, {6})".format(int(data[key_map['SEQNO']]), data[key_map['NODE_ID']],\
     ext_value, data[key_map['INT_T']], datetime.now(), int_value, data[key_map['BATTERY']])

    rps_sql = "REPLACE INTO itri_MOEA_current_sensor(sn, mac_addr, ext_temperature, pyranometer, datetime, int_temperature, battery_volt) \
    VALUES({0}, '{1}', {2}, {3}, '{4}', {5}, {6})".format(int(data[key_map['SEQNO']]), data[key_map['NODE_ID']],\
     ext_value, data[key_map['INT_T']], datetime.now(), int_value, data[key_map['BATTERY']])
    try:
        # Execute the SQL command
      print (insert_sql)
      cursor.execute(insert_sql)
      db.commit()
      cursor.execute(rps_sql)
        # Commit your changes in the database
      db.commit()
      print 'insert sensor success'
    except:
          # Rollback in case there is any error
          #db.rollback()
      print 'insert DB failed, do not rollback'
      

      ## topology part


    sql = "INSERT INTO itri_topology_neighbors(mode, neighborNum, devAddr,PDR, parentAddr, datetime, SN, rank, n1, rssi1)VALUES('{0}', {1}, '{2}', {3}, '{4}', '{5}', {6}, {7}, '{8}', {9})"\
    .format('111', 1, data[key_map['NODE_ID']], pdr, data[key_map['BEST_NEIGHBOR']], datetime.now(), int(data[key_map['SEQNO']]), int(data[key_map['RTMETRIC']]), data[key_map['BEST_NEIGHBOR']], rssi)
    
    rps_sql = "REPLACE INTO itri_topology_current_neighbors(mode, neighborNum, devAddr,PDR, parentAddr, datetime, SN, rank, n1, rssi1)VALUES('{0}', {1}, '{2}', {3}, '{4}', '{5}', {6}, {7}, '{8}', {9})"\
    .format('111', 1, data[key_map['NODE_ID']], pdr, data[key_map['BEST_NEIGHBOR']], datetime.now(), int(data[key_map['SEQNO']]), int(data[key_map['RTMETRIC']]), data[key_map['BEST_NEIGHBOR']], rssi)
   
    try:
          # Execute the SQL command
      print sql
      cursor.execute(sql)
      db.commit()
      cursor.execute(rps_sql)
          # Commit your changes in the database
      db.commit()
      print 'insert neighbor success'
    except:
          # Rollback in case there is any error
          #db.rollback()
      print 'insert DB failed, do not rollback'


    db.close()
  except:
    print '3.connect to SQL failed !'




while True:
    data = ser.readline()
    if data:
      print(data)
      try:
        split_data = data.split(' ',13)
        print 'split_data={0}, s0={1}, s3={2}'.format(split_data, split_data[key_map['DATA_LEN']], split_data[key_map['NODE_ID']])
        upload_to_DB(split_data)
      except:
        pass
