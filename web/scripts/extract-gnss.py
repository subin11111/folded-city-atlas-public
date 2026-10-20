"""Read-only extraction of unique iOS/RTAB-Map GPS timestamps. Standard library only."""
import argparse
import hashlib
import json
from pathlib import Path
import sqlite3
import struct

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('database',type=Path)
p.add_argument('output',type=Path)
a=p.parse_args()
source=a.database.resolve()
db=sqlite3.connect(source.as_uri()+'?mode=ro',uri=True)
rows=db.execute('SELECT id,stamp,gps FROM Node WHERE gps IS NOT NULL ORDER BY stamp').fetchall()
db.close()
unique={}
for node,stamp,blob in rows:
    if not blob or len(blob)!=48:continue
    t,lon,lat,alt,accuracy,bearing=struct.unpack('<6d',blob)
    if not(t>0 and -90<=lat<=90 and -180<=lon<=180 and accuracy>0):continue
    unique.setdefault(t,{'timestamp':t*1000,'coords':{'latitude':lat,'longitude':lon,'accuracy':accuracy,'accuracyConvention':'unspecified_radius','heading':None,'speed':None},'node':node})
value={'schema':'folded-city-gnss-v1','simulated':False,'source':source.name,'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'description':'Recorded iOS OS location fixes from RTAB-Map GPS blobs, deduplicated by GPS timestamp. Not raw satellite observables. Bearing is not used without speed.','records':[unique[t] for t in sorted(unique)]}
a.output.parent.mkdir(parents=True,exist_ok=True)
a.output.write_bytes((json.dumps(value,ensure_ascii=False,indent=2)+'\n').encode('utf-8'))
print(json.dumps({'source':source.name,'rows':len(rows),'unique_fixes':len(unique)}))
