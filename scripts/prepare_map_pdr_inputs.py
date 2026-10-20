"""Align Sensor Logger native streams for local map-PDR comparisons, no map fit."""
import argparse,csv,hashlib,io,json,zipfile
from pathlib import Path
import numpy as np
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--zip',type=Path,required=True)
parser.add_argument('--attitude',type=Path,required=True,help='Existing NWU-to-relative-ZXY conversion JSON')
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
with zipfile.ZipFile(args.zip) as z:
    def read(name):
        rows=list(csv.DictReader(io.StringIO(z.read(name+'.csv').decode('utf-8-sig'))))
        return {k:np.array([float(r[k]) for r in rows]) for k in rows[0]}
    inc,linear,gyro,baro=[read(n) for n in ['AccelerometerUncalibrated','Accelerometer','Gyroscope','Barometer']]
t=inc['seconds_elapsed']
def axes(d,scale=1):return np.column_stack([np.interp(t,d['seconds_elapsed'],d[k])*scale for k in ['x','y','z']])
att=json.loads(args.attitude.read_text('utf-8'))
angles=np.asarray(att['rows'])
assert np.max(np.abs(t-angles[:,0]))<1e-8
samples=np.column_stack([t,axes(inc,9.80665),axes(linear),axes(gyro,180/np.pi),angles[:,1:]])
data={'source_sha256':hashlib.sha256(args.zip.read_bytes()).hexdigest(),
      'columns':['seconds','inc_x_mps2','inc_y_mps2','inc_z_mps2','linear_x_mps2','linear_y_mps2','linear_z_mps2',
                 'gyro_beta_deg_s','gyro_gamma_deg_s','gyro_alpha_deg_s','relative_alpha_deg','beta_deg','gamma_deg'],
      'samples':samples.tolist(),'barometer':np.column_stack([baro['seconds_elapsed'],baro['relativeAltitude']]).tolist(),
      'native_recording_not_safari':True,'arbitrary_attitude_yaw_origin_degrees':att['arbitrary_yaw_origin_degrees']}
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(data,separators=(',',':')),encoding='utf-8')
print(json.dumps({'samples':len(samples),'output':str(args.output),'source_sha256':data['source_sha256']}))
