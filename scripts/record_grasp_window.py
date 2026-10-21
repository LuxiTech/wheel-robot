"""Record only an Isaac window, ending at this grasp run's success/failure."""
import argparse,json,time
from pathlib import Path
import cv2
import numpy as np
from record_relay_window import WindowCapture
p=argparse.ArgumentParser();p.add_argument('window',type=lambda x:int(x,0));p.add_argument('run',type=Path);a=p.parse_args()
c=WindowCapture(a.window);fps=12
writer=cv2.VideoWriter(str(a.run/'execution.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),fps,(c.width,c.height))
if not writer.isOpened():raise RuntimeError('Video encoder unavailable')
start=time.time();frames=0;end=None
try:
 while time.time()-start<1800:
  tick=time.monotonic();frame=np.frombuffer(c.read(),np.uint8).reshape(c.height,c.width,4)[:,:,:3]
  writer.write(frame);frames+=1
  if frames%fps==0:cv2.imwrite(str(a.run/'viewer.png'),frame)
  report=a.run/'relay_report.json'
  relay=json.loads(report.read_text()) if report.exists() else {}
  if (a.run/'simulate.result.json').exists() or (a.run/'simulate.error.json').exists() or relay.get('passed') or relay.get('skill_passed') or relay.get('error'):
   if end is None:end=time.time()+3
   if time.time()>end:break
  time.sleep(max(0,1/fps-(time.monotonic()-tick)))
finally:
 writer.release();c.close()
 (a.run/'video.json').write_text(json.dumps(dict(frames=frames,fps=fps,scope='Isaac window only',started_at=start,ended_at=time.time()),indent=2))
