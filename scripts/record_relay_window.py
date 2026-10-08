"""Record only the Isaac viewer's compositor buffer; finalize MP4 at task end.

Requires X11/XWayland, libXcomposite, and an ffmpeg executable. No physics hooks.
"""
import argparse
import ctypes as C
import json
import os
from pathlib import Path
import signal
import subprocess
import time


class XImage(C.Structure):
    _fields_ = [('width',C.c_int),('height',C.c_int),('xoffset',C.c_int),
                ('format',C.c_int),('data',C.c_void_p),('byte_order',C.c_int),
                ('bitmap_unit',C.c_int),('bitmap_bit_order',C.c_int),
                ('bitmap_pad',C.c_int),('depth',C.c_int),('bytes_per_line',C.c_int),
                ('bits_per_pixel',C.c_int),('red_mask',C.c_ulong),
                ('green_mask',C.c_ulong),('blue_mask',C.c_ulong)]


class WindowCapture:
    def __init__(self, window):
        self.x=C.CDLL('libX11.so.6'); self.composite=C.CDLL('libXcomposite.so.1')
        self.x.XOpenDisplay.argtypes=[C.c_char_p];self.x.XOpenDisplay.restype=C.c_void_p
        self.x.XGetGeometry.argtypes=[C.c_void_p,C.c_ulong,C.POINTER(C.c_ulong),
            C.POINTER(C.c_int),C.POINTER(C.c_int),C.POINTER(C.c_uint),C.POINTER(C.c_uint),
            C.POINTER(C.c_uint),C.POINTER(C.c_uint)]
        self.x.XGetImage.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_int,C.c_uint,C.c_uint,C.c_ulong,C.c_int]
        self.x.XGetImage.restype=C.POINTER(XImage)
        self.x.XDestroyImage.argtypes=[C.POINTER(XImage)]
        self.x.XFreePixmap.argtypes=[C.c_void_p,C.c_ulong]
        self.x.XCloseDisplay.argtypes=[C.c_void_p]
        self.composite.XCompositeNameWindowPixmap.argtypes=[C.c_void_p,C.c_ulong]
        self.composite.XCompositeNameWindowPixmap.restype=C.c_ulong
        self.display=self.x.XOpenDisplay(None)
        if not self.display:raise RuntimeError('Cannot open display')
        # Existing desktop compositor owns redirection; do not redirect desktop.
        self.pixmap=self.composite.XCompositeNameWindowPixmap(self.display,window)
        root=C.c_ulong();x=C.c_int();y=C.c_int();w=C.c_uint();h=C.c_uint();border=C.c_uint();depth=C.c_uint()
        self.x.XGetGeometry(self.display,self.pixmap,C.byref(root),C.byref(x),C.byref(y),
            C.byref(w),C.byref(h),C.byref(border),C.byref(depth))
        self.width,self.height=w.value,h.value

    def read(self):
        ptr=self.x.XGetImage(self.display,self.pixmap,0,0,self.width,self.height,C.c_ulong(-1).value,2)
        if not ptr:raise RuntimeError('Viewer window image unavailable')
        try:
            im=ptr.contents
            # XGetImage on a pixmap may omit visual channel masks (all zero).
            if im.bits_per_pixel!=32 or im.byte_order!=0 or im.red_mask not in (0,0xff0000):
                raise RuntimeError('Unsupported window pixel format')
            if im.bytes_per_line==self.width*4:
                return C.string_at(im.data,im.bytes_per_line*self.height)
            return b''.join(C.string_at(im.data+y*im.bytes_per_line,self.width*4) for y in range(self.height))
        finally:self.x.XDestroyImage(ptr)

    def close(self):
        self.x.XFreePixmap(self.display,self.pixmap)
        self.x.XCloseDisplay(self.display)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--window',type=lambda x:int(x,0),required=True)
    parser.add_argument('--ffmpeg',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--fps',type=int,default=12)
    parser.add_argument('--crop',help='Optional viewport rectangle w:h:x:y')
    parser.add_argument('--after-session',help='Wait for a new physics session so recording starts at initialization')
    parser.add_argument('--preview',action='store_true')
    args=parser.parse_args()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    capture=WindowCapture(args.window)
    source=[args.ffmpeg,'-hide_banner','-loglevel','warning','-f','rawvideo','-pixel_format','bgra',
        '-video_size',f'{capture.width}x{capture.height}','-framerate',str(args.fps),'-i','pipe:0']
    if args.preview:
        subprocess.run(source+['-frames:v','1','-y',str(args.output)],input=capture.read(),check=True)
        capture.close();return
    status=Path(__file__).resolve().parents[1]/'outputs/robot_cameras/operator_status.json'
    initial=json.loads(status.read_text())
    deadline=time.monotonic()+600
    while args.after_session and initial['session']==args.after_session:
        if time.monotonic()>deadline:raise RuntimeError('New physics session did not start')
        time.sleep(.1)
        initial=json.loads(status.read_text())
    session=initial['session']
    metadata=dict(started_at=time.time(),session=session,start_simulation_seconds=initial['simulation_seconds'],
        start_completed=initial['operator']['completed'],frames=0,fps=args.fps,
        scope='Viewer window only; from initialization' if args.after_session else 'Viewer window only; recording begins mid-task',
        width=capture.width,height=capture.height,output=str(args.output))
    stop=[]
    signal.signal(signal.SIGINT,lambda *_:stop.append('interrupted'))
    signal.signal(signal.SIGTERM,lambda *_:stop.append('interrupted'))
    working=args.output.with_suffix('.recording.mkv')
    with args.output.with_suffix('.ffmpeg.log').open('w') as log:
        filters=('crop='+args.crop+',' if args.crop else '')+'scale=1280:-2'
        encoder=subprocess.Popen(source+['-an','-vf',filters,'-c:v','libx264',
            '-preset','ultrafast','-crf','23','-pix_fmt','yuv420p','-threads','2','-y',str(working)],stdin=subprocess.PIPE,stderr=log)
        ended=None;last_write=0
        try:
            while not stop:
                tick=time.monotonic()
                encoder.stdin.write(capture.read())
                metadata['frames']+=1
                if tick-last_write>=2:
                    s=json.loads(status.read_text());metadata['last_status']=s
                    metadata['updated_at']=time.time()
                    args.output.with_suffix('.json').write_text(json.dumps(metadata,indent=2))
                    if s['session']!=session:stop.append('session_changed')
                    if s['operator']['finished'] and ended is None:ended=tick
                    if time.time()-s['updated_at']>60:stop.append('status_stale')
                    last_write=tick
                if ended is not None and tick-ended>=4:stop.append('task_finished')
                time.sleep(max(0,1/args.fps-(time.monotonic()-tick)))
        except Exception as exc:
            stop.append(f'{type(exc).__name__}: {exc}')
        finally:
            encoder.stdin.close();code=encoder.wait();capture.close()
        metadata.update(stop_reason=stop,ended_at=time.time(),encoder_exit_code=code)
        if code==0:
            result=subprocess.run([args.ffmpeg,'-hide_banner','-loglevel','warning','-i',str(working),
                '-c','copy','-movflags','+faststart','-y',str(args.output)],stderr=log)
            metadata['export_exit_code']=result.returncode
            if result.returncode==0:working.unlink()
        args.output.with_suffix('.json').write_text(json.dumps(metadata,indent=2))


if __name__=='__main__':main()
