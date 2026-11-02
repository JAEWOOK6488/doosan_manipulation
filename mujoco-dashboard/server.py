"""Local dashboard for the instrumented Week9 simulator work copy."""
import io, json, math, os, sys
from pathlib import Path
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
ROOT = Path(__file__).resolve().parent
KIT = Path(os.environ.get('MUJOCO_KIT', '/home/jwj/Documents/ChatGPT/Lecture_ADV/assignment-lab'))
sys.path.insert(0, str(KIT))
import numpy as np
import mujoco
import matplotlib.image as mpimg
import servo_controller, torque_controller, collision_detector, hover_servo, hybrid_detector
from sim.runner import run_episode
STATE = {}
def clean(x):
    if isinstance(x, dict): return {k: clean(v) for k,v in x.items()}
    if isinstance(x, (list, tuple)): return [clean(v) for v in x]
    if isinstance(x, np.ndarray): return clean(x.tolist())
    if isinstance(x, np.generic): return clean(x.item())
    if isinstance(x, float) and not math.isfinite(x): return None
    return x

def simulate(p):
    mass = float(p.get('mass',8)); force = float(p.get('force',40)); when = float(p.get('time',3))
    com_mm = np.array([
        float(p.get('comX', 20)),
        float(p.get('comY', -10)),
        float(p.get('comZ', 30)),
    ])
    values = [mass, force, when, *com_mm]
    if not all(map(math.isfinite, values)) or not (
        0 <= mass <= 8 and 0 <= force <= 100 and 0 <= when <= 5.8
        and np.all(np.abs(com_mm) <= 100)
    ):
        raise ValueError('질량 0~8kg, COM 각 축 -100~100mm, 힘 0~100N, 충돌 시각 0~5.8s 범위로 입력하세요.')
    box_com = tuple(1e-3 * com_mm)
    mode = p.get('mode','improved')
    choices = {
        'improved': (hover_servo.Controller, hybrid_detector.Detector),
        'servo': (servo_controller.Controller, collision_detector.Detector),
        'torque': (torque_controller.Controller, collision_detector.Detector),
    }
    if mode not in choices: raise ValueError('지원하지 않는 제어기입니다.')
    axes = {
        '+X': np.array([1., 0., 0.]), '-X': np.array([-1., 0., 0.]),
        '+Y': np.array([0., 1., 0.]), '-Y': np.array([0., -1., 0.]),
        '+Z': np.array([0., 0., 1.]), '-Z': np.array([0., 0., -1.]),
    }
    axis = p.get('axis', '+Y')
    if axis not in axes: raise ValueError('지원하지 않는 충돌 방향입니다.')
    base, detector_class = choices[mode]
    class Recorder(base):
        def compute(self,t,th,thd,tau_s,grip,q_d,qd_d,qdd_d):
            return super().compute(t,th,thd,tau_s,grip,q_d,qd_d,qdd_d)
    collision = (when, force * axes[axis]) if p.get('collision') else None
    res,log = run_episode(
        Recorder(), box_mass=mass, box_com=box_com,
        detector=detector_class(), collision=collision,
    )
    # A transition alarm is only attributable to the injected collision when the
    # matching no-collision run is quiet around the same detection instant.
    res['distinguishable'] = None
    res['reference_alarm_near_detection'] = None
    if collision is not None and res.get('detect_delay_ms') is not None and not res.get('alarm_before_collision'):
        _, reference = run_episode(
            Recorder(), box_mass=mass, box_com=box_com,
            detector=detector_class(),
        )
        detected_at = when + res['detect_delay_ms'] / 1000.0
        k = int(round(detected_at / 0.001))
        lo, hi = max(0, k - 20), min(len(reference['detect']), k + 21)
        reference_alarm = bool(reference['detect'][lo:hi].any())
        res['reference_alarm_near_detection'] = reference_alarm
        res['distinguishable'] = not reference_alarm
    n=len(log['t'])
    if STATE.get('renderer'): STATE['renderer'].close()
    STATE.clear(); STATE.update(log=log,n=n,params=p,version=os.urandom(6).hex())
    data={k:log[k][:n] for k in ['t','q','q_d','tau','r','f_residual','detect','e_mm','f_table','f_ext']}
    data.update(th=log['th'][:n],thd=log['thd'][:n],tau_s=log['tau_s'][:n],tau_m=log['tau_m'][:n])
    STATE['data']=clean(dict(metrics=res,series=data,params=p,version=STATE['version']))
    return STATE['data']

class Handler(BaseHTTPRequestHandler):
    def send(self,body,mime='application/json',code=200):
        if not isinstance(body,bytes): body=json.dumps(clean(body),allow_nan=False).encode()
        self.send_response(code); self.send_header('Content-Type',mime); self.send_header('Content-Length',str(len(body))); self.send_header('Cache-Control','no-store'); self.end_headers(); self.wfile.write(body)
    def do_POST(self):
        if self.path!='/api/run': return self.send({'error':'Not found'},code=404)
        origin=self.headers.get('Origin')
        if origin and origin not in ('http://127.0.0.1:8765','http://localhost:8765'): return self.send({'error':'Origin rejected'},code=403)
        try:
            size=int(self.headers.get('Content-Length',0))
            if not 0<size<4096: raise ValueError('요청 크기 오류')
            self.send(simulate(json.loads(self.rfile.read(size))))
        except (ValueError,TypeError,KeyError) as e: self.send({'error':str(e)},code=400)
        except Exception as e: self.send({'error':str(e)},code=500)
    def do_GET(self):
        u=urlparse(self.path)
        try:
            if u.path=='/api/frame':
                if not STATE: return self.send({'error':'먼저 실행하세요'},code=409)
                k=max(0,min(STATE['n']-1,int(parse_qs(u.query).get('k',['0'])[0])))
                log=STATE['log']; m=log['plant'].m
                if not STATE.get('renderer'):
                    STATE['renderer']=mujoco.Renderer(m,480,640); STATE['d']=mujoco.MjData(m)
                d=STATE['d']; d.qpos[:]=log['qpos'][k]; mujoco.mj_forward(m,d)
                cam=mujoco.MjvCamera(); cam.lookat[:]=[.25,.1,.3]; cam.distance=2.8; cam.azimuth=150; cam.elevation=-20
                STATE['renderer'].update_scene(d,camera=cam)
                b=io.BytesIO(); mpimg.imsave(b,STATE['renderer'].render(),format='png'); return self.send(b.getvalue(),'image/png')
            if u.path=='/api/dynamics':
                if not STATE: return self.send({'error':'먼저 실행하세요'},code=409)
                k=max(0,min(STATE['n']-1,int(parse_qs(u.query).get('k',['0'])[0])))
                from sim.nominal import Nominal
                q=STATE['log']['q'][k]; params=STATE['params']
                empty=Nominal(); loaded=Nominal()
                box_com=1e-3*np.array([
                    float(params.get('comX',20)), float(params.get('comY',-10)),
                    float(params.get('comZ',30)),
                ])
                loaded.set_payload(float(params.get('mass',8)),box_com)
                return self.send({
                    'M_empty':empty.mass_matrix(q),'g_empty':empty.gravity(q),
                    'M_loaded':loaded.mass_matrix(q),'g_loaded':loaded.gravity(q),
                    'J_tcp':empty.jacobian(q),
                })
            if u.path=='/api/data': return self.send(STATE.get('data',{}))
            paths={'/':'index.html','/app.js':'app.js','/style.css':'style.css'}
            if u.path not in paths: return self.send({'error':'Not found'},code=404)
            f=ROOT/'static'/paths[u.path]
            mime={'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8','.css':'text/css; charset=utf-8'}[f.suffix]
            self.send(f.read_bytes(),mime)
        except Exception as e: self.send({'error':str(e)},code=500)

if __name__=='__main__':
    print('MuJoCo dashboard: http://127.0.0.1:8765',flush=True)
    HTTPServer(('127.0.0.1',8765),Handler).serve_forever()
