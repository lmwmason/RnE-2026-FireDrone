#!/usr/bin/env python3
"""Taranis/USB HID FPV front end. C99 controller + fixed-step C physics."""
import argparse
import csv
import ctypes as C
import json
import hashlib
import math
import os
from pathlib import Path
import time

ROOT = Path(__file__).resolve().parents[1]
CHANNELS = ('roll', 'pitch', 'yaw', 'throttle')
HAZARDS = ('WIND / GUST', 'PROP DAMAGE', 'MOTOR STOP', 'ACCEL FREEZE',
           'GYRO BIAS', 'ACCEL SHOCK', 'RC LOSS', 'GPS LOSS', 'IMU LOSS', 'ALL FAULTS')


class View(C.Structure):
    _fields_ = [('t', C.c_double), ('pos', C.c_double * 3), ('q', C.c_double * 4),
                ('vel', C.c_double * 3), ('cmd', C.c_double * 8), ('eff', C.c_double * 8),
                ('wind', C.c_double * 3), ('arm_angle', C.c_double),
                ('arm_target', C.c_double), ('arm_torque', C.c_double), ('impact', C.c_double),
                *[(key, C.c_int) for key in ('armed', 'crashed', 'failed_mask', 'events',
                                            'hazard', 'cycles', 'control_lost')]]


class Simulation:
    def __init__(self, config, seed):
        self.lib = C.CDLL(str(ROOT / 'build/libfiredrone_live.so'))
        self.lib.live_create.argtypes = [C.c_char_p, C.c_uint]
        self.lib.live_create.restype = C.c_void_p
        self.lib.live_destroy.argtypes = [C.c_void_p]
        self.lib.live_step.argtypes = [C.c_void_p, C.POINTER(C.c_uint16), C.c_int, C.c_int, C.c_uint]
        self.lib.live_view.argtypes = [C.c_void_p, C.POINTER(View)]
        self.handle = self.lib.live_create(os.fsencode(config), seed)
        if not self.handle:
            raise RuntimeError('Invalid X8 configuration')

    def step(self, rc, connected, hazards, ticks):
        if not self.handle: raise RuntimeError('Simulation is closed')
        if not isinstance(ticks,int) or not 0<=ticks<=1000:
            raise ValueError('ticks must be an integer in 0..1000')
        if len(rc)!=5 or any(not isinstance(v,int) or not 1000<=v<=2000 for v in rc):
            raise ValueError('five RC channels in 1000..2000 us required')
        self.lib.live_step(self.handle, (C.c_uint16 * 5)(*rc), connected, hazards, ticks)

    def view(self):
        view = View()
        self.lib.live_view(self.handle, C.byref(view))
        return view

    def close(self):
        if self.handle:
            self.lib.live_destroy(self.handle)
            self.handle = None


def switch_request(requested, ready):
    """After keyboard disarm/reconnect, a held-high switch cannot re-arm."""
    if not requested:
        return False, True
    return bool(ready), bool(ready)


def normalized(raw, mapping, throttle=False):
    """Piecewise calibration handles reversed axes and asymmetric center travel."""
    if not math.isfinite(raw):
        raise ValueError("Non-finite joystick sample")
    center, positive = mapping['center'], mapping['positive']
    direction = 1 if positive > center else -1
    delta = (raw - center) * direction
    if throttle:
        return max(0.0, min(1.0, delta / abs(positive - center)))
    negative = mapping['negative']
    travel = abs(positive - center) if delta >= 0 else abs(negative - center)
    value = max(-1.0, min(1.0, delta / travel))
    deadband = 0.03
    return 0.0 if abs(value) <= deadband else math.copysign((abs(value)-deadband)/(1-deadband), value)


def valid_profile(profile, axes):
    try:
        if set(profile['channels']) != set(CHANNELS):
            return False
        used = set()
        for name in CHANNELS:
            m = profile['channels'][name]
            index = m['axis']
            if not isinstance(index, int) or not 0 <= index < axes or index in used:
                return False
            used.add(index)
            if not all(math.isfinite(m[k]) and abs(m[k]) <= 1.1 for k in ('center', 'positive')):
                return False
            if abs(m['positive']-m['center']) < 0.25:
                return False
            if name != 'throttle':
                if not math.isfinite(m['negative']) or abs(m['negative']) > 1.1:
                    return False
                if (m['negative']-m['center'])*(m['positive']-m['center']) >= 0:
                    return False
                if abs(m['negative']-m['center']) < 0.25:
                    return False
        if 'arm_button' in profile and (not isinstance(profile['arm_button'],int) or profile['arm_button']<0):
            return False
        if 'arm_axis' in profile:
            m=profile['arm_axis']
            if not isinstance(m['axis'],int) or not 0<=m['axis']<axes or m['axis'] in used:
                return False
            if not all(math.isfinite(m[k]) and abs(m[k])<=1.1 for k in ('low','high')):
                return False
            if abs(m['high']-m['low'])<=0.8: return False
        return True
    except (KeyError, TypeError, ValueError):
        return False


class Calibration:
    prompts = ('Center roll/pitch/yaw, throttle LOW. ENTER',
               'ROLL fully RIGHT, other sticks unchanged. ENTER',
               'ROLL fully LEFT. ENTER',
               'PITCH fully FORWARD. ENTER', 'PITCH fully BACK. ENTER',
               'YAW fully RIGHT. ENTER', 'YAW fully LEFT. ENTER',
               'THROTTLE fully HIGH. ENTER')

    def __init__(self):
        self.stage = 0
        self.baseline = []
        self.channels = {}
        self.error = ''

    def capture(self, axes):
        self.error = ''
        if len(axes)<4 or not all(math.isfinite(v) and abs(v)<=1.1 for v in axes):
            self.error='Invalid joystick samples'; return False
        if self.baseline and len(axes)!=len(self.baseline):
            self.error='Device axes changed. Restart calibration.'; return False
        if self.stage == 0:
            self.baseline = axes[:]
        else:
            name = {1:'roll', 2:'roll', 3:'pitch', 4:'pitch', 5:'yaw', 6:'yaw', 7:'throttle'}[self.stage]
            if self.stage in (1, 3, 5, 7):
                used = {m['axis'] for m in self.channels.values()}
                candidates = [i for i in range(len(axes)) if i not in used]
                if not candidates:
                    self.error = 'Four independent axes required'; return False
                index = max(candidates, key=lambda i: abs(axes[i]-self.baseline[i]))
                if abs(axes[index]-self.baseline[index]) < 0.25:
                    self.error = 'Move the requested stick fully'; return False
                self.channels[name] = dict(axis=index, center=self.baseline[index], positive=axes[index])
            else:
                m = self.channels[name]
                value = axes[m['axis']]
                if abs(value-m['center']) < 0.25 or (value-m['center'])*(m['positive']-m['center']) >= 0:
                    self.error = 'Move to the opposite endpoint'; return False
                m['negative'] = value
        self.stage += 1
        return self.stage == len(self.prompts)


def rotation(q):
    import numpy as np
    w,x,y,z = q
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])


def render(screen, font, small, view, device_name, rc, status, calibrating, axes, paused, hazards, slowed=False):
    import numpy as np
    import pygame as pg
    width, height = screen.get_size()
    screen.fill((25, 46, 66))
    R = rotation(view.q)
    camera = np.array(view.pos) + R @ np.array([0.12, 0, 0.18])
    focal = width * 0.55

    def project(point):
        body = R.T @ (np.array(point)-camera)
        if body[0] < 0.15:
            return None
        return (int(width/2 - focal*body[1]/body[0]), int(height/2-focal*body[2]/body[0]))

    def line(a,b,color,thickness=1):
        # Clip against the near plane in camera space before projection.
        aa=R.T@(np.asarray(a)-camera); bb=R.T@(np.asarray(b)-camera)
        if aa[0]<0.15 and bb[0]<0.15:
            return
        if aa[0]<0.15:
            aa=aa+(bb-aa)*((0.15-aa[0])/(bb[0]-aa[0]))
        if bb[0]<0.15:
            bb=bb+(aa-bb)*((0.15-bb[0])/(aa[0]-bb[0]))
        pts=[(int(width/2-focal*v[1]/v[0]),int(height/2-focal*v[2]/v[0])) for v in (aa,bb)]
        pg.draw.line(screen,color,*pts,thickness)

    polygon=[(0.,0.),(float(width),0.),(float(width),float(height)),(0.,float(height))]
    def ground_side(point):
        x,y=point
        return R[2,0]+R[2,1]*(width/2-x)/focal+R[2,2]*(height/2-y)/focal
    clipped=[]
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        va,vb=ground_side(a),ground_side(b)
        if va<=0: clipped.append(a)
        if (va<=0)!=(vb<=0):
            fraction=va/(va-vb)
            clipped.append((a[0]+fraction*(b[0]-a[0]),a[1]+fraction*(b[1]-a[1])))
    if len(clipped)>=3: pg.draw.polygon(screen,(26,49,44),clipped)
    # Ground grid and flight gates are navigation markers, not collision objects.
    cx,cy=(int(v//5)*5 for v in view.pos[:2])
    for n in range(-12,13):
        line((cx+n*5,cy-60,0),(cx+n*5,cy+60,0),(52,85,88))
        line((cx-60,cy+n*5,0),(cx+60,cy+n*5,0),(52,85,88))
    for x,y in ((10,0),(20,10),(10,20),(0,10)):
        corners=[(x,y-2,0),(x,y-2,5),(x,y+2,5),(x,y+2,0)]
        for a,b in zip(corners,corners[1:]): line(a,b,(72,210,187),3)
    origin=project((0,0,0))
    if origin and -100<origin[0]<width+100 and -100<origin[1]<height+100:
        pg.draw.circle(screen,(245,193,87),origin,8,2)
    pg.draw.line(screen,(235,242,248),(width//2-10,height//2),(width//2+10,height//2),1)
    pg.draw.line(screen,(235,242,248),(width//2,height//2-10),(width//2,height//2+10),1)

    def text(value,x,y,color=(220,234,241),f=small):
        screen.blit(f.render(value,True,color),(x,y))
    pg.draw.rect(screen,(10,20,31),(0,0,width,108))
    text('FIREDRONE  /  X8  /  ANGLE',24,16,(83,226,190),font)
    text('USB pilot input  |  UKF attitude  |  tilt limit 35 deg  |  no waypoint autopilot',24,48)
    text('UNCALIBRATED 10-inch design study  /  single-link arm approximation',24,74,(242,191,104))
    text(f'{view.t:7.1f}s   ALT {view.pos[2]:6.2f} m   SPEED {math.sqrt(sum(v*v for v in view.vel)):5.1f} m/s',24,125,f=font)
    text(('ARMED' if view.armed else 'DISARMED') + ('  |  PAUSED' if paused else ''),24,157,(255,196,101))
    text(device_name,24,186)
    text(status,24,213,(255,195,101))
    text(f'Random faults: {"ON" if hazards else "OFF"}  |  events {view.events}  |  cycles {view.cycles}',24,242)
    if view.hazard>=0: text('Last: '+HAZARDS[view.hazard],24,268,(253,127,127))
    if slowed: text('Rendering slow - physics timestep retained',24,294,(242,191,104))
    if view.crashed or view.control_lost:
        text('CRASH / CONTROL LOST  -  R: new airframe',width//2-220,height//2+40,(255,118,118),font)
    # Instrument inset exposes the ACTUAL simulated arm joint state.
    base=(width-145,180)
    tip=(int(base[0]+80*math.sin(view.arm_angle)),int(base[1]+80*math.cos(view.arm_angle)))
    pg.draw.line(screen,(235,192,91),base,tip,6); pg.draw.circle(screen,(83,226,190),tip,8)
    text('ARM SIDE VIEW',width-230,126)
    text(f'{math.degrees(view.arm_angle):+.1f} deg / {view.arm_torque:+.3f} Nm',width-230,282)
    for j in range(8):
        x=24+j*60; y=height-128
        color=(244,100,110) if view.eff[j]<0.01 else (242,187,91) if view.eff[j]<0.99 else (74,214,183)
        pg.draw.rect(screen,(30,48,60),(x,y,35,55))
        pg.draw.rect(screen,color,(x,y+55-int(view.cmd[j]*55),35,int(view.cmd[j]*55)))
        text(f'M{j+1}',x,y+59); text(f'{view.eff[j]*100:.0f}%',x,y-22,color)
    text(f'R {rc[0]}  P {rc[1]}  T {rc[2]}  Y {rc[3]}',width-390,height-88)
    text('V view  |  C calibrate  |  SPACE arm  |  B bind arm switch  |  F5 random faults  |  P pause  |  R reset  |  ESC exit',24,height-32)
    if calibrating:
        box=pg.Rect(80,height//2-75,width-160,160)
        pg.draw.rect(screen,(9,18,29),box,border_radius=10)
        text(f'CALIBRATION {calibrating.stage+1}/8',100,box.y+15,(83,226,190),font)
        text(calibrating.prompts[calibrating.stage],100,box.y+55)
        text(calibrating.error or 'Return other sticks to center; throttle low unless requested.',100,box.y+88,(242,191,104))
        text('Raw: '+' '.join(f'{i}:{a:+.2f}' for i,a in enumerate(axes)),100,box.y+118)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config',type=Path,default=ROOT/'configs/x8_design_study.conf')
    ap.add_argument('--seed',type=int,default=42)
    ap.add_argument('--view',choices=('dashboard','fpv'),default='dashboard')
    ap.add_argument('--frames',type=int,default=0,help='automated UI smoke test limit')
    ap.add_argument('--screenshot',type=Path)
    ap.add_argument('--list-devices',action='store_true')
    args=ap.parse_args()
    if not 0<=args.seed<=0xffffffff: ap.error('seed must be in 0..4294967295')
    if args.frames<0: ap.error('frames must be nonnegative')
    import pygame as pg
    pg.display.init(); pg.font.init(); pg.joystick.init()
    if args.list_devices:
        for i in range(pg.joystick.get_count()):
            j=pg.joystick.Joystick(i)
            print(f'{i}: {j.get_name()} | axes={j.get_numaxes()} buttons={j.get_numbuttons()} GUID={j.get_guid()}')
        return
    screen=pg.display.set_mode((1280,800),pg.RESIZABLE)
    pg.display.set_caption('FireDrone X8 - Taranis X9D Plus - ANGLE')
    font=pg.font.SysFont('DejaVu Sans',21); small=pg.font.SysFont('DejaVu Sans',15)
    from live_dashboard import Dashboard
    dashboard=Dashboard(args.config); view_mode=args.view
    clock=pg.time.Clock(); joystick=None; profile=None; calibrating=None
    arm=False; hazards=True; paused=False; binding=False; bind_axes=[]; switch_ready=False
    status='Connect X9D Plus in USB joystick mode, then press C.'
    folder=ROOT/'out'/'joystick'; folder.mkdir(parents=True,exist_ok=True)
    sim=Simulation(args.config,args.seed); frame=0; flight_no=0; accumulator=0.0
    logpath=ROOT/'out'/f'fpv_{time.time_ns()}.csv'
    log=logpath.open('w',newline=''); writer=csv.writer(log)
    writer.writerow(['connected','faults_enabled','paused','control_lost','motor_fault_mask','flight','seed','t','x','y','z','qw','qx','qy','qz','roll_us','pitch_us','throttle_us','yaw_us','arm_us',
                     'armed','crashed','events','last_hazard','arm_angle','arm_torque',
                     *[f'u{i+1}' for i in range(8)],*[f'eff{i+1}' for i in range(8)]])
    metadata=dict(seed=args.seed,config=args.config.read_text(),
        model='UNCALIBRATED_X8',control='MANUAL_ANGLE',physics_dt=0.0005,control_dt=0.001,
        binary_sha256=hashlib.sha256((ROOT/'build/libfiredrone_live.so').read_bytes()).hexdigest(),
        profiles=[],flights=[dict(flight=0,seed=args.seed,config=args.config.read_text())])

    def save_metadata():
        logpath.with_suffix('.json').write_text(json.dumps(metadata,indent=2))

    def record_profile():
        metadata['profiles'].append(dict(flight=flight_no,t=sim.view().t,
                                        profile=json.loads(json.dumps(profile))))
        save_metadata()

    save_metadata()

    def save_profile():
        (folder/f'{joystick.get_guid()}.json').write_text(json.dumps(profile,indent=2))
        record_profile()

    def connect():
        nonlocal joystick,profile,status,arm,switch_ready
        if joystick is not None: return
        for i in range(pg.joystick.get_count()):
            candidate=pg.joystick.Joystick(i)
            # Linux virtual mice can appear as js0; never use a two-axis mouse.
            if candidate.get_numaxes()<4: continue
            joystick=candidate; arm=False; profile=None; switch_ready=False
            path=folder/f'{joystick.get_guid()}.json'
            try:
                data=json.loads(path.read_text())
                if valid_profile(data,joystick.get_numaxes()):
                    profile=data; record_profile()
            except (OSError,ValueError): pass
            status='Calibration loaded. Throttle low, SPACE to arm.' if profile else 'USB controller detected. Press C to calibrate.'
            break

    running=True
    slow_until=0.0
    clock.tick()  # exclude library/config startup from the first simulation frame
    try:
        while running:
            elapsed=clock.tick(60)/1000
            connect()
            for event in pg.event.get():
                if event.type==pg.QUIT: running=False
                elif event.type==pg.JOYDEVICEREMOVED and joystick and event.instance_id==joystick.get_instance_id():
                    joystick=None; profile=None; arm=False; calibrating=None; binding=False; bind_axes=[]; switch_ready=False
                    status='USB DISCONNECTED - controller failsafe; reconnect and re-arm explicitly.'
                elif event.type==pg.WINDOWFOCUSLOST:
                    arm=False; switch_ready=False; paused=True; status='Focus lost: PAUSED, arm request cleared. P to resume.'
                elif event.type==pg.JOYBUTTONDOWN and joystick and event.instance_id==joystick.get_instance_id() and binding:
                    if profile:
                        profile.pop('arm_axis',None); profile['arm_button']=event.button; save_profile(); status=f'Arm switch bound to button {event.button}'
                    binding=False; arm=False; switch_ready=False
                elif event.type==pg.KEYDOWN:
                    if event.key==pg.K_ESCAPE: running=False
                    elif event.key==pg.K_c and joystick:
                        calibrating=Calibration(); arm=False; binding=False; switch_ready=False; status='Calibrating - simulation paused.'
                    elif event.key==pg.K_RETURN and calibrating and joystick:
                        axes=[joystick.get_axis(i) for i in range(joystick.get_numaxes())]
                        if calibrating.capture(axes):
                            profile=dict(name=joystick.get_name(),guid=joystick.get_guid(),channels=calibrating.channels)
                            save_profile(); calibrating=None; arm=False
                            status='Saved. Throttle LOW; SPACE to arm. Sticks centered = level.'
                    elif event.key==pg.K_SPACE and profile and not calibrating:
                        if 'arm_button' in profile or 'arm_axis' in profile:
                            arm=False; switch_ready=False
                            status='Keyboard DISARM. Cycle the radio switch OFF -> ON to arm again.'
                        else:
                            arm=not arm
                    elif event.key==pg.K_b and profile:
                        binding=True; arm=False; bind_axes=[joystick.get_axis(i) for i in range(joystick.get_numaxes())]; status='Move an unused arm switch LOW -> HIGH, or press an arm button.'
                    elif event.key==pg.K_v: view_mode='fpv' if view_mode=='dashboard' else 'dashboard'
                    elif event.key==pg.K_F5: hazards=not hazards
                    elif event.key==pg.K_p:
                        paused=not paused; arm=False; switch_ready=False
                    elif event.key==pg.K_r:
                        new_seed=(args.seed+1)&0xffffffff
                        replacement=Simulation(args.config,new_seed)
                        sim.close(); sim=replacement; args.seed=new_seed
                        dashboard=Dashboard(args.config)
                        switch_ready=False; arm=False; accumulator=0; status=f'New airframe, seed {args.seed}. Throttle LOW before arming.'
                        flight_no+=1; log.flush()
                        metadata['flights'].append(dict(flight=flight_no,seed=args.seed,config=args.config.read_text()))
                        save_metadata()
            rc=[1500,1500,1000,1500,1000]; axes=[]
            connected=joystick is not None and profile is not None and not calibrating
            if joystick:
                try:
                    axes=[joystick.get_axis(i) for i in range(joystick.get_numaxes())]
                    if binding and profile and bind_axes:
                        used={m['axis'] for m in profile['channels'].values()}
                        changed=[i for i in range(len(axes)) if i not in used and abs(axes[i]-bind_axes[i])>0.8]
                        if changed:
                            i=max(changed,key=lambda j:abs(axes[j]-bind_axes[j]))
                            profile.pop('arm_button',None)
                            profile['arm_axis']=dict(axis=i,low=bind_axes[i],high=axes[i])
                            save_profile(); binding=False; arm=False; switch_ready=False
                            status=f'Arm switch assigned to axis {i}. Switch OFF before arming.'
                    if connected:
                        v={name:normalized(axes[m['axis']],m,name=='throttle') for name,m in profile['channels'].items()}
                        # UI yaw right is negative body-FLU yaw rate.
                        rc=[int(1500+500*v['roll']),int(1500+500*v['pitch']),
                            int(1000+1000*v['throttle']),int(1500-500*v['yaw']),1000]
                        if 'arm_button' in profile and not binding:
                            button=profile['arm_button']
                            if 0<=button<joystick.get_numbuttons():
                                requested=bool(joystick.get_button(button))
                                arm,switch_ready=switch_request(requested,switch_ready)
                        if 'arm_axis' in profile and not binding:
                            m=profile['arm_axis']
                            if 0<=m['axis']<len(axes) and abs(m['high']-m['low'])>0.8:
                                requested=(axes[m['axis']]-m['low'])/(m['high']-m['low'])>0.75
                                arm,switch_ready=switch_request(requested,switch_ready)
                        rc[4]=2000 if arm and not binding else 1000
                except (pg.error, ValueError):
                    joystick=None; profile=None; connected=False; arm=False
            if not paused and not calibrating:
                accumulator+=min(elapsed,0.1)
                ticks=min(100,int(accumulator*1000))
                accumulator-=ticks/1000
                if elapsed>0.1: slow_until=time.monotonic()+2
                sim.step(rc,int(connected),int(hazards),ticks)
            else: accumulator=0
            view=sim.view()
            if arm and not view.armed and not view.crashed and not view.control_lost:
                status='Arming requires LOW throttle and a fresh OFF -> ON arm action.'
            dashboard.sample(view)
            draw=dashboard.render if view_mode=='dashboard' else render
            draw(screen,font,small,view,joystick.get_name() if joystick else 'No four-axis USB controller detected',
                   rc,status,calibrating,axes,paused,hazards,time.monotonic()<slow_until)
            writer.writerow([int(bool(connected)),int(hazards),int(bool(paused or calibrating)),view.control_lost,view.failed_mask,flight_no,args.seed,view.t,*view.pos,*view.q,*rc,view.armed,view.crashed,view.events,view.hazard,
                             view.arm_angle,view.arm_torque,*view.cmd,*view.eff])
            if frame%60==0: log.flush()
            pg.display.flip(); frame+=1
            if args.frames and frame>=args.frames: running=False
        if args.screenshot: pg.image.save(screen,str(args.screenshot))
    finally:
        sim.close(); log.close(); pg.quit()
    print(f'Flight log: {logpath}')


if __name__=='__main__':
    main()
