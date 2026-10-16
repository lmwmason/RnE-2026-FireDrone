"""Read-only live telemetry dashboard; no control or physics state mutations."""
from collections import deque
import math
import numpy as np
import pygame as pg

BG = (11, 19, 32)
GRID = (43, 55, 72)
TEXT = (211, 222, 235)
BLUE = (63, 180, 229)
TEAL = (51, 209, 183)
GOLD = (246, 189, 88)
RED = (246, 115, 129)


class Dashboard:
    def __init__(self, config):
        values = {}
        for line in config.read_text().splitlines():
            line = line.split("#", 1)[0]
            if "=" in line:
                k, v = line.split("=", 1)
                values[k.strip()] = float(v)
        self.arm = values["arm"]
        self.link = values["arm_length"]
        self.history = deque(maxlen=1801)
        self.events = deque(maxlen=200)
        self.last_events = 0

    def sample(self, v):
        if self.history and v.t < self.history[-1][0]:
            self.history.clear(); self.events.clear(); self.last_events = 0
        if v.events != self.last_events:
            self.events.append((v.t, v.hazard))
            self.last_events = v.events
        if self.history and v.t-self.history[-1][0] < .033:
            return
        w, x, y, z = v.q
        roll = math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)))
        pitch = math.degrees(math.asin(max(-1, min(1, 2*(w*y-z*x)))))
        self.history.append((v.t, v.pos[2], roll, pitch, math.degrees(v.arm_angle), tuple(v.pos)))

    def render(self, screen, font, small, v, device, rc, status, calibration, axes, paused, hazards, slowed=False):
        screen.fill(BG)
        width, height = screen.get_size()
        # Fixed logical canvas keeps every control legible on smaller windows.
        canvas = pg.Surface((1440, 900))
        canvas.fill(BG)
        def text(s, x, y, color=TEXT, large=False):
            canvas.blit((font if large else small).render(s, True, color), (x, y))
        text("FIREDRONE SITL  /  X8 COAXIAL  /  MANUAL ANGLE", 30, 20, TEAL, True)
        text("UNCALIBRATED design study  |  10-inch prop plan  |  single-link arm approximation", 30, 53, GOLD)
        text(f"t {v.t:7.2f} s    POS {v.pos[0]:+.2f}  {v.pos[1]:+.2f}  {v.pos[2]:+.2f} m", 35, 103)
        text(f"SPEED {math.sqrt(sum(a*a for a in v.vel)):.2f} m/s    " + ("ARMED" if v.armed else "DISARMED") + ("  /  PAUSED" if paused else ""), 35, 129, GOLD)
        text(f"Random faults {'ON' if hazards else 'OFF'}    events {v.events}    cycles {v.cycles}    failed mask 0x{v.failed_mask:02X}", 35, 155)
        text(device, 35, 183)
        names = ("Wind", "Prop damage", "Motor stop", "Accel freeze", "Gyro bias", "Accel shock", "RC loss", "GPS loss", "IMU outage", "ALL")
        if 0 <= v.hazard < len(names):
            text("Last fault: " + names[v.hazard], 470, 183, RED)
        text(status, 35, 805, GOLD)
        text(f"RC  roll {rc[0]}   pitch {rc[1]}   throttle {rc[2]}   yaw {rc[3]}", 35, 832)
        text("V view  |  C calibrate  |  SPACE arm  |  B bind switch  |  F5 faults  |  P pause  |  R reset  |  ESC exit", 30, 872)
        if slowed: text("Render delay: fixed physics timestep retained", 35, 776, GOLD)
        if v.crashed or v.control_lost:
            text("CRASH / CONTROL LOST  -  R: new airframe", 35, 745, RED, True)

        # Orthographic external camera following the actual position. Geometry is
        # expressed in metres; rotor radius and coaxial separation are visual only.
        w,x,y,z = v.q
        R = np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                      [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                      [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
        center = np.array(v.pos)
        right = np.array([.80, -.60, 0.])
        up = np.array([.30, .40, .8660254])
        scale = 420
        def project(p):
            d = np.asarray(p)-center
            return (round(420+scale*np.dot(d, right)), round(470-scale*np.dot(d, up)))
        def line(a, b, color=GRID, thickness=1):
            pg.draw.line(canvas, color, project(a), project(b), thickness)
        def body(p): return center + R @ np.array(p)
        scene = pg.Rect(20, 217, 785, 473)
        canvas.set_clip(scene)
        floor = max(0., math.floor(center[2]*2)/2-.5)
        x0, y0 = np.floor(center[:2]*2)/2
        for k in range(-3, 4):
            line((x0+k*.5,y0-1.5,floor),(x0+k*.5,y0+1.5,floor))
            line((x0-1.5,y0+k*.5,floor),(x0+1.5,y0+k*.5,floor))
        if len(self.history) > 1:
            trail = np.asarray([s[5] for s in self.history])-center
            points = np.column_stack((420+scale*(trail@right),470-scale*(trail@up)))
            pg.draw.lines(canvas,(39,107,148),False,points,2)
        origin = np.array([x0-1, y0-1, floor])
        for delta, name, color in [((1,0,0), "East [m]", BLUE), ((0,1,0), "North [m]", TEAL), ((0,0,1), "Up [m]", GOLD)]:
            end = origin+delta
            line(origin, end, color, 2)
            px, py = project(end)
            text(name, px+6, py, color)
        a = self.arm/math.sqrt(2)
        mounts = [(-a,-a), (a,-a), (-a,a), (a,a)]
        # Sort discs by camera depth so upper/lower and distant arms overlap naturally.
        eye = np.cross(right, up)
        rotors = [(i, (x,y,.045 if i<4 else -.045)) for i, (x,y) in enumerate(mounts*2)]
        for x,y in mounts: line(body((0,0,0)), body((x,y,0)), (204,215,228), 5)
        for i, point in sorted(rotors, key=lambda item: np.dot(R@np.array(item[1]),eye)):
            color = RED if v.eff[i]<.01 else GOLD if v.eff[i]<.99 else (TEAL if i<4 else BLUE)
            x,y,z = point
            ring = [project(body((x+.127*math.cos(t),y+.127*math.sin(t),z))) for t in np.linspace(0,2*math.pi,33)]
            pg.draw.lines(canvas,color,True,ring,2)
            phase = v.t*22*v.cmd[i]+i
            line(body((x-.105*math.cos(phase),y-.105*math.sin(phase),z)),body((x+.105*math.cos(phase),y+.105*math.sin(phase),z)),color,2)
            px,py=project(body(point)); text(f"M{i+1}",px+8,py-17,color)
        pg.draw.circle(canvas,TEXT,project(center),8)
        line(center,body((.19,0,0)),RED,4)
        tip = body((self.link*math.sin(v.arm_angle),0,-self.link*math.cos(v.arm_angle)))
        line(center,tip,GOLD,7)
        pg.draw.circle(canvas,GOLD,project(tip),6)
        canvas.set_clip(None)
        text(f"ARM {math.degrees(v.arm_angle):+.1f} deg   /   reaction {v.arm_torque:+.3f} Nm",35,704,GOLD)
        text(f"Following camera / grid 0.5 m at z={floor:.1f} m (not ground)",35,731)
        text("Prop radius / vertical motor spacing: display assumptions",35,753)

        history = list(self.history)
        end = max(30.,v.t); start = end-30
        def chart(y, title, series, lo, hi):
            rect = pg.Rect(870,y+29,530,158)
            text(title,840,y, TEXT, True)
            pg.draw.rect(canvas,(15,25,42),rect)
            for t,h in self.events:
                if start<=t<=end:
                    px=rect.x+int((t-start)/30*rect.w)
                    pg.draw.line(canvas,(92,67,48),(px,rect.top),(px,rect.bottom),2)
            for i in range(5):
                yy=rect.bottom-i*rect.h/4
                pg.draw.line(canvas,GRID,(rect.x,yy),(rect.right,yy))
                text(f"{lo+(hi-lo)*i/4:.1f}",817,int(yy)-9)
            for i in range(7):
                xx=rect.x+i*rect.w/6
                pg.draw.line(canvas,GRID,(xx,rect.y),(xx,rect.bottom))
                text(f"{start+5*i:.0f}s",int(xx)-10,rect.bottom+5)
            canvas.set_clip(rect)
            for index,label,color in series:
                points=[(rect.x+(s[0]-start)/30*rect.w,rect.bottom-(s[index]-lo)/(hi-lo)*rect.h) for s in history if s[0]>=start]
                if len(points)>1: pg.draw.lines(canvas,color,False,points,2)
            canvas.set_clip(None)
            for j,(_,label,color) in enumerate(series): text(label,1050+j*115,y+3,color)
        visible=[s for s in history if s[0]>=start]
        heights=[s[1] for s in visible] or [0]
        chart(101,"Altitude [m]",[(1,"",BLUE)],min(0,min(heights)-.2),max(1,max(heights)+.2))
        limit=max(45, math.ceil(max((abs(s[i]) for s in visible for i in (2,3,4)),default=0)/15)*15)
        chart(339,"Angle [deg]",[(2,"Roll",RED),(3,"Pitch",TEAL),(4,"Arm",GOLD)],-limit,limit)
        text("Motor command / PWM equivalent",840,580,TEXT,True)
        text("Upper: M1-4  /  Lower: M5-8  |  labels = remaining efficiency",840,609)
        for i in range(8):
            x=865+i*66; y=655; h=109
            color=RED if v.eff[i]<.01 else GOLD if v.eff[i]<.99 else (TEAL if i<4 else BLUE)
            pg.draw.rect(canvas,(27,40,57),(x,y,44,h))
            fill=round(max(0,min(1,v.cmd[i]))*h)
            pg.draw.rect(canvas,color,(x,y+h-fill,44,fill))
            text(f"{1000+1000*v.cmd[i]:.0f}",x-2,y+h+6)
            text(f"M{i+1}",x+6,y+h+28,color)
            text(f"{v.eff[i]*100:.0f}%",x+2,y-23,color)
        text("Amber lines: fault injection times / trailing 30 seconds",840,827,GOLD)
        if calibration:
            box=pg.Rect(110,365,1220,175)
            pg.draw.rect(canvas,(8,15,26),box,border_radius=10)
            pg.draw.rect(canvas,TEAL,box,2,border_radius=10)
            text(f"CALIBRATION {calibration.stage+1}/8",135,380,TEAL,True)
            text(calibration.prompts[calibration.stage],135,424)
            text(calibration.error or "Return other sticks to center; throttle low unless requested.",135,455,GOLD)
            text("Raw: "+" ".join(f"{i}:{a:+.2f}" for i,a in enumerate(axes)),135,490)
        ratio=min(width/1440,height/900)
        size=(max(1,round(1440*ratio)),max(1,round(900*ratio)))
        screen.blit(pg.transform.smoothscale(canvas,size),((width-size[0])//2,(height-size[1])//2))
