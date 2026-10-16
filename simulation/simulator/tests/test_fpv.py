import math
from pathlib import Path
import sys
import unittest
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from fpv import Calibration, Simulation, normalized, valid_profile, switch_request


class RadioTests(unittest.TestCase):
    def test_asymmetric_and_reversed_axes(self):
        m=dict(axis=2, center=0.1, positive=-0.8, negative=0.95)
        self.assertEqual(normalized(0.1,m),0)
        self.assertEqual(normalized(-0.8,m),1)
        self.assertEqual(normalized(0.95,m),-1)
        self.assertEqual(normalized(1,dict(center=1,positive=-1),True),0)
        self.assertEqual(normalized(-1,dict(center=1,positive=-1),True),1)

    def test_calibration_discovers_channel_order(self):
        c=Calibration()
        # Taranis model outputs may be reordered and reversed; do not assume AETR.
        samples=[[0,1,0,0], [0,1,0,-1], [0,1,0,1],
                 [1,1,0,0],[-1,1,0,0], [0,1,1,0], [0,1,-1,0], [0,-1,0,0]]
        for i,sample in enumerate(samples): self.assertEqual(c.capture(sample),i==7)
        profile=dict(channels=c.channels)
        self.assertTrue(valid_profile(profile,4))
        self.assertEqual(c.channels['roll']['axis'],3)
        self.assertEqual(c.channels['throttle']['axis'],1)
        profile['channels']['pitch']['axis']=3
        self.assertFalse(valid_profile(profile,4))

    def test_bad_samples_and_switch_disarm(self):
        c=Calibration()
        self.assertFalse(c.capture([0,0,float('nan'),-1]))
        self.assertEqual(c.stage,0)
        with self.assertRaises(ValueError):
            normalized(float('nan'),dict(center=0,positive=1,negative=-1))
        self.assertEqual(switch_request(True,False),(False,False))
        self.assertEqual(switch_request(False,False),(False,True))
        self.assertEqual(switch_request(True,True),(True,True))

    def test_missing_travel_does_not_calibrate(self):
        c=Calibration(); c.capture([0,0,0,-1])
        self.assertFalse(c.capture([0,0,0,-1]))
        self.assertEqual(c.stage,1)


class LiveTests(unittest.TestCase):
    def setUp(self):
        self.sim=Simulation(ROOT/'configs/x8_design_study.conf',42)

    def tearDown(self): self.sim.close()

    def arm(self):
        self.sim.step([1500,1500,1000,1500,1000],1,0,100)
        self.sim.step([1500,1500,1000,1500,2000],1,0,100)
        self.assertTrue(self.sim.view().armed)

    def test_invalid_configuration_and_bridge_inputs(self):
        with self.assertRaises(ValueError):
            self.sim.step([1500,1500,1000,1500,1000],1,0,-1)
        with self.assertRaises(ValueError):
            self.sim.step([1500,1500,0,1500,1000],1,0,1)
        original=(ROOT/'configs/x8_design_study.conf').read_text()
        for bad in ('arm_mass = 3', 'ixx = 0.001', 'thrust = nan'):
            key=bad.split(' = ')[0]
            lines=[bad if line.startswith(key+' = ') else line for line in original.splitlines()]
            with tempfile.TemporaryDirectory() as folder:
                path=Path(folder)/'invalid.conf'; path.write_text('\n'.join(lines))
                with self.assertRaises(RuntimeError): Simulation(path,42)

    def test_no_autonomous_takeoff_or_hotplug_arming(self):
        self.sim.step([1500,1500,1000,1500,2000],1,0,1000)
        self.assertFalse(self.sim.view().armed)
        self.sim.step([1500,1500,1000,1500,1000],1,0,10)
        self.sim.step([1500,1500,1500,1500,2000],1,0,1000)
        self.assertFalse(self.sim.view().armed)
        self.sim.step([1500,1500,1000,1500,2000],1,0,1000)
        self.assertFalse(self.sim.view().armed) # lowering throttle is not an arm edge
        self.assertEqual(self.sim.view().pos[2],0)

    def test_angle_sticks_and_level_recovery(self):
        self.arm()
        for _ in range(10): self.sim.step([1500,1500,1300,1500,2000],1,0,500)
        for _ in range(3): self.sim.step([1700,1500,1300,1500,2000],1,0,500)
        q=self.sim.view().q
        tilted=math.degrees(math.acos(max(-1,min(1,1-2*(q[1]**2+q[2]**2)))))
        self.assertGreater(tilted,8)
        for _ in range(7): self.sim.step([1500,1500,1300,1500,2000],1,0,500)
        v=self.sim.view(); q=v.q
        level=math.degrees(math.acos(max(-1,min(1,1-2*(q[1]**2+q[2]**2)))))
        self.assertLess(level,5)
        self.assertFalse(v.crashed)
        self.assertGreater(abs(v.arm_angle),0.01)

    def test_disarm_and_deterministic_disturbances(self):
        self.arm()
        other=Simulation(ROOT/'configs/x8_design_study.conf',42)
        try:
            other.step([1500,1500,1000,1500,1000],1,0,100)
            other.step([1500,1500,1000,1500,2000],1,0,100)
            for _ in range(20):
                rc=[1500,1500,1300,1500,2000]
                self.sim.step(rc,1,1,1000); other.step(rc,1,1,1000)
                a,b=self.sim.view(),other.view()
                self.assertEqual(list(a.pos),list(b.pos))
                self.assertEqual(list(a.eff),list(b.eff))
                self.assertEqual(a.events,b.events)
            self.assertGreater(self.sim.view().events,0)
            self.sim.step([1500,1500,1000,1500,1000],1,0,1)
            self.assertFalse(self.sim.view().armed)
            self.assertEqual(list(self.sim.view().cmd),[0]*8)
        finally: other.close()


if __name__=='__main__': unittest.main()
