import sys,unittest,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from identify import fit_axis,read_csv
import tempfile
class IdentifyTests(unittest.TestCase):
    def test_known_lagged_plant(self):
        dt=.001;w=0;lag=0;rows=[]
        for i in range(10000):
            t=i*dt;u=20*math.sin(2*math.pi*.7*t)+12*math.sin(2*math.pi*2.3*t)
            rows.append((t,w,u));lag+=dt/.026*(u-lag);w+=dt*(2*lag-1.5*w+.2)
        fit=fit_axis(rows,25)
        self.assertTrue(fit["usable_for_review"])
        self.assertAlmostEqual(fit["effectiveness_rad_s2_per_pid_unit"],2,delta=.1)
    def test_reject_quiet_log(self):
        with self.assertRaises(ValueError):fit_axis([(i*.001,0,0) for i in range(3000)],25)
    def test_reject_gap(self):
        rows=[(i*.001,math.sin(i*.05),20*math.sin(i*.1)) for i in range(3000)]
        rows[1500]=(5,rows[1500][1],rows[1500][2])
        with self.assertRaises(ValueError):fit_axis(rows,25)

    def test_actual_decoder_column_names(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/"decoded.csv"
            p.write_text("time (us), gyroADC[0] (deg/s), gyroADC[1] (deg/s), gyroADC[2] (deg/s), axisP[0],axisI[0],axisD[0],axisF[0],axisP[1],axisI[1],axisD[1],axisF[1],axisP[2],axisI[2],axisF[2]\n1000000,180,0,-180,1,2,3,4,1,2,3,4,1,2,4\n")
            axes=read_csv(p,.9,1.1)
            self.assertAlmostEqual(axes[0][0][1],math.pi)
            self.assertEqual(axes[0][0][2],10)
            self.assertAlmostEqual(axes[2][0][1],-math.pi)
            self.assertEqual(axes[2][0][2],7)
