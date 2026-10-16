import unittest,tempfile,csv,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from analyze_rpm_log import analyze
class RpmLogTests(unittest.TestCase):
 def test_units_stale_and_unknown(self):
  with tempfile.TemporaryDirectory() as tmp:
   p=Path(tmp)/"flight.01.csv"
   names=["time (us)","flightModeFlags","fdTelemetryMask"]+[f"fdErpm[{i}]" for i in range(4)]+[f"fdErpmAge[{i}]" for i in range(4)]+[f"fdGyro[{i}]" for i in range(3)]+[f"fdPidSum[{i}]" for i in range(3)]+[f"gyroADC[{i}] (deg/s)" for i in range(3)]
   with p.open("w") as f:
    w=csv.writer(f);w.writerow(names)
    w.writerow([0,3,15]+[600]*4+[2]*4+[123,-234,0]+[0]*3+[1,-2,0])
    w.writerow([625,3,15]+[600]*4+[200]*4+[123,-234,0]+[0]*3+[1,-2,0])
   p.with_name("flight.01.headers.csv").write_text('motor_poles,"12"\n')
   r=analyze(p);self.assertEqual(r["rpm_max"],[10000]*4);self.assertEqual(r["fresh_armed_fraction"],[.5]*4)
   self.assertLess(max(r["gyro_fine_vs_standard_max_deg_s"]),.5)
 def test_old_log_not_silently_accepted(self):
  with tempfile.TemporaryDirectory() as tmp:
   p=Path(tmp)/"flight.01.csv";p.write_text("time (us),flightModeFlags\n0,3\n")
   self.assertFalse(analyze(p)["valid_schema"])

 def test_identification_uses_fine_units(self):
  import numpy as np
  from review_identification import axis_signals
  d={"fdGyro[0]":np.array([123.,-234.]),"fdPidSum[0]":np.array([125.,-250.]),"gyroADC[0] (deg/s)":np.array([1.,-2.]),"axisP[0]":np.array([1.,-2.])}
  w,u=axis_signals(d,0)
  np.testing.assert_allclose(w,np.array([1.23,-2.34])*np.pi/180)
  np.testing.assert_allclose(u,[1.25,-2.5])
