import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from identify_instrumental import fit_fixed
from review_rpm_model import low
class InstrumentalTests(unittest.TestCase):
 def test_closed_loop_known_gain_with_sensor_noise(self):
  dt=.004;n=30000;t=np.arange(n)*dt;rng=np.random.default_rng(17)
  rc=low(rng.normal(size=n),dt,2)*3+np.sin(t*8)*.4
  w=np.zeros(n);u=np.zeros(n);measured=np.zeros(n);actuator=0.;b=2.5
  for i in range(n-1):
   measured[i]=w[i]+rng.normal(scale=.025)
   u[i]=12*(rc[i]-measured[i])
   w[i+1]=w[i]+dt*(b*actuator-2*w[i]+.3*np.sin(t[i]*.7))
   actuator+=dt/(dt+.04)*(u[i]-actuator)
  measured[-1]=w[-1];u[-1]=u[-2]
  blocks=np.floor(t/6).astype(int);inside=(t>4)&((t%6)>.5)&((t%6)<5.5)
  result=fit_fixed(measured,u,rc,dt,1,40,inside&(blocks%2==0),inside&(blocks%2==1))
  self.assertLess(abs(result["b"]/b-1),.15,result)
  self.assertLess(result["validation"]["projected_moment_error"],.15,result)
 def test_no_excitation_rejected(self):
  n=5000;t=np.arange(n)*.004;z=np.zeros(n)
  with self.assertRaises(ValueError):fit_fixed(z,z,z,.004,1,40,t<10,t>=10)
