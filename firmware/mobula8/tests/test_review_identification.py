import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from review_identification import fit
class ReviewTests(unittest.TestCase):
 def test_held_out_plant(self):
  dt=.001;t=np.arange(12000)*dt;u=20*np.sin(2*np.pi*.7*t)+12*np.sin(2*np.pi*2.3*t)
  w=np.zeros(len(t));lag=0
  for i in range(1,len(t)):
   lag+=dt/(dt+.025)*(u[i-1]-lag);w[i]=w[i-1]+dt*(2*lag-1.5*w[i-1]+.2)
  r=fit(w,u,dt,25)
  self.assertAlmostEqual(r["b"],2,delta=.1)
  self.assertGreater(r["validation_r2"],.95)
 def test_wrong_validation_plant_rejected(self):
  dt=.001;t=np.arange(12000)*dt;u=20*np.sin(2*np.pi*.7*t)+12*np.sin(2*np.pi*2.3*t)
  w=np.zeros(len(t));lag=0
  for i in range(1,len(t)):
   lag+=dt/(dt+.025)*(u[i-1]-lag);w[i]=w[i-1]+dt*((2 if i<8000 else .1)*lag-1.5*w[i-1]+.2)
  r=fit(w,u,dt,25)
  self.assertLess(r["validation_r2"],.6)
