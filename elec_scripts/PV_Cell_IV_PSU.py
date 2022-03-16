#Simulate PV cell (follow measured IV curve) with Rigol DP800

import DP800Wrapper
import pyvisa as pv

#Open connection
targetID = ""
resourceManager = pv.ResourceManager()

#Create and initialize object
PSU = DP800Wrapper.DP800()
PSU.selectChannel(1)

#Start with 2V, 2A

#Maintain IV curve
while 1:
    #Read current
    i = PSU.measureValue(1, "Current")

    #Compute target voltage
    target_voltage = (30.03*i**3 - 56.22*i**2 - 3436*i + 11480) / (i**2 - 2852*i + 10170) #Fit from eve module, current increased

    #Set target voltage
    
