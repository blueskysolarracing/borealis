#Simulate PV cell (follow measured IV curve) with Rigol DP800

import DP800
import pyvisa as pv

init_current = 2.0 #Initial current to get initial operating point on the IV curve

def compVolt(curr):
    #Simulates 2 cells in series with I_sc of ~3.5A
    return (30.03*curr**3 - 56.22*curr**2 - 3436*curr + 11480) / (curr**2 - 2852*curr + 10170) #Fit from eve module, current increased

#Open connection
targetID = "USB0::6833::3601::DP8A234200572::0::INSTR"
resourceManager = pv.ResourceManager()

#Create and initialize object
PSU = DP800.DP800(resourceManager.open_resource(targetID))

#Select channel 1
PSU.selectChannel(1)
                  
#Set OVP and OCP
PSU.setOVP(4.0) #4V
PSU.setOCP(4.0) #4A; must be higher than I_sc
                  
#Set initial IV
PSU.setCURR(init_current) #2A
PSU.setVOLT(compVolt(init_current)) #Set voltage corresponding to 2A
PSU.channelON()           

#Maintain IV curve
while 1:
    #Read current
    i = PSU.measureValue("Current") #Measure current with 0.0s delay
    print("Current: " + str(i))

    #Set target voltage
    PSU.setVOLT(compVolt(i), 0.0) #Set voltage corresponding to current with 0.0s delay