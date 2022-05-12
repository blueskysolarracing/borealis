#Simulate PV cell (follow measured IV curve) with Rigol DP800

import DP800
import pyvisa as pv

init_current = 2.0 #Initial current to get initial operating point on the IV curve
max_current = 3.4 #Max. allowed current from PSU (although DP800 can only reach 5A)

#Methods
def compVolt(curr):
    #Simulates 2 cells in series with I_sc of ~3.5A
    return (30.03*curr**3 - 56.22*curr**2 - 3436*curr + 11480) / (curr**2 - 2852*curr + 10170) #Fit from eve module, current increased

def getMPP():
    #Determines the maximum voltage and current (MPP) on the given IV curve
    num_points = 10000
    interval = max_current / num_points
    powerList = [] #List of maxPowerList's
    maxPowerList = [0, 0, 0] #Carries current, voltage and power for each point

    for i in range (num_points):
        current = interval * i
        voltage = compVolt(current)

        newList = [current, voltage, current*voltage]
        powerList.append(newList)

        if newList[2] > maxPowerList[2]: #Update maximum power point
            maxPowerList = newList

    return maxPowerList

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

#Greet user
MPP = getMPP()
print("Welcome to the Solar Cell Simulator.\n Max. power is " + "{:10.2f}".format(MPP[2]) + "W (at " + "{:10.2f}".format(MPP[1]) + "V, " + "{:10.2f}".format(MPP[0]) + "A)")

#Maintain IV curve
while 1:
    #Read current
    i = PSU.measureValue("Current") #Measure current with 0.0s delay
    
    #Set target voltage
    PSU.setVOLT(compVolt(i), 0.0) #Set voltage corresponding to current with 0.0s delay