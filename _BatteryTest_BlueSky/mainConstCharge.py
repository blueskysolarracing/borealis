import ClassBattery as Battery
import DP800 as DPWrapper
import pyvisa as pv
import time
import datetime
import os

def executeCharge(inBatteryObj, device):

    # Initialize variables
    targetVoltage = inBatteryObj.m_voltageBounds[1]
    inputCurrent =  inBatteryObj.genInCurrent()
    currVoltage = 0
    totalTime = 0
    result = True

    if inputCurrent == 0:

        print("Input Current cannot be 0 ... \n")
        return False

    # setup device to run constant battery charge
    result = device.reset()

    if not result:

        print("Could not reset DC Power Supply ... \n")
        return False

    # select Channel 1 on the power supply since it has the 
    # range of interest
    result = device.selectChannel(1)

    if device.queryChannel() != 1:
        
        print("Could not select CH1 for battery testing ... \n")
        return False

    device.channelON()
    currVoltage = device.measureValue("Voltage") 
    device.channelOFF()

    if (abs(currVoltage - targetVoltage) <= 0.05):
        print("Cannot charge battery, near full capacity ... \n")
        return False

    # turn the power supply ON after saving the first set of data
    inBatteryObj.logMeasurement(currVoltage, 0, 0)
    
    # set input current
    result = device.setCURR(inputCurrent)

    if not result:
        print("Input current is too large - defaulted to maximum ... \n")
    
    # set voltage a bit higher to allow enough current to be drawn
    result = device.setVOLT(targetVoltage + 0.5)
    
    if not result:
        print("Voltage is out of bounds, please review your battery characteristics ... \n")
        return False

    # set OVP
    device.setOVP(targetVoltage)
    
    # turn on DC Power Supply (selected Channel)
    device.channelON()

    while (True):

        prevTime = time.time()

        currVoltage = device.measureValue("Voltage")
        currCurrent = device.measureValue("Current")

        time.sleep(0.95)

        timeDelta = time.time() - prevTime
        totalTime += timeDelta

        inBatteryObj.logMeasurement(currVoltage, currCurrent, timeDelta)

        if (abs(currVoltage - targetVoltage) <= 0.05):
            break

    # turn the power supply OFF
    device.channelOFF() 
    device.reset()

    return True

if __name__ == '__main__':

    targetID_pSupply = ""

    # initialize e-Load and DC Power Supply
    resourceManager = pv.ResourceManager()
    for resource in resourceManager.list_resources():
        if "DP8A234200572" in resource:
            targetID_pSupply = resource
            # ex: targetID_pSupply = "USB0::6833::3601::DP8A234200572::0::INSTR"
            break
    
    # initialize test sepcifications (ONLY CONSTANT CURRENT DISCHARGING FOR NOW)
    
    cellCapacity = 3500                                         # nominal capacity in mAh
    cellNum = 14                                                # number of cells in parallel
    testSetting = 1                                             # -1: constant discharge | 1: constant charge | 0: HCCP
    voltageBounds = [2.5, 4.2]                                  # [cutoff voltage, max. charge voltage]
    CRateIn = 1/30

    # initialize BatteryObject
    batteryObj = Battery.BatteryObj(cellCapacity, cellNum, testSetting, voltageBounds, CRate = CRateIn)
    pSupply = DPWrapper.DP800(resourceManager.open_resource(targetID_pSupply))

    # run battery unit tests
    result = executeCharge(batteryObj, pSupply)

    # log all accumulated data
    if result:
        now = datetime.datetime.now()
        nowStr = now.strftime("%Y%m%d_%H%M%S")
        outDir = "output"
        if not os.path.exists(outDir):
            os.makedirs(outDir)
        fileName = "Results_ConstCharge_" + nowStr + ".csv"
        batteryObj.dumpData(os.path.join(outDir, fileName))
