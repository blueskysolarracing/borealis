import ClassBattery as Battery
import DL3000 as DLwrapper
import pyvisa as pv
import time
import datetime

# from mainConstCharge import CRate

def executeDischarge(inBatteryObj, device):

    # Initialize variables
    targetVoltage = inBatteryObj.m_voltageBounds[0]
    inputCurrent = inBatteryObj.genInCurrent()
    currVoltage = 0
    totalTime = 0
    result = True

    if inputCurrent == 0:

        print("Input Current cannot be 0 ... \n")
        return False

    # setup device to run constant battery discharge
    device.reset()
    device.simAPP_Key()

    result = device.setBATT_Curr(inputCurrent)

    if not result:
        print("Discharge current is outside of the allowable range (40 A)! \n")
        return False

    currVoltage = device.voltage()
    result = device.setBATT_VStop(currVoltage, targetVoltage)

    if not result:
        print("Target voltage is lower than the cutoff voltage ({} V)! \n".format(inBatteryObj.m_voltageBounds[0]))
        return False

    if (abs(currVoltage - targetVoltage) <= 0.05):
        print("Cannot discharge battery, near end of capacity ... \n")
        return False

    inBatteryObj.logMeasurement(currVoltage, 0, 0)
    device.enable()
    
    while (True):
            
        prevTime = time.time()

        time.sleep(0.95)

        currVoltage = device.voltage()
        currCurrent = device.current()

        timeDelta = time.time() - prevTime
        totalTime += timeDelta

        inBatteryObj.logMeasurement(currVoltage, currCurrent, timeDelta)

        print("Voltage: " + str(currVoltage) + "V")

        if (abs(currVoltage - targetVoltage) <= 0.05):
            break

    device.disable()
    device.reset()

    return True

if __name__ == '__main__':

    # initialize test sepcifications (ONLY CONSTANT CURRENT DISCHARGING FOR NOW)
    targetID_eLoad = "USB0::6833::3601::DL3A192600119::0::INSTR"
    cellCapacity = 3500                                         # nominal capacity in mAh
    cellNum = 14                                                # number of cells in parallel
    testSetting = -1                                             # -1: constant discharge | 1: constant charge | 0: HCCP
    voltageBounds = [2.5, 4.2]                                  # [cutoff voltage, max. charge voltage]

    # initialize BatteryObject
    batteryObj = Battery.BatteryObj(cellCapacity, cellNum, testSetting, voltageBounds, 0.408)

    # initialize e-Load and DC Power Supply
    resourceManager = pv.ResourceManager()
    eLoad = DLwrapper.DL3000(resourceManager.open_resource(targetID_eLoad))
    
    # run battery unit tests
    result = executeDischarge(batteryObj, eLoad)

    # log all accumulated data
    if result:
        now = datetime.datetime.now()
        nowStr = now.strftime("%Y%m%d_%H%M%S")
        fileName = "Results_ConstDischarge_" + nowStr + ".csv"
        batteryObj.dumpData(fileName)
