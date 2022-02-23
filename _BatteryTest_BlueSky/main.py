import ClassBattery as Battery
import DL3000 as DLwrapper
import pyvisa as pv
import time

def executeDischarge(inBatteryObj, device):

    if inBatteryObj.m_setting != -1:
        return False

    # Initialize variables
    targetVoltage = inBatteryObj.m_voltageBounds[0]
    inputCurrent = inBatteryObj.m_inputCurrent[0]
    currVoltage = 0
    totalTime = 0
    result = True

    # setup device to run constant battery discharge
    device.reset()
    device.simAPP_Key()

    result = device.setBATT_Curr(inputCurrent)

    if not result:
        print("Discharge current is outside of the allowable range (40 A)! \n")
        return False

    result = device.setBATT_VStop(targetVoltage, inBatteryObj.m_VoltageBounds[0])

    if not result:
        print("Target voltage is lower than the cutoff voltage ({} V)! \n".format(inBatteryObj.m_voltageBounds[0]))
        return False

    currVoltage = device.voltage()

    if currVoltage > inBatteryObj.m_voltageBounds[1]:
        print("Wrong Battery for the simulation setup ... \n")
        return False

    inBatteryObj.logMeasurement(0, currVoltage)
    
    while (True):
            
        prevTime = time.time()

        time.sleep(0.95)

        currVoltage = device.voltage()

        timeDelta = time.time() - prevTime
        totalTime += timeDelta

        inBatteryObj.logMeasurement(timeDelta, currVoltage)

        if (abs(currVoltage - targetVoltage) <= 0.05):
            break

    device.disable()
    device.reset()

    return True

if __name__ == '__main__':

    # initialize test sepcifications (ONLY CONSTANT CURRENT DISCHARGING FOR NOW)
    targetID_eLoad = "USB0::6833::3601::DL3A192600119::0::INSTR"
    targetID_DCSupply = ""
    cellCapacity = 3500                                         # nominal capacity in mAh
    cellNum = 14                                                # number of cells in parallel
    testSetting = -1                                             # -1: constant discharge | 1: constant charge | 0: dynamic
    voltageBounds = [2.5, 4.2]                                  # [cutoff voltage, max. charge voltage]
    CRate = 1/30

    # initialize BatteryObject
    batteryObj = Battery.BatteryObj(cellCapacity, cellNum, testSetting, voltageBounds)
    batteryObj.setupCurrentProfile(CRate)

    # initialize e-load
    resourceManager = pv.ResourceManager()
    eLoad = DLwrapper.DL3000(resourceManager.open_resource(targetID_eLoad))
    
    # run battery unit tests
    result = executeDischarge(batteryObj, eLoad)

    # log all accumulated data
    if result:
        batteryObj.dumpData("sampleResult.csv")
