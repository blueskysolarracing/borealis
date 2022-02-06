import ClassBattery as Battery
import DL3000 as DLwrapper
import visa as pv
import time

def constantTest(inBatteryObj, device):

    targetVoltage = ((inBatteryObj.m_setting == 1)*inBatteryObj.voltageBound[1] + (inBatteryObj.m_setting == -1)*inBatteryObj.voltageBound[0])
    inputCurrent = inBatteryObj.m_inputCurrent[0]

    currVoltage = 0
    totalTime = 0

    # save initial voltage without applying current (need to check if it is done like this)
    device.enable()
    currVoltage = device.voltage()
    inBatteryObj.logMeasurement(0, currVoltage)
    device.disable()
    
    while (True):

        device.set_cc_current(inputCurrent)
            
        prevTime = time.time()
        device.enable()

        time.sleep(0.85)

        currVoltage = device.voltage()
        device.disable()

        timeDelta = time.time() - prevTime
        totalTime += timeDelta

        inBatteryObj.logMeasurement(timeDelta, currVoltage)

        if (abs(currVoltage - targetVoltage) <= 0.05):
            break

    return True

def execute(inBatteryObj, device):

    if abs(inBatteryObj.m_setting) == 1:

        constantTest(inBatteryObj, device)
        return True

    print("Input setting {} to BatteryObj type is invalid... \n".format(inBatteryObj.m_setting))

    return False

if __name__ == '__main__':

    # initialize test sepcifications
    targetID = "USB0::6833::3601::DL3A192600119::0::INSTR"
    cellCapacity = 3500                                         # nominal capacity in mAh
    cellNum = 14                                                # number of cells in parallel
    testSetting = 1                                             # -1: constant discharge | 1: constant charge | 0: dynamic
    voltageBounds = [2.5, 4.2]                                  # [cutoff voltage, max. charge voltage]
    CRate = 0.5

    # initialize BatteryObject
    batteryObj = Battery.BatteryObj(cellCapacity, cellNum, testSetting)
    batteryObj.setupCurrentProfile(CRate)

    # initialize e-load
    resourceManager = pv.ResourceManager()
    eLoad = DLwrapper.DL3000(resourceManager.open_resource(targetID))
    eLoad.reset()
    eLoad.set_mode("CURRENT")

    # run battery unit test
    result = execute(batteryObj, eLoad)

    # log all accumulated data
    if result:
        batteryObj.dumpData("sampleResult.csv")
