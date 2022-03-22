import ClassBattery as Battery
import DL3000 as DLwrapper
import DP800 as DPWrapper
import visa as pv
import time

def executeHCCP(inBatteryObj, eLoad, pSupply):
    return True

def executeCharge(inBatteryObj, device):

    # Initialize variables
    targetVoltage = inBatteryObj.m_voltageBounds[1]
    inputCurrent = inBatteryObj.m_inputCurrent
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

    # select channel 2 on the power supply since it has the 
    # range of interest
    result = device.selectChannel(2)

    if device.queryChannel != 2:
        
        print("Could not select CH1 for battery testing ... \n")
        return False

        

    return True

def executeDischarge(inBatteryObj, device):

    # Initialize variables
    targetVoltage = inBatteryObj.m_voltageBounds[0]
    inputCurrent = inBatteryObj.m_inputCurrent[0]
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

    result = device.setBATT_VStop(targetVoltage, targetVoltage)

    if not result:
        print("Target voltage is lower than the cutoff voltage ({} V)! \n".format(inBatteryObj.m_voltageBounds[0]))
        return False

    currVoltage = device.voltage()

    if currVoltage > inBatteryObj.m_voltageBounds[1]:
        print("Wrong Battery for the simulation setup ... \n")
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

        if (abs(currVoltage - targetVoltage) <= 0.05):
            break

    device.disable()
    device.reset()

    return True

def run(batObj, eLoad, pSupply):

    if batObj.m_setting == 1:
        return executeDischarge(batObj, eLoad)
    elif batObj.m_setting == -1:
        return executeCharge(batObj, pSupply)
    elif batObj.m_setting == 0:
        return executeHCCP(batObj, eLoad, pSupply)
    
    print("Invalid battery test mode ... \n")

    return False

if __name__ == '__main__':

    # initialize test sepcifications (ONLY CONSTANT CURRENT DISCHARGING FOR NOW)
    targetID_eLoad = "USB0::6833::3601::DL3A192600119::0::INSTR"
    targetID_pSupply = ""
    cellCapacity = 3500                                         # nominal capacity in mAh
    cellNum = 14                                                # number of cells in parallel
    testSetting = -1                                             # -1: constant discharge | 1: constant charge | 0: HCCP
    voltageBounds = [2.5, 4.2]                                  # [cutoff voltage, max. charge voltage]
    CRate = 1/30
    HCCPRates = [0.75, 1]

    # initialize BatteryObject
    batteryObj = Battery.BatteryObj(cellCapacity, cellNum, testSetting, voltageBounds, CRate, HCCPRates)
    batteryObj.genInCurrent(CRate, HCCPRates)

    # initialize e-Load and DC Power Supply
    resourceManager = pv.ResourceManager()
    eLoad = DLwrapper.DL3000(resourceManager.open_resource(targetID_eLoad))
    pSupply = DPWrapper.DP800(resourceManager.open_resource(targetID_pSupply))
    
    # run battery unit tests
    result = executeDischarge(batteryObj, eLoad)

    # log all accumulated data
    if result:
        batteryObj.dumpData("sampleResult.csv")
