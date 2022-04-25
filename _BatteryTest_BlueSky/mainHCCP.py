import ClassBattery as Battery
import DP800 as DPWrapper
import DL3000 as DLwrapper
import pyvisa as pv
import time

from HCCP import testHCCP

def executeHCCP(inHCCPObj):

    inHCCPObj.initLogging()

    result = inHCCPObj.initELoad()
    time.sleep(2)
    result = result & inHCCPObj.initDCSupply()
    time.sleep(2)
    inHCCPObj.runHCCP()

    return result

if __name__ == '__main__':

    # initialize test sepcifications (ONLY CONSTANT CURRENT DISCHARGING FOR NOW)
    targetID_eLoad = "USB0::6833::3601::DL3A192600119::0::INSTR"
    targetID_pSupply = "USB0::6833::3601::DP8A234200572::0::INSTR"
    cellCapacity = 3500                                         # nominal capacity in mAh
    cellNum = 14                                                # number of cells in parallel
    testSetting = 1                                             # -1: constant discharge | 1: constant charge | 0: HCCP
    voltageBounds = [2.5, 4.2]                                  # [cutoff voltage, max. charge voltage]
    HCCPRatesIn = [0.75, 0.80]
    
    # initialize BatteryObject 
    batteryObj = Battery.BatteryObj(cellCapacity, cellNum, testSetting, voltageBounds, HCCPRates = HCCPRatesIn)

    # initialize e-Load and DC Power Supply
    resourceManager = pv.ResourceManager()
    eLoad = DLwrapper.DL3000(resourceManager.open_resource(targetID_eLoad))
    pSupply = DPWrapper.DP800(resourceManager.open_resource(targetID_pSupply))

    # TestHPPC object
    HCCPObj = testHCCP(batteryObj, eLoad, pSupply, 0)
    
    # run battery unit tests
    result = executeHCCP(HCCPObj)