import ClassBattery as Battery
import DP800 as DPWrapper
import DL3000 as DLwrapper
import HPPC 
import pyvisa as pv
import time
import datetime

from HPPC import testHPPC

def executeHPPC(inHPPCObj):

    result = inHPPCObj.initELoad()
    time.sleep(2)
    result = result & inHPPCObj.initDCSupply()
    time.sleep(2)
    inHPPCObj.runHPPC()

    return result

if __name__ == '__main__':

    # initialize test sepcifications (ONLY CONSTANT CURRENT DISCHARGING FOR NOW)
    targetID_eLoad = "USB0::6833::3601::DL3A192600119::0::INSTR"
    targetID_pSupply = "USB0::6833::3601::DP8A234200572::0::INSTR"
    cellCapacity = 3500                                         # nominal capacity in mAh
    cellNum = 14                                                # number of cells in parallel
    testSetting = 1                                             # -1: constant discharge | 1: constant charge | 0: HCCP
    voltageBounds = [2.5, 4.2]                                  # [cutoff voltage, max. charge voltage]
    HPPCRatesIn = [0.75, 0.85]
    cyclesIn = 2
    
    # initialize BatteryObject 
    batteryObj = Battery.BatteryObj(cellCapacity, cellNum, testSetting, voltageBounds, HPPCRates = HPPCRatesIn, cycles = cyclesIn)

    # initialize e-Load and DC Power Supply
    resourceManager = pv.ResourceManager()
    eLoad = DLwrapper.DL3000(resourceManager.open_resource(targetID_eLoad))
    pSupply = DPWrapper.DP800(resourceManager.open_resource(targetID_pSupply))

    # TestHPPC object
    HPPCObj = HPPC.testHPPC(batteryObj, eLoad, pSupply)
    
    # run battery unit tests
    result = executeHPPC(batteryObj, pSupply)

    # log all accumulated data
    if result:
        now = datetime.datetime.now()
        nowStr = now.strftime("%Y%m%d_%H%M%S")
        fileName = "Results_HPPC_" + nowStr + ".csv"
        batteryObj.dumpData(fileName)
