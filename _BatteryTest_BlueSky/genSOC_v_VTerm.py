import pandas as pd
import time
import matplotlib.pyplot as plt
import os

def generate_SOC_vs_VTerm(in_file_path, out_dir, nominal_capacity, mode):
    # Data format inside file (.CSV)
    # {'TimeStamp','TimeDelta','InputCurrent','OutputVoltage'}
    # nominal_capacity - nominal capacity in mAh (Q)
    # mode - (1) or (-1) for discharging or charging
    data = pd.read_csv(in_file_path)
    dt_idx = 1
    in_curr_idx = 2
    out_volt_idx = 3
    n = len(data)

    Cte = 1000/3600 # coulomb_factor 

    # Assume we start from a fully charged battery.
    out_SOC = [1]
    out_VTerm = [data.iloc[1, out_volt_idx]]

    # Do Coulomb counting.
    for i in range(2, n):
        temp_SOC = out_SOC[i - 2] - (Cte*mode*data.iloc[i, dt_idx]*data.iloc[i, in_curr_idx])/nominal_capacity

        if temp_SOC < 0:
            break
        else:
            out_SOC.append(temp_SOC)
            out_VTerm.append(data.iloc[i, out_volt_idx])

    # Create output file.
    plt.plot(out_VTerm, out_SOC)
    plt.title('Terminal Voltage vs SOC')
    plt.legend(['SOC'], loc='upper left')
    plt.xlim([out_VTerm[-2], out_VTerm[0]])
    plt.ylim([0, 1])
    plt.xlabel('Voltage (V)')
    plt.ylabel('SOC')
    plt.show()

    current_time = time.localtime()
    file_name = 'VTerm_v_SOC_' + str(current_time.tm_hour) + str(current_time.tm_min) + '.csv'
    out_table = pd.DataFrame({'VTerm': out_VTerm, 'SOC': out_SOC})
    if not os.path.exists(out_dir):
        os.makedirs(out_dir)
    out_table.to_csv(os.path.join(out_dir, file_name))

    return out_table