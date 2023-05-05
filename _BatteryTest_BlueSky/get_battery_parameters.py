import pandas as pd
import time, os


def get_battery_parameters(in_file_path, out_dir, CRate):
    # Load data from in_file_path (.CSV) formatted as below
    # ['TimeStamp', 'TimeDelta', 'InputCurrent', 'OutputVoltage']
    data = pd.read_csv(in_file_path)

    idx_Time = 0
    idx_DeltaT = 1
    idx_InCurr = 2
    idx_VoltOut = 3

    # Time for the 1st pulse to start (seconds)
    T_pulse_start = 50
    # duration of pulse (seconds) for 10% SOC
    T_pulse_end = 360 / CRate + T_pulse_start

    I_Rin_0 = 0
    V_Rin_0 = 0
    V_R_CT_0 = 0
    I_R_CT = 0
    T_Rin = 0
    T_RCT = 0
    compute_stage = 0

    n = len(data)
    prev_Volt = 0

    for i in range(1, n):
        if data.iloc[i, idx_Time] > T_pulse_start:
            deltaV = data.iloc[i, idx_VoltOut] - prev_Volt

            if abs(deltaV) > 0.05 and compute_stage == 0:
                I_Rin_0 = data.iloc[i, idx_InCurr]
                V_Rin_0 = data.iloc[i-1, idx_VoltOut]
                T_Rin = data.iloc[i-1, idx_Time]
                compute_stage = 1
                continue

            if compute_stage == 1 and data.iloc[i, idx_Time] - T_Rin > 0.015:
                R_in = (V_Rin_0 - data.iloc[i, idx_VoltOut]) / I_Rin_0
                T_RCT = data.iloc[i, idx_Time] + 1
                compute_stage = 2
                continue

            if compute_stage == 2 and data.iloc[i, idx_Time] > T_RCT:
                V_R_CT_0 = data.iloc[i, idx_VoltOut]
                I_R_CT = data.iloc[i, idx_InCurr]
                compute_stage = 3
                continue

            if compute_stage == 3 and deltaV > 0 and data.iloc[i, idx_Time] > T_pulse_end:
                R_CT = (V_R_CT_0 - data.iloc[i-1, idx_VoltOut]) / I_R_CT - R_in
                T_RCT = data.iloc[i, idx_Time]
                compute_stage = 4
                continue

            if compute_stage == 4 and deltaV != 0 and deltaV < 0.00009:
                C_CT = (data.iloc[i, idx_Time] - T_RCT) / R_CT
                current_time = time.localtime()
                out_file_name = 'battery_parameters' + str(current_time.tm_hour) + str(current_time.tm_min) + '.csv'
                out_table = pd.DataFrame({'R_in': [R_in], 'R_CT': [R_CT], 'C_CT': [C_CT]})
                if not os.path.exists(out_dir):
                    os.makedirs(out_dir)
                out_table.to_csv(os.path.join(out_dir, out_file_name))
                return out_table

        prev_Volt = data.iloc[i, idx_VoltOut]