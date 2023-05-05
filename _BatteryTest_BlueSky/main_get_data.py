import argparse
from genSOC_v_VTerm import generate_SOC_vs_VTerm
from get_battery_parameters import get_battery_parameters

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Get battery parameters and SOC data.')
    parser.add_argument('in_file_path', type=str, default='data.csv', help='Path to input file.')
    parser.add_argument('--nominal_capacity', type=float, default=1000, help='Nominal capacity in mAh.')
    parser.add_argument('--charging_mode', type=int, default=-1, help='(1) or (-1) for discharging or charging.')
    parser.add_argument('--charging_rate', type=float, default=1, help='rate at which a battery is charged or discharged relative to its capacity. A C-rate of 1C represents a charge or discharge current that will discharge the battery in one hour. For example, a 1C discharge rate for a battery with a capacity of 2000 mAh would be a discharge current of 2000 mA.')
    args = parser.parse_args()

    out_dir = 'output'
    
    # Get battery parameters.
    battery_parameters = get_battery_parameters(args.in_file_path, out_dir, args.charging_rate)


    # Generate SOC vs VTerm.
    generate_SOC_vs_VTerm(args.in_file_path, out_dir, args.nominal_capacity, args.charging_mode)