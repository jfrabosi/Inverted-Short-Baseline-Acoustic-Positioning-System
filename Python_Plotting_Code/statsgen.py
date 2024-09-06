import re
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.patches import Ellipse
from scipy import stats  # Add this import


custom_title_str = ""


def parse_log_file(log_file_path, axis_mask):
    # Initialize lists to store parsed data
    axis = []
    angle = []
    x_kf_sf, y_kf_sf, z_kf_sf = [], [], []
    x_r_isbl, y_r_isbl, z_r_isbl = [], [], []
    x_kf_isbl, y_kf_isbl, z_kf_isbl = [], [], []
    x_true, y_true, z_true = [], [], []
    roll, pitch, yaw = [], [], []

    # Regular expression pattern to match both integer and float for angle
    pattern = r'([A-Z]), (-?\d+(?:\.\d+)?): ([-\d.]+), ([-\d.]+), ([-\d.]+); ([-\d.]+), ([-\d.]+), ([-\d.]+); ([-\d.]+), ([-\d.]+), ([-\d.]+); ([-\d.]+), ([-\d.]+), ([-\d.]+); ([-\d.]+), ([-\d.]+), ([-\d.]+);'

    # Open and read the log file
    with open(log_file_path, 'r') as file:
        discard_next = False
        for line in file:
            line = line.strip()
            if line == "DISCARD":
                discard_next = True
                continue
            if discard_next:
                discard_next = False
                continue
            match = re.match(pattern, line.strip())
            if match and match.group(1) in axis_mask:
                # Extract data from the matched groups
                axis.append(match.group(1))
                angle.append(float(match.group(2)))
                x_kf_sf.append(float(match.group(3)))
                y_kf_sf.append(float(match.group(4)))
                z_kf_sf.append(float(match.group(5)))
                x_kf_isbl.append(float(match.group(6)))
                y_kf_isbl.append(float(match.group(7)))
                z_kf_isbl.append(float(match.group(8)))
                x_r_isbl.append(float(match.group(9)))
                y_r_isbl.append(float(match.group(10)))
                z_r_isbl.append(float(match.group(11)))
                x_true.append(float(match.group(12)))
                y_true.append(float(match.group(13)))
                z_true.append(float(match.group(14)))
                roll.append(float(match.group(15)))
                pitch.append(float(match.group(16)))
                yaw.append(float(match.group(17)))

    return [axis, angle, x_kf_sf, y_kf_sf, z_kf_sf, x_r_isbl, y_r_isbl, z_r_isbl,
            x_kf_isbl, y_kf_isbl, z_kf_isbl, x_true, y_true, z_true, roll, pitch, yaw]


def remove_outliers(data, print_removed=False):
    axis, angle, x_kf_sf, y_kf_sf, z_kf_sf, x_r_isbl, y_r_isbl, z_r_isbl, x_kf_isbl, y_kf_isbl, z_kf_isbl, x_true, y_true, z_true, roll, pitch, yaw = data

    # Convert lists to numpy arrays for element-wise operations
    x_kf_sf, y_kf_sf, z_kf_sf = map(np.array, [x_kf_sf, y_kf_sf, z_kf_sf])
    x_r_isbl, y_r_isbl, z_r_isbl = map(np.array, [x_r_isbl, y_r_isbl, z_r_isbl])
    x_kf_isbl, y_kf_isbl, z_kf_isbl = map(np.array, [x_kf_isbl, y_kf_isbl, z_kf_isbl])
    x_true, y_true, z_true = map(np.array, [x_true, y_true, z_true])

    # Calculate differences
    differences = np.array([
        x_kf_sf - x_true, y_kf_sf - y_true, z_kf_sf - z_true,
        x_r_isbl - x_true, y_r_isbl - y_true, z_r_isbl - z_true,
        x_kf_isbl - x_true, y_kf_isbl - y_true, z_kf_isbl - z_true
    ])

    # Calculate mean and standard deviation for each difference
    means = np.mean(differences, axis=1)
    stds = np.std(differences, axis=1)

    # Create a mask for values within 4 standard deviations
    outlier_mask = np.abs(differences - means[:, np.newaxis]) > 4 * stds[:, np.newaxis]
    row_mask = np.any(outlier_mask, axis=0)

    # Identify removed rows
    removed_rows = np.where(row_mask)[0]
    total_rows = len(axis)
    removed_count = len(removed_rows)
    removed_percentage = (removed_count / total_rows) * 100

    print(f"Removed {removed_count} out of {total_rows} rows ({removed_percentage:.2f}%)")

    if print_removed and removed_count > 0:
        print("Removed rows:")
        for idx in removed_rows:
            outlier_indices = np.where(outlier_mask[:, idx])[0]

            # Format the row string with proper separators
            row_str = f"{axis[idx]}, {angle[idx]}: "

            values = [
                (x_kf_sf[idx], y_kf_sf[idx], z_kf_sf[idx]),
                (x_r_isbl[idx], y_r_isbl[idx], z_r_isbl[idx]),
                (x_kf_isbl[idx], y_kf_isbl[idx], z_kf_isbl[idx]),
                (x_true[idx], y_true[idx], z_true[idx]),
                (roll[idx], pitch[idx], yaw[idx])
            ]

            for i, triple in enumerate(values):
                for j, value in enumerate(triple):
                    if i * 3 + j in outlier_indices:
                        row_str += f"*{value:.2f}*"
                    else:
                        row_str += f"{value:.2f}"

                    if j < 2:
                        row_str += ", "
                    elif i < 4:
                        row_str += "; "

            print(f"Row {idx}: {row_str}")

    # Apply the mask to all data lists
    return [
        [d for d, m in zip(sublist, ~row_mask) if m]
        for sublist in data
    ]


def process_log_file(log_file_path, plot_hist, num_iterations=3, axis_mask="ABCXYZ"):
    # Parse the log file
    print("Parsing log file...")
    data = parse_log_file(log_file_path, axis_mask)

    # Store the original data for plotting
    data_before = data.copy()

    print(f"\nNumber of rows before outlier removal: {len(data[0])}")

    # Apply outlier removal for the specified number of iterations
    for i in range(num_iterations):
        print(f"\nIteration {i+1} of outlier removal:")
        data = remove_outliers(data)
        print(f"Remaining rows after iteration {i+1}: {len(data[0])}")

    # Count number of datapoints
    num_datapoints = len(data[0])

    # Ensure all numeric data is converted to native Python float
    data = [
        [float(x) if isinstance(x, np.number) else x for x in sublist]
        for sublist in data
    ]

    # Print the final number of remaining rows
    print(f"\n---------\n")

    # Plot histograms
    if plot_hist:
        plot_difference_histograms(data_before, data)

    return data, num_datapoints


def plot_difference_histograms(data_before, data_after):
    # Extract relevant data
    variables = ['x_kf_sf', 'y_kf_sf', 'z_kf_sf',
                 'x_r_isbl', 'y_r_isbl', 'z_r_isbl',
                 'x_kf_isbl', 'y_kf_isbl', 'z_kf_isbl']

    def calculate_differences(data):
        x_kf_sf, y_kf_sf, z_kf_sf, x_r_isbl, y_r_isbl, z_r_isbl, x_kf_isbl, y_kf_isbl, z_kf_isbl, x_true, y_true, z_true = data[
                                                                                                                           2:14]
        return [
            np.array(x_kf_sf) - np.array(x_true),
            np.array(y_kf_sf) - np.array(y_true),
            np.array(z_kf_sf) - np.array(z_true),
            np.array(x_r_isbl) - np.array(x_true),
            np.array(y_r_isbl) - np.array(y_true),
            np.array(z_r_isbl) - np.array(z_true),
            np.array(x_kf_isbl) - np.array(x_true),
            np.array(y_kf_isbl) - np.array(y_true),
            np.array(z_kf_isbl) - np.array(z_true)
        ]

    differences_before = calculate_differences(data_before)
    differences_after = calculate_differences(data_after)

    # Create subplots
    fig, axes = plt.subplots(3, 3, figsize=(15, 15))
    fig.suptitle('Histograms of Differences (Measured - True)', fontsize=16)

    for i, (ax, var) in enumerate(zip(axes.flat, variables)):
        # Combine before and after data to determine common range and bins
        combined_data = np.concatenate([differences_before[i], differences_after[i]])
        min_val, max_val = np.min(combined_data), np.max(combined_data)

        # Calculate number of bins (you can adjust this if needed)
        num_bins = 50

        # Create common bins
        bins = np.linspace(min_val, max_val, num_bins + 1)

        # Plot histogram for data before outlier removal
        ax.hist(differences_before[i], bins=bins, alpha=0.5, label='Before', color='blue', density=True)

        # Plot histogram for data after outlier removal
        ax.hist(differences_after[i], bins=bins, alpha=0.5, label='After', color='red', density=True)

        ax.set_title(var)
        ax.set_xlabel('Difference')
        ax.set_ylabel('Density')
        ax.legend()

        # Set x-axis limits to ensure consistency across before and after
        ax.set_xlim(min_val, max_val)

    plt.tight_layout()
    plt.show()


def process_multiple_files(file_list, folder, num_outlier_removals, axes_list):
    all_processed_data = []
    cum_datapoints = 0
    for file in file_list:
        log_file_path = folder + file
        print(f"Processing file: {file}")
        processed_data, num_datapoints = process_log_file(log_file_path, False, num_outlier_removals, axes_list)
        cum_datapoints += num_datapoints
        all_processed_data.append(processed_data)
    return all_processed_data, cum_datapoints


def generate_statistics_table(all_processed_data, file_list, combine_rgs=False):
    stats_data = []

    # Create a dictionary to group data by position
    position_data = {}

    for data, file in zip(all_processed_data, file_list):
        # Extract position from filename
        parts = file.split('_')
        pos = parts[1:4]
        x_pos, y_pos, z_pos = [int(p[1:]) for p in pos]
        position_key = (x_pos, y_pos, z_pos)

        # Calculate Euclidean distance and yaw angle
        euclidean_distance = np.sqrt(x_pos**2 + y_pos**2 + z_pos**2)
        yaw_angle = np.degrees(np.arctan2(y_pos, x_pos))

        # Extract file type (d_g, d_r, or s)
        file_type = '_'.join(parts[4:]).split('.')[0]

        # Prepare data for this file
        file_data = {
            'data': data,
            'distance': euclidean_distance,
            'yaw_angle': yaw_angle,
            'file_type': file_type
        }

        # Group data by position
        if position_key not in position_data:
            position_data[position_key] = []
        position_data[position_key].append(file_data)

    # Process grouped data
    for i, (position, group_data) in enumerate(position_data.items(), 1):
        x_pos, y_pos, z_pos = position

        if combine_rgs:
            # Combine all data for this position
            combined_data = [d['data'] for d in group_data]
            combined_stats = calculate_stats(combined_data, x_pos, y_pos, z_pos, i)
            combined_stats['Distance'] = group_data[0]['distance']
            combined_stats['Yaw Angle'] = group_data[0]['yaw_angle']
            stats_data.append(combined_stats)
        else:
            # Process each file separately
            for file_data in group_data:
                file_stats = calculate_stats([file_data['data']], x_pos, y_pos, z_pos, i)
                file_stats['Distance'] = file_data['distance']
                file_stats['Yaw Angle'] = file_data['yaw_angle']
                file_stats['Type'] = file_data['file_type']
                stats_data.append(file_stats)

    # Add combined statistics for different groups
    groups = {'d_g': [], 'd_r': [], 's': [], 'All': []}
    for data, file in zip(all_processed_data, file_list):
        file_type = '_'.join(file.split('_')[4:]).split('.')[0]
        if file_type in groups:
            groups[file_type].append(data)
        groups['All'].append(data)

    group_names = {
        'd_g': 'Gradual',
        'd_r': 'Rapid',
        's': 'Stationary',
        'All': 'All Combined'
    }

    # Add this after the groups are populated
    print("\nGroup sizes:")
    for group_name, group_data_list in groups.items():
        print(f"{group_name}: {len(group_data_list)} files")

    for group_name, group_data_list in groups.items():
        if group_data_list:
            group_stats = calculate_stats(group_data_list, group_names[group_name], group_names[group_name], group_names[group_name], len(stats_data) + 1)
            group_stats['Distance'] = 'N/A'
            group_stats['Yaw Angle'] = 'N/A'
            stats_data.append(group_stats)

    # Create DataFrame
    df = pd.DataFrame(stats_data)

    # Define column groups
    column_groups = {
        'TX': ['X-pos', 'Y-pos', 'Z-pos', 'Distance', 'Yaw Angle'],
        'iSBL-SF Kalman Filter': ['iSBL-SF KF Y-diff Mean', 'iSBL-SF KF Y-diff StDev', 'iSBL-SF KF Z-diff Mean', 'iSBL-SF KF Z-diff StDev'],
        'iSBL Kalman Filter': ['iSBL KF Y-diff Mean', 'iSBL KF Y-diff StDev', 'iSBL KF Z-diff Mean', 'iSBL KF Z-diff StDev'],
        'iSBL Raw Position Estimates': ['iSBL Raw Y-diff Mean', 'iSBL Raw Y-diff StDev', 'iSBL Raw Z-diff Mean', 'iSBL Raw Z-diff StDev']
    }

    # Create multi-level columns
    columns = pd.MultiIndex.from_tuples([('Test Number', '')] +
                                        [(group, col) for group, cols in column_groups.items() for col in cols])
    if not combine_rgs:
        columns = columns.insert(1, ('TX', 'Type'))

    # Reorder DataFrame columns
    df = df.reindex(columns=columns.get_level_values(1))

    # Set multi-level columns
    df.columns = columns

    return df

def calculate_stats(data_list, x_pos, y_pos, z_pos, index):
    # Combine data from all files in the group
    y_kf_sf = np.concatenate([data[3] for data in data_list])
    z_kf_sf = np.concatenate([data[4] for data in data_list])
    y_r_isbl = np.concatenate([data[6] for data in data_list])
    z_r_isbl = np.concatenate([data[7] for data in data_list])
    y_kf_isbl = np.concatenate([data[9] for data in data_list])
    z_kf_isbl = np.concatenate([data[10] for data in data_list])
    y_true = np.concatenate([data[12] for data in data_list])
    z_true = np.concatenate([data[13] for data in data_list])

    # Calculate differences
    y_kf_sf_diff = y_kf_sf - y_true
    z_kf_sf_diff = z_kf_sf - z_true
    y_kf_isbl_diff = y_kf_isbl - y_true
    z_kf_isbl_diff = z_kf_isbl - z_true
    y_r_isbl_diff = y_r_isbl - y_true
    z_r_isbl_diff = z_r_isbl - z_true

    return {
        'Test Number': index,
        'X-pos': x_pos,
        'Y-pos': y_pos,
        'Z-pos': z_pos,
        'iSBL-SF KF Y-diff Mean': np.mean(y_kf_sf_diff),
        'iSBL-SF KF Y-diff StDev': np.std(y_kf_sf_diff),
        'iSBL-SF KF Z-diff Mean': np.mean(z_kf_sf_diff),
        'iSBL-SF KF Z-diff StDev': np.std(z_kf_sf_diff),
        'iSBL KF Y-diff Mean': np.mean(y_kf_isbl_diff),
        'iSBL KF Y-diff StDev': np.std(y_kf_isbl_diff),
        'iSBL KF Z-diff Mean': np.mean(z_kf_isbl_diff),
        'iSBL KF Z-diff StDev': np.std(z_kf_isbl_diff),
        'iSBL Raw Y-diff Mean': np.mean(y_r_isbl_diff),
        'iSBL Raw Y-diff StDev': np.std(y_r_isbl_diff),
        'iSBL Raw Z-diff Mean': np.mean(z_r_isbl_diff),
        'iSBL Raw Z-diff StDev': np.std(z_r_isbl_diff),
    }


def save_stats_table(combine_rgs=False):
    all_processed_data, _ = process_multiple_files(file_list, folder, num_outlier_removals, axes_list)
    df = generate_statistics_table(all_processed_data, file_list, combine_rgs)

    # Save to CSV
    csv_filename = 'statistics_table_combined.csv' if combine_rgs else 'statistics_table_separate.csv'
    df.to_csv(csv_filename)

    # Save to Excel
    excel_filename = 'statistics_table_combined.xlsx' if combine_rgs else 'statistics_table_separate.xlsx'
    df.to_excel(excel_filename)

    print(f"Statistics table saved as {csv_filename} and {excel_filename}.")


# ENTER FILE AND OFFSET
folder = "C:/Users/frabb/OneDrive - Cal Poly/Documents (Cloud)/0 CALPOLY/000Thesis/"
file_list = ["test_x1426_y0_z-694_d_g.log",
"test_x2645_y0_z-682_d_g.log",
"test_x3880_y0_z-652_d_g.log",
"test_x1426_y-152_z-694_d_g.log",
"test_x1426_y-305_z-694_d_g.log",
"test_x2051_y305_z-707_d_g.log",
"test_x2051_y610_z-702_d_g.log",
"test_x2203_y762_z-726_d_g.log",
"test_x2645_y457_z-708_d_g.log",
"test_x1426_y0_z-694_d_r.log",
"test_x2645_y0_z-682_d_r.log",
"test_x3880_y0_z-652_d_r.log",
"test_x1426_y-152_z-694_d_r.log",
"test_x1426_y-305_z-694_d_r.log",
"test_x2051_y305_z-707_d_r.log",
"test_x2051_y610_z-702_d_r.log",
"test_x2203_y762_z-726_d_r.log",
"test_x2645_y457_z-708_d_r.log",
"test_x2645_y0_z-682_s.log",
"test_x3880_y0_z-652_s.log",
"test_x1426_y-152_z-694_s.log",
"test_x1426_y-305_z-694_s.log",
"test_x2051_y305_z-707_s.log",
"test_x2051_y610_z-702_s.log",
"test_x2203_y762_z-726_s.log",
"test_x2645_y457_z-708_s.log",
]

offset_x = 0
offset_y = 90
num_outlier_removals = 3
axes_list = "XYZABC"

save_stats_table(combine_rgs=True)
