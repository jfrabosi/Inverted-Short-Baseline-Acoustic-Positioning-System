import re
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from scipy import stats


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


def plot_comparison_data(all_processed_data, xoffset, yoffset, axis_mask, num_datapoints, custom_title=None, subtract_means=True):
    # Combine data from all processed files
    y_kf_sf = np.concatenate([np.array(data[3]) for data in all_processed_data])
    z_kf_sf = np.concatenate([np.array(data[4]) for data in all_processed_data])
    y_r_isbl = np.concatenate([np.array(data[6]) for data in all_processed_data])
    z_r_isbl = np.concatenate([np.array(data[7]) for data in all_processed_data])
    y_kf_isbl = np.concatenate([np.array(data[9]) for data in all_processed_data])
    z_kf_isbl = np.concatenate([np.array(data[10]) for data in all_processed_data])
    y_true = np.concatenate([np.array(data[12]) for data in all_processed_data])
    z_true = np.concatenate([np.array(data[13]) for data in all_processed_data])

    # Calculate differences (subtract mean differences)
    y_kf_sf_mean = np.mean(y_kf_sf)
    z_kf_sf_mean = np.mean(z_kf_sf)
    y_kf_isbl_mean = np.mean(y_kf_isbl)
    z_kf_isbl_mean = np.mean(z_kf_isbl)
    y_r_isbl_mean = np.mean(y_r_isbl)
    z_r_isbl_mean = np.mean(z_r_isbl)
    y_true_mean = np.mean(y_true)
    z_true_mean = np.mean(z_true)

    # Calculate differences based on subtract_means flag
    if subtract_means:
        diff_y_kf_sf = y_kf_sf - y_true - (y_kf_sf_mean - y_true_mean)
        diff_z_kf_sf = z_kf_sf - z_true - (z_kf_sf_mean - z_true_mean)
        diff_y_kf_isbl = y_kf_isbl - y_true - (y_kf_isbl_mean - y_true_mean)
        diff_z_kf_isbl = z_kf_isbl - z_true - (z_kf_isbl_mean - z_true_mean)
        diff_y_r_isbl = y_r_isbl - y_true - (y_r_isbl_mean - y_true_mean)
        diff_z_r_isbl = z_r_isbl - z_true - (z_r_isbl_mean - z_true_mean)
    else:
        diff_y_kf_sf = y_kf_sf - y_true
        diff_z_kf_sf = z_kf_sf - z_true
        diff_y_kf_isbl = y_kf_isbl - y_true
        diff_z_kf_isbl = z_kf_isbl - z_true
        diff_y_r_isbl = y_r_isbl - y_true
        diff_z_r_isbl = z_r_isbl - z_true

    # Calculate statistics
    stats = [
        ('iSBL-SF KF Y', np.mean(diff_y_kf_sf), np.std(diff_y_kf_sf), np.ptp(diff_y_kf_sf)),
        ('iSBL-SF KF Z', np.mean(diff_z_kf_sf), np.std(diff_z_kf_sf), np.ptp(diff_z_kf_sf)),
        ('iSBL KF Y', np.mean(diff_y_kf_isbl), np.std(diff_y_kf_isbl), np.ptp(diff_y_kf_isbl)),
        ('iSBL KF Z', np.mean(diff_z_kf_isbl), np.std(diff_z_kf_isbl), np.ptp(diff_z_kf_isbl)),
        ('Raw iSBL Y', np.mean(diff_y_r_isbl), np.std(diff_y_r_isbl), np.ptp(diff_y_r_isbl)),
        ('Raw iSBL Z', np.mean(diff_z_r_isbl), np.std(diff_z_r_isbl), np.ptp(diff_z_r_isbl))
    ]

    # Find the best (lowest) values for each statistic
    best_values = {
        'y_mean': min(abs(stats[i][1]) for i in [0, 2, 4]),
        'y_std': min(stats[i][2] for i in [0, 2, 4]),
        'y_range': min(stats[i][3] for i in [0, 2, 4]),
        'z_mean': min(abs(stats[i][1]) for i in [1, 3, 5]),
        'z_std': min(stats[i][2] for i in [1, 3, 5]),
        'z_range': min(stats[i][3] for i in [1, 3, 5])
    }

    # Set title
    if custom_title:
        title = custom_title
    else:
        # Use the first file in file_list to generate the title
        first_file = file_list[0]
        match = re.search(r'test_x(\d+)_y(-?\d+)_z(-?\d+)_(d_g|d_r|s)', first_file)
        if match:
            x, y, z = match.group(1), match.group(2), match.group(3)
            movement = {"d_g": "Gradual Movement", "d_r": "Rapid Movement", "s": "Stationary"}[match.group(4)]
            if (movement != "Stationary") and (axis_mask != "XYZABC"):
                title = f"Y vs Z Differences, Transmitter Location: (x={x}mm, y={y}mm, z={z}mm), {movement}, Axis List: ({axis_mask})"
            else:
                title = f"Y vs Z Differences, Transmitter Location: (x={x}mm, y={y}mm, z={z}mm), {movement}"
        else:
            title = f"Y vs Z Differences"

    # Create figure with one row of subplots
    fig, axs = plt.subplots(1, 3, figsize=(18, 8))
    fig.suptitle(title, fontsize=20, y=0.89)  # Adjusted title position
    fig.text(0.43, 0.835, f"Number of datapoints per subplot: {num_datapoints}")

    # Find global min and max for consistent axes
    all_y_diffs = np.concatenate([diff_y_kf_sf, diff_y_kf_isbl, diff_y_r_isbl])
    all_z_diffs = np.concatenate([diff_z_kf_sf, diff_z_kf_isbl, diff_z_r_isbl])

    min_val = min(np.min(all_y_diffs), np.min(all_z_diffs))
    max_val = max(np.max(all_y_diffs), np.max(all_z_diffs))

    # Add a small margin (e.g., 5%) to ensure all points are visible
    margin = 0.05 * (max_val - min_val)
    axis_limit = max(abs(min_val - margin), abs(max_val + margin))

    # Plot data and add statistics
    plot_data = [
        (axs[0], diff_y_kf_sf, diff_z_kf_sf, 'iSBL-SF Kalman Filter', stats[0], stats[1], 'red'),
        (axs[1], diff_y_kf_isbl, diff_z_kf_isbl, 'iSBL Kalman Filter', stats[2], stats[3], 'green'),
        (axs[2], diff_y_r_isbl, diff_z_r_isbl, 'Raw iSBL Data', stats[4], stats[5], 'blue')
    ]

    for ax, x_diff, y_diff, title, x_stat, y_stat, color in plot_data:
        ax.scatter(x_diff, y_diff, alpha=0.5, color=color, label='Data points')
        ax.set_title(title, pad=10, fontsize=16)  # Add padding to the subplot title
        ax.set_xlabel('Y Difference (mm)', fontsize=12)
        ax.set_ylabel('Z Difference (mm)', fontsize=12)
        ax.set_xlim(-axis_limit + xoffset, axis_limit + xoffset)
        ax.set_ylim(-axis_limit + yoffset, axis_limit + yoffset)
        ax.set_xlim(-368 + xoffset, 368 + xoffset)
        ax.set_ylim(-368 + yoffset, 368 + yoffset)
        ax.set_aspect('equal', 'box')

        # Add mean point
        ax.scatter(np.mean(x_diff), np.mean(y_diff), color='black', s=100, marker='x', label='Mean')

        # Add 95% confidence ellipse
        cov = np.cov(x_diff, y_diff)
        lambda_, v = np.linalg.eig(cov)
        lambda_ = np.sqrt(lambda_)
        ell = Ellipse(xy=(np.mean(x_diff), np.mean(y_diff)),
                      width=lambda_[0] * 2 * 1.96, height=lambda_[1] * 2 * 1.96,
                      angle=np.rad2deg(np.arctan2(*v[:, 0][::-1])),
                      color='black', fill=False, label='95% CI')
        ax.add_artist(ell)

        # Add statistics to the plot
        y_texts = [
            (f"Y-Diff Mean: {x_stat[1]:.1f}mm", abs(x_stat[1]) == best_values['y_mean']),
            (f"Y-Diff StDev: {x_stat[2]:.1f}mm", x_stat[2] == best_values['y_std']),
            (f"Y-Diff Range: {x_stat[3]:.1f}mm", x_stat[3] == best_values['y_range'])
        ]

        z_texts = [
            (f"Z-Diff Mean: {y_stat[1]:.1f}mm", abs(y_stat[1]) == best_values['z_mean']),
            (f"Z-Diff StDev: {y_stat[2]:.1f}mm", y_stat[2] == best_values['z_std']),
            (f"Z-Diff Range: {y_stat[3]:.1f}mm", y_stat[3] == best_values['z_range'])
        ]

        # Add Y-diff statistics on the left
        for i, (text, is_best) in enumerate(y_texts):
            y_pos = 0.98 - 0.06 * i  # Adjusted vertical position for each text
            if is_best and i != 0:
                ax.text(0.05, y_pos, text, transform=ax.transAxes, verticalalignment='top', fontsize=12,
                        fontweight='bold')
            else:
                ax.text(0.05, y_pos, text, transform=ax.transAxes, verticalalignment='top', fontsize=12)

        # Add Z-diff statistics on the right
        for i, (text, is_best) in enumerate(z_texts):
            y_pos = 0.98 - 0.06 * i  # Adjusted vertical position for each text
            if is_best and i != 0:
                ax.text(0.55, y_pos, text, transform=ax.transAxes, verticalalignment='top', fontsize=12,
                        fontweight='bold')
            else:
                ax.text(0.55, y_pos, text, transform=ax.transAxes, verticalalignment='top', fontsize=12)

        # Add legend to each subplot
        ax.legend(loc='lower right', fontsize=10)

    # Adjust layout and display the plot
    plt.tight_layout()
    plt.subplots_adjust(top=0.90, hspace=0.4)  # Adjust top margin and increase space between subplots
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


def plot_histogram_comparison(all_processed_data, axis_mask, subtract_means=False, max_axes=False, custom_title=None, vertical_shift=0):
    # Combine data from all processed files
    y_kf_sf = np.concatenate([np.array(data[3]) for data in all_processed_data])
    z_kf_sf = np.concatenate([np.array(data[4]) for data in all_processed_data])
    y_r_isbl = np.concatenate([np.array(data[6]) for data in all_processed_data])
    z_r_isbl = np.concatenate([np.array(data[7]) for data in all_processed_data])
    y_kf_isbl = np.concatenate([np.array(data[9]) for data in all_processed_data])
    z_kf_isbl = np.concatenate([np.array(data[10]) for data in all_processed_data])
    y_true = np.concatenate([np.array(data[12]) for data in all_processed_data])
    z_true = np.concatenate([np.array(data[13]) for data in all_processed_data])

    # Calculate means
    y_kf_sf_mean = np.mean(y_kf_sf)
    z_kf_sf_mean = np.mean(z_kf_sf)
    y_kf_isbl_mean = np.mean(y_kf_isbl)
    z_kf_isbl_mean = np.mean(z_kf_isbl)
    y_r_isbl_mean = np.mean(y_r_isbl)
    z_r_isbl_mean = np.mean(z_r_isbl)
    y_true_mean = np.mean(y_true)
    z_true_mean = np.mean(z_true)

    # Calculate differences based on subtract_means flag
    if subtract_means:
        diff_y_kf_sf = y_kf_sf - y_true - (y_kf_sf_mean - y_true_mean)
        diff_z_kf_sf = z_kf_sf - z_true - (z_kf_sf_mean - z_true_mean)
        diff_y_kf_isbl = y_kf_isbl - y_true - (y_kf_isbl_mean - y_true_mean)
        diff_z_kf_isbl = z_kf_isbl - z_true - (z_kf_isbl_mean - z_true_mean)
        diff_y_r_isbl = y_r_isbl - y_true - (y_r_isbl_mean - y_true_mean)
        diff_z_r_isbl = z_r_isbl - z_true - (z_r_isbl_mean - z_true_mean)
    else:
        diff_y_kf_sf = y_kf_sf - y_true
        diff_z_kf_sf = z_kf_sf - z_true
        diff_y_kf_isbl = y_kf_isbl - y_true
        diff_z_kf_isbl = z_kf_isbl - z_true
        diff_y_r_isbl = y_r_isbl - y_true
        diff_z_r_isbl = z_r_isbl - z_true

    # Create figure with 3 rows and 2 columns of subplots
    fig, axs = plt.subplots(3, 2, figsize=(15, 22))

    # Set title
    if custom_title:
        title = custom_title
    else:
        # Use the first file in file_list to generate the title
        first_file = file_list[0]
        match = re.search(r'test_x(\d+)_y(-?\d+)_z(-?\d+)_(d_g|d_r|s)', first_file)
        if match:
            x, y, z = match.group(1), match.group(2), match.group(3)
            movement = {"d_g": "Gradual Movement", "d_r": "Rapid Movement", "s": "Stationary"}[match.group(4)]
            if (movement != "Stationary") and (axis_mask != "XYZABC"):
                title = f"Y vs Z Differences, Transmitter Location: (x={x}mm, y={y}mm, z={z}mm), {movement}, Axis List: ({axis_mask})"
            else:
                title = f"Y vs Z Differences, Transmitter Location: (x={x}mm, y={y}mm, z={z}mm), {movement}"
        else:
            title = f"Y vs Z Differences"

    fig.suptitle(title, fontsize=16, y=1)  # Moved up slightly
    fig.text(0.43, 0.97, f"Number of datapoints per subplot: {len(z_true)}")

    # Data to plot
    data_to_plot = [
        (diff_y_kf_sf, diff_z_kf_sf, 'iSBL-SF Kalman Filter'),
        (diff_y_kf_isbl, diff_z_kf_isbl, 'iSBL Kalman Filter'),
        (diff_y_r_isbl, diff_z_r_isbl, 'Raw ISBL Data')
    ]

    # Find global min and max for consistent axes
    all_diffs = np.concatenate(
        [diff_y_kf_sf, diff_z_kf_sf, diff_y_kf_isbl, diff_z_kf_isbl, diff_y_r_isbl, diff_z_r_isbl])
    global_min, global_max = np.min(all_diffs), np.max(all_diffs)

    # Plot histograms with normal distribution overlay
    for i, (y_diff, z_diff, title) in enumerate(data_to_plot):
        for j, (diff, label) in enumerate([(y_diff, 'Y'), (z_diff, 'Z')]):
            ax = axs[i, j]

            # Plot histogram
            n, bins, patches = ax.hist(diff, bins=100, density=True, alpha=0.7, edgecolor='black')

            # Calculate mean and std dev
            mean = np.mean(diff)
            std = np.std(diff)

            # Create a normal distribution curve
            xmin, xmax = ax.get_xlim()
            x = np.linspace(xmin, xmax, 100)
            p = stats.norm.pdf(x, mean, std)

            # Plot the normal distribution curve
            ax.plot(x, p, 'r-', linewidth=2)

            ax.set_title(f'{title} - {label} Differences', pad=10)
            ax.set_xlabel('Difference (mm)')
            ax.set_ylabel('Density')

            if max_axes:
                ax.set_xlim(-325, 325)
            else:
                ax.set_xlim(global_min*1.1, global_max*1.1)

            # Add mean and std dev to the plot
            ax.axvline(mean, color='r', linestyle='dashed', linewidth=2)
            ax.text(0.05, 0.95, f'Mean: {mean:.2f}\nStd Dev: {std:.2f}',
                    transform=ax.transAxes, va='top')

    plt.tight_layout()

    # Apply vertical shift
    plt.subplots_adjust(top=0.95, bottom=0.05 + vertical_shift, hspace=0.3)

    plt.show()

# ENTER FILE AND OFFSET
folder = "C:/Users/frabb/OneDrive - Cal Poly/Documents (Cloud)/0 CALPOLY/000Thesis/"
file_list = [
"test_x2051_y305_z-707_s.log",
]

offset_x = 0
offset_y = 90
num_outlier_removals = 5
axes_list = "XYZABC"
sub_meas = False

# custom_title_str = "Y vs Z Differences, All Transmitter Locations, Gradual Movement, Axis List: (XYZABC)"
# custom_title_str = "Y vs Z Differences, Transmitter Location: (x=2051mm, y=305mm, z=-707mm), Rapid and Gradual Movement"

all_processed_data, num_rows = process_multiple_files(file_list, folder, num_outlier_removals, axes_list)

plot_histogram_comparison(all_processed_data, axes_list, subtract_means=sub_meas, max_axes=False, custom_title=custom_title_str, vertical_shift=.1)

plot_comparison_data(all_processed_data, offset_x, offset_y, axes_list, num_rows, custom_title_str, subtract_means=sub_meas)
# plot_combined_comparison_data(all_processed_data, file_list, offset_x, offset_y)
