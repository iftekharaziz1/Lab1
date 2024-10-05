import bagpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np

# Function to convert bag file data to CSV
def to_csv(bag):
    csv_files = []
    for t in bag.topics:
        data = bag.message_by_topic(t)
        csv_files.append(data)
    return pd.read_csv(csv_files[0])

def process_gps_data(df):
    df['utm_easting_offset'] = df['utm_easting'] - df['utm_easting'].iloc[0]
    df['utm_northing_offset'] = df['utm_northing'] - df['utm_northing'].iloc[0]
    return df

def calculate_centroid(df):
    centroid_easting = df['utm_easting_offset'].mean()
    centroid_northing = df['utm_northing_offset'].mean()
    df['deviation_easting'] = df['utm_easting_offset'] - centroid_easting
    df['deviation_northing'] = df['utm_northing_offset'] - centroid_northing
    df['euclidean_distance'] = np.sqrt(df['deviation_easting']**2 + df['deviation_northing']**2)
    return (centroid_easting, centroid_northing), df  # Return centroid as tuple and df

# plotting function
def plot_northing_vs_easting(df_open, df_occ, centroid_open, centroid_occ, title):
    fig, ax = plt.subplots()
    
    ax.scatter(df_open['deviation_easting'], df_open['deviation_northing'], label='Open', marker='o', s=10)
    ax.scatter(df_occ['deviation_easting'], df_occ['deviation_northing'], label='Occluded', marker='x', s=10)
   
    ax.scatter(centroid_open[0], centroid_open[1], label='Centroid Open', marker='D', color='blue')
    ax.scatter(centroid_occ[0], centroid_occ[1], label='Centroid Occluded', marker='D', color='red')

    ax.text(centroid_open[0], centroid_open[1], f'Centroid Open: ({centroid_open[0]:.2f}, {centroid_open[1]:.2f})', 
            color='blue', fontsize=10, verticalalignment='bottom')
    ax.text(centroid_occ[0], centroid_occ[1], f'Centroid Occluded: ({centroid_occ[0]:.2f}, {centroid_occ[1]:.2f})', 
            color='red', fontsize=10, verticalalignment='top')
    
    # Titles and labels
    plt.title(f'{title} Northing vs Easting Scatterplot')
    plt.xlabel('Deviation Easting (m)')
    plt.ylabel('Deviation Northing (m)')
    plt.legend()
    plt.show()

def plot_altitude_vs_time(df_open, df_occ, title):
    fig, ax = plt.subplots()

    # Plot open & occluded data with lines and markers
    ax.plot(df_open['Time'], df_open['altitude'], label='Open', marker='o', markersize=5, linestyle='-', color='blue')
    ax.plot(df_occ['Time'], df_occ['altitude'], label='Occluded', marker='x', markersize=5, linestyle='-', color='red')

    plt.title(f'{title} Altitude vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Altitude (m)')
    plt.legend()
    plt.show()


# plot histograms for Euclidean distances
def plot_histogram(df, title):
    plt.figure()
    plt.hist(df['euclidean_distance'], bins=30, color='blue', alpha=0.7)
    plt.title(f'Histogram of Euclidean Distance - {title}')
    plt.xlabel('Euclidean Distance (m)')
    plt.ylabel('Frequency')
    plt.show()

# Stationary Data
st_open = bagreader('/home/iftekhar/catkin_ws/src/EECE5554/Lab1/gnss/Data/open_2.bag')
st_occ = bagreader('/home/iftekhar/catkin_ws/src/EECE5554/Lab1/gnss/Data/occluded_2.bag')

df_open_st = process_gps_data(to_csv(st_open))
df_occ_st = process_gps_data(to_csv(st_occ))

centroid_open_st, df_open_st = calculate_centroid(df_open_st)
centroid_occ_st, df_occ_st = calculate_centroid(df_occ_st)

# Stationary plots
plot_northing_vs_easting(df_open_st, df_occ_st, centroid_open_st, centroid_occ_st, 'Stationary')
plot_altitude_vs_time(df_open_st, df_occ_st, 'Stationary')

# Plot histograms of Euclidean distances
plot_histogram(df_open_st, 'Open Stationary Data')
plot_histogram(df_occ_st, 'Occluded Stationary Data')


# Moving Data
mov_open = bagreader('/home/iftekhar/catkin_ws/src/EECE5554/Lab1/gnss/Data/walking_2.bag')
mov_occ = bagreader('/home/iftekhar/catkin_ws/src/EECE5554/Lab1/gnss/Data/occluded_2.bag')

df_open_mov = process_gps_data(to_csv(mov_open))
df_occ_mov = process_gps_data(to_csv(mov_occ))

centroid_open_mov, df_open_mov = calculate_centroid(df_open_mov)
centroid_occ_mov, df_occ_mov = calculate_centroid(df_occ_mov)

# Moving plots
plot_northing_vs_easting(df_open_mov, df_occ_mov, centroid_open_mov, centroid_occ_mov, 'Moving')
plot_altitude_vs_time(df_open_mov, df_occ_mov, 'Moving')


