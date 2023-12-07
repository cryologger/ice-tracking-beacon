#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Aug 16 11:36:36 2023

@author: adam
"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import matplotlib.dates as mdates
import numpy as np
import seaborn as sns
import pyproj

# -----------------------------------------------------------------------------
# Plotting attributes
# -----------------------------------------------------------------------------

# Seaborn configuration
sns.set_theme(style="ticks")
sns.set_context("talk") # talk, paper, poster

# Set colour palette
sns.set_palette("colorblind")

# Graph attributes
lw = 1
interval = 30
date_format = "%Y-%m-%d"

# Figure DPI
dpi = 300

# -----------------------------------------------------------------------------
# Folder paths
# -----------------------------------------------------------------------------

path = "/Users/adam/Library/CloudStorage/GoogleDrive-adam.garbo@gmail.com/My Drive/Cryologger/Publications/Garbo & Mueller/"

# Data directory
path_data = "/Users/adam/Library/CloudStorage/GoogleDrive-adam.garbo@gmail.com/My Drive/Cryologger/Publications/Garbo & Mueller/Data/

# Figure directory
path_figures = "/Users/adam/Library/CloudStorage/GoogleDrive-adam.garbo@gmail.com/My Drive/Cryologger/Publications/Garbo & Mueller/Figures/"


# -----------------------------------------------------------------------------
# Load data
# -----------------------------------------------------------------------------

# Load most recent output file exported directly from MariaDB
df = pd.read_csv("/Users/adam/Downloads/2018_itb_amundsen.csv", index_col=False)


# -----------------------------------------------------------------------------
# Prepare data
# -----------------------------------------------------------------------------

# Convert unixtime to datetime
df["datetime"] = pd.to_datetime(df["unixtime"], unit="s")

# Convert IMEI to string
df["imei"] = df["imei"].astype(str)

# -----------------------------------------------------------------------------
# Subset data according to deployment time of beacon
# -----------------------------------------------------------------------------

# Beacon: 
df1 = df[df["imei"].isin(["300434063418130"])]



# -----------------------------------------------------------------------------
# Calculate velocities
# -----------------------------------------------------------------------------

# Ensure rows are sorted by datetime. Uses datetime_data first and datatime_transmit
df.sort_values(by="datetime", inplace=True)

# Initialize pyproj with appropriate ellipsoid
geodesic = pyproj.Geod(ellps="WGS84")

# Calculate forward azimuth and great circle distance between modelled coordinates
df["direction"], back_azimuth, df["distance"] = geodesic.inv(
    df["longitude"].shift().tolist(),
    df["latitude"].shift().tolist(),
    df["longitude"].tolist(),
    df["latitude"].tolist(),
)

# Convert azimuth from (-180° to 180°) to (0° to 360°)
df["direction"] = (df["direction"] + 360) % 360

# Calculate time delta between rows (in seconds)
df["time_delta"] = df["datetime"].diff().dt.total_seconds()

# Calculate speed in m/s
df["speed"] = df["distance"] / df["time_delta"]




# -----------------------------------------------------------------------------
# Plot velocities
# -----------------------------------------------------------------------------




# Transmit Duration
fig, ax = plt.subplots(figsize=(10,5))
ax.grid(ls="dotted")
sns.scatterplot(x="datetime", y="speed", data=df)
ax.set(xlabel=None, ylabel="Speed (m/s)")
plt.xticks(rotation=45, horizontalalignment="right")
ax.xaxis.set_major_formatter(mdates.DateFormatter(date_format))
#ax.xaxis.set_major_locator(mdates.DayLocator(interval=interval))
#ax.xaxis.set_major_locator(mdates.MonthLocator(interval=interval)) 
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="Station")
plt.savefig(path_figures + "10_transmit_duration.png", dpi=dpi, transparent=False, bbox_inches="tight")












