#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 17 10:25:21 2023

@author: adam
"""

# -----------------------------------------------------------------------------
# Load librarires
# -----------------------------------------------------------------------------
 
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import matplotlib.dates as mdates
import cartopy.crs as ccrs
import cartopy.feature as cfeature
import pyproj
import seaborn as sns
from cartopy.mpl.ticker import (LongitudeFormatter, 
                                LatitudeFormatter,
                                LongitudeLocator,
                                LatitudeLocator)
import xarray as xr

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
# Mapping attributes
# -----------------------------------------------------------------------------

# Add Natural Earth coastline
coast = cfeature.NaturalEarthFeature("physical", "land", "10m",
                                     edgecolor="black",
                                     facecolor="lightgray",
                                     lw=0.75)

# Add Natural Earth coastline
coastline = cfeature.NaturalEarthFeature("physical", "coastline", "10m",
                                         edgecolor="black",
                                         facecolor="none",
                                         lw=0.75)

# -----------------------------------------------------------------------------
# Folder paths
# -----------------------------------------------------------------------------

# Data directory
path_data = "/Users/adam/Documents/GitHub/cryologger-ice-tracking-beacon/Software/Python/"

# Figure directory
path_figures = "/Users/adam/Documents/GitHub/cryologger-ice-tracking-beacon/Software/Python/Figures/"


# -----------------------------------------------------------------------------
# Load and clean data exported from MariaDB
# -----------------------------------------------------------------------------

# Load most recent output file exported directly from MariaDB
df = pd.read_csv(path_data + "2022_itb_milne_fiord.csv", index_col=False)

df = pd.read_csv("/Users/adam/Downloads/cryologger_itb.csv", index_col=False)

# Convert unixtime to datetime
df["datetime"] = pd.to_datetime(df["unixtime"], unit="s")

# Remove datetime outliers
df = df[df["datetime"] > "2018-01-01 00:00:00"]

# Remove latitude outliers
df["latitude"].values[df["latitude"] <= 0] = np.nan
df["latitude"].values[df["latitude"] >= 90] = np.nan

# Convert IMEI to string
df["imei"] = df["imei"].astype(str)

# Clean temperature anomalys
df["temperature"].values[df["temperature"] > 50] = np.nan
df["temperature"].values[df["temperature"] == 0.0] = np.nan

# Convert pressure to float
df["pressure"] = df["pressure"].replace({",":""},regex=True).apply(pd.to_numeric,1)
df["pressure"] = df["pressure"] / 10 # Convert to kPa


# Set zero values to NaN
df["transmit_duration"].replace(0, np.nan, inplace=True)

# Sort data by earliest datetime first
df.sort_values(by="datetime", ascending = True, inplace=True)

# Set hue order
hue_order = ["300434063298940",
             "300434063392080"]

# -----------------------------------------------------------------------------
# Optional: Download and clean data exported from WordPress
# -----------------------------------------------------------------------------

# Load most recent output file downloaded from WordPress
df = pd.read_csv(path_data + "2022_itb_grise_fiord.csv", index_col=False)

df = pd.read_csv("/Users/adam/Downloads/2022_itb_grise_fiord.csv", index_col=False)

df = pd.read_csv("/Users/adam/Downloads/2023_itb_amundsen.csv", index_col=False)


# Convert unixtime datetime
df["datetime"] = pd.to_datetime(df["unixtime"].astype(str), format="%Y-%m-%d %H:%M:%S")

# Convert IMEI to string
df["imei"] = df["imei"].astype(str)

# Set zero values to NaN
df["transmit_duration"].replace(0, np.nan, inplace=True)

# -----------------------------------------------------------------------------
# Subset data
# -----------------------------------------------------------------------------

# Subset by IMEI
df = df[df["imei"].isin(["300434063298940"])]

# Subset by datetime
df = df[(df["datetime"] > "2023-09-01 00:00")]


# Convert pressure to float
df["pressure_int"] = df["pressure_int"].replace({",":""},regex=True).apply(pd.to_numeric,1)
df["pressure_int"] = df["pressure_int"] / 10 # Convert to kPa

# Concat dataset
df0 = pd.concat([df1, df2])

# Export CSV file
df0.to_csv(path_data + "2021_cleaned.csv", index=False)

# -----------------------------------------------------------------------------
# Calculate speed and distance and reproject data to UTM
# -----------------------------------------------------------------------------

# Calculate drift distance and speed
df1 = calculate_drift(df1)
df2 = calculate_drift(df2)

# Concat dataset
df0 = pd.concat([df1, df2])

# Plot maps on distance axis
plot_distance(df1, 1)
plot_distance(df2, 2)

# -----------------------------------------------------------------------------
# Plots
# -----------------------------------------------------------------------------

# Temperature
fig, ax = plt.subplots(figsize=(10,5))
ax.grid(ls="dotted")
sns.lineplot(x="datetime", y="temperature_int", data=df, errorbar=None, lw=lw, hue="uid")
ax.set(xlabel=None, ylabel="Temperature (°C)")
plt.xticks(rotation=45, horizontalalignment="right")
ax.xaxis.set_major_formatter(mdates.DateFormatter(date_format))
#ax.xaxis.set_major_locator(mdates.MonthLocator(interval=interval)) 
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="UID")
fig.savefig(path_figures + "01_temperature.png", dpi=dpi, transparent=False, bbox_inches="tight")

# Humidity
fig, ax = plt.subplots(figsize=(10,5))
ax.grid(ls="dotted")
sns.lineplot(x="datetime", y="humidity_int", data=df, errorbar=None, lw=lw, hue="uid")
ax.set(xlabel=None, ylabel="Humidity (%)")
plt.xticks(rotation=45, horizontalalignment="right")
ax.xaxis.set_major_formatter(mdates.DateFormatter(date_format))
#ax.xaxis.set_major_locator(mdates.MonthLocator(interval=interval)) 
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="UID")
fig.savefig(path_figures + "02_humidity.png", dpi=dpi, transparent=False, bbox_inches="tight")


# Pressure
fig, ax = plt.subplots(figsize=(10,5))
ax.grid(ls="dotted")
sns.lineplot(x="datetime", y="pressure_int", data=df, errorbar=None, lw=lw, hue="uid")
ax.set(xlabel=None, ylabel="Pressure (kPa)")
plt.xticks(rotation=45, horizontalalignment="right")
ax.xaxis.set_major_formatter(mdates.DateFormatter(date_format))
#ax.xaxis.set_major_locator(mdates.MonthLocator(interval=interval)) 
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="UID")
fig.savefig(path_figures + "03_pressure.png", dpi=dpi, transparent=False, bbox_inches="tight")

# Pitch
fig, ax = plt.subplots(figsize=(10,5))
sns.lineplot(x="datetime", y="pitch", data=df, ci=None, lw=lw, hue="uid")
ax.set(xlabel=None, ylabel="Pitch (°)")
ax.grid(ls="dotted")
ax.xaxis.set_major_formatter(mdates.DateFormatter(date_format))
#ax.xaxis.set_major_locator(mdates.MonthLocator(interval=interval)) 
plt.xticks(rotation=45, horizontalalignment="right")
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="UID")
fig.savefig(path_figures + "04_pitch.png", dpi=dpi, transparent=False, bbox_inches="tight")

# Roll
fig, ax = plt.subplots(figsize=(10,5))
sns.lineplot(x="datetime", y="roll", data=df, ci=None, lw=lw, hue="uid")
ax.set(xlabel="Datetime", ylabel="Roll (°)")
ax.grid(ls="dotted")
ax.xaxis.set_major_formatter(mdates.DateFormatter(date_format))
#ax.xaxis.set_major_locator(mdates.MonthLocator(interval=interval)) 
plt.xticks(rotation=45, horizontalalignment="right")
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="UID")
fig.savefig(path_figures + "05_roll.png", dpi=dpi, transparent=False, bbox_inches="tight")

# Heading
fig, ax = plt.subplots(figsize=(10,5))
sns.scatterplot(x="datetime", y="heading", data=df,  hue="uid")
ax.set(xlabel="Datetime", ylabel="Heading (°)")
ax.grid(ls="dotted")
ax.xaxis.set_major_formatter(mdates.DateFormatter(date_format))
#ax.xaxis.set_major_locator(mdates.MonthLocator(interval=interval)) 
plt.xticks(rotation=45, horizontalalignment="right")
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="IMEI")
fig.savefig(path_figures + "06_heading.png", dpi=dpi, transparent=False, bbox_inches="tight")

# Satellites
fig, ax = plt.subplots(figsize=(10,5))
sns.lineplot(x="datetime", y="satellites", data=df, ci=None, lw=lw, hue="uid")
ax.set(xlabel="Datetime", ylabel="Satellites")
ax.grid(ls="dotted")
ax.xaxis.set_major_formatter(mdates.DateFormatter(date_format))
#ax.xaxis.set_major_locator(mdates.MonthLocator(interval=interval)) 
plt.xticks(rotation=45, horizontalalignment="right")
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="UID")
fig.savefig(path_figures + "07_satellites.png", dpi=dpi, transparent=False, bbox_inches="tight")

# Voltage
fig, ax = plt.subplots(figsize=(10,5))
sns.lineplot(x="datetime", y="voltage", data=df, ci=None, lw=lw, hue="uid")
ax.set(xlabel=None, ylabel="Voltage (V)") #, ylim=(7,7.3)
ax.grid(ls="dotted")
ax.xaxis.set_major_formatter(mdates.DateFormatter(date_format))
#ax.xaxis.set_major_locator(mdates.MonthLocator(interval=interval)) 
plt.xticks(rotation=45, horizontalalignment="right")
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="UID")
fig.savefig(path_figures + "08_voltage.png", dpi=dpi, transparent=False, bbox_inches="tight")

# Transmit Duration
fig, ax = plt.subplots(figsize=(10,5))
sns.scatterplot(x="datetime", y="transmit_duration", data=df, hue="uid")
ax.set(xlabel=None, ylabel="Transmit Duration (s)",)
ax.grid(ls="dotted")
ax.xaxis.set_major_formatter(mdates.DateFormatter(date_format))
#ax.xaxis.set_major_locator(mdates.MonthLocator(interval=interval)) 
plt.xticks(rotation=45, horizontalalignment="right")
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="UID")
fig.savefig(path_figures + "09_transmit_duration.png", dpi=dpi, transparent=False, bbox_inches="tight")

# Counter
fig, ax = plt.subplots(figsize=(10,5))
ax.grid(ls="dotted")
sns.lineplot(x="datetime", y="message_counter", data=df, errorbar=None, lw=lw, hue="uid")
ax.set(xlabel=None, ylabel="Message Counter")
plt.xticks(rotation=45, horizontalalignment="right")
ax.xaxis.set_major_formatter(mdates.DateFormatter(date_format))
#ax.xaxis.set_major_locator(mdates.MonthLocator(interval=interval)) 
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="UID")
fig.savefig(path_figures + "10_counter.png", dpi=dpi, transparent=False, bbox_inches="tight")



# -----------------------------------------------------------------------------
# Map of 2021 Cryologger deployments
# -----------------------------------------------------------------------------

# Load deployment
df = pd.read_csv("/Users/adam/Google Drive/Fieldwork/2021 - Amundsen/Cryologger ITB/Deployments/deployments_2021.csv", index_col=False)

# Convert from int64 to object
df["imei_short"] = df["imei_short"].astype(str)
df["beacon"] = df["beacon"].astype(str)
df["year"] = df["year"].astype(str)

# Convert to datetime
df["deployment"] = pd.to_datetime(df["deployment"].astype(str), format="%Y-%m-%d")

# Set extents for 2021 deployments (zoom)
x1 = -80
x2 = -57
y1 = 65
y2 = 79.5

# Cryospheric
x1 = -85
x2 = -70
y1 = 72.5
y2 = 79.5

# Set colour palette
sns.set_palette("turbo", 2)

# Plot map
plt.figure(figsize=(10,10))
ax = plt.axes(projection=ccrs.LambertConformal((0.5 * (x1 + x2)), (0.5 * (y1 + y2)))) # Centre of extents
ax.set_extent([x1, x2, y1, y2])
ax.add_feature(coast)
gl = ax.gridlines(crs=ccrs.PlateCarree(), draw_labels=True,
                  color="black", alpha=0.25, linestyle="dotted",
                  x_inline=False, y_inline=False)
gl.top_labels = False
gl.right_labels = False
gl.rotate_labels = False
gl.xlocator = mticker.FixedLocator(np.arange(-100,-50,5))
gl.ylocator = mticker.FixedLocator(np.arange(60,90,1))
gl.xformatter = LongitudeFormatter()
gl.yformatter = LatitudeFormatter()
gl.xpadding=10
sns.scatterplot(x="longitude", y="latitude", hue="beacon",
                markers=True, marker="o",
                data=df, ci=None, zorder=10, 
                s=500, linewidth=1, edgecolor="black", legend="full",
                transform=ccrs.PlateCarree())
ax.legend(loc=1, title="Beacon")

# Add scale bar
scale_bar(ax, 100, (0.8, 0.025), 3)

# Save figure
plt.savefig(path_figures + "01_deployment_map_2021_cryosphereic.png", dpi=dpi, transparent=False, bbox_inches="tight")

                      
# -----------------------------------------------------------------------------
# Map of all beacon positions
# -----------------------------------------------------------------------------

# Set extents for subset
x1 = -67.5
x2 = -50
y1 = 61.5
y2 = 70.5

# Set colour palette
sns.set_palette("turbo", 10)

# Plot map
plt.figure(figsize=(10,10))
ax = plt.axes(projection=ccrs.LambertConformal((0.5 * (x1 + x2)), (0.5 * (y1 + y2)))) # Centre of extents
#ax.set_extent([x1, x2, y1, y2])
ax.add_feature(coast)
gl = ax.gridlines(crs=ccrs.PlateCarree(), draw_labels=True,
                  color="black", alpha=0.25, linestyle="dotted",
                  x_inline=False, y_inline=False)
gl.rotate_labels = False
gl.top_labels = False
gl.right_labels = False
gl.xlocator = mticker.FixedLocator(np.arange(-80,-50,2))
gl.ylocator = mticker.FixedLocator(np.arange(60,80,1))
gl.xformatter = LongitudeFormatter()
gl.yformatter = LatitudeFormatter()
gl.xpadding=5

sns.scatterplot(x="longitude", y="latitude", hue="imei",
                data=df, ci=None, s=50, linewidth=0,
                legend="full",
                transform=ccrs.PlateCarree())

ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="IMEI")

plt.savefig(path_figures + "01_map.png", dpi=dpi, transparent=False, bbox_inches="tight")



# -----------------------------------------------------------------------------
# Map of Cryologger deployments
# -----------------------------------------------------------------------------

# Load deployment
df = pd.read_csv("/Users/adam/Google Drive/Fieldwork/2021 - Amundsen/Cryologger ITB/Deployments/deployments.csv", index_col=False)

# Convert from int64 to object
df["imei_short"] = df["imei_short"].astype(str)
df["year"] = df["year"].astype(str)

# Convert to datetime
df["deployment"] = pd.to_datetime(df["deployment"].astype(str), format="%Y-%m-%d")

# Set extents
x1 = -86
x2 = -50
y1 = 60
y2 = 83.5

# Set colour palette
#sns.set_palette(["cyan","lime","magenta"])
sns.set_palette("turbo", 3)

# Plot map
plt.figure(figsize=(10,10))
ax = plt.axes(projection=ccrs.LambertConformal((0.5 * (x1 + x2)), (0.5 * (y1 + y2)))) # Centre of extents
ax.set_extent([x1, x2, y1, y2])
ax.add_feature(coast)
gl = ax.gridlines(crs=ccrs.PlateCarree(), draw_labels=True,
                  color="black", alpha=0.25, linestyle="dotted",
                  x_inline=False, y_inline=False)
gl.top_labels = False
gl.right_labels = False
gl.rotate_labels = False
gl.xlocator = mticker.FixedLocator(np.arange(-180,180,10))
gl.ylocator = mticker.FixedLocator(np.arange(40,90,5))
gl.xformatter = LongitudeFormatter()
gl.yformatter = LatitudeFormatter()
gl.xpadding=10
sns.scatterplot(x="longitude", y="latitude", hue="year",
                data=df, ci=None, zorder=10, 
                s=250, linewidth=1, edgecolor="black",legend="full",
                transform=ccrs.PlateCarree())
ax.legend(loc=3, title="Year")

# Save figure
plt.savefig(path_figures + "01_map_deployments_all.png", dpi=dpi, transparent=False, bbox_inches="tight")


    