#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Sep  8 17:58:05 2023

@author: adam
"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import matplotlib.dates as mdates
import numpy as np
import seaborn as sns
import requests

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

path = "/Users/adam/Documents/GitHub/cryologger-ice-tracking-beacon/Software/Python/"

# Data directory
path_data = "/Users/adam/Documents/GitHub/cryologger-ice-tracking-beacon/Software/Python"

# Figure directory
path_figures = "/Users/adam/Documents/GitHub/cryologger-ice-tracking-beacon/Software/Python/Figures/"


# -----------------------------------------------------------------------------
# Load data
# -----------------------------------------------------------------------------

# API key
headers = {'x-api-key': "xxx"}

# Station UID
url = "https://api.cryologger.org/itb?uid=LGO&records=1000"
response = requests.get(url, headers=headers)
df = pd.read_json(response.text)

# View content
#response.content

# Output file
#df.to_csv(path_data + "oeg.csv")

# -----------------------------------------------------------------------------
# Prepare data
# -----------------------------------------------------------------------------

# Convert unixtime to datetime
df["datetime"] = pd.to_datetime(df["unixtime"], unit="s")

# Remove 0 s transmit durations
#df.loc[df["transmit_duration"] == 0, "transmit_duration"] = np.nan

# Remove voltages less than 12 V
#df.loc[df["voltage"] < 12.5, "voltage"] = np.nan

# -----------------------------------------------------------------------------
# Subset data
# -----------------------------------------------------------------------------

# By datetime
df = df[(df["datetime"] > "2023-09-01 00:00")]

df = df[(df["unixtime"] >= 1685126041)]


# -----------------------------------------------------------------------------
# Plot
# -----------------------------------------------------------------------------

date_format = "%Y-%m-%d"

interval = 7

# Temperature Internal (°C)
fig, ax = plt.subplots(figsize=(10,5))
ax.grid(ls="dotted")
sns.lineplot(x="datetime", y="temperature_int", data=df, errorbar=None, lw=lw, label="Internal")
ax.set(xlabel=None, ylabel="Temperature (°C)")
plt.xticks(rotation=45, horizontalalignment="right")
ax.xaxis.set_major_formatter(mdates.DateFormatter(date_format))
#ax.xaxis.set_major_locator(mdates.DayLocator(interval=interval))
#ax.xaxis.set_major_locator(mdates.MonthLocator(interval=interval)) 
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="Station")
plt.savefig(path_figures + "01_temperature.png", dpi=dpi, transparent=False, bbox_inches="tight")

# Humidity (%)
fig, ax = plt.subplots(figsize=(10,5))
ax.grid(ls="dotted")
sns.lineplot(x="datetime", y="humidity_int", data=df, errorbar=None, lw=lw, label="Internal")
ax.set(xlabel=None, ylabel="Humidity (%)", ylim=(0,100))
plt.xticks(rotation=45, horizontalalignment="right")
ax.xaxis.set_major_formatter(mdates.DateFormatter(date_format))
#ax.xaxis.set_major_locator(mdates.DayLocator(interval=interval))
#ax.xaxis.set_major_locator(mdates.MonthLocator(interval=interval)) 
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="Station")
plt.savefig(path_figures + "02_humidity.png", dpi=dpi, transparent=False, bbox_inches="tight")

# Wind Speed
fig, ax = plt.subplots(figsize=(10,5))
ax.grid(ls="dotted")
sns.lineplot(x="datetime", y="wind_speed", data=df, errorbar=None, lw=lw, hue="uid")
ax.set(xlabel=None, ylabel="Wind Speed (km/h)")
plt.xticks(rotation=45, horizontalalignment="right")
ax.xaxis.set_major_formatter(mdates.DateFormatter(date_format))
#ax.xaxis.set_major_locator(mdates.DayLocator(interval=interval)) 
#ax.xaxis.set_major_locator(mdates.MonthLocator(interval=interval)) 
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="Station")
plt.savefig(path_figures + "03_wind_speed.png", dpi=dpi, transparent=False, bbox_inches="tight")

# Wind Gust Speed 
fig, ax = plt.subplots(figsize=(10,5))
ax.grid(ls="dotted")
sns.lineplot(x="datetime", y="wind_gust_speed", data=df, errorbar=None, lw=lw, hue="uid")
ax.set(xlabel=None, ylabel="Wind Gust Speed (km/h)")
plt.xticks(rotation=45, horizontalalignment="right")
ax.xaxis.set_major_formatter(mdates.DateFormatter(date_format))
#ax.xaxis.set_major_locator(mdates.DayLocator(interval=interval)) 
#ax.xaxis.set_major_locator(mdates.MonthLocator(interval=interval)) 
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="Station")
plt.savefig(path_figures + "04_wind_gust_speed.png", dpi=dpi, transparent=False, bbox_inches="tight")

# Wind Direction (°)
fig, ax = plt.subplots(figsize=(10,5))
ax.grid(ls="dotted")
sns.lineplot(x="datetime", y="wind_direction", data=df, errorbar=None, lw=lw, hue="uid")
ax.set(xlabel=None, ylabel="Wind Direction (°)")
plt.xticks(rotation=45, horizontalalignment="right")
ax.xaxis.set_major_formatter(mdates.DateFormatter(date_format))
#ax.xaxis.set_major_locator(mdates.DayLocator(interval=interval)) 
#ax.xaxis.set_major_locator(mdates.MonthLocator(interval=interval)) 
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="Station")
plt.savefig(path_figures + "05_wind_direction.png", dpi=dpi, transparent=False, bbox_inches="tight")

# Pitch (°)
fig, ax = plt.subplots(figsize=(10,5))
ax.grid(ls="dotted")
sns.lineplot(x="datetime", y="pitch", data=df, errorbar=None, lw=lw, hue="uid")
ax.set(xlabel=None, ylabel="Pitch (°)")
plt.xticks(rotation=45, horizontalalignment="right")
ax.xaxis.set_major_formatter(mdates.DateFormatter(date_format))
#ax.xaxis.set_major_locator(mdates.DayLocator(interval=interval))
#ax.xaxis.set_major_locator(mdates.MonthLocator(interval=interval)) 
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="Station")
plt.savefig(path_figures + "06_pitch.png", dpi=dpi, transparent=False, bbox_inches="tight")

# Roll (°)
fig, ax = plt.subplots(figsize=(10,5))
ax.grid(ls="dotted")
sns.lineplot(x="datetime", y="roll", data=df, errorbar=None, lw=lw, hue="uid")
ax.set(xlabel=None, ylabel="Roll (°)")
plt.xticks(rotation=45, horizontalalignment="right")
ax.xaxis.set_major_formatter(mdates.DateFormatter(date_format))
#ax.xaxis.set_major_locator(mdates.DayLocator(interval=interval))
#ax.xaxis.set_major_locator(mdates.MonthLocator(interval=interval)) 
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="Station")
plt.savefig(path_figures + "07_roll.png", dpi=dpi, transparent=False, bbox_inches="tight")

# Solar ()
fig, ax = plt.subplots(figsize=(10,5))
ax.grid(ls="dotted")
sns.lineplot(x="datetime", y="solar", data=df, errorbar=None, lw=lw, hue="uid")
ax.set(xlabel=None, ylabel="Solar (°)")
plt.xticks(rotation=45, horizontalalignment="right")
ax.xaxis.set_major_formatter(mdates.DateFormatter(date_format))
#ax.xaxis.set_major_locator(mdates.DayLocator(interval=interval))
#ax.xaxis.set_major_locator(mdates.MonthLocator(interval=interval)) 
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="Station")
plt.savefig(path_figures + "08_solar.png", dpi=dpi, transparent=False, bbox_inches="tight")

# Snow ()
fig, ax = plt.subplots(figsize=(10,5))
ax.grid(ls="dotted")
sns.lineplot(x="datetime", y="snow_depth", data=df, errorbar=None, lw=lw, hue="uid")
ax.set(xlabel=None, ylabel="Snow depth (mm)")
plt.xticks(rotation=45, horizontalalignment="right")
ax.xaxis.set_major_formatter(mdates.DateFormatter(date_format))
ax.xaxis.set_major_locator(mdates.DayLocator(interval=interval))
#ax.xaxis.set_major_locator(mdates.MonthLocator(interval=interval)) 
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="Station")
plt.savefig(path_figures + "09_snow_fon.png", dpi=dpi, transparent=False, bbox_inches="tight")


# Transmit Duration
fig, ax = plt.subplots(figsize=(10,5))
ax.grid(ls="dotted")
sns.scatterplot(x="datetime", y="transmit_duration", data=df, hue="uid")
ax.set(xlabel=None, ylabel="Transmit Duration (s)", ylim=(-15,255))
plt.xticks(rotation=45, horizontalalignment="right")
ax.xaxis.set_major_formatter(mdates.DateFormatter(date_format))
#ax.xaxis.set_major_locator(mdates.DayLocator(interval=interval))
#ax.xaxis.set_major_locator(mdates.MonthLocator(interval=interval)) 
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="Station")
plt.savefig(path_figures + "10_transmit_duration.png", dpi=dpi, transparent=False, bbox_inches="tight")

# Voltage
fig, ax = plt.subplots(figsize=(10,5))
ax.grid(ls="dotted")
sns.lineplot(x="datetime", y="voltage", data=df, errorbar=None, lw=lw, hue="uid")
ax.set(xlabel=None, ylabel="Voltage Min (V)")
plt.xticks(rotation=45, horizontalalignment="center")
ax.xaxis.set_major_formatter(mdates.DateFormatter(date_format))
#ax.xaxis.set_major_locator(mdates.DayLocator(interval=interval))
#ax.xaxis.set_major_locator(mdates.MonthLocator(interval=interval)) 
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="Station")
plt.savefig(path_figures + "11_voltage.png", dpi=dpi, transparent=False, bbox_inches="tight")
