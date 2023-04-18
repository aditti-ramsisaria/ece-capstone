import pandas as pd
import matplotlib.pyplot as plt

sampling_freq = 10

df = pd.read_csv("scentbot_multi.csv")

# drop zero readings
df.drop(df[(df['ens_tvoc'] <= 0.0) | (df['ens_co2'] <= 0.0)].index, inplace=True)

sm = pd.plotting.scatter_matrix(df, figsize=(12, 12))
plt.show()

print("len of original dataset: ", len(df))

# normalization
df["gm_voc_v"] = (df["gm_voc_v"] - df["gm_voc_v"].min()) / (df["gm_voc_v"].max() - df["gm_voc_v"].min())
df["gm_co_v"] = (df["gm_co_v"] - df["gm_co_v"].min()) / (df["gm_co_v"].max() - df["gm_co_v"].min())
df["gm_no2_v"] = (df["gm_no2_v"] - df["gm_no2_v"].min()) / (df["gm_no2_v"].max() - df["gm_no2_v"].min())
df["gm_eth_v"] = (df["gm_eth_v"] - df["gm_eth_v"].min()) / (df["gm_eth_v"].max() - df["gm_eth_v"].min())

df["ens_tvoc"] = (df["ens_tvoc"] - df["ens_tvoc"].min()) / (df["ens_tvoc"].max() - df["ens_tvoc"].min())
df["ens_co2"] = (df["ens_co2"] - df["ens_co2"].min()) / (df["ens_co2"].max() - df["ens_co2"].min())

df["bme_temp"] = (df["bme_temp"] - df["bme_temp"].min()) / (df["bme_temp"].max() - df["bme_temp"].min())
df["bme_pressure"] = (df["bme_pressure"] - df["bme_pressure"].min()) / (df["bme_pressure"].max() - df["bme_pressure"].min())
df["bme_altitude"] = (df["bme_altitude"] - df["bme_altitude"].min()) / (df["bme_altitude"].max() - df["bme_altitude"].min())
df["bme_humidity"] = (df["bme_humidity"] - df["bme_humidity"].min()) / (df["bme_humidity"].max() - df["bme_humidity"].min())

new_df = pd.DataFrame(columns=["g_voc_min", "g_voc_max", "g_voc_std", "g_voc_rms", "g_voc_avg",
                               "g_eth_min", "g_eth_max", "g_eth_std", "g_eth_rms", "g_eth_avg",
                               "g_no2_min", "g_no2_max", "g_no2_std", "g_no2_rms", "g_no2_avg",
                               "g_co_min", "g_co_max", "g_co_std", "g_co_rms", "g_co_avg",
                               "ens_tvoc_min", "ens_tvoc_max", "ens_tvoc_std", "ens_tvoc_rms", "ens_tvoc_avg",
                               "ens_co2_min", "ens_co2_max", "ens_co2_std", "ens_co2_rms", "ens_co2_avg",
                               "temp_min", "temp_max", "temp_std", "temp_rms", "temp_avg",
                               "pressure_min", "pressure_max", "pressure_std", "pressure_rms", "pressure_avg",
                               "humidity_min", "humidity_max", "humidity_std", "humidity_rms", "humidity_avg",
                               "altitude_min", "altitude_max", "altitude_std", "altitude_rms", "altitude_avg", "target"])

for i in range(0, len(df) // sampling_freq):
    j = i * 10
    subset = df[j : j + sampling_freq]

    if (subset.target.nunique() == 1):

        # for grove
        new_df.at[i, "g_voc_min"] = subset["gm_voc_v"].min()
        new_df.at[i, "g_voc_max"] = subset["gm_voc_v"].max()
        new_df.at[i, "g_voc_std"] = subset["gm_voc_v"].std()
        new_df.at[i, "g_voc_rms"] = ((subset["gm_voc_v"]**2).sum())**0.5
        new_df.at[i, "g_voc_avg"] = subset["gm_voc_v"].mean()

        new_df.at[i, "g_eth_min"] = subset["gm_eth_v"].min()
        new_df.at[i, "g_eth_max"] = subset["gm_eth_v"].max()
        new_df.at[i, "g_eth_std"] = subset["gm_eth_v"].std()
        new_df.at[i, "g_eth_rms"] = ((subset["gm_eth_v"]**2).sum())**0.5
        new_df.at[i, "g_eth_avg"] = subset["gm_eth_v"].mean()

        new_df.at[i, "g_no2_min"] = subset["gm_no2_v"].min()
        new_df.at[i, "g_no2_max"] = subset["gm_no2_v"].max()
        new_df.at[i, "g_no2_std"] = subset["gm_no2_v"].std()
        new_df.at[i, "g_no2_rms"] = ((subset["gm_no2_v"]**2).sum())**0.5
        new_df.at[i, "g_no2_avg"] = subset["gm_no2_v"].mean()

        new_df.at[i, "g_co_min"] = subset["gm_co_v"].min()
        new_df.at[i, "g_co_max"] = subset["gm_co_v"].max()
        new_df.at[i, "g_co_std"] = subset["gm_co_v"].std()
        new_df.at[i, "g_co_rms"] = ((subset["gm_co_v"]**2).sum())**0.5
        new_df.at[i, "g_co_avg"] = subset["gm_co_v"].mean()

        # for ens
        new_df.at[i, "ens_tvoc_min"] = subset["ens_tvoc"].min()
        new_df.at[i, "ens_tvoc_max"] = subset["ens_tvoc"].max()
        new_df.at[i, "ens_tvoc_std"] = subset["ens_tvoc"].std()
        new_df.at[i, "ens_tvoc_rms"] = ((subset["ens_tvoc"]**2).sum())**0.5
        new_df.at[i, "ens_tvoc_avg"] = subset["ens_tvoc"].mean()

        new_df.at[i, "ens_co2_min"] = subset["ens_co2"].min()
        new_df.at[i, "ens_co2_max"] = subset["ens_co2"].max()
        new_df.at[i, "ens_co2_std"] = subset["ens_co2"].std()
        new_df.at[i, "ens_co2_rms"] = ((subset["ens_co2"]**2).sum())**0.5
        new_df.at[i, "ens_co2_avg"] = subset["ens_co2"].mean()

        # for bme
        new_df.at[i, "temp_min"] = subset["bme_temp"].min()
        new_df.at[i, "temp_max"] = subset["bme_temp"].max()
        new_df.at[i, "temp_std"] = subset["bme_temp"].std()
        new_df.at[i, "temp_rms"] = ((subset["bme_temp"]**2).sum())**0.5
        new_df.at[i, "temp_avg"] = subset["bme_temp"].mean()

        new_df.at[i, "pressure_min"] = subset["bme_pressure"].min()
        new_df.at[i, "pressure_max"] = subset["bme_pressure"].max()
        new_df.at[i, "pressure_std"] = subset["bme_pressure"].std()
        new_df.at[i, "pressure_rms"] = ((subset["bme_pressure"]**2).sum())**0.5
        new_df.at[i, "pressure_avg"] = subset["bme_pressure"].mean()

        new_df.at[i, "humidity_min"] = subset["bme_humidity"].min()
        new_df.at[i, "humidity_max"] = subset["bme_humidity"].max()
        new_df.at[i, "humidity_std"] = subset["bme_humidity"].std()
        new_df.at[i, "humidity_rms"] = ((subset["bme_humidity"]**2).sum())**0.5
        new_df.at[i, "humidity_avg"] = subset["bme_humidity"].mean()

        new_df.at[i, "altitude_min"] = subset["bme_altitude"].min()
        new_df.at[i, "altitude_max"] = subset["bme_altitude"].max()
        new_df.at[i, "altitude_std"] = subset["bme_altitude"].std()
        new_df.at[i, "altitude_rms"] = ((subset["bme_altitude"]**2).sum())**0.5
        new_df.at[i, "altitude_avg"] = subset["bme_altitude"].mean()

        new_df.at[i, "target"] = subset["target"].mean()

print(len(new_df))
new_df.to_csv("preprocessed_multi.csv")




