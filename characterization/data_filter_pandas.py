import json

import pandas as pd
import matplotlib.pyplot as plt

# Name associated with each test run
DATASETS = ["slow-forward", "slow-backward", "fast-forward", "fast-backward"]
COLUMNS = ["TIME", "BATTERY", "AUTOSPEED", "L_VOLTS", "R_VOLTS", "L_ENCODER_P", "R_ENCODER_P", "L_ENCODER_V", "R_ENCODER_V", "GYRO_ANGLE"]

filename = "characterization-data20210117-2330.json"

# Load the data from the JSON file
with open(filename) as file:
    data = json.load(file)

frames = {}
for dataset in DATASETS:
    print(f"Dataset: {dataset}")
    frames[dataset] = pd.DataFrame(data[dataset], columns=COLUMNS)
    frames[dataset]["dLP"] = frames[dataset]["L_ENCODER_P"].diff()
    frames[dataset]["dRP"] = frames[dataset]["R_ENCODER_P"].diff()
    frames[dataset] = frames[dataset].query('abs(dLP) > 0.0001 and abs(dRP) > 0.0001')
    frames[dataset]["L_ENCODER_V"] = frames[dataset].L_ENCODER_V.rolling(window=3, center=True).median()
    frames[dataset]["R_ENCODER_V"] = frames[dataset].R_ENCODER_V.rolling(window=3, center=True).median()
    frames[dataset] = frames[dataset].iloc[10:].iloc[:-1]
    frames[dataset] = frames[dataset].drop("dLP", axis=1).drop("dRP", axis=1)
    data[dataset] = frames[dataset].to_dict(orient="split")["data"]
    print(frames[dataset].head(2).to_string())

    # # Create GUI plots
    # df = frames[dataset]
    # # plot = df.plot.scatter(x="TIME", y="L_ENCODER_V", label="V")
    # # df.plot.scatter(x="TIME", y="L_ENCODER_V2", ax=plot, label="V2", color="Red")
    # plot = df.plot.scatter(x="TIME", y="L_ENCODER_V", label="Original R", color="LightBlue")
    # df.plot.scatter(x="TIME", y="L_ENCODER_V2", ax=plot, label="Clean L")
    # df.plot.scatter(x="TIME", y="R_ENCODER_V", ax=plot, label="Original R", color="Pink")
    # df.plot.scatter(x="TIME", y="R_ENCODER_V2", ax=plot, label="Clean R", color="Red")
    # plot.set_xlabel("Time")
    # plot.set_ylabel("Velocity")
    # plot.set_title(dataset)

with open("out.json", "w") as f:
    f.write(json.dumps(data, indent=2))

plt.show()
