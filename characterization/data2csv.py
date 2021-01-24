import json

filename = "characterization-data20210117-2330.json"

with open(filename) as file:
    data = json.load(file)

print("Loaded:")
for key in data.keys():
    if isinstance(data[key], list):
        print("  %s: [%d records]" % (key, len(data[key])))
    else:
        print("  %s: %s" % (key, data[key]))

with open("slow-forward.csv", "w") as file:
    for rec in data["slow-forward"]:
        file.write("%s,%s,%s,%s,%s,%s,%s,%s\n" % (rec[0], rec[2], rec[3], rec[4], rec[5], rec[6], rec[7], rec[8]))
