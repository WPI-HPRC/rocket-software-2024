import math
import csv

pb = 101325
Tb = 288.15
Lb = -0.0065
hb = 0
R = 8.31432
g0 = 9.80665
M = 0.0289644

def altToPressure(alt: float) -> float:
    pressure_Pa = math.pow((alt - hb) / (Tb / Lb) + 1, 1.0 / (-R * Lb / (g0 * M))) * pb

    return pressure_Pa / 100.0

# for testing only
def pressureToAlt(pressure_mBar: float) -> float:
    pressure_Pa = pressure_mBar * 100

    return hb + (Tb / Lb) * (math.pow((pressure_Pa / pb), (-R * Lb / (g0 * M))) - 1)

with open("irec2023-flash.csv", newline="") as fd:
    reader = csv.DictReader(fd)
    for row in reader:
        print(f'{float(row["AccelX"]) / 100},{float(row["AccelY"]) / 100},{float(row["AccelZ"]) / 100},{float(row["GyroX"]) / 10},{float(row["GyroY"]) / 10},{float(row["GyroZ"]) / 10},0,0,0,{altToPressure(float(row["Altitude"]))},{float(row["Temperature"])},0,0,0,0,0,0,0,0,0,0,{row["Timestamp"]}')
