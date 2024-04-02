import random
import sys

if len(sys.argv) < 2:
    print("Please provide a keyframes csv file")
    exit(1)

keyframes = open(sys.argv[1], "r")
keyframes = list(map(lambda str: str[:-1].split(","), keyframes.readlines()))


# for i in keyframes:
#     print(i)

time = 0

while time <= int(keyframes[-1][-1]):
    index = -1;
    lerp = 0
    for i in range(3, len(keyframes) - 1):
        if int(keyframes[i][-1]) <= time and int(keyframes[i+1][-1]) > time:
            index = i
            lerp = (time - int(keyframes[i][-1])) / (int(keyframes[i+1][-1]) - int(keyframes[i][-1]))


    if index == -1:
        time += 25
        continue

    timestep = []
    for i in range(len(keyframes[0])-1):
        timestep.append(float(keyframes[index][i]) + lerp * (float(keyframes[index + 1][i]) - float(keyframes[index][i])));
        timestep[i] += (random.random()* 2 -1) * float(keyframes[2][i])

        match(keyframes[1][i]):
            case 'float':
                pass
            case 'int':
                timestep[i] = int(timestep[i])
            case 'bool':
                timestep[i] = int(timestep[i] > 0.5)

    timestep.append(time)
            
                
    print(",".join(map(str, timestep)))
    time += 25
