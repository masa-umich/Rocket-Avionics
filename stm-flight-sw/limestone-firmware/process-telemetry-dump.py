# NOT DONE YET

import base64

with open("telemetry.bin", "r") as f:
    i = 0
    for line in f:
        if(i > 20):
            break
        i += 1
        packet = line.strip()
        raw_lmp = base64.b64decode(packet)
        print(raw_lmp.hex())