import re

input_path = "t21_debug_log.txt"
output_path = "log_filtered.txt"

with open(input_path, "r") as fin, open(output_path, "w") as fout:
    prev_send = None
    prev_send_geom = None

    for line in fin:
        if line.startswith("SEND"):
            prev_send = line
            match = re.findall(r"[-+]?\d*\.\d+|\d+", line)
            prev_send_geom = match[-1] if match else None
        elif line.startswith("RECV") and prev_send is not None:
            match = re.findall(r"[-+]?\d*\.\d+|\d+", line)
            recv_geom = match[-1] if match else None
            if prev_send_geom is not None and recv_geom is not None:
                try:
                    if abs(float(prev_send_geom) - float(recv_geom)) < 1:
                        prev_send = None
                        continue
                except Exception:
                    pass
            fout.write(prev_send)
            fout.write(line)
            prev_send = None
        else:
            fout.write(line)
            prev_send = None
