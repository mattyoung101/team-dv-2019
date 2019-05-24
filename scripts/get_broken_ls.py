broken_string = "111111111111111100111111111111001111111100110000"

for i, sensor in enumerate(broken_string):
    if sensor == "0":
        print("Broken sensor:", str(i))