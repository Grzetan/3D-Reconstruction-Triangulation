import sys
import xmltodict
import json


class InvalidFile(Exception):
    pass


try:
    input = sys.argv[1]
    if ".xcp" not in input:
        raise InvalidFile()
    output = sys.argv[1].split(".xcp")[-2] + ".txt"
    try:
        output = sys.argv[2]
    except:
        pass
except IndexError:
    print(
        "The structure of arguments is: 'python3 convert_xcp.py path/to/input.xcp path/to/output.txt' where output path is not required"
    )
except Exception:
    print("Input file must be a XCP file")

with open(input, "r") as f:
    dict_data = xmltodict.parse(f.read())
    json_data = json.dumps(dict_data, indent=4)
    print(json_data)

with open(output, "w") as f:
    for c in dict_data["Cameras"]["Camera"]:
        f.write(f"Camera ID: {c['@DEVICEID']}\n")
        control_frame = c["ControlFrames"]["ControlFrame"]
        f.write(f"Focal Length: {control_frame["@FOCAL_LENGTH"]}\n")
        f.write(f"Orientation: {control_frame["@ORIENTATION"]}\n")
        f.write(f"Position: {control_frame["@POSITION"]}\n")
        f.write(f"Princial point: {control_frame["@PRINCIPAL_POINT"]}\n")
        f.write("\n")

