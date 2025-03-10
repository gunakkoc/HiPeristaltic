#this script (kind of) generates the corresponding commands to be used in Python using the Arduino code.
with open("HiPeristaltic.ino") as file:
    lines = [line.strip() for line in file]

arduino_func_map = []
arduino_func_map.append("void (*cmd_fnc_lst[])() = {")

python_func_map = []

i = 0
cmd_ind = -1 #number of fixed commands
while (i < len(lines)):
    l = lines[i]
    if l.startswith("void get_") or l.startswith("void set_"):
        params = l.split(" ")
        func_name = params[1].replace("(){","").strip(" ")
        i = i + 1
        l2 = lines[i]
        params = l2.split("=")
        if ("unsigned long" in l2) or ("uint32_t" in l2):
            arg_type = "np.uint32"
        else:
            arg_type = "np.uint8"
        cmd_ind = cmd_ind + 1
        arduino_func_map.append('  &' + func_name + ',')
        python_func_map.append(f"_cmd_map['{func_name}'] = CommandStructure(cmd_ind={cmd_ind}, var_type={arg_type})")
    i = i + 1

with open('cmds_python.txt', 'w') as f:
    for l in python_func_map:
        f.write(f"{l}\n")


arduino_func_map.append('};')
with open('cmds_arduino.txt', 'w') as f:
    for l in arduino_func_map:
        f.write(f"{l}\n")