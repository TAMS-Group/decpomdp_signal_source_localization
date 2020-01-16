import subprocess
import re
def main():
    cmd = ["iwlist", "scanning"]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    points = proc.stdout.read()
    CellArray = points.split("Cell")
    Output = []
    for cell in CellArray:
        cellinfo = cell.split()
        cellinfo = list(filter(
            lambda x: re.match(r"ESSID*", x) or re.match(r"level*", x),
            cellinfo
        ))
        Output.append(cellinfo)
    return Output


if __name__ == '__main__':
    value = main()
    print(value)
    input("Press Enter to continue...")
