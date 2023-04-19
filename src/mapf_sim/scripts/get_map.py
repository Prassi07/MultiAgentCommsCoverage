#!/usr/bin/python3
import argparse
import glob
from pathlib import Path


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    f.close()
    return my_map, rows, columns

if __name__ == '__main__':
    
    my_map, map_size, _ = import_mapf_instance('../misc/small_map.txt')
    x_offset = -map_size / 2
    y_offset = -map_size / 2
    resolution = 1.0
    
    id = 1
    for i in range(map_size):
        for j in range(map_size):
            if (my_map[i][j]):
                x = (i + 1 + x_offset) * resolution
                y = (j + 1 + y_offset) * resolution
                if abs(x) > 3 or abs(y) > 3:
                    print(" - {{ id: {}, x: {},  y: {},  width: 1.0, length: 1.0, height: 20.0 }}".format(id, x, y))
                id += 1
    