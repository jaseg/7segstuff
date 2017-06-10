#!/usr/bin/env python3

from matplotlib import pyplot as plt

f, ax = plt.subplots(1, figsize=(3, 8))
bar_width = 1

ax.bar([-bar_width/2], top, bottom=bottom, width=bar_width, label='foo')
ax.set_xticks([0], [filename])
ax.set_ylabel('Memory usage (B)')
ax.set_xlabel('')

ax.set_xlim([-bar_width/2, bar_width/2])
ax.set_ylim([0, mem_max])

if __name__ == '__main__':
    import argparse
    import mapparse
    parser = argparse.ArgumentParser(description='Visualize program memory usage using GCC map file')
    parser.add_argument('mapfile', type=argparse.FileType('r'), description='Input GCC .map file')
    args = parser.parse_args()

    mapping = mapparse.MapFile(args.mapfile.read())
    mapping.

