#!/usr/bin/env python3

import re

with open('chibi_2024.kicad_pcb') as f:
    lines = f.readlines()

def mangled(lines):
    for l in lines:
        if 'fp_text' in l and not l.strip().endswith('hide'):
            at_re = '\((at\s\S+\s\S+)(\s\S+)?\)'
            match = re.search(at_re, l)
            if not match:
                raise Exception()
            rot = int(match.group(2) or '0')
            rot = (rot+180)%360
            yield re.sub(at_re, r'(\1 {})'.format(rot), l)
        else:
            yield l

with open('out.kicad_pcb', 'w') as f:
    f.write(''.join(mangled(lines)))
