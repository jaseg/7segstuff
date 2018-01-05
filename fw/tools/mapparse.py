
import re
from collections import defaultdict, namedtuple

Section = namedtuple('Section', ['name', 'offset', 'objects'])
ObjectEntry = namedtuple('ObjectEntry', ['filename', 'object', 'offset', 'size'])
FileEntry = namedtuple('FileEntry', ['section', 'object', 'offset', 'length'])

class Memory:
    def __init__(self, name, origin, length, attrs=''):
        self.name, self.origin, self.length, self.attrs = name, origin, length, attrs
        self.sections = {}
        self.files = defaultdict(lambda: [])
        self.totals = defaultdict(lambda: 0)

    def add_toplevel(self, name, offx, length):
        self.sections[name] = Section(offx, length, [])

    def add_obj(self, name, offx, length, fn, obj):
        base_section, sep, subsec = name[1:].partition('.')
        base_section = '.'+base_section
        if base_section in self.sections:
            sec = secname, secoffx, secobjs = self.sections[base_section]
            secobjs.append(ObjectEntry(fn, obj, offx, length))
        else:
            sec = None
        self.files[fn].append(FileEntry(sec, obj, offx, length))
        self.totals[fn] += length

class MapFile:
    def __init__(self, s):
        self._lines = s.splitlines()
        self.memcfg = {}
        self.defaultmem = Memory('default', 0, 0xffffffffffffffff)
        self._parse()

    def __getitem__(self, offx_or_name):
        ''' Lookup a memory area by name or address '''
        if offx_or_name in self.memcfg:
            return self.memcfg[offx_or_name]

        elif isinstance(offx_or_name, int):
            for mem in self.memcfg.values():
                if mem.origin <= offx_or_name < mem.origin+mem.length:
                    return mem
            else:
                return self.defaultmem

        raise ValueError('Invalid argument type for indexing')

    def _skip(self, regex):
        matcher = re.compile(regex)
        for l in self:
            if matcher.match(l):
                break

    def __iter__(self):
        while self._lines:
            yield self._lines.pop(0)

    def _parse(self):
        self._skip('^Memory Configuration')

        # Parse memory segmentation info
        self._skip('^Name')
        for l in self:
            if not l:
                break
            name, origin, length, *attrs = l.split()
            if not name.startswith('*'):
                self.memcfg[name] = Memory(name, int(origin, 16), int(length, 16), attrs[0] if attrs else '')

        # Parse section information
        toplevel_m = re.compile('^(\.[a-zA-Z0-9_.]+)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)')
        secondlevel_m = re.compile('^ (\.[a-zA-Z0-9_.]+)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\s+(.*)$')
        secondlevel_linebreak_m = re.compile('^ (\.[a-zA-Z0-9_.]+)\n')
        filelike = re.compile('^(/?[^()]*\.[a-zA-Z0-9-_]+)(\(.*\))?')
        linebreak_section = None
        for l in self:
            # Toplevel section
            match = toplevel_m.match(l)
            if match:
                name, offx, length = match.groups()
                offx, length = int(offx, 16), int(length, 16)
                self[offx].add_toplevel(name, offx, length)

            match = secondlevel_linebreak_m.match(l)
            if match:
                linebreak_section, = match.groups()
                continue

            if linebreak_section:
                l = ' {} {}'.format(linebreak_section, l)
                linebreak_section = None

            # Second-level section
            match = secondlevel_m.match(l)
            if match:
                name, offx, length, misc = match.groups()
                match = filelike.match(misc)
                if match:
                    fn, obj = match.groups()
                    obj = obj.strip('()') if obj else None
                    offx, length = int(offx, 16), int(length, 16)
                    self[offx].add_obj(name, offx, length, fn, obj)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Parser GCC map file')
    parser.add_argument('mapfile', type=argparse.FileType('r'), help='The GCC .map file to parse')
    parser.add_argument('-m', '--memory', type=str, help='The memory segments to print, comma-separated')
    args = parser.parse_args()
    mf = MapFile(args.mapfile.read())
    args.mapfile.close()

    mems = args.memory.split(',') if args.memory else mf.memcfg.keys()

    for name in mems:
        mem = mf.memcfg[name]
        print('Symbols by file for memory', name)
        for tot, fn in reversed(sorted( (tot, fn) for fn, tot in mem.totals.items() )):
            print('    {:>8} {}'.format(tot, fn))
            for length, offx, sec, obj in reversed(sorted(( (length, offx, sec, obj) for sec, obj, offx, length in
                mem.files[fn] ), key=lambda e: e[0] )):
                name = sec.name if sec else None
                print('       {:>8} {:>#08x} {}'.format(length, offx, obj))
        #print('{:>16} 0x{:016x} 0x{:016x} ({:>24}) {}'.format(name, origin, length, length, attrs))

