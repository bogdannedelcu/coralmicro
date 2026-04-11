#!/usr/bin/env python3
"""Generate C header with embedded file data."""
import sys

def main():
    infile, outfile, varname = sys.argv[1], sys.argv[2], sys.argv[3]
    data = open(infile, 'rb').read()
    with open(outfile, 'w') as f:
        f.write('// Auto-generated from %s - do not edit\n' % infile.split('/')[-1])
        f.write('#pragma once\n')
        f.write('static const unsigned char %s[] = {\n' % varname)
        for i in range(0, len(data), 16):
            chunk = data[i:i+16]
            f.write('  ' + ', '.join('0x%02x' % b for b in chunk) + ',\n')
        f.write('};\n')
        f.write('static const unsigned int %s_len = %u;\n' % (varname, len(data)))

if __name__ == '__main__':
    main()
