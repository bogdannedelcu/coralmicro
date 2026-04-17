#!/usr/bin/env python3
"""Generate a .cc file with help.txt embedded in SDRAM section."""
import sys

def main():
    infile, outfile = sys.argv[1], sys.argv[2]
    data = open(infile, 'rb').read()
    with open(outfile, 'w') as f:
        f.write('// Auto-generated from help.txt — do not edit\n')
        f.write('extern "C" __attribute__((section(".help_data")))\n')
        f.write('const unsigned char help_txt_data[] = {\n')
        for i in range(0, len(data), 16):
            chunk = data[i:i+16]
            f.write('  ' + ', '.join('0x%02x' % b for b in chunk) + ',\n')
        f.write('};\n')
        f.write('extern "C" const unsigned int help_txt_data_len = %u;\n' % len(data))

if __name__ == '__main__':
    main()
