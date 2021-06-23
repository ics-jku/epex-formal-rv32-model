#!/bin/python3
import sys

DELIML = '%'
DELIMR = DELIML

if len(sys.argv) > 1:
    name = sys.argv[1]
    with open(name, 'r') as f:
        txt = f.read()
        txt = txt.replace(f'bv1101', f'bv0')
        txt = txt.replace(f'bv1102', f'bv4')

        for r in range(1, 33):
            if r == 2:
                txt = txt.replace(f'bv22{r:02d}', f'bv0')
                txt = txt.replace(f'bv44{r:02d}', f'bv7')
            else:
                txt = txt.replace(f'bv22{r:02d}', f'bv0')
                txt = txt.replace(f'bv44{r:02d}', f'bv0')

        print(txt)
