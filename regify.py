#!/bin/python3
import sys

DELIML = '%'
DELIMR = DELIML

if len(sys.argv) > 1:
    name = sys.argv[1]
    with open(name, 'r') as f:
        txt = f.read()
        txt = txt.replace(f'bv1101', f'bv{DELIML}65{DELIMR}')
        txt = txt.replace(f'bv1102', f'bv{DELIML}66{DELIMR}')

        for r in range(1, 33):
            txt = txt.replace(f'bv22{r:02d}', f'bv{DELIML}{r}{DELIMR}')
            txt = txt.replace(f'bv44{r:02d}', f'bv{DELIML}{r+32}{DELIMR}')

        print(txt)
