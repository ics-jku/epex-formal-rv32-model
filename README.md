## epex-model
This project contains a formal 32-bit RISC-V model for program synthesis at the instruction level.
Program synthesis is performed by first specifying the initial (pre) register values and the target (post) register values
ans then solving using an smt solver. If a program that satisfies these conditions exists it 

# Usage
To generate an smt2 description for *i* unrolled instructions run `make smt n=i`.
In the resulting smt2 description the *pre* and *post* registers are replaced by *boost format* placeholders using
the following naming scheme:
```
pre x0 = %1%
pre x1 = %2%
...
pre x31 = %32%

post x0 = %33%
post x1 = %34%
...

pre pc = %65%
post pc = %66%
```

The naming scheme can be modified in `regify.py`.

To obtain the generated instructions read the `nondet*` variables where * is an integer i > 0 representing the ith instruction.
`nondet0` always contains the initial program counter value.

## Example
To generate and run an smt2 instance with exemplary *pre* and *post* states run `make run n=i`.
The exemplary *pre* and *post* state can be modified in `example.py`.

## CBMC
The model supports CBMC version 5.11.

# Reference
If you use or find this project helpful, please cite the following paper:

```
@InProceedings{KG:2021,
    author        = {Lucas Klemmer and Daniel Gro{\ss}e},
    title         = {{EPEX:} Processor Verification by Equivalent Program Execution},
    booktitle     = {ACM Great Lakes Symposium on VLSI},
    year          = 2021
}
```

The paper is available at: https://www.ics.jku.at/files/2021GLSVLSI_EPEX.pdf


# Acknowledgements
We thank Vladimir Herdt for many interesting discussions and valuable advices.
