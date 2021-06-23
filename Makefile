n?=1

smt:
	cbmc rv32.c --smt2 --outfile model.smt2 -DCYCLES=$(n)
	sed '/^(get-value/d' model.smt2 > model.smt2.tmp
	sed '/^(check-sat/d' model.smt2.tmp > model.smt2.tmp.2
	mkdir -p smt2
	python3 regify.py model.smt2.tmp.2 > smt2/rv32-$(n).smt2

example:
	cbmc rv32.c --smt2 --outfile example1.smt2 -DCYCLES=$(n)
	awk '/\(get-value \(\|nondet/;!/\(get-value \(\|/' example1.smt2 > example2.smt2
	mkdir -p smt2
	python3 example.py example2.smt2 > smt2/rv32-$(n)-example.smt2 

run: example
	z3 smt2/rv32-$(n)-example.smt2

clean:
	rm -rf model.smt2.tmp model.smt2 model.smt2.tmp.2 example*.smt2 smt2/
