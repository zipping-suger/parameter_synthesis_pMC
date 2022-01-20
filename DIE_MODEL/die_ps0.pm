const double x = 0.39079514565065837;
const double y = 0.30303105353097465;

dtmc
module die


	// local state
	s : [0..7] init 0;
	// value of the die
	d : [0..6] init 0;

	[] s=0 -> x : (s'=1) + 1-x : (s'=2);
	[] s=1 -> y : (s'=3) + 1-y : (s'=4);
	[] s=2 -> y : (s'=5) + 1-y : (s'=6);
	[] s=3 -> x : (s'=1) + 1-x : (s'=7) & (d'=1);
	[] s=4 -> 1-x : (s'=7) & (d'=2) + x : (s'=7) & (d'=3);
        [] s=5 -> x : (s'=2) + 1-x : (s'=7) & (d'=4);
	[] s=6 -> 1-x : (s'=7) & (d'=5) + x : (s'=7) & (d'=6);
	[] s=7 -> (s'=7);

endmodule

