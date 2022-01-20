const double x = 0.39079514565065837;
const double y = 0.30303105353097465;

dtmc
module die


	// local state
	s : [0..4] init 0;
	// value of the die
	d : [0..1] init 0;

	[] s=0 -> y : (s'=1) + 1-y : (s'=2);
	[] s=1 -> x : (s'=0) + 1-x : (s'=3) & (d'=1);
        [] s=2 -> (s'=2);
	[] s=3 -> (s'=3);

endmodule

