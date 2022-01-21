# README



## Prequisite

`python 3.8` (tested)

`Gurobi 9.5` with python package `gurobipy` installed

`PRISIM 4.6` (optional)



## Description

### Knuth_Yao_pMC.py

The main running file `Knuth_Yao_pMC.py` offers four different methods to solve parameter synthesis for the Knuth -Yao algorithm pMC model. (see `reprot.pdf`)

* Function `original_qcqp` solves the original nonconvex QCQP problem using a nonlinear Gurobi optimizer.
* Function `penalty_ccp` uses the convex-concave procedure to iteratively solve the convexified QCQP problem.
* Function `penalty_scp` uses sequential convex programming to iteratively solve an approximated LP problem.
* Function `iteration_LP` substitutes parts of the probability variables with results obtained by model checking, and iteratively solves the approximated LP problem.

### GRB_optimize.py

The file `GRB_optimize.py` contains the formulation of the subproblem in each iteration, which are solved by the Gurobi optimizer (e.g. LP, QCP, nonlinear optimizer).

### model_check.py

The file model_check.py offers two methods for probabilistic model checking.

The function model_check_PRISM uses the probabilistic model checker **PRISM** for model checking. If `PRISM 4.6` is not installed, this function is not recommended.

And the default function model_check_LE check the probability by solving a linear equation system (see `report.pdf`).

