#!/usr/bin/env python3.7

# Example of Knuth_Yao algorithm， Using penalty CCP, penalty SCP and a new linearized formulation
from GRB_optimize import gurobi_dc, gurobi_scp, gurobi_bilinear, gurobi_lp
from model_check import model_check_PRISM, model_check_LE


def original_qcqp(c_lambda=3 / 20,):
    opt_vars = gurobi_bilinear(c_lambda=c_lambda)
    opt_vars = model_check_LE(opt_vars)
    print("\n======\nSolution found by solving original nonconvex QCQP problem")
    print("x: %f\ny; %f" % (opt_vars['x'], opt_vars['y']))
    print("The probability value is {}\n======\n".format(opt_vars['ps0']))
    return opt_vars


def penalty_CCP(initial_point, c_lambda=3 / 20):
    # main loop
    opt_vars = initial_point
    while True:
        # Solve penalty DC problem
        opt_vars = gurobi_dc(app_point=opt_vars, c_lambda=c_lambda, tau=10e5)
        # Model Checking
        opt_vars = model_check_LE(opt_vars)
        if opt_vars['ps0'] <= c_lambda:
            print("\n======\nSolution found by penalty CCP")
            print("x: %f\ny; %f" % (opt_vars['x'], opt_vars['y']))
            print("The probability value is {}\n======\n".format(opt_vars['ps0']))
            return opt_vars


def penalty_SCP(initial_point, c_lambda=3 / 20, gamma=1.5, initial_delta=2, omega=10e-4):
    opt_vars = initial_point  # initial guess
    delta = initial_delta
    beta = 1  # initial optimal value
    while True:
        # Solve local linearized LP problem
        opt_vars = gurobi_scp(app_point=opt_vars, c_lambda=c_lambda, delta=1 + delta, tau=10e5)

        # Model Checking
        check_vars = model_check_LE(opt_vars)

        if check_vars['ps0'] <= c_lambda:
            opt_vars = check_vars
            print("\n======\nSolution found by penalty SCP")
            print("x: %f\ny; %f" % (opt_vars['x'], opt_vars['y']))
            print("The probability value is {}\n======\n".format(opt_vars['ps0']))
            return opt_vars

        elif check_vars['ps0'] <= beta:
            opt_vars = check_vars
            beta = opt_vars['ps0']  # update new optimal value
        else:
            delta = delta / gamma  # update gamma
            if delta < omega:  # check the size of the trust region
                print("\n======\nFail to find an instantiation")
                print("x: %f\ny; %f" % (opt_vars['x'], opt_vars['y']))
                print("The probability value is {}\n======\n".format(opt_vars['ps0']))
                return None


def iteration_LP(initial_point, c_lambda=3 / 20, gamma=1.5, initial_delta=2, omega=10e-4):
    opt_vars = initial_point  # initial guess
    delta = initial_delta
    beta = 1  # initial optimal value
    while True:
        # Solve local linearized LP problem
        opt_vars = gurobi_lp(app_point=opt_vars, c_lambda=c_lambda, delta=1 + delta)

        # Model Checking
        check_vars = model_check_LE(opt_vars)

        if check_vars['ps0'] <= c_lambda:
            opt_vars = check_vars
            print("\n======\nSolution found by iteration linearization")
            print("x: %f\ny; %f" % (opt_vars['x'], opt_vars['y']))
            print("The probability value is {}\n======\n".format(opt_vars['ps0']))
            return opt_vars

        elif check_vars['ps0'] <= beta:
            opt_vars = check_vars
            beta = opt_vars['ps0']  # update new optimal value
        else:
            delta = delta / gamma  # update gamma
            if delta < omega:  # check the size of the trust region
                print("\n======\nFail to find an instantiation")
                print("x: %f\ny; %f" % (opt_vars['x'], opt_vars['y']))
                print("The probability value is {}\n======\n".format(opt_vars['ps0']))
                return None


if __name__ == '__main__':
    # initialize
    init = {'x': 0.4, 'y': 0.8, 'ps0': 0.01, 'ps1': 0.3, 'ps3': 0.9}  # initial guess
    c_lambda = 0.001
    original_qcqp(c_lambda=c_lambda)
    penalty_SCP(initial_point=init, c_lambda=c_lambda, initial_delta=10)
    penalty_CCP(initial_point=init, c_lambda=c_lambda)
    iteration_LP(initial_point=init, c_lambda=c_lambda,initial_delta=10)
