#!/usr/bin/env python3.7
from typing import Any
import gurobipy as gp
from gurobipy import GRB


# original QCQP
def gurobi_bilinear(c_lambda=1 / 6, epsilon=10e-5):
    """
    :param c_lambda:
    :param epsilon:
    :return:
    """
    # Create a new model
    m = gp.Model("qcp")
    m.setParam("NonConvex", 2)
    m.setParam('OutputFlag', 0)

    # Create variables
    ps0 = m.addVar(name="ps0")
    ps1 = m.addVar(name="ps1")
    ps3 = m.addVar(name="ps3")  # probability variables
    ps7 = 1

    x = m.addVar(name="x")
    y = m.addVar(name="y")  # polynomial parameters

    # Set objective: ps0
    obj = 1.0 * ps0
    m.setObjective(obj, GRB.MINIMIZE)

    # Add linear constraint
    m.addConstr(x >= epsilon, "c0")
    m.addConstr(y >= epsilon, "c1")
    m.addConstr(1 - x >= epsilon, "c2")
    m.addConstr(1 - y >= epsilon, "c3")
    m.addConstr(c_lambda >= ps0, "c4")

    # Add quadratic constrains
    m.addConstr(-ps0 + x * ps1 <= 0, "qc0")
    m.addConstr(-ps1 + y * ps3 <= 0, "qc1")
    m.addConstr(-ps3 + x * ps1 + (1 - x) * ps7 <= 0, "qc2")

    m.optimize()
    opt_vars: dict[Any, Any] = {}
    for v in m.getVars():
        opt_vars[v.VarName] = v.X
    return opt_vars


# convexified DC problem
def gurobi_dc(app_point, tau,
              c_lambda=1 / 6,
              epsilon=10e-5):
    """
    :param app_point:
    :param tau: penalty coefficient
    :param c_lambda: upper bound
    :param epsilon: graphic reserve
    :return:
    """

    # approximation point
    h_x, h_y, h_ps0, h_ps1, h_ps3 = \
        (app_point['x'], app_point['y'], app_point['ps0'], app_point['ps1'], app_point['ps3'])
    h_ps7 = 1

    # Create a new model
    m = gp.Model("qcp")
    m.setParam('OutputFlag', 0)

    # Create variables
    ps0 = m.addVar(name="ps0")
    ps1 = m.addVar(name="ps1")
    ps3 = m.addVar(name="ps3")  # probability variables
    ps7 = 1

    x = m.addVar(name="x")
    y = m.addVar(name="y")  # polynomial parameters

    k0 = m.addVar(name="k0")
    k1 = m.addVar(name="k1")
    k3 = m.addVar(name="k3")  # penalty coefficient

    # Set objective: ps0
    obj = 1.0 * ps0 + tau * (k0 + k1 + k3)
    m.setObjective(obj, GRB.MINIMIZE)

    # Add linear constraint
    m.addConstr(x >= epsilon, "c0")
    m.addConstr(y >= epsilon, "c1")
    m.addConstr(1 - x >= epsilon, "c2")
    m.addConstr(1 - y >= epsilon, "c3")
    m.addConstr(c_lambda >= ps0, "c4")
    m.addConstr(k0 >= 0, "c5")
    m.addConstr(k1 >= 0, "c6")
    m.addConstr(k3 >= 0, "c7")

    # Add quadratic constrains
    m.addConstr(-k0 - ps0 + 0.5 * (x + ps1) ** 2 - 1.5 * (h_x ** 2 + h_ps1 ** 2) + h_x * x + h_ps1 * ps1 <= 0, "qc0")
    m.addConstr(-k3 - ps3 + 0.5 * (x + ps1) ** 2 - 1.5 * (h_x ** 2 + h_ps1 ** 2) + h_x * x + h_ps1 * ps1
                + 0.5 * (1 - x + ps7) ** 2 - 1.5 * ((1 - h_x) ** 2 + h_ps7 ** 2) + (1 - h_x) * (
                        1 - x) + h_ps7 * ps7 <= 0, "qc1")
    m.addConstr(-k1 - ps1 + 0.5 * (y + ps3) ** 2 - 1.5 * (h_y ** 2 + h_ps3 ** 2) + h_y * y + h_ps3 * ps3 <= 0, "qc2")

    m.optimize()
    opt_vars: dict[Any, Any] = {}
    for v in m.getVars():
        opt_vars[v.VarName] = v.X
    return opt_vars


# Convexified LP programming
def gurobi_scp(app_point, tau,
               c_lambda=1 / 6,
               delta=2,
               epsilon=10e-5):
    """
    :param app_point: approximation point
    :param tau: penalty coefficient
    :param c_lambda: reachability specification $lambda$
    :param delta: trust region coefficient
    :param epsilon: graph preserving constant
    :return:
    """
    # approximation point
    h_x, h_y, h_ps0, h_ps1, h_ps3 = \
        (app_point['x'], app_point['y'], app_point['ps0'], app_point['ps1'], app_point['ps3'])

    # Create a new model
    m = gp.Model("lp")
    m.setParam('OutputFlag', 0)

    # Create variables
    ps0 = m.addVar(name="ps0")
    ps1 = m.addVar(name="ps1")
    ps3 = m.addVar(name="ps3")  # probability variables
    ps7 = 1

    x = m.addVar(name="x")
    y = m.addVar(name="y")  # polynomial parameters

    k0 = m.addVar(name="k0")
    k1 = m.addVar(name="k1")
    k3 = m.addVar(name="k3")  # penalty coefficient

    # Set objective: ps0
    obj = 1.0 * ps0 + tau * (k0 + k1 + k3)
    m.setObjective(obj, GRB.MINIMIZE)

    # Add linear constraint
    m.addConstr(x >= epsilon, "c0")
    m.addConstr(y >= epsilon, "c1")
    m.addConstr(1 - x >= epsilon, "c2")
    m.addConstr(1 - y >= epsilon, "c3")
    m.addConstr(c_lambda >= ps0, "c4")
    m.addConstr(k0 >= 0, "c5")
    m.addConstr(k1 >= 0, "c6")
    m.addConstr(k3 >= 0, "c7")

    # Add approximation constrains
    m.addConstr(-k0 - ps0 + h_x * ps1 + h_ps1 * x - h_x * h_ps1 <= 0, "c8")
    m.addConstr(-k1 - ps1 + h_y * ps3 + h_ps3 * y - h_y * h_ps1 <= 0, "c9")
    m.addConstr(-k3 - ps3 + h_x * ps1 + h_ps1 * x - h_x * h_ps1 + 1 - x <= 0, "c9")

    # Trust region constrains
    m.addConstr(ps0 - h_ps0 * delta <= 0, "t0")
    m.addConstr(-ps0 + h_ps0 / delta <= 0, "t1")
    m.addConstr(ps1 - h_ps1 * delta <= 0, "t2")
    m.addConstr(-ps1 + h_ps1 / delta <= 0, "t3")
    m.addConstr(ps3 - h_ps3 * delta <= 0, "t4")
    m.addConstr(-ps3 + h_ps3 / delta <= 0, "t5")
    m.addConstr(x - h_x * delta <= 0, "t6")
    m.addConstr(-x + x / delta <= 0, "t7")
    m.addConstr(y - h_y * delta <= 0, "t8")
    m.addConstr(-y + h_y / delta <= 0, "t9")

    m.optimize()
    opt_vars: dict[Any, Any] = {}
    for v in m.getVars():
        opt_vars[v.VarName] = v.X
    return opt_vars


# iteration LP
def gurobi_lp(app_point, delta=2, c_lambda=1 / 6, epsilon=10e-5):
    # approximation point
    h_x, h_y, h_ps0, h_ps1, h_ps3 = \
        (app_point['x'], app_point['y'], app_point['ps0'], app_point['ps1'], app_point['ps3'])

    # Create a new model
    m = gp.Model("lp")
    m.setParam("NonConvex", 2)
    m.setParam('OutputFlag', 0)

    # Create variables
    ps0 = m.addVar(name="ps0")
    ps1 = m.addVar(name="ps1")
    ps3 = m.addVar(name="ps3")  # probability variables
    ps7 = 1

    x = m.addVar(name="x")
    y = m.addVar(name="y")  # polynomial parameters

    # Set objective: ps0
    obj = 1.0 * ps0
    m.setObjective(obj, GRB.MINIMIZE)

    # Add linear constraint
    m.addConstr(x >= epsilon, "c0")
    m.addConstr(y >= epsilon, "c1")
    m.addConstr(1 - x >= epsilon, "c2")
    m.addConstr(1 - y >= epsilon, "c3")
    m.addConstr(c_lambda >= ps0, "c4")

    # linear approximation
    m.addConstr(-ps0 + x * h_ps1 <= 0, "a0")
    m.addConstr(-ps1 + y * h_ps3 <= 0, "a1")
    m.addConstr(-ps3 + x * h_ps1 + (1 - x) * ps7 <= 0, "a2")

    # Trust region constrains
    m.addConstr(ps0 - h_ps0 * delta <= 0, "t0")
    m.addConstr(-ps0 + h_ps0 / delta <= 0, "t1")
    m.addConstr(ps1 - h_ps1 * delta <= 0, "t2")
    m.addConstr(-ps1 + h_ps1 / delta <= 0, "t3")
    m.addConstr(ps3 - h_ps3 * delta <= 0, "t4")
    m.addConstr(-ps3 + h_ps3 / delta <= 0, "t5")
    m.addConstr(x - h_x * delta <= 0, "t6")
    m.addConstr(-x + x / delta <= 0, "t7")
    m.addConstr(y - h_y * delta <= 0, "t8")
    m.addConstr(-y + h_y / delta <= 0, "t9")

    m.optimize()
    opt_vars: dict[Any, Any] = {}
    for v in m.getVars():
        opt_vars[v.VarName] = v.X
    return opt_vars


if __name__ == '__main__':
    init = {'x': 0.5, 'y': 0.5, 'ps0': 0.3, 'ps1': 0.5, 'ps3': 0.5}
    opt_vars = init
    for i in range(100):
        opt_vars = gurobi_lp(opt_vars)
    print(opt_vars)
