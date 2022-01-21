from subprocess import Popen, PIPE
import numpy as np
import re


def update_par(x, y):
    '''
    :param x: parameter x
    :param y: parameter y
    :return: None
    '''
    for name in ['ps0', 'ps1', 'ps3']:
        with open("./DIE_MODEL/die_{}.pm".format(name), "r") as f:
            data = f.readlines()
            data[0] = "const double x = {};\n".format(x)
            data[1] = "const double y = {};\n".format(y)

        with open("./DIE_MODEL/die_{}.pm".format(name), "w") as f:
            f.writelines(data)
    return None


# get estimated probability value
def get_P_value(node_name):
    '''
    :param node_name:
    :return: estimated probability value
    '''
    ps0_check = Popen(["./PRISM/prism-4.6-src/prism/bin/prism", "./DIE_MODEL/die_{}.pm".format(node_name),
                       "./DIE_MODEL/die_{}.pctl".format(node_name)], stdout=PIPE)
    text = ps0_check.communicate()[0].decode("utf-8")
    number = re.findall(r"(?<=Value in the initial state: )\d+\.?\d*E?-?\d*", text)
    return (float(number[0]))


# Model Checking with PRISM
def model_check_PRISM(opt_vars):
    update_par(x=opt_vars['x'], y=opt_vars['y'])  # update affine paramters

    for name in ['ps0', 'ps1', 'ps3']:
        opt_vars[name] = get_P_value(name)  # get evaluation
    return opt_vars

# Model Checking by solving linear equations
def model_check_LE(opt_vars):
    x, y = opt_vars['x'], opt_vars['y']
    A = np.array([[0, x, 0],
                  [0, 0, y],
                  [0, x, 0]]);
    b = np.array([0, 0, 1 - x])
    ps = np.linalg.solve(np.identity(3) - A, b)
    opt_vars['ps0'], opt_vars['ps1'], opt_vars['ps3'] = tuple(ps)
    return opt_vars


if __name__ == '__main__':
    pass
