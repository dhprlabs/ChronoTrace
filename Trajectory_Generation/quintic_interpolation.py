import sympy as sy
import numpy as np 


t0_sym, tf_sym = sy.symbols('t0 tf', real=True) # defining the variables
q0_sym, qf_sym = sy.symbols('q0 qf', real=True) # defining the variables

A = sy.Matrix([ 
    [1, t0_sym, t0_sym**2, t0_sym**3, t0_sym**4, t0_sym**5],
    [1, tf_sym, tf_sym**2, tf_sym**3, tf_sym**4, tf_sym**5],
    [0, 1, 2*t0_sym, 3*t0_sym**2, 4*t0_sym**3, 5*t0_sym**4 ],
    [0, 1, 2*tf_sym, 3*tf_sym**2, 4*tf_sym**3, 5*tf_sym**4],
    [0, 0, 2, 6*t0_sym, 12*t0_sym**2, 20*t0_sym**3 ],
    [0, 0, 2, 6*tf_sym, 12*tf_sym**2, 20*tf_sym**3],
])

b = sy.Matrix([ 
    [q0_sym],
    [qf_sym],
    [0],
    [0],
    [0],
    [0]
])

A_inv = A.inv()
x = A_inv * b

eq = []
for i in range(len(x)):
    eq.append(x[i])

# for i in range(len(eq)):
#     print(eq[i])

def quintic_interpolation(t, t0, tf, q0, qf):
    q0 = np.asarray(q0, dtype=float)
    qf = np.asarray(qf, dtype=float)

    if q0.shape != qf.shape:
        raise ValueError("q0 and qf must have the same shape.")

    q = np.zeros_like(q0, dtype=float)
    q_dot = np.zeros_like(q0, dtype=float)
    q_dot_dot = np.zeros_like(q0, dtype=float)

    for i in range(q0.shape[0]):
        subs = {
            t0_sym: t0,
            tf_sym: tf,
            q0_sym: q0[i],
            qf_sym: qf[i],
        }

        a0, a1, a2, a3, a4, a5 = [float(expr.subs(subs)) for expr in eq]
        q[i] = a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5
        q_dot[i] = a1 + 2*a2*t + 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4
        q_dot_dot[i] = 2*a2 + 6*a3*t + 12*a4*t**2 + 20*a5*t**3

    return q, q_dot, q_dot_dot