# -*- coding: utf-8 -*-
"""Nav_algo_1.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1d-W4UYxB0MmTkqTxEXzUO5kp5wN71oC9
"""

import numpy as np
import math
x = np.array([[2],[3]])
alpha = 1.2
epslon = 0.5
delta = 1
vo= 1
c = 2

"""1. Goal to goal"""

xg = np.array([[10],[15]])
egtg = xg-x
print(egtg)
e1 = np.linalg.norm(egtg)
print(e1)
kgtg = vo*(1-math.exp(-alpha*np.power(e1,2)))/e1
print(kgtg)
eogtg = -kgtg*egtg
ugtg = -eogtg
print(eogtg)

"""2. Avoid obstacle"""

xo = np.array([[6],[4]])
eao = xo-x
e2 = np.linalg.norm(eao)
print(e2)
kao = c/(e2*(np.power(e2,2)+epslon))
print(kao)
uao = kao*eao
print(uao)

"""3. Follow wall"""

t = np.array([[0,1],[-1,0]])
ucw = alpha*(t@uao)
uccw = alpha*(-t@uao)
#Unit vectors:
uv0 = ugtg/np.linalg.norm(ugtg)   
uv1 = ucw/np.linalg.norm(ucw)
uv2 = uccw/np.linalg.norm(uccw)
uv3 = uao/np.linalg.norm(uao)

print(uccw)
dirc = np.dot(uv0.T,uv1)
dircc = np.dot(uv0.T,uv2)
print(dirc,dircc)
dirao = np.dot(uv0.T,uv3)
print(dirao)

"""4. Automation"""

