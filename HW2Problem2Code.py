# -*- coding: utf-8 -*-
"""
Created on Mon Mar 15 18:58:06 2021

@author: Moon
"""

# Import of the pyomo module
from pyomo.environ import *

import numpy as np

import math

#get_ipython().run_line_magic('matplotlib', 'inline')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

#import shutil
#import sys
#import os.path

# Creation of a Concrete Model
Optimal_values_I=[]
result_w=[]
Optimal_values=[]
M=100000

B=75000



model_I = ConcreteModel()

# Initialize state and actions
model_I.i = Set(initialize=['1','2','3'], doc='State i')
model_I.j = Set(initialize=['1','2','3'], doc='State j')
model_I.a = Set(initialize=['a1','a2', 'a3'], doc='Action')

# User cost
model_I.u_cost = Param(model_I.i, initialize={'1':10,'2':20,'3':30}, doc='UserCost')

# Action costs
d_cost = {
    ('a1','1'): 0.0,
    ('a1','2'): 0.0,
    ('a1','3'): 0.0,
    ('a2','1'): 5.0,
    ('a2','2'): 10.0,
    ('a2','3'): 15.0,
    ('a3','1'): 20.0,
    ('a3','2'): 20.0,
    ('a3','3'): 20.0,
    }

model_I.a_cost = Param(model_I.a, model_I.i, initialize=d_cost, doc='ActionCost')

# Transition Propability

P_DN={
    ('1','1') : 0.5,
    ('1','2') : 0.5,
    ('1','3') : 0,
    ('2','1') : 0,
    ('2','2') : 0.5,
    ('2','3') : 0.5,
    ('3','1') : 0,
    ('3','2') : 0,
    ('3','3') : 1,
    }

P_RM={
    ('1','1'):0.8,
    ('1','2'):0.2,
    ('1','3'):0,
    ('2','1'):0,
    ('2','2'):0.8,
    ('2','3'):0.2,
    ('3','1'):0,
    ('3','2'):0,
    ('3','3'):1,
    }
P_Resurf={
    ('1','1'): 1.0,
    ('1','2'): 0,
    ('1','3'): 0,
    ('2','1'): 0.8,
    ('2','2'): 0.2,
    ('2','3'): 0,
    ('3','1'): 0.6,
    ('3','2'): 0.4,
    ('3','3'): 0,
    }

P={"a1":P_DN,"a2":P_RM,"a3":P_Resurf}

model_I.P = Param(model_I.a, initialize=P, doc='Transition probability')
model_I.w = Var(model_I.a, model_I.i, bounds=(0,1), doc='% of the pavement with certain action')

# Define constraints
def const_1(model_I,a,i):
    print(model_I.w[a,i] >= 0)
    return model_I.w[a,i] >= 0

model_I.const_1 = Constraint(model_I.a, model_I.i, rule=const_1, doc='Wai>=0')
   
def const_2(model_I):
    sum_wai2 = 0.0
    for ka in model_I.a:
        for ki in model_I.i:
            sum_wai2 += model_I.w[ka,ki]
    print(sum_wai2==1)
    return sum_wai2==1

model_I.const_2 = Constraint(rule=const_2,doc='w_constraints = 1')
model_I.const_2.display()

def const_3(model_I,i):
    sum_wai3=0
    sum_pw3=0
    for ka in model_I.a:
        sum_wai3+=model_I.w[ka,i]
        for kj in model_I.j:
            sum_pw3 += model_I.P[ka][kj,i]*model_I.w[ka,kj]
            #return sum_wai==sum_pw
    #print(sum_pw)
    #print(sum_wai)
    print(sum_wai3 ==sum_pw3)
    return sum_wai3 == sum_pw3

model_I.const_3 = Constraint(model_I.i, rule=const_3,doc='sum_wai==sum_pw')
   
def const_4(model_I):
    sum_cw = 0.0
    for ka in model_I.a:
        for ki in model_I.i:
            sum_cw += model_I.w[ka,ki]*model_I.a_cost[ka,ki]
    print(sum_cw*M <=B)
    return sum_cw*M<=B

model_I.const_4 = Constraint(rule=const_4,doc='w_constraints = 1')
def const_5(model_I,i):
    sum_wai=0
    for ka in model_I.a:
        sum_wai+=model_I.w[ka,i]
    print(sum_wai>= 0.1)
    return sum_wai >= 0.0
model_I.const_5 = Constraint(model_I.i,rule=const_5,doc='w_constraints >= 0.2')

def const_6(model_I,i):
    sum_wai=0
    for ka in model_I.a:
        sum_wai+=model_I.w[ka,i]
    #print(sum_wai<= 0)
    return sum_wai <= 1

model_I.const_6 = Constraint(model_I.i,rule=const_6,doc='w_constraints <= 1.0')
def objective_rule(model_I):
    obj=0
    for ki in model_I.i:
        for ka in model_I.a:
            obj+=model_I.w[ka,ki]*(model_I.u_cost[ki] + model_I.a_cost[ka,ki])
    #print(obj*M)
    return obj*M

model_I.objective = Objective(rule=objective_rule, sense=minimize, doc='Define objective function')


def pyomo_postprocess(options=None, instance=None, results=None):
  model_I.w.display()

# This is an optional code path that allows the script to be run outside of
# pyomo command-line.  For example:  python transport.py
if __name__ == '__main__':
    import pyomo.environ
    from pyomo.opt import SolverFactory
    opt = SolverFactory("glpk")
    results = opt.solve(model_I)
    
    # Printout results
    results.write()
    print("\nDisplaying Solution\n" + '-'*60)
    pyomo_postprocess(None, model_I, results)

print(model_I.objective())
  

# =============================================================================
# # Save the optimal results
# for ka in model_I.a:
#     for ki in model_I.i:
#         w = value(model_I.w[ka,ki])
#         result_w.append([B,ka,ki,w])
#     
# result_df=pd.DataFrame(result_w,columns=['B','ka','ki','w'])
# result_df['wai']=result_df['ka']+result_df['ki']
# =============================================================================
