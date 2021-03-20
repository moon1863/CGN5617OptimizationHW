# -*- coding: utf-8 -*-
"""
Created on Sun Mar  7 00:53:27 2021

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



# Creation of a Concrete Model
Optimal_values_I=[]
result_w=[]
Optimal_values=[]
M=100000

B_list=list(range(75000,150001,25000))

for B in B_list:

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

    model_I.const_4 = Constraint(rule=const_4,doc='w_constraints ')
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
  
    
    # Save the optimal results
    for ka in model_I.a:
        for ki in model_I.i:
            w = value(model_I.w[ka,ki])
            result_w.append([B,ka,ki,w])
        
    result_df=pd.DataFrame(result_w,columns=['B','ka','ki','w'])
    result_df['wai']=result_df['ka']+result_df['ki']


    # Creation of a Concrete Model
    for T in range (25,201,25):
        model_F = ConcreteModel()
        
        T0=T-1
        model_F.i = Set(initialize=['1','2','3'], doc='i')
        model_F.j = Set(initialize=['1','2','3'], doc='j')
        model_F.a = Set(initialize=['a1','a2', 'a3'], doc='a')
        model_F.t = Set(initialize=np.arange(T)+1, doc='states')
        model_F.t0 = Set(initialize=np.arange(T0)+1, doc='states')
        model_F.u_cost = Param(model_F.i, initialize={'1':10,'2':20,'3':30}, doc='user_cost')
        model_F.a_cost = Param(model_F.a, model_F.i, initialize=d_cost, doc='action_cost[s,a]')
        model_F.P = Param(model_F.a, initialize=P, doc='tr_probability_routine_maintenance P_ji')
        model_F.w = Var(model_F.a, model_F.i,model_F.t, bounds=(0,1), doc='% of the pavement with certain action')

        def const_1(model,a,i,t):
            return model.w[a,i,t] >= 0


        model_F.const_1 = Constraint(model_F.a, model_F.i, model_F.t, rule=const_1, doc='Wai>=0')


        def const_2(model_F,t):
            sum_wai2 = 0.0
            for ka in model_F.a:
                for ki in model_F.i:
                    sum_wai2 += model_F.w[ka,ki,t]
            return sum_wai2==1.0
        model_F.const_2 = Constraint(model_F.t,rule=const_2,doc='sum_wai2==1')


        def const_3(model_F,i,t0):
            sum_wai3=0
            sum_pw3=0
            for ka in model_F.a:
                sum_wai3+=model_F.w[ka,i,t0+1]
                for kj in model_F.j:
                    sum_pw3 += model_F.P[ka][kj,i]*model_F.w[ka,kj,t0]
            return sum_wai3 == sum_pw3

        model_F.const_3 = Constraint(model_F.i,model_F.t0,rule=const_3,doc='sum_wai==sum_pw')

        def const_4(model_F,t):
            sum_cw = 0.0
            for ka in model_F.a:
                for ki in model_F.i:
                    sum_cw += model_F.w[ka,ki,t]*model_F.a_cost[ka,ki]
            return sum_cw*M <= B

        model_F.const_4 = Constraint(model_F.t,rule=const_4,doc='w_constraints = 1')

        def const_5(model_F,i,t):
            sum_wai5=0
            for ka in model_F.a:
                sum_wai5+=model_F.w[ka,i,t]
            return sum_wai5 >= 0.0

        model_F.const_5 = Constraint(model_F.i,model_F.t,rule=const_5,doc='w_constraints >= 0')


        def const_6(model_F,i,t):
            sum_wai6=0
            for ka in model_F.a:
                sum_wai6+=model_F.w[ka,i,t]
            return sum_wai6 <= 1

        model_F.const_6 = Constraint(model_F.i,model_F.t,rule=const_6,doc='w_constraints <= 1.0')

        def const_7(model_F,i):
            sum_wai7=0
            for ka in model_F.a:
                sum_wai7+=model_F.w[ka,i,1]
            if i == '1':
                
                return sum_wai7 == 0
            elif i == '2':
               
                return sum_wai7 == 1
            elif i == '3':
              
                return sum_wai7 == 0

        model_F.const_7 = Constraint(model_F.i,rule=const_7,doc='boundary1')


        w_t={
        ('a1', '1'):0.0,
        ('a1', '2'):0.043,
        ('a1', '3'):0.872,
        ('a2', '1'):0.064,
        ('a2', '2'):0.0,
        ('a2', '3'):0.0,
        ('a3', '1'):0.0,
        ('a3', '2'):0.0,
        ('a3', '3'):0.021,
        }

        def const_8(model_F,i):
            sum_wai8=0
            sum_pw8=0
            for ka in model_F.a:
                sum_wai8+=model_F.w[ka,i,T]
                sum_pw8 += value(model_I.w[ka,i])
            #print(sum_wai8 ==sum_pw8)
            return sum_wai8 == sum_pw8

        model_F.const_8 = Constraint(model_F.i, rule=const_8, doc='boundary2')


        def objective_rule(model_F):
            obj=0
            for kt in model_F.t:
                for ka in model_F.a:
                    for ki in model_F.i:
                        obj+=(model_F.w[ka,ki,kt])*(model_F.u_cost[ki] + model_F.a_cost[ka,ki])*(0.97)**(kt)
            return obj*M + (0.97**T/(1-0.97))*model_I.objective()

        model_F.objective = Objective(rule=objective_rule, sense=minimize, doc='Define objective function')





        ## Display of the output ##
        def pyomo_postprocess(options=None, instance=None, results=None):
          model_F.w.display()


        try:

            if __name__ == '__main__':
                # This emulates what the pyomo command-line tools does
                import pyomo.environ
                from pyomo.opt import SolverFactory
                opt = SolverFactory("glpk",validate=False)
                results = opt.solve(model_F)
                #sends results to stdout
                results.write()
                print("\nDisplaying Solution\n" + '-'*60)
                pyomo_postprocess(None, model_F, results)

                objective_value_F = value(model_F.objective)
                objective_value_I = value(model_I.objective)
        except:
                objective_value_F= np.NAN
                objective_value_I= np.NAN


        Optimal_values.append([T, B, objective_value_F, objective_value_I])



df_optimal=pd.DataFrame(Optimal_values,columns=["T","B","ObjectiveValueF", "ObjectiveValueI"])

df_optimal.to_csv("df_optimal_problem4.csv")


# Plot for different budget and optimal costs

import matplotlib.pyplot as plt

for B in B_list:
    
    fig,ax=plt.subplots(figsize=(10,6))
    
    df_plot=df_optimal[df_optimal['B']==B]
    
    plt.plot(df_plot["T"],df_plot['ObjectiveValueF'])  
    
    plt.xlabel('Short Term Horizon in Years', size="16")
    plt.ylabel('Optimal Cost',size="16")
    plt.title('Finite Horizon Problem, Budget :'+str(B),size="16" )
    plt.tight_layout()
    
    plt.savefig(str(B)+"variation.png",dpi=500)
    plt.show()



# Optimal cost vaition at t=200 short term horizon
    
# Finite Horizon 

Th=200

fig2,ax2=plt.subplots(figsize=(10,6))
ax2.set_xlim(xmin=70000, xmax=110000)

df_plotT=df_optimal[df_optimal['T']==Th]

plt.plot(df_plotT["B"],df_plotT['ObjectiveValueF'])

plt.xlabel('Budget', size="16")
plt.ylabel('Optimal Cost',size="16")
plt.title('Finite Horizon Problem, Time :'+str(Th),size="16" )
plt.tight_layout()

plt.savefig(str(Th)+"variation.png",dpi=500)
plt.show()
    
 
# Finite Horizon 
       
fig3,ax3=plt.subplots(figsize=(10,6))
ax3.set_xlim(xmin=70000, xmax=110000)

df_plotT=df_optimal[df_optimal['T']==Th]

plt.plot(df_plotT["B"],df_plotT['ObjectiveValueI'])


plt.xlabel('Budget', size="16")
plt.ylabel('Optimal Cost',size="16")
plt.title('Infinte Horizon Problem, Time :'+str(Th),size="16" )
plt.tight_layout()

plt.savefig(str(Th)+"INFvariation.png",dpi=500)
plt.show()

fig,ax=plt.subplots(figsize=(10,6))


# Optimal solutions for InFinite Horizon for different budget 


for B in B_list:
          
    print(B)
    
    df_plot_w=result_df[result_df['B']==B]
    
    ax.plot(df_plot_w["wai"],df_plot_w['w'],'s-',label=str(B))
         
    
    plt.xlabel('action and state', size="16")
    plt.ylabel('Wai',size="16")
    plt.title('Infinte Horizon Problem',size="16" )
    plt.legend()
    plt.tight_layout()
    
    
plt.savefig("wa_variation.png",dpi=500)
   
plt.show()



