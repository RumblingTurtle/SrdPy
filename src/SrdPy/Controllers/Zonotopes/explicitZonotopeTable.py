import numpy as np
import cvxpy as cp
def explicitZonotopeTable(A_table,B_table,W_matrix=np.diag([0.001,0.001,0.001,0.001]),
zonotope_order=250, cost_weights_G=[1111],cost_weights_T=1,cost_weights_b=1,cyclical=False):

    size_x = A_table.shape[2]
    size_u = B_table.shape[2]
    count  = A_table.shape[0]
    size_p = A_table.shape[3]
    size_z = zonotope_order

    G,T = [],[]

    for i in range(count):
        G+=[cp.Variable((size_x, size_z+size_x),name="G_{}".format(i))]
        T+=[cp.Variable((size_u, size_z+size_x),name="T_{}".format(i))]
    G+=[cp.Variable((size_x, size_z+size_x),name="G_N")]

    bounding_box_margin = cp.Variable((size_x, count),nonneg=True)

    cost_norm = 2

    cost = cost_weights_b * cp.norm(cp.vec(bounding_box_margin), cost_norm)

    if type(cost_weights_T)==int:
        cost = cost + cost_weights_T * cp.norm(cp.vec(cp.hstack(T)), cost_norm)
    else:
        for i in range(size_u):
            cost = cost + cost_weights_T[i] * cp.norm(T[i], cost_norm)
        


    if type(cost_weights_G)==int:
        cost = cost + cost_weights_G * cp.norm(cp.vec(cp.hstack(G)), cost_norm)
    else:
        for i in range(size_x):
            cost = cost + cost_weights_G[i] * cp.norm(G[i], cost_norm)
        

    constraints = []
    if cyclical:
        constraints.append(G[0] == G[count])

    for i in range(count):
        GG = []
        for j in range(size_p):
            GG+=[A_table[i,:,:, j]@G[i] + B_table[i,:,:,  j]@T[i]]
        

        F_TEMP = cp.hstack((GG[0]+GG[1],GG[0]-GG[1]))/2

        #ReaZOR
        F_rea = cp.hstack((F_TEMP[:, :size_x], F_TEMP[:, 8:], F_TEMP[:, 4:7]))
        F_1 = F_rea[:,:size_z-size_x]
        F_2 = F_rea[:, (size_z-size_x):]
        for j in range(size_x):
            constraints+=[cp.norm(F_2[j],1) <= bounding_box_margin[j,i]]
        
        constraints+=[G[i+1] == cp.hstack((F_1,cp.diag(bounding_box_margin[:,i]), W_matrix))]

    prob = cp.Problem(cp.Minimize(cost),constraints)
    prob.solve(verbose=True)
    if prob.status not in ["infeasible", "unbounded"]:
        print("Optimal value: %s" % prob.value)
        K_table = np.zeros([size_u, size_x, count])
        res_G = []
        res_T = []
        for variable in prob.variables():
            if variable.name()[0]=="G":
                res_G.append(variable.value)
            if variable.name()[0]=="T":
                res_T.append(variable.value)
        for i in range(count):
            K_table[:,:,i] = res_T[i] @ np.linalg.pinv(res_G[i])
        

        return K_table, np.stack(res_G), np.stack(res_T)