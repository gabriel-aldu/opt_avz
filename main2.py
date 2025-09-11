from gurobipy import Model, GRB, quicksum


def build_model(data, sense=GRB.MINIMIZE):

    # SETS
    N = data['BIG_N'] # BIG_N: all terminals
    D = data['D'] # d_k[k]: depots for line k
    A = data['A']
    K = data['K'] # K: lines
    N_k = data['N'] # N_k[k]: terminals for line k
    A_k = data['A_k'] # A_k[k]: arcs for line k
    M = data['M'] # M[k]: buses for line k
    L = data['L'] # L[m,k]: trips for bus m of line k
    PHI = data['PHI']
    PHI_depo = data['PHI_depo']
    PHI_term = data['PHI_term']
    PHI_hour = data['PHI_hour']
    PHI_demand = data['PHI_demand']
    Z = data['Z']
    H = data['H']
    Z_peak = data['Z_peak']
    H_peak = data['H_peak']
    PHI_h = data['PHI_h']
    PHI_z = data['PHI_z']

    # PARAMETERS
    d_k = data['d_k']
    o_k = data['o_k']
    rho = data['rho']
    u_k = data['u_k']
    soc_up = data['soc_up']
    soc_low = data['soc_low']
    c_depo_to_term = data['c_depo_to_term']
    c_term_to_depo = data['c_term_to_depo']
    c_arc = data['c_arc']
    t_depo_dep = data['t_depo_dep']
    t_depo_arr = data['t_depo_arr']
    t_temr_dep = data['t_temr_dep']
    t_term_arr = data['t_term_arr']
    t_start_h = data['t_start_h']
    t_end_h = data['t_end_h']
    t_start_z = data['t_start_z']
    t_end_z = data['t_end_z']
    p_on_route = data['p_on_route']
    p_depo = data['p_depo']
    theta_on_route = data['theta_on_route']
    theta_depo = data['theta_depo']
    gamma = data['gamma']
    psi_on = data['psi_on']
    psi_off = data['psi_off']
    psi_on = data['psi_on']
    psi_off = data['psi_off']
    omega = data['omega']

    # MODEL
    model = Model("Bus_Scheduling")
    
    # VARIABLES
    e_depo_arr = model.addVars([(d, k ,m) for k in K for d in d_k[k] for m in M[k]],
                                vtype=GRB.CONTINUOUS, name="e_depo_arr")
    e_depo_dep = model.addVars([(d, k ,m) for k in K for d in d_k[k] for m in M[k]], 
                               vtype=GRB.CONTINUOUS, name="e_depo_dep")
    e_term_arr = model.addVars([(i,k,m,l) for k in K for i in N_k[k] for m in M[k] for l in L[m,k]], 
                               vtype=GRB.CONTINUOUS, name="e_term_arr")
    e_term_dep = model.addVars([(i,k,m,l) for k in K for i in N_k[k] for m in M[k] for l in L[m,k]], 
                               vtype=GRB.CONTINUOUS, name="e_term_dep")

    p_term = model.addVars([(i,k,m,phi) for k in K for i in N_k[k] for m in M[k] for phi in PHI], 
                           vtype=GRB.CONTINUOUS, name="p_term")
    p_depo = model.addVars([(d,k,m,phi) for k in K for d in d_k[k] for m in M[k] for phi in PHI_depo], 
                           vtype=GRB.CONTINUOUS, name="p_depo")

    eta_term = model.addVars([(i, z) for i in N for z in Z], vtype=GRB.CONTINUOUS, name="eta_term")
    eta_depo = model.addVars([(d, z) for d in D for z in Z], vtype=GRB.CONTINUOUS, name="eta_depo")

    sigma_term_on = model.addVars([(i) for i in N], vtype=GRB.CONTINUOUS, name="sigma_term_on")
    sigma_term_off = model.addVars([(i) for i in N], vtype=GRB.CONTINUOUS, name="sigma_term_off")
    sigma_depo_on = model.addVars([(d) for d in D], vtype=GRB.CONTINUOUS, name="sigma_depo_on")
    sigma_depo_off = model.addVars([(d) for d in D], vtype=GRB.CONTINUOUS, name="sigma_depo_off")

    #CONSTRAINTS

    # Initial battery energy
    # R1
    model.addConstrs((e_depo_dep[d,k,m] == soc_up * u_k[k] for k in K for d in d_k[k] for m in M[k]),
                      name="initial_soc_depo")

    # Battery energy change
    # R2
    model.addConstrs(
        (e_term_dep[o,k,m,1] == e_depo_dep[d,k,m] - c_depo_to_term[d,o,k,m]
          for k in K for m in M[k] for d in d_k[k] for o in o_k[k]), name="soc_depo_to_term")
    # R3
    model.addConstrs(
        (e_term_arr[j,k,m,l] == e_term_dep[i,k,m,l] - c_arc[i,k,m,l]
          for k in K for m in M[k] for i, j in A_k[k] for l in L[m,k]), name="soc_arc")
    # R6
    model.addConstrs(
        (e_depo_arr[d,k,m] == e_term_dep[o,k,m,l] - c_term_to_depo[o,d,k,m]
          for k in K for m in M[k] for d in d_k[k] for o in o_k[k] for l in L[m,k]), name="soc_term_to_depo")
    # R28
    model.addConstrs(
        (e_term_dep[i,k,m,l] == e_term_arr[i,k,m,l] + theta_on_route * 
        quicksum(p_term[i,k,m,phi] * rho for phi in PHI_term) 
        for k in K for m in M[k] for l in L[m,k] for i in N_k[k] if i not in o_k[k]),
        name="soc_not_o_term")
    # R29
    model.addConstrs(
        (e_term_dep[i,k,m,l] == e_term_arr[i,k,m,l-1] + theta_on_route * 
        quicksum(p_depo[i,k,m,phi] * rho for phi in PHI_term)
        for k in K for m in M[k] for l in L[m,k] if l >= 2 for i in o_k[k]),
        name="soc_o_term")
    # R30
    model.addConstrs((e_depo_dep[d,k,m] == e_depo_arr[d,k,m] + theta_depo * 
                    quicksum(p_depo[d,k,m,phi] * rho for phi in PHI_depo) 
                    for k in K for m in M[k] for d in d_k[k]), 
                    name="soc_depo")
    # R31
    model.addConstrs((p_term[i,k,m,phi] >= 0 
                    for k in K for m in M[k] for i in N_k[k] for phi in PHI), 
                    name="p_term_nonneg")
    # R32
    model.addConstrs((p_term[i,k,m,phi] == 0 
                    for k in K for m in M[k] for i in N_k[k] if i not in o_k[k]
                    for phi in PHI if phi not in (PHI_term.get((i, k, m, l), []) for l in L[k][m])),
                    name="p_term_if_not_idle") 
    # R33
    model.addConstrs((p_term[i,k,m,phi] == 0 
                    for k in K for m in M[k] for i in o_k[k] for l in L[k][m] if l >= 2 
                    for phi in PHI if phi not in (PHI_term.get((i, k, m, l), []))),
                    name="p_term_if_not_idle")
    # R34
    model.addConstrs((quicksum(p_term[i,k,m,phi]
                    for m in M[k]) <= p_on_route[i,k] for k in K for i in N_k[k] for phi in PHI),
                    name="p_term_on_route_limit")
    # R35
    model.addConstrs((p_depo[d,k,m,phi] >= 0 
                    for k in K for m in M[k] for d in d_k[k] for phi in PHI), 
                    name="p_depo_nonneg")
    # R36
    model.addConstrs((p_depo[d,k,m,phi] == 0 
                    for k in K for m in M[k] for d in d_k[k] for phi in PHI if phi not in PHI_depo), 
                    name="p_depo_if_not_idle")
    # R37
    model.addConstrs((quicksum(p_depo[d,k,m,phi] for m in M[k]) <= p_depo[d,k] 
                    for k in K for d in d_k[k] for phi in PHI), 
                    name="p_depo_limit")
    # R38
     
    # Battery energy lower and upper bounds

    # R8
    model.addConstrs((soc_low * u_k[k] <= e_depo_arr[d,k,m] 
                    for k in K for d in d_k[k] for m in M[k]), 
                    name="soc_depo_arr_low")
    model.addConstrs((e_depo_arr[d,k,m] <= soc_up * u_k[k] 
                    for k in K for d in d_k[k] for m in M[k]), 
                    name="soc_depo_arr_up")

    #R9 
    model.addConstrs((soc_low * u_k[k] <= e_depo_dep[d,k,m] 
                    for k in K for d in d_k[k] for m in M[k]), 
                    name="soc_depo_dep_low")
    model.addConstrs((e_depo_dep[d,k,m] <= soc_up * u_k[k] 
                    for k in K for d in d_k[k] for m in M[k]), 
                    name="soc_depo_dep_up")

    # R10
    model.addConstrs((soc_low * u_k[k] <= e_term_arr[i,k,m,l] 
                    for k in K for i in N_k[k] for m in M[k] for l in L[m,k]), 
                    name="soc_term_arr_low")
    model.addConstrs((e_term_arr[i,k,m,l] <= soc_up * u_k[k] 
                    for k in K for i in N_k[k] for m in M[k] for l in L[m,k]), 
                    name="soc_term_arr_up")

    # R11
    model.addConstrs((soc_low * u_k[k] <= e_term_dep[i,k,m,l] 
                    for k in K for i in N_k[k] for m in M[k] for l in L[m,k]), 
                    name="soc_term_dep_low")
    model.addConstrs((e_term_dep[i,k,m,l] <= soc_up * u_k[k] 
                    for k in K for i in N_k[k] for m in M[k] for l in L[m,k]), 
                    name="soc_term_dep_up")

    # OBJECTIVE FUNCTION
    EC_on = psi_on * quicksum(
        quicksum(p_term[i,k,m,phi] * rho for phi in PHI_h[h] for i in N_k[k]) +
        quicksum(p_depo[d_k[k],k,m,phi] * rho for phi in PHI_h[h])
        for k in K for m in M[k] for h in H_peak)
    EC_off = psi_off * quicksum(
        quicksum(p_term[i, k, m, phi] * rho for phi in PHI_h[h] for i in N_k[k]) +
        quicksum(p_depo[d_k[k], k, m, phi] * rho for phi in PHI_h[h])
        for k in K for m in M[k] for h in H if h not in H_peak
    )

