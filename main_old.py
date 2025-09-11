from gurobipy import Model, GRB, quicksum


def build_model(data, sense=GRB.MINIMIZE):

    # SETS
    N = data['N'] # BIG_N: all terminals
    D = data['D'] # d_k[k]: depots for line k
    K = data['K'] # K: lines
    N_k = data['N_k'] # N_k[k]: terminals for line k
    A_k = data['A_k'] # A_k[k]: arcs for line k
    M = data['M'] # M[k]: buses for line k
    L = data['L'] # L[m,k]: trips for bus m of line k
    PHI = data['PHI']
    PHI_depo = data['PHI_depo']
    PHI_term = data['PHI_term']
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
    p_on_route_max = data['p_on_route_max']
    p_depo_max = data['p_depo_max']
    theta_on_route = data['theta_on_route']
    theta_depo = data['theta_depo']
    gamma = data['gamma']
    psi_on = data['psi_on']
    psi_off = data['psi_off']
    pi_on = data['pi_on']
    pi_off = data['pi_off']
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
    p_depo = model.addVars([(d,k,m,phi) for k in K for d in d_k[k] for m in M[k] for phi in PHI], 
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
        (e_term_arr[j,k,m,l] == e_term_dep[i,k,m,l] - c_arc[i,j,k,m,l]
          for k in K for m in M[k] for i, j in A_k[k] for l in L[m,k]), name="soc_arc")
    # R6
    model.addConstrs(
        (e_depo_arr[d,k,m] == e_term_dep[o,k,m,l] - c_term_to_depo[o,d,k,m]
          for k in K for m in M[k] for d in d_k[k] for o in o_k[k] for l in L[m,k]), name="soc_term_to_depo")
    # R28
    model.addConstrs(
        (e_term_dep[i,k,m,l] == e_term_arr[i,k,m,l] + theta_on_route * 
        quicksum(p_term[i,k,m,phi] * rho for phi in PHI_term.get((i, k, m, l), [])) 
        for k in K for m in M[k] for l in L[m,k] for i in N_k[k] if i not in o_k[k]),
        name="soc_not_o_term")
    # R29
    model.addConstrs(
        (e_term_dep[i,k,m,l] == e_term_arr[i,k,m,l-1] + theta_on_route * 
        quicksum(p_depo[i,k,m,phi] * rho for phi in PHI_term.get((i, k, m, l), []))
        for k in K for m in M[k] for l in L[m,k] if l >= 2 for i in o_k[k]),
        name="soc_o_term")
    # R30
    model.addConstrs((e_depo_dep[d,k,m] == e_depo_arr[d,k,m] + theta_depo * 
                    quicksum(p_depo[d,k,m,phi] * rho for phi in PHI_depo.get((k, m), [])) 
                    for k in K for m in M[k] for d in d_k[k]), 
                    name="soc_depo")
    # R31
    model.addConstrs((p_term[i,k,m,phi] >= 0 
                    for k in K for m in M[k] for i in N_k[k] for phi in PHI), 
                    name="p_term_nonneg")
    # R32
    model.addConstrs((p_term[i,k,m,phi] == 0 
                    for k in K for m in M[k] for i in N_k[k] if i not in o_k[k]
                    for phi in PHI if phi not in (PHI_term.get((i, k, m, l), []) for l in L[m,k])),
                    name="p_term_if_not_idle") 
    # R33
    model.addConstrs((p_term[i,k,m,phi] == 0 
                    for k in K for m in M[k] for i in o_k[k] for l in L[m,k] if l >= 2 
                    for phi in PHI if phi not in (PHI_term.get((i, k, m, l), []))),
                    name="p_term_if_not_idle")
    # R34
    model.addConstrs((quicksum(p_term[i,k,m,phi]
                    for m in M[k]) <= p_on_route_max[i,k] for k in K for i in N_k[k] for phi in PHI),
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
    model.addConstrs((quicksum(p_depo[d,k,m,phi] for m in M[k]) <= p_depo_max[d] 
                    for k in K for d in d_k[k] for phi in PHI), 
                    name="p_depo_limit")
    # R39
    model.addConstrs((eta_term[i,z] == (1/gamma) *
                    quicksum(p_term[i,k,m,phi] * rho for phi in PHI_z[z] for k in K for m in M[k] if i in N_k[k])
                    for i in N for z in Z), name="average_eta_term")
    # R40
    model.addConstrs((eta_depo[d,z] == (1/gamma) *
                    quicksum(p_depo[d,k,m,phi] * rho for phi in PHI for k in K for m in M[k] if d in d_k[k])
                    for d in D for z in Z), name="average_eta_depo")
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

    # R41
    model.addConstrs((sigma_term_on[i] == eta_term[i,z] for i in N for z in Z_peak),
                     name="sigma_term_on")
    # R42
    model.addConstrs((sigma_term_off[i] == eta_term[i,z] for i in N for z in Z if z not in Z_peak),
                     name="sigma_term_off")
    # R43
    model.addConstrs((sigma_depo_on[d] == eta_depo[d,z] for d in D for z in Z_peak),
                     name="sigma_depo_on")
    # R44
    model.addConstrs((sigma_depo_off[d] == eta_depo[d,z] for d in D for z in Z if z not in Z_peak),
                     name="sigma_depo_off")
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

    EC = EC_on + EC_off

    DC = pi_on * (quicksum(sigma_term_on[i] for i in N) + quicksum(sigma_depo_on[d] for d in D)) + pi_off * (quicksum(sigma_term_off[i] for i in N) + quicksum(sigma_depo_off[d] for d in D))

    model.setObjective(EC + omega * DC, GRB.MINIMIZE)

if __name__ == "__main__":
    # One depot, one line, one bus, two terminals, two loops (toy)
    N = ["A", "B"]
    D     = ["D1"]
    K     = ["L1"]

    N_k   = {"L1": ["A", "B"]}
    A_k   = {"L1": [("A","B"), ("B","A")]}
    M     = {"L1": ["B1"]}
    L  = {("B1","L1"): [1, 2]}           # <-- note (m,k) key

    # Start terminal and depot mapping
    o_k   = {"L1": "A"}                     # scalar; model handles list too
    d_k   = {"L1": ["D1"]}                  # list to allow multi-depot lines

    # Time partitions
    # -------------------------
    # 24h time sets (configurable resolution)
    # -------------------------
    HOURS = 4
    RES_MIN = 60          # change to 15, 5, or 1 for finer grids
    H = list(range(HOURS))   # hours 0..23
    SLOTS_PER_HOUR = 60 // RES_MIN
    T = HOURS * SLOTS_PER_HOUR

    # Discrete time points Φ = {1..T} (1-based labels)
    PHI = list(range(1, T + 1))

    # Map each hour h to its Φ-indices inside that hour
    # hour 0 gets indices 1..SLOTS_PER_HOUR, hour 1 gets next block, etc.
    PHI_h = {
        h: list(range(h * SLOTS_PER_HOUR + 1, (h + 1) * SLOTS_PER_HOUR + 1))
        for h in H
    }

    # Demand periods Z: use hours as demand periods (paper-compatible)
    Z = H[:]                       # Z = {0..23}
    PHI_z = {z: PHI_h[z] for z in Z}

    # Pick on-peak hour set (example: 16:00–21:00)
    ON_PEAK_HOURS = {16, 17, 18, 19, 20, 21}
    H_peak = list(ON_PEAK_HOURS)
    Z_peak = list(ON_PEAK_HOURS)   # same periods as hours in this setup

    # Time-step weight ρ and averaging divisor γ for Eq. (39)–(40)
    # In your model you use (1/γ) * Σ (p * ρ). With hourly slots, ρ=1 and γ=SLOTS_PER_HOUR=1.
    rho = 1.0
    gamma = SLOTS_PER_HOUR

    PHI_term = {
    ("A", "L1", "B1", 1): PHI_h[1],   # all slots in 08:00–09:00
    ("B", "L1", "B1", 2): PHI_h[2],  # all slots in 17:00–18:00
    # add more windows as needed...
    }

    # Example depot idling: same bus idles at the depot in hours 0 and 23
    PHI_depo = {
        ("L1", "B1"): PHI_h[0] + PHI_h[3]
    }

    # Params
    rho = 1.0
    u_k = {"L1": 330.0}
    soc_low, soc_up = 0.2, 0.9

    # Energy consumptions (kWh)
    c_depo_to_term = {("D1","A","L1","B1"): 5.0}
    c_term_to_depo = {("A","D1","L1","B1"): 5.0}
    c_arc = {("A","B","L1","B1",1): 10.0,
             ("B","A","L1","B1",1): 10.0,
             ("A","B","L1","B1",2): 10.0,
             ("B","A","L1","B1",2): 10.0}

    # Charger caps (kW)
    p_on_route_max  = {("A","L1"): 325.0, ("B","L1"): 325.0}
    p_depo_max  = {"D1": 130.0}

    # Efficiencies
    theta_on_route = 0.9
    theta_depo     = 0.9

    # Prices
    psi_on, psi_off = 0.050209, 0.033889   # $/kWh
    pi_on,  pi_off  = 15.40, 0.0           # $/kW
    omega = 1.0/30.0

    data = dict(
        N=N, D=D, K=K, N_k=N_k, A_k=A_k, M=M, L=L,
        PHI=PHI, Z=Z, H=H, Z_peak=Z_peak, H_peak=H_peak, PHI_h=PHI_h, PHI_z=PHI_z,
        PHI_term=PHI_term, PHI_depo=PHI_depo,
        d_k=d_k, o_k=o_k, rho=rho, u_k=u_k, soc_up=soc_up, soc_low=soc_low,
        c_depo_to_term=c_depo_to_term, c_term_to_depo=c_term_to_depo, c_arc=c_arc,
        p_on_route_max=p_on_route_max, p_depo_max=p_depo_max,
        theta_on_route=theta_on_route, theta_depo=theta_depo,
        gamma=len(PHI_z[1]),  # example: divisor equals |Φ_z|
        psi_on=psi_on, psi_off=psi_off, pi_on=pi_on, pi_off=pi_off, omega=omega
    )

    model = build_model(data)
    model.Params.OutputFlag = 1
    model.optimize()

    if model.status == GRB.OPTIMAL:
        print(f"Optimal objective = {model.objVal:.6f}")
        with open("constraints.txt", "w") as f:
            f.write("Constraint,Slack\n")
            for constr in model.getConstrs():
                f.write(f"{constr.ConstrName},{constr.Slack}\n")
        model.write("solucion.sol")