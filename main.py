from gurobipy import Model, GRB, quicksum
from soc_visualizer import soc_timeseries_for_bus

def build_model(data, sense=GRB.MINIMIZE):

    # ---------------------
    # SETS (as in your data)
    # ---------------------
    N      = data['N']        # all terminals (list/iterable of i)
    D      = data['D']            # depots (list/iterable of d)
    K      = data['K']            # lines (list/iterable of k)

    N_k    = data['N_k']            # dict: N_k[k] -> list of terminals on line k
    A_k    = data['A_k']          # dict: A_k[k] -> list of arcs (i,j) on line k
    M      = data['M']            # dict: M[k]   -> list of buses m on line k
    L_km   = data['L']            # dict: L_km[(m,k)] -> list of trips/loops l for bus m on line k

    # Time partitions
    PHI    = data['PHI']          # all discrete time points
    Z      = data['Z']            # demand periods
    H      = data['H']            # hours
    Z_peak = set(data['Z_peak'])  # subset of Z
    H_peak = set(data['H_peak'])  # subset of H
    PHI_h  = data['PHI_h']        # dict: PHI_h[h] -> list of phi in hour h
    PHI_z  = data['PHI_z']        # dict: PHI_z[z] -> list of phi in demand period z

    # Idling windows (IMPORTANT: mappings, not global lists)
    # - PHI_term[(i,k,m,l)] -> list of phi where (k,m) idles at terminal i in loop l
    # - PHI_depo[(k,m)]     -> list of phi where (k,m) idles at its depot d(k)
    PHI_term = data['PHI_term']
    PHI_depo = data['PHI_depo']

    # ---------------------
    # PARAMETERS
    # ---------------------
    d_k     = data['d_k']          # dict: d_k[k] -> list/set of depots serving line k (often one)
    o_k     = data['o_k']          # dict: o_k[k] -> start terminal (single i); if you store a list, take o_k[k][0]
    rho     = data['rho']          # scalar time weight (paper writes rho); if you prefer 1, set rho=1
    u_k     = data['u_k']          # dict: u_k[k] battery capacity (kWh)
    soc_up  = data['soc_up']       # scalar fraction
    soc_low = data['soc_low']      # scalar fraction

    # Energy consumptions (kWh)
    # c_depo_to_term[(d,o,k,m)], c_term_to_depo[(o,d,k,m)], c_arc[(i,j,k,m,l)]
    c_depo_to_term = data['c_depo_to_term']
    c_term_to_depo = data['c_term_to_depo']
    c_arc          = data['c_arc']

    # Charger caps (kW)
    # on-route per terminal & line: p_on_route[(i,k)]
    # depot per depot (aggregated across its lines): p_depot_cap[d]
    p_on_route   = data['p_on_route']
    p_depot_cap  = data['p_depo']     # <-- your key is 'p_depo'; renamed locally to avoid shadowing a var

    # Charging efficiencies (fractions)
    theta_on_route = data['theta_on_route']
    theta_depo     = data['theta_depo']

    # Averaging divisor (paper gamma)
    gamma = data['gamma']

    # Energy prices ($/kWh)
    psi_on  = data['psi_on']
    psi_off = data['psi_off']

    # Demand prices ($/kW)
    pi_on  = data['pi_on']
    pi_off = data['pi_off']

    # Demand → daily factor
    omega = data['omega']

    # ---------------------
    # MODEL
    # ---------------------
    model = Model("Bus_Scheduling")
    model.ModelSense = sense

    # ---------------------
    # VARIABLES
    # ---------------------
    # Depot energies (kWh)
    e_depo_arr = model.addVars(
        [(d,k,m) for k in K for d in d_k[k] for m in M[k]],
        lb=0.0, vtype=GRB.CONTINUOUS, name="e_depo_arr"
    )
    e_depo_dep = model.addVars(
        [(d,k,m) for k in K for d in d_k[k] for m in M[k]],
        lb=0.0, vtype=GRB.CONTINUOUS, name="e_depo_dep"
    )

    # Terminal energies (kWh)
    e_term_arr = model.addVars(
        [(i,k,m,l) for k in K for m in M[k] for l in L_km[(m,k)] for i in N_k[k]],
        lb=0.0, vtype=GRB.CONTINUOUS, name="e_term_arr"
    )
    e_term_dep = model.addVars(
        [(i,k,m,l) for k in K for m in M[k] for l in L_km[(m,k)] for i in N_k[k]],
        lb=0.0, vtype=GRB.CONTINUOUS, name="e_term_dep"
    )

    # Charging power (kW)
    p_term = model.addVars(
        [(i,k,m,phi) for k in K for m in M[k] for i in N_k[k] for phi in PHI],
        lb=0.0, vtype=GRB.CONTINUOUS, name="p_term"
    )
    p_depot = model.addVars(
        [(d,k,m,phi) for k in K for d in d_k[k] for m in M[k] for phi in PHI],
        lb=0.0, vtype=GRB.CONTINUOUS, name="p_depot"
    )

    # Average power per demand period (paper’s η_{i,ζ} and η_{d,ζ})
    eta_term = model.addVars([(i,z) for i in N for z in Z], lb=0.0, vtype=GRB.CONTINUOUS, name="eta_term")
    eta_depo = model.addVars([(d,z) for d in D for z in Z], lb=0.0, vtype=GRB.CONTINUOUS, name="eta_depo")

    # Peak power (linearized max)
    sigma_term_on  = model.addVars(N, lb=0.0, vtype=GRB.CONTINUOUS, name="sigma_term_on")
    sigma_term_off = model.addVars(N, lb=0.0, vtype=GRB.CONTINUOUS, name="sigma_term_off")
    sigma_depo_on  = model.addVars(D, lb=0.0, vtype=GRB.CONTINUOUS, name="sigma_depo_on")
    sigma_depo_off = model.addVars(D, lb=0.0, vtype=GRB.CONTINUOUS, name="sigma_depo_off")

    # ---------------------
    # CONSTRAINTS
    # ---------------------

    # R1: initial depot departure energy (you fixed it at SOC upper bound)
    model.addConstrs(
        (e_depo_dep[d,k,m] == soc_up * u_k[k] for k in K for d in d_k[k] for m in M[k]),
        name="initial_soc_depo"
    )

    # Helpers
    def start_terminal(k):
        ok = o_k[k]
        return ok[0] if isinstance(ok, (list, tuple)) else ok

    # R2: depot -> start terminal at first loop
    model.addConstrs(
        (e_term_dep[start_terminal(k), k, m, min(L_km[(m,k)])] ==
         e_depo_dep[d, k, m] - c_depo_to_term[(d, start_terminal(k), k, m)]
         for k in K for m in M[k] for d in d_k[k]),
        name="soc_depo_to_term"
    )

    # R3: along each arc within the same loop l
    model.addConstrs(
        (e_term_arr[j, k, m, l] ==
         e_term_dep[i, k, m, l] - c_arc[(i, j, k, m, l)]
         for k in K for m in M[k] for l in L_km[(m,k)] for (i, j) in A_k[k]),
        name="soc_arc"
    )

    # R6: end (last loop at start terminal) -> depot
    model.addConstrs(
        (e_depo_arr[d, k, m] ==
         e_term_dep[start_terminal(k), k, m, max(L_km[(m,k)])] - c_term_to_depo[(start_terminal(k), d, k, m)]
         for k in K for m in M[k] for d in d_k[k]),
        name="soc_term_to_depo"
    )

    # R28: energy at terminals (i != o(k)) with on-route charging accumulation
    model.addConstrs(
        (e_term_dep[i, k, m, l] ==
         e_term_arr[i, k, m, l] +
         theta_on_route * quicksum(p_term[i, k, m, phi] * rho for phi in PHI_term.get((i, k, m, l), []))
         for k in K for m in M[k] for l in L_km[(m,k)] for i in N_k[k] if i != start_terminal(k)),
        name="soc_not_o_term"
    )

    # R29: energy at start terminal using previous loop arrival (l >= 2)
    model.addConstrs(
        (e_term_dep[start_terminal(k), k, m, l] ==
         e_term_arr[start_terminal(k), k, m, l-1] +
         theta_on_route * quicksum(p_term[start_terminal(k), k, m, phi] * rho
                                   for phi in PHI_term.get((start_terminal(k), k, m, l), []))
         for k in K for m in M[k] for l in L_km[(m,k)] if l >= 2),
        name="soc_o_term"
    )

    # R30: energy at depot with depot charging
    model.addConstrs(
        (e_depo_dep[d, k, m] ==
         e_depo_arr[d, k, m] +
         theta_depo * quicksum(p_depot[d, k, m, phi] * rho for phi in PHI_depo.get((k, m), []))
         for k in K for m in M[k] for d in d_k[k]),
        name="soc_depo"
    )

    # R31: (already ensured by lb=0)

    # R32/R33: zero terminal charging outside idling windows
    # For each (i,k,m), compute the union of all idling phi over its loops l
    for k in K:
        for m in M[k]:
            for i in N_k[k]:
                allowed = set().union(*(set(PHI_term.get((i, k, m, l), [])) for l in L_km[(m,k)]))
                for phi in PHI:
                    if phi not in allowed:
                        model.addConstr(p_term[i, k, m, phi] == 0.0,
                                        name=f"p_term_not_idle[{i},{k},{m},{phi}]")

    # R34: on-route charger capacity by terminal & line
    model.addConstrs(
        (quicksum(p_term[i, k, m, phi] for m in M[k]) <= p_on_route[(i, k)]
         for k in K for i in N_k[k] for phi in PHI),
        name="p_term_on_route_limit"
    )

    # R35: (already ensured by lb=0)

    # R36: zero depot charging outside depot idling windows
    for k in K:
        for m in M[k]:
            allowed = set(PHI_depo.get((k, m), []))
            for d in d_k[k]:
                for phi in PHI:
                    if phi not in allowed:
                        model.addConstr(p_depot[d, k, m, phi] == 0.0,
                                        name=f"p_depot_not_idle[{d},{k},{m},{phi}]")

    # R37: depot charger capacity per depot (aggregated across lines it serves)
    model.addConstrs(
        (quicksum(p_depot[d, k, m, phi] for k in K for m in M[k] if d in d_k[k]) <= p_depot_cap[d]
         for d in D for phi in PHI),
        name="p_depot_limit"
    )

    # R39: average terminal power per demand period (η_{i,ζ})
    model.addConstrs(
        (eta_term[i, z] ==
         (1.0 / gamma) *
         quicksum(p_term[i, k, m, phi] * rho for phi in PHI_z[z] for k in K for m in M[k] if i in N_k[k])
         for i in N for z in Z),
        name="average_eta_term"
    )

    # R40: average depot power per demand period (η_{d,ζ})
    model.addConstrs(
        (eta_depo[d, z] ==
         (1.0 / gamma) *
         quicksum(p_depot[d, k, m, phi] * rho for phi in PHI_z[z] for k in K for m in M[k] if d in d_k[k])
         for d in D for z in Z),
        name="average_eta_depo"
    )

    # Battery energy lower/upper bounds: R8–R11
    # Lower bounds
    model.addConstrs((e_depo_arr[d, k, m] >= soc_low * u_k[k]
                      for k in K for d in d_k[k] for m in M[k]), name="lb_depo_arr")
    model.addConstrs((e_depo_dep[d, k, m] >= soc_low * u_k[k]
                      for k in K for d in d_k[k] for m in M[k]), name="lb_depo_dep")
    model.addConstrs((e_term_arr[i, k, m, l] >= soc_low * u_k[k]
                      for k in K for m in M[k] for l in L_km[(m,k)] for i in N_k[k]), name="lb_term_arr")
    model.addConstrs((e_term_dep[i, k, m, l] >= soc_low * u_k[k]
                      for k in K for m in M[k] for l in L_km[(m,k)] for i in N_k[k]), name="lb_term_dep")

    # Upper bounds
    model.addConstrs((e_depo_arr[d, k, m] <= soc_up * u_k[k]
                      for k in K for d in d_k[k] for m in M[k]), name="ub_depo_arr")
    model.addConstrs((e_depo_dep[d, k, m] <= soc_up * u_k[k]
                      for k in K for d in d_k[k] for m in M[k]), name="ub_depo_dep")
    model.addConstrs((e_term_arr[i, k, m, l] <= soc_up * u_k[k]
                      for k in K for m in M[k] for l in L_km[(m,k)] for i in N_k[k]), name="ub_term_arr")
    model.addConstrs((e_term_dep[i, k, m, l] <= soc_up * u_k[k]
                      for k in K for m in M[k] for l in L_km[(m,k)] for i in N_k[k]), name="ub_term_dep")

    # Peak linearization (use >=, not ==): R41–R44
    model.addConstrs((sigma_term_on[i]  >= eta_term[i, z] for i in N for z in Z if z in Z_peak),  name="sigma_term_on")
    model.addConstrs((sigma_term_off[i] >= eta_term[i, z] for i in N for z in Z if z not in Z_peak), name="sigma_term_off")
    model.addConstrs((sigma_depo_on[d]  >= eta_depo[d, z] for d in D for z in Z if z in Z_peak),  name="sigma_depo_on")
    model.addConstrs((sigma_depo_off[d] >= eta_depo[d, z] for d in D for z in Z if z not in Z_peak), name="sigma_depo_off")

    # ---------------------
    # OBJECTIVE
    # ---------------------
    # Energy charge (sum by on/off hours)
    EC_on = quicksum(
        psi_on * (
            quicksum(p_term[i, k, m, phi] * rho for phi in PHI_h[h] for i in N_k[k]) +
            quicksum(p_depot[d, k, m, phi] * rho for phi in PHI_h[h] for d in d_k[k])
        )
        for k in K for m in M[k] for h in H if h in H_peak
    )
    EC_off = quicksum(
        psi_off * (
            quicksum(p_term[i, k, m, phi] * rho for phi in PHI_h[h] for i in N_k[k]) +
            quicksum(p_depot[d, k, m, phi] * rho for phi in PHI_h[h] for d in d_k[k])
        )
        for k in K for m in M[k] for h in H if h not in H_peak
    )
    EC = EC_on + EC_off

    # Demand charge
    DC = pi_on  * (quicksum(sigma_term_on[i]  for i in N) + quicksum(sigma_depo_on[d]  for d in D)) \
       + pi_off * (quicksum(sigma_term_off[i] for i in N) + quicksum(sigma_depo_off[d] for d in D))

    model.setObjective(EC + omega * DC, GRB.MINIMIZE)

    return model


# -------------------------
# Minimal stub matching YOUR keys
# -------------------------
if __name__ == "__main__":
    # One depot, one line, one bus, two terminals, two loops (toy)
    N = ["A", "B"]
    D     = ["D1"]
    K     = ["L1"]

    N_k   = {"L1": ["A", "B"]}
    A_k   = {"L1": [("A","B"), ("B","A")]}
    M     = {"L1": ["B1"]}
    L_km  = {("B1","L1"): [1, 2]}           # <-- note (m,k) key

    # Start terminal and depot mapping
    o_k   = {"L1": "A"}                     # scalar; model handles list too
    d_k   = {"L1": ["D1"]}                  # list to allow multi-depot lines

    # Time partitions
    # -------------------------
    # 24h time sets (configurable resolution)
    # -------------------------
    RES_MIN = 60          # change to 15, 5, or 1 for finer grids
    H = list(range(24))   # hours 0..23
    SLOTS_PER_HOUR = 60 // RES_MIN
    T = 24 * SLOTS_PER_HOUR

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
    ("A", "L1", "B1", 1): PHI_h[8],   # all slots in 08:00–09:00
    ("B", "L1", "B1", 2): PHI_h[17],  # all slots in 17:00–18:00
    # add more windows as needed...
    }

    # Example depot idling: same bus idles at the depot in hours 0 and 23
    PHI_depo = {
        ("L1", "B1"): PHI_h[0] + PHI_h[23]
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
    p_on_route  = {("A","L1"): 325.0, ("B","L1"): 325.0}
    p_depo_cap  = {"D1": 130.0}

    # Efficiencies
    theta_on_route = 0.9
    theta_depo     = 0.9

    # Prices
    psi_on, psi_off = 0.050209, 0.033889   # $/kWh
    pi_on,  pi_off  = 15.40, 0.0           # $/kW
    omega = 1.0/30.0

    data = dict(
        N=N, D=D, K=K, N_k=N_k, A_k=A_k, M=M, L=L_km,
        PHI=PHI, Z=Z, H=H, Z_peak=Z_peak, H_peak=H_peak, PHI_h=PHI_h, PHI_z=PHI_z,
        PHI_term=PHI_term, PHI_depo=PHI_depo,
        d_k=d_k, o_k=o_k, rho=rho, u_k=u_k, soc_up=soc_up, soc_low=soc_low,
        c_depo_to_term=c_depo_to_term, c_term_to_depo=c_term_to_depo, c_arc=c_arc,
        p_on_route=p_on_route, p_depo=p_depo_cap,
        theta_on_route=theta_on_route, theta_depo=theta_depo,
        gamma=len(PHI_z[1]),  # example: divisor equals |Φ_z|
        psi_on=psi_on, psi_off=psi_off, pi_on=pi_on, pi_off=pi_off, omega=omega
    )

    model = build_model(data)
    model.Params.OutputFlag = 1
    model.optimize()

    plots_dir = "plots/"
    file_name = "toy_example"
    dir = plots_dir + file_name
    if model.status == GRB.OPTIMAL:
        soc_timeseries_for_bus(model, data, k = "L1", m = "B1", make_plot=True, dir_name=dir)
        print(f"Optimal objective = {model.objVal:.6f}")
        model.write("solucion.sol")
