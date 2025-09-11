# -*- coding: utf-8 -*-
"""
Gurobi model for the discretized linear reformulation (D-CSM-LP)
of the fast-charging BEB scheduling & management problem.

Implements (paper numbering):
(1)   depot->start energy link
(3)   along-arc energy consumption
(6)   end->depot energy link
(8)-(11) SoC bounds
(28)-(30) discretized energy accumulation from charging
(31)-(37) charging feasibility & power limits
(38)  energy charge (kWh * rate)
(39)-(40) average power per demand period
(41)-(44) linearization of peak demand (max)
(26),(27) demand charge & total daily cost objective
"""
from gurobipy import Model, GRB, quicksum


def build_model(data, sense=GRB.MINIMIZE):
    """
    Build and return the Gurobi model.

    Required keys in `data`:
      Sets / maps:
        D                     # depots
        K                     # lines
        K_by_depot            # dict: d -> [k in K with d(k)=d]
        M                     # dict: M[k] -> iterable of BEBs on line k
        N                     # dict: N[k] -> iterable of terminals on line k
        L                     # dict: L[k][m] -> iterable of service loops for BEB m on line k
        H                     # hours
        H_peak                # subset of H with on-peak hours
        Z                     # demand periods
        Z_peak                # subset of Z with on-peak demand periods
        Tau                   # discretized time periods

        # Idling windows (lists of tau)
        T_term                # dict: T_term[(i,k,m,l)] -> iterable of tau when (k,m) idles at terminal i in loop l
        T_depot               # dict: T_depot[(k,m)]    -> iterable of tau when (k,m) idles at its depot d(k)

        # Hour / demand → tau
        T_hour                # dict: T_hour[h]  -> iterable of tau in hour h
        T_demand              # dict: T_demand[z]-> iterable of tau in demand period z

      Line/vehicle/graph:
        o                     # dict: o[k] -> start terminal
        d                     # dict: d[k] -> depot id for line k
        A                     # dict: A[k] -> iterable of arcs (i,j)

      Energy (kWh):
        c_dep_to_o            # dict: c_dep_to_o[(k,m)]
        c_o_to_dep            # dict: c_o_to_dep[(k,m)]
        c_arc                 # dict: c_arc[(k,m,l,i,j)]

      Charger caps (kW):
        p_onroute_max         # dict: p_onroute_max[(i,k)]
        p_depot_max           # dict: p_depot_max[d]

      Efficiencies:
        eta_on                # on-route
        eta_depot             # depot

      Battery bounds (kWh):
        u_lo                  # dict: u_lo[k]
        u_up                  # dict: u_up[k]

      Prices:
        c_on                  # $/kWh on-peak
        c_off                 # $/kWh off-peak
        lam_on                # $/kW on-peak
        lam_off               # $/kW off-peak
        alpha                 # demand→daily factor (e.g., 1/30)
    """
    # Unpack
    D = data["D"]
    K = data["K"]
    K_by_depot = data["K_by_depot"]
    M = data["M"]
    N = data["N"]
    L = data["L"]
    H = data["H"]
    H_peak = set(data["H_peak"])
    Z = data["Z"]
    Z_peak = set(data["Z_peak"])
    Tau = data["Tau"]

    T_term = data["T_term"]
    T_depot = data["T_depot"]
    T_hour = data["T_hour"]
    T_demand = data["T_demand"]

    o = data["o"]
    d_of = data["d"]
    A = data["A"]

    c_dep_to_o = data["c_dep_to_o"]
    c_o_to_dep = data["c_o_to_dep"]
    c_arc = data["c_arc"]

    p_onroute_max = data["p_onroute_max"]
    p_depot_max = data["p_depot_max"]

    eta_on = data["eta_on"]
    eta_depot = data["eta_depot"]

    u_lo = data["u_lo"]
    u_up = data["u_up"]

    c_on = data["c_on"]
    c_off = data["c_off"]
    lam_on = data["lam_on"]
    lam_off = data["lam_off"]
    alpha = data["alpha"]

    # Model
    m = Model("D-CSM-LP")
    m.ModelSense = sense

    # -------------------------
    # Decision variables
    # -------------------------
    # Terminal charging power (kW)
    p_term = {}
    for k in K:
        for m_id in M[k]:
            for i in N[k]:
                for tau in Tau:
                    p_term[i, k, m_id, tau] = m.addVar(lb=0.0, vtype=GRB.CONTINUOUS,
                                                       name=f"p_term[{i},{k},{m_id},{tau}]")

    # Depot charging power (kW)
    p_dep = {}
    for k in K:
        for m_id in M[k]:
            for tau in Tau:
                p_dep[k, m_id, tau] = m.addVar(lb=0.0, vtype=GRB.CONTINUOUS,
                                               name=f"p_dep[{k},{m_id},{tau}]")

    # Energy at terminals (kWh)
    e_arr_term, e_dep_term = {}, {}
    for k in K:
        for m_id in M[k]:
            for l_id in L[k][m_id]:
                for i in N[k]:
                    e_arr_term[i, k, m_id, l_id] = m.addVar(lb=0.0, vtype=GRB.CONTINUOUS,
                                                            name=f"e_arr_term[{i},{k},{m_id},{l_id}]")
                    e_dep_term[i, k, m_id, l_id] = m.addVar(lb=0.0, vtype=GRB.CONTINUOUS,
                                                            name=f"e_dep_term[{i},{k},{m_id},{l_id}]")

    # Energy at depots (kWh) — with depot index d
    e_arr_dep, e_dep_dep = {}, {}
    for d in D:
        for k in K_by_depot.get(d, []):
            for m_id in M[k]:
                e_arr_dep[d, k, m_id] = m.addVar(lb=0.0, vtype=GRB.CONTINUOUS,
                                                 name=f"e_arr_dep[{d},{k},{m_id}]")
                e_dep_dep[d, k, m_id] = m.addVar(lb=0.0, vtype=GRB.CONTINUOUS,
                                                 name=f"e_dep_dep[{d},{k},{m_id}]")

    # Average power per demand period (kW)
    pbar_term = {}
    for k in K:
        for i in N[k]:
            for z in Z:
                pbar_term[i, k, z] = m.addVar(lb=0.0, vtype=GRB.CONTINUOUS,
                                              name=f"pbar_term[{i},{k},{z}]")

    pbar_dep = {}
    for d in D:
        for z in Z:
            pbar_dep[d, z] = m.addVar(lb=0.0, vtype=GRB.CONTINUOUS,
                                      name=f"pbar_dep[{d},{z}]")

    # Linearized peak power (kW)
    p_on_i, p_off_i = {}, {}
    for k in K:
        for i in N[k]:
            p_on_i[i, k] = m.addVar(lb=0.0, vtype=GRB.CONTINUOUS, name=f"p_on_i[{i},{k}]")
            p_off_i[i, k] = m.addVar(lb=0.0, vtype=GRB.CONTINUOUS, name=f"p_off_i[{i},{k}]")

    p_on_d = {d: m.addVar(lb=0.0, vtype=GRB.CONTINUOUS, name=f"p_on_d[{d}]") for d in D}
    p_off_d = {d: m.addVar(lb=0.0, vtype=GRB.CONTINUOUS, name=f"p_off_d[{d}]") for d in D}

    # -------------------------
    # Constraints
    # -------------------------

    # (31)-(33): zero terminal charging when not idling
    for k in K:
        for m_id in M[k]:
            for i in N[k]:
                for tau in Tau:
                    idles_some_loop = any(tau in T_term.get((i, k, m_id, l_id), [])
                                          for l_id in L[k][m_id])
                    if not idles_some_loop:
                        m.addConstr(p_term[i, k, m_id, tau] == 0.0,
                                    name=f"no_idle_term[{i},{k},{m_id},{tau}]")

    # (34): on-route charger capacity at each terminal i for its line k
    for k in K:
        for i in N[k]:
            for tau in Tau:
                m.addConstr(
                    quicksum(p_term[i, k, m_id, tau] for m_id in M[k]) <= p_onroute_max[i, k],
                    name=f"cap_onroute[{i},{k},{tau}]"
                )

    # (35)-(36): zero depot charging when not idling at depot
    for k in K:
        for m_id in M[k]:
            idle_set = set(T_depot.get((k, m_id), []))
            for tau in Tau:
                if tau not in idle_set:
                    m.addConstr(p_dep[k, m_id, tau] == 0.0,
                                name=f"no_idle_depot[{k},{m_id},{tau}]")

    # (37): depot charger capacity — per depot, sum across lines served by that depot
    for d in D:
        for tau in Tau:
            m.addConstr(
                quicksum(p_dep[k, m_id, tau]
                         for k in K_by_depot.get(d, [])
                         for m_id in M[k]) <= p_depot_max[d],
                name=f"cap_depot[{d},{tau}]"
            )

    # (28): energy accumulation at terminals (i != o(k))
    for k in K:
        for m_id in M[k]:
            for l_id in L[k][m_id]:
                for i in N[k]:
                    if i != o[k]:
                        m.addConstr(
                            e_dep_term[i, k, m_id, l_id] ==
                            e_arr_term[i, k, m_id, l_id] +
                            eta_on * quicksum(p_term[i, k, m_id, tau]
                                              for tau in T_term.get((i, k, m_id, l_id), [])),
                            name=f"energy_acc_i_not_o[{i},{k},{m_id},{l_id}]"
                        )

    # (29): energy at start terminal i = o(k) using previous loop arrival (for l>=2)
    for k in K:
        for m_id in M[k]:
            i0 = o[k]
            for l_id in L[k][m_id]:
                if l_id == min(L[k][m_id]):
                    continue
                m.addConstr(
                    e_dep_term[i0, k, m_id, l_id] ==
                    e_arr_term[i0, k, m_id, l_id - 1] +
                    eta_on * quicksum(p_term[i0, k, m_id, tau]
                                      for tau in T_term.get((i0, k, m_id, l_id), [])),
                    name=f"energy_acc_o_prev[{i0},{k},{m_id},{l_id}]"
                )

    # (30): energy at depot with depot charging (per depot d)
    for k in K:
        for m_id in M[k]:
            d = d_of[k]
            m.addConstr(
                e_dep_dep[d, k, m_id] ==
                e_arr_dep[d, k, m_id] +
                eta_depot * quicksum(p_dep[k, m_id, tau]
                                     for tau in T_depot.get((k, m_id), [])),
                name=f"energy_acc_depot[{d},{k},{m_id}]"
            )

    # (1): depart depot -> first loop start o(k)
    for k in K:
        for m_id in M[k]:
            d = d_of[k]
            i0 = o[k]
            l1 = min(L[k][m_id])
            m.addConstr(
                e_arr_term[i0, k, m_id, l1] ==
                e_dep_dep[d, k, m_id] - c_dep_to_o[k, m_id],
                name=f"trip_dep_to_o[{d},{k},{m_id}]"
            )

    # (3): along arcs inside each loop
    for k in K:
        for m_id in M[k]:
            for l_id in L[k][m_id]:
                for (i, j) in A[k]:
                    m.addConstr(
                        e_arr_term[j, k, m_id, l_id] ==
                        e_dep_term[i, k, m_id, l_id] - c_arc[k, m_id, l_id, i, j],
                        name=f"arc_energy[{i}->{j},{k},{m_id},{l_id}]"
                    )

    # (6): last loop end -> depot arrival
    for k in K:
        for m_id in M[k]:
            d = d_of[k]
            i0 = o[k]
            l_last = max(L[k][m_id])
            m.addConstr(
                e_arr_dep[d, k, m_id] ==
                e_arr_term[i0, k, m_id, l_last] - c_o_to_dep[k, m_id],
                name=f"trip_o_to_dep[{d},{k},{m_id}]"
            )

    # (8)-(11): SoC / energy bounds
    for k in K:
        for m_id in M[k]:
            d = d_of[k]
            # Lower bounds
            m.addConstr(e_arr_dep[d, k, m_id] >= u_lo[k],
                        name=f"lb_arr_dep[{d},{k},{m_id}]")
            m.addConstr(e_dep_dep[d, k, m_id] >= u_lo[k],
                        name=f"lb_dep_dep[{d},{k},{m_id}]")
            for l_id in L[k][m_id]:
                for i in N[k]:
                    m.addConstr(e_arr_term[i, k, m_id, l_id] >= u_lo[k],
                                name=f"lb_arr_term[{i},{k},{m_id},{l_id}]")
                    m.addConstr(e_dep_term[i, k, m_id, l_id] >= u_lo[k],
                                name=f"lb_dep_term[{i},{k},{m_id},{l_id}]")
            # Upper bounds
            m.addConstr(e_arr_dep[d, k, m_id] <= u_up[k],
                        name=f"ub_arr_dep[{d},{k},{m_id}]")
            m.addConstr(e_dep_dep[d, k, m_id] <= u_up[k],
                        name=f"ub_dep_dep[{d},{k},{m_id}]")
            for l_id in L[k][m_id]:
                for i in N[k]:
                    m.addConstr(e_arr_term[i, k, m_id, l_id] <= u_up[k],
                                name=f"ub_arr_term[{i},{k},{m_id},{l_id}]")
                    m.addConstr(e_dep_term[i, k, m_id, l_id] <= u_up[k],
                                name=f"ub_dep_term[{i},{k},{m_id},{l_id}]")

    # (39): average terminal power per demand period
    for k in K:
        for i in N[k]:
            for z in Z:
                taus = list(T_demand[z])
                denom = max(1, len(taus))
                m.addConstr(
                    pbar_term[i, k, z] ==
                    (1.0 / denom) * quicksum(p_term[i, k, m_id, tau]
                                             for m_id in M[k] for tau in taus),
                    name=f"avg_term[{i},{k},{z}]"
                )

    # (40): average depot power per demand period (per depot, across its lines)
    for d in D:
        for z in Z:
            taus = list(T_demand[z])
            denom = max(1, len(taus))
            m.addConstr(
                pbar_dep[d, z] ==
                (1.0 / denom) * quicksum(p_dep[k, m_id, tau]
                                         for k in K_by_depot.get(d, [])
                                         for m_id in M[k]
                                         for tau in taus),
                name=f"avg_dep[{d},{z}]"
            )

    # (41)-(44): linearization of peak demand
    for k in K:
        for i in N[k]:
            for z in Z:
                if z in Z_peak:
                    m.addConstr(p_on_i[i, k]  >= pbar_term[i, k, z],
                                name=f"peak_on_term[{i},{k},{z}]")
                else:
                    m.addConstr(p_off_i[i, k] >= pbar_term[i, k, z],
                                name=f"peak_off_term[{i},{k},{z}]")

    for d in D:
        for z in Z:
            if z in Z_peak:
                m.addConstr(p_on_d[d]  >= pbar_dep[d, z], name=f"peak_on_dep[{d},{z}]")
            else:
                m.addConstr(p_off_d[d] >= pbar_dep[d, z], name=f"peak_off_dep[{d},{z}]")

    # (38): energy charge (sum over hours with on/off rates)
    EC_on = quicksum(
        c_on * (quicksum(p_term[i, k, m_id, tau]
                         for k in K for m_id in M[k] for i in N[k] for tau in T_hour[h]) +
                quicksum(p_dep[k, m_id, tau]
                         for k in K for m_id in M[k] for tau in T_hour[h]))
        for h in H_peak
    )
    EC_off = quicksum(
        c_off * (quicksum(p_term[i, k, m_id, tau]
                          for k in K for m_id in M[k] for i in N[k] for tau in T_hour[h]) +
                 quicksum(p_dep[k, m_id, tau]
                          for k in K for m_id in M[k] for tau in T_hour[h]))
        for h in set(H) - set(H_peak)
    )
    EC = EC_on + EC_off

    # (26): demand charge (sum over terminals and depots)
    DC = quicksum(lam_on  * p_on_i[i, k]  for k in K for i in N[k]) \
       + quicksum(lam_on  * p_on_d[d]     for d in D) \
       + quicksum(lam_off * p_off_i[i, k] for k in K for i in N[k]) \
       + quicksum(lam_off * p_off_d[d]    for d in D)

    # (27): total daily cost
    m.setObjective(EC + alpha * DC, sense)

    m.Params.OutputFlag = 1
    return m


# -------------------------
# Minimal runnable stub (toy data) — replace with your real data
# -------------------------
if __name__ == "__main__":
    # One depot, one line, one bus, two terminals, two loops (toy)
    D = ["D1"]
    K = ["L1"]
    K_by_depot = {"D1": ["L1"]}
    M = {"L1": ["B1"]}
    N = {"L1": ["A", "B"]}
    L = {"L1": {"B1": [1, 2]}}
    o = {"L1": "A"}
    d_map = {"L1": "D1"}
    A = {"L1": [("A", "B"), ("B", "A")]}

    # Time discretization (toy)
    Tau = [1, 2, 3, 4]
    H = [1, 2]
    H_peak = [2]
    Z = [1, 2]
    Z_peak = [2]
    T_hour = {1: [1, 2], 2: [3, 4]}
    T_demand = {1: [1, 2], 2: [3, 4]}

    # Idling windows
    T_term = {
        ("A", "L1", "B1", 1): [1],
        ("B", "L1", "B1", 1): [2],
        ("A", "L1", "B1", 2): [3],
        ("B", "L1", "B1", 2): [4],
    }
    T_depot = {("L1", "B1"): [1, 2]}

    # Charger limits (kW)
    p_onroute_max = {("A", "L1"): 325.0, ("B", "L1"): 325.0}
    p_depot_max = {"D1": 130.0}

    # Efficiencies
    eta_on = 0.90
    eta_depot = 0.90

    # Battery bounds (kWh)
    u_lo = {"L1": 0.2 * 330.0}
    u_up = {"L1": 0.9 * 330.0}

    # Energy consumptions (kWh)
    c_dep_to_o = {("L1", "B1"): 5.0}
    c_o_to_dep = {("L1", "B1"): 5.0}
    c_arc = {}
    for l_id in L["L1"]["B1"]:
        for (i, j) in A["L1"]:
            c_arc[("L1", "B1", l_id, i, j)] = 10.0  # toy value

    # Prices
    c_on = 0.050209
    c_off = 0.033889
    lam_on = 15.40
    lam_off = 0.0
    alpha = 1.0 / 30.0

    data = dict(
        D=D, K=K, K_by_depot=K_by_depot,
        M=M, N=N, L=L, H=H, H_peak=H_peak, Z=Z, Z_peak=Z_peak, Tau=Tau,
        T_term=T_term, T_depot=T_depot, T_hour=T_hour, T_demand=T_demand,
        o=o, d=d_map, A=A,
        c_dep_to_o=c_dep_to_o, c_o_to_dep=c_o_to_dep, c_arc=c_arc,
        p_onroute_max=p_onroute_max, p_depot_max=p_depot_max,
        eta_on=eta_on, eta_depot=eta_depot,
        u_lo=u_lo, u_up=u_up,
        c_on=c_on, c_off=c_off, lam_on=lam_on, lam_off=lam_off, alpha=alpha
    )

    m = build_model(data)
    m.optimize()

    if m.status == GRB.OPTIMAL:
        print(f"Optimal TC = {m.objVal:.6f}")
