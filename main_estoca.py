import numpy as np
from gurobipy import Model, GRB, quicksum
from datetime import datetime
import os
from generate_data import generate_data
import time
import json
# Semilla para reproducibilidad
np.random.seed(42)

def build_stochastic_model(data):
    print("Construyendo modelo estocástico de dos etapas...")
    
    # CONJUNTOS
    # Conjuntos deterministas
    N, D, K = data['N'], data['D'], data['K']
    N_k, d_k, o_k = data['N_k'], data['d_k'], data['o_k']
    M_bus, L_mk = data['M'], data['L']
    A_k = data['A_k']
    
    # TIEMPO
    PHI, Z, H = data['PHI'], data['Z'], data['H']
    Z_peak, H_peak = set(data['Z_peak']), set(data['H_peak'])
    PHI_h, PHI_z = data['PHI_h'], data['PHI_z']
    PHI_term, PHI_depo = data['PHI_term'], data['PHI_depo']
    
    # ESCENARIOS
    SCENARIOS = data['SCENARIOS'] # Lista de indices de escenarios
    PROBS = data['PROBS']         # Diccionario de probabilidades

    # PARAMETROS
    rho, gamma, omega = data['rho'], data['gamma'], data['omega']
    psi_on, psi_off = data['psi_on'], data['psi_off']
    pi_on, pi_off = data['pi_on'], data['pi_off']
    
    u_k = data['u_k']
    epsilon_up, epsilon_low = data['epsilon_up'], data['epsilon_low']
    
    # Limites Físicos
    p_on_route = data['p_on_route']
    p_depo_cap = data['p_depo']
    theta_on_route = data['theta_on_route']
    theta_depo = data['theta_depo']

    # PARAMETROS ESTOCÁSTICOS
    c_depo_to_term = data['c_depo_to_term'] 
    c_term_to_depo = data['c_term_to_depo']
    c_arc = data['c_arc']
    
    # PENALIZACION
    BIG_M_PENALTY = 1000.0 

    model = Model("Stochastic_Bus_Scheduling")

    #VARIABLES
    print("Creando variables...")

    # ETAPA 1
    sigma_term_on  = model.addVars(N, lb=0.0, vtype=GRB.CONTINUOUS, name="sigma_term_on")
    sigma_term_off = model.addVars(N, lb=0.0, vtype=GRB.CONTINUOUS, name="sigma_term_off")
    sigma_depo_on  = model.addVars(D, lb=0.0, vtype=GRB.CONTINUOUS, name="sigma_depo_on")
    sigma_depo_off = model.addVars(D, lb=0.0, vtype=GRB.CONTINUOUS, name="sigma_depo_off")

    # ETAPA 2
    # Potencia de carga real (p)
    p_term = model.addVars(
        [(i,k,m,phi,s) for k in K for m in M_bus[k] for i in N_k[k] for phi in PHI for s in SCENARIOS],
        lb=0.0, vtype=GRB.CONTINUOUS, name="p_term"
    )
    p_depot = model.addVars(
        [(d_k[k],k,m,phi,s) for k in K for m in M_bus[k] for phi in PHI for s in SCENARIOS],
        lb=0.0, vtype=GRB.CONTINUOUS, name="p_depot"
    )

    # Estado de energía (e)
    e_depo_arr = model.addVars(
        [(d_k[k],k,m,s) for k in K for m in M_bus[k] for s in SCENARIOS],
        lb=0.0, vtype=GRB.CONTINUOUS, name="e_depo_arr"
    )
    e_depo_dep = model.addVars(
        [(d_k[k],k,m,s) for k in K for m in M_bus[k] for s in SCENARIOS],
        lb=0.0, vtype=GRB.CONTINUOUS, name="e_depo_dep"
    )
    e_term_arr = model.addVars(
        [(i,k,m,l,s) for k in K for m in M_bus[k] for l in L_mk[(m,k)] for i in N_k[k] for s in SCENARIOS],
        lb=0.0, vtype=GRB.CONTINUOUS, name="e_term_arr"
    )
    e_term_dep = model.addVars(
        [(i,k,m,l,s) for k in K for m in M_bus[k] for l in L_mk[(m,k)] for i in N_k[k] for s in SCENARIOS],
        lb=0.0, vtype=GRB.CONTINUOUS, name="e_term_dep"
    )
    
    #  Variables de Holgura (Delta) para evitar infactibilidad
    # Si falta batería, delta > 0 y se paga multa.
    delta_depo_arr = model.addVars(
        [(d_k[k],k,m,s) for k in K for m in M_bus[k] for s in SCENARIOS],
        lb=0.0, vtype=GRB.CONTINUOUS, name="delta_depo_arr"
    )
    delta_term_arr = model.addVars(
        [(i,k,m,l,s) for k in K for m in M_bus[k] for l in L_mk[(m,k)] for i in N_k[k] for s in SCENARIOS],
        lb=0.0, vtype=GRB.CONTINUOUS, name="delta_term_arr"
    )

    # Potencia promedio
    eta_term = model.addVars([(i,z,s) for i in N for z in Z for s in SCENARIOS], 
                             lb=0.0, vtype=GRB.CONTINUOUS, name="eta_term")
    eta_depo = model.addVars([(d,z,s) for d in D for z in Z for s in SCENARIOS], 
                             lb=0.0, vtype=GRB.CONTINUOUS, name="eta_depo")


    # RESTRICCIONES
    print("Creando restricciones...")

    for s in SCENARIOS:
        
        # R1: Energía inicial 
        model.addConstrs(
            (e_depo_dep[d_k[k],k,m,s] == epsilon_up * u_k[k] 
             for k in K for m in M_bus[k]),
            name=f"R1_init_S{s}"
        )

        # R2: Deposito a Terminal
        model.addConstrs(
            (e_term_dep[o_k[k], k, m, min(L_mk[(m,k)]), s] ==
             e_depo_dep[d_k[k], k, m, s] - c_depo_to_term[(d_k[k], o_k[k], k, m, s)]
             for k in K for m in M_bus[k]),
            name=f"R2_dep2term_S{s}"
        )

        # R3: Consumo en arcos
        model.addConstrs(
            (e_term_arr[j, k, m, l, s] ==
             e_term_dep[i, k, m, l, s] - c_arc[(i, j, k, m, l, s)]
             for k in K for m in M_bus[k] for l in L_mk[(m,k)] for (i, j) in A_k[k]),
            name=f"R3_arc_S{s}"
        )

        # R6: Terminal a Deposito
        model.addConstrs(
            (e_depo_arr[d_k[k], k, m, s] ==
             e_term_dep[o_k[k], k, m, max(L_mk[(m,k)]), s] - c_term_to_depo[(o_k[k], d_k[k], k, m, s)]
             for k in K for m in M_bus[k]),
            name=f"R6_term2dep_S{s}"
        )

        # R28: Carga en terminal intermedio
        model.addConstrs(
            (e_term_dep[i, k, m, l, s] ==
             e_term_arr[i, k, m, l, s] +
             theta_on_route * quicksum(p_term[i, k, m, phi, s] * rho for phi in PHI_term.get((i, k, m, l), []))
             for k in K for m in M_bus[k] for l in L_mk[(m,k)] for i in N_k[k] if i != o_k[k]),
            name=f"R28_charge_mid_S{s}"
        )
        
        # R29: Carga en terminal inicial
        model.addConstrs(
            (e_term_dep[o_k[k], k, m, l, s] ==
             e_term_arr[o_k[k], k, m, l-1, s] +
             theta_on_route * quicksum(p_term[o_k[k], k, m, phi, s] * rho for phi in PHI_term.get((o_k[k], k, m, l), []))
             for k in K for m in M_bus[k] for l in L_mk[(m,k)] if l >= 2),
            name=f"R29_charge_loop_S{s}"
        )

        # R30: Carga en deposito
        model.addConstrs(
            (e_depo_dep[d_k[k], k, m, s] ==
             e_depo_arr[d_k[k], k, m, s] +
             theta_depo * quicksum(p_depot[d_k[k], k, m, phi, s] * rho for phi in PHI_depo.get((k, m), []))
             for k in K for m in M_bus[k]),
            name=f"R30_charge_depo_S{s}"
        )
         
        # Potencia 0 si no está en ventana de idling
        for k in K:
            for m in M_bus[k]:
                for i in N_k[k]:
                    allowed = set().union(*(set(PHI_term.get((i, k, m, l), [])) for l in L_mk[(m,k)]))
                    for phi in PHI:
                        if phi not in allowed:
                            p_term[i, k, m, phi, s].ub = 0.0 
                            
                allowed_depo = set(PHI_depo.get((k, m), []))
                for phi in PHI:
                    if phi not in allowed_depo:
                        p_depot[d_k[k], k, m, phi, s].ub = 0.0

        # R34: Limite del cargador
        model.addConstrs(
            (quicksum(p_term[i, k, m, phi, s] for m in M_bus[k]) <= p_on_route[(i, k)]
             for k in K for i in N_k[k] for phi in PHI),
            name=f"R34_phys_lim_term_S{s}"
        )
        model.addConstrs(
            (quicksum(p_depot[d_k[k], k, m, phi, s] for k in K for m in M_bus[k] if d_k[k] in d_k[k]) <= p_depo_cap[d]
             for d in D for phi in PHI),
            name=f"R37_phys_lim_depo_S{s}"
        )

        # Demanda promedio
        model.addConstrs(
            (eta_term[i, z, s] == (1.0 / gamma) *
             quicksum(p_term[i, k, m, phi, s] * rho for phi in PHI_z[z] for k in K for m in M_bus[k] if i in N_k[k])
             for i in N for z in Z),
            name=f"calc_eta_term_S{s}"
        )
        model.addConstrs(
            (eta_depo[d, z, s] == (1.0 / gamma) *
             quicksum(p_depot[d, k, m, phi, s] * rho for phi in PHI_z[z] for k in K for m in M_bus[k] if d in d_k[k])
             for d in D for z in Z),
            name=f"calc_eta_depo_S{s}"
        )

        # Baterias deben cubrir demanda mínima (con holgura)
        # Llegada al deposito
        model.addConstrs(
            (e_depo_arr[d_k[k], k, m, s] + delta_depo_arr[d_k[k],k,m,s] >= epsilon_low * u_k[k]
             for k in K for m in M_bus[k]), 
            name=f"lb_depo_arr_S{s}"
        )
        # Llegada a terminal
        model.addConstrs(
            (e_term_arr[i, k, m, l, s] + delta_term_arr[i,k,m,l,s] >= epsilon_low * u_k[k]
             for k in K for m in M_bus[k] for l in L_mk[(m,k)] for i in N_k[k]), 
            name=f"lb_term_arr_S{s}"
        )

        model.addConstrs((e_depo_arr[d_k[k], k, m, s] <= epsilon_up * u_k[k] for k in K for m in M_bus[k]), name=f"ub_depo_arr_S{s}")
        model.addConstrs((e_term_arr[i, k, m, l, s] <= epsilon_up * u_k[k] for k in K for m in M_bus[k] for l in L_mk[(m,k)] for i in N_k[k]), name=f"ub_term_arr_S{s}")


    # La capacidad reservada (Etapa 1) debe ser mayor que la demanda
    print("Creando linking constraints...")
    
    for s in SCENARIOS:
        model.addConstrs((sigma_term_on[i]  >= eta_term[i, z, s] for i in N for z in Z if z in Z_peak),  name=f"link_term_on_S{s}")
        model.addConstrs((sigma_term_off[i] >= eta_term[i, z, s] for i in N for z in Z if z not in Z_peak), name=f"link_term_off_S{s}")
        model.addConstrs((sigma_depo_on[d]  >= eta_depo[d, z, s] for d in D for z in Z if z in Z_peak),  name=f"link_depo_on_S{s}")
        model.addConstrs((sigma_depo_off[d] >= eta_depo[d, z, s] for d in D for z in Z if z not in Z_peak), name=f"link_depo_off_S{s}")



    # FUNCIÓN OBJETIVO
    print("Creando función objetivo...")
    
    # Cargos por demanda
    DC_cost = pi_on  * (quicksum(sigma_term_on[i]  for i in N) + quicksum(sigma_depo_on[d]  for d in D)) \
            + pi_off * (quicksum(sigma_term_off[i] for i in N) + quicksum(sigma_depo_off[d] for d in D))

    # Coste del energía
    Expected_Second_Stage = 0
    
    for s in SCENARIOS:
        prob = PROBS[s]
        
        # EC por escenario
        EC_on_s = quicksum(psi_on * (quicksum(p_term[i,k,m,phi,s]*rho for phi in PHI_h[h] for i in N_k[k]) +
                                     quicksum(p_depot[d_k[k],k,m,phi,s]*rho for phi in PHI_h[h]))
                           for k in K for m in M_bus[k] for h in H if h in H_peak)
                           
        EC_off_s = quicksum(psi_off * (quicksum(p_term[i,k,m,phi,s]*rho for phi in PHI_h[h] for i in N_k[k]) +
                                       quicksum(p_depot[d_k[k],k,m,phi,s]*rho for phi in PHI_h[h]))
                            for k in K for m in M_bus[k] for h in H if h not in H_peak)
                            
        # Penalizacion 
        Penalty_s = BIG_M_PENALTY * (
            quicksum(delta_depo_arr[d_k[k],k,m,s] for k in K for m in M_bus[k]) +
            quicksum(delta_term_arr[i,k,m,l,s] for k in K for m in M_bus[k] for l in L_mk[(m,k)] for i in N_k[k])
        )
        
        Expected_Second_Stage += prob * (EC_on_s + EC_off_s + Penalty_s)

    # Objetivo Total
    model.setObjective(omega * DC_cost + Expected_Second_Stage, GRB.MINIMIZE)

    print("Modelo construido correctamente.")
    return model

def generate_stochastic_data():
    """Genera una instancia SIMPLE con escenarios estocásticos"""
    
    # Configuracion Básica (1 Linea, 2 Buses, 1 Deposito, 2 Terminales)
    N = ["A", "B"]
    D = ["D1"]
    K = ["L1"] # Solo una linea
    
    N_k = {"L1": ["A", "B"]}
    d_k = {"L1": ["D1"]}
    o_k = {"L1": "A"}
    A_k = {"L1": [("A","B"), ("B","A")]}
    M   = {"L1": ["B1", "B2"]}
    L_mk = {("B1","L1"): [1, 2], ("B2","L1"): [1, 2]} # 2 vueltas cada uno
    
    # Tiempo
    HOURS = 24
    RES_MIN = 60 # Bloques de 1 hora para que sea rapido
    SLOTS_PER_HOUR = 60 // RES_MIN
    T = HOURS * SLOTS_PER_HOUR
    PHI = list(range(1, T + 1))
    H = list(range(HOURS))
    PHI_h = {h: list(range(h*SLOTS_PER_HOUR+1, (h+1)*SLOTS_PER_HOUR+1)) for h in H}
    Z = H[:]
    PHI_z = {z: PHI_h[z] for z in Z}
    
    # Horas punta (ej. tarde)
    H_peak = {18, 19, 20, 21}
    Z_peak = list(H_peak)
    
    rho = 1.0 / SLOTS_PER_HOUR
    gamma = SLOTS_PER_HOUR
    
    # Ventanas de carga (Simplificadas para que calcen)
    # Asumimos que B1 llega a B a las 10am y a las 14pm, etc.
    PHI_term = {
        ("B", "L1", "B1", 1): PHI_h[10],
        ("B", "L1", "B1", 2): PHI_h[14],
        ("B", "L1", "B2", 1): PHI_h[11],
        ("B", "L1", "B2", 2): PHI_h[15]
    }
    PHI_depo = {
        ("L1", "B1"): PHI_h[23] + PHI_h[0] + PHI_h[1], # Carga nocturna
        ("L1", "B2"): PHI_h[23] + PHI_h[0] + PHI_h[1]
    }
    
    # Parametros Deterministas
    u_k = {"L1": 300.0} # Bateria chica para forzar carga
    epsilon_up, epsilon_low = 0.95, 0.15
    p_on_route = {("A","L1"): 400.0, ("B","L1"): 400.0}
    p_depo = {"D1": 100.0}
    theta_on_route, theta_depo = 0.95, 0.95
    psi_on, psi_off = 100.0, 50.0  # Precios Energia
    pi_on, pi_off = 2000.0, 500.0  # Precios Demanda
    omega = 1.0/30.0

    # --- GENERACIÓN ESTOCÁSTICA ---
    NUM_SCENARIOS = 5 # 5 Escenarios
    SCENARIOS = list(range(NUM_SCENARIOS))
    PROBS = {s: 1.0/NUM_SCENARIOS for s in SCENARIOS} # Equiprobables
    
    # Consumo Base
    base_c_arc = 60.0 # kWh por tramo
    base_c_depo = 10.0
    
    # Diccionarios estocásticos
    c_depo_to_term_scen = {}
    c_term_to_depo_scen = {}
    c_arc_scen = {}
    
    for s in SCENARIOS:
        # Factor de variabilidad (Normal centrada en 1.0, desv std 0.2)
        # Algunos escenarios consumiran 20% más, otros menos.
        factor = np.random.normal(1.0, 0.2) 
        factor = max(0.5, factor) # Evitar negativos
        
        # Llenar diccionarios
        # Deposito -> Terminal
        for k in K:
            for m in M[k]:
                d = d_k[k]
                o = o_k[k]
                c_depo_to_term_scen[(d,o,k,m,s)] = base_c_depo * factor
                c_term_to_depo_scen[(o,d,k,m,s)] = base_c_depo * factor
                
                # Arcos
                for l in L_mk[(m,k)]:
                    for (i,j) in A_k[k]:
                         # Variabilidad extra por arco
                        local_factor = np.random.normal(factor, 0.05)
                        c_arc_scen[(i,j,k,m,l,s)] = base_c_arc * local_factor

    # Empaquetar
    data = {
        'N': N, 'D': D, 'K': K, 'N_k': N_k, 'd_k': d_k, 'o_k': o_k, 'A_k': A_k, 'M': M, 'L': L_mk,
        'PHI': PHI, 'Z': Z, 'H': H, 'Z_peak': Z_peak, 'H_peak': H_peak, 'PHI_h': PHI_h, 'PHI_z': PHI_z,
        'PHI_term': PHI_term, 'PHI_depo': PHI_depo,
        'rho': rho, 'gamma': gamma, 'omega': omega,
        'psi_on': psi_on, 'psi_off': psi_off, 'pi_on': pi_on, 'pi_off': pi_off,
        'u_k': u_k, 'epsilon_up': epsilon_up, 'epsilon_low': epsilon_low,
        'p_on_route': p_on_route, 'p_depo': p_depo, 'theta_on_route': theta_on_route, 'theta_depo': theta_depo,
        # Estocasticos
        'SCENARIOS': SCENARIOS, 'PROBS': PROBS,
        'c_depo_to_term_scen': c_depo_to_term_scen,
        'c_term_to_depo_scen': c_term_to_depo_scen,
        'c_arc_scen': c_arc_scen
    }
    return data

def export_parameters(data, filepath):
    """Exporta los parámetros estocásticos a un archivo"""
    
    os.makedirs("parameters", exist_ok=True)
    
    # Convertir diccionarios complejos a formatos serializables
    data_to_export = {}
    for key, value in data.items():
        if isinstance(value, dict):
            # Convert dict with tuple keys to dict with string keys
            data_to_export[key] = {str(k): v for k, v in value.items()}
        elif isinstance(value, list):
            data_to_export[key] = value
        else:
            data_to_export[key] = value
    
    with open(filepath, 'w') as f:
        json.dump(data_to_export, f, indent=2, default=str)
    
    print(f"Parámetros exportados a: {filepath}")

if __name__ == "__main__":
    ini_time = time.time()
    exec_time = datetime.now().strftime("%m-%d_%H-%M-%S")
    data = generate_data(num_lineas = 1, num_buses = 20, num_vueltas = 100, hora_inicio = 5)
    export_parameters(data, f"parameters/params_estoca_{exec_time}.json")
    # data = generate_data_224()
    model = build_stochastic_model(data)
    
    model.Params.OutputFlag = 1
    model.optimize()
    
    if model.status == GRB.OPTIMAL:
        print(f"\nOBJETIVO ÓPTIMO: {model.objVal:.2f}")
        
        
        logs_base = "logs"
        paths = [os.path.join(logs_base, "constraints"), os.path.join(logs_base, "sols"), os.path.join(logs_base, "ilps")]
        for p in paths:
            os.makedirs(p, exist_ok=True)

        model.write(os.path.join(logs_base, "sols", f"solucion_estoca_20b_100l.sol"))
        for v in model.getVars():
            if "sigma" in v.VarName and v.X > 0.01:
                print(f"{v.VarName}: {v.X:.2f} kW")
        
        total_penalty = 0
        for v in model.getVars():
            if "delta" in v.VarName and v.X > 0.01:
                print(f"MULTA ACTIVADA en {v.VarName}: {v.X:.2f} kWh faltantes")
                total_penalty += v.X
        
        if total_penalty == 0:
            print("El sistema es robusto: Ningún escenario violó los límites de batería.")
    else:
        print("No se encontró solución óptima.")
    
    fin_time = time.time()
    print(f"\nTiempo total de ejecución: {fin_time - ini_time:.2f} segundos.")