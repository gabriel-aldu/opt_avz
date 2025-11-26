from gurobipy import Model, GRB, quicksum
from datetime import datetime
import os
from generate_data import generate_data
import json
from main_estoca import export_parameters

def build_model(data):

    print("Cargando datos...")
    # CONJUNTOS
    N = data['N'] # Todo los terminales
    D = data['D'] # Todos los depositos
    K = data['K'] # Todas las lineas

    N_k = data['N_k'] # Terminales en la linea k
    A_k = data['A_k'] # Arcos en la linea k
    M = data['M'] # Buses en la linea k
    L_mk = data['L'] # Viajes del bus m en la linea k

    PHI = data['PHI'] # Tiempo discretizado
    Z = data['Z'] # Periodos de demanda
    H = data['H'] # Horas
    Z_peak = set(data['Z_peak']) # Periodos de demanda peak
    H_peak = set(data['H_peak']) # Horas peak (para costos de energia)
    PHI_h  = data['PHI_h'] # Periodos de tiempo que pertenece a cada hora h
    PHI_z  = data['PHI_z'] # Periodos de tiempo que pertenece a cada periodo de demanda z
    PHI_term = data['PHI_term'] # Ventanas de idling en terminales
    PHI_depo = data['PHI_depo'] # Ventanas de idling en depositos

    # PARAMETROS
    d_k = data['d_k'] # Depositos de la linea k
    o_k = data['o_k'] # Terminal inicial de la linea k
    rho = data['rho'] # Peso del paso de tiempo (1 si es horario)

    u_k = data['u_k'] # Capacidad de bateria para los buses de la linea k (kWh)
    epsilon_up = data['epsilon_up'] # Cota superior de la bateria (fraccion de la capacidad)
    epsilon_low = data['epsilon_low'] # Cota inferior de la bateria (fraccion de la capacidad)

    c_depo_to_term = data['c_depo_to_term'] # Consumo de energia del deposito al terminal (kWh)
    c_term_to_depo = data['c_term_to_depo'] # Consumo de energia del terminal al deposito (kWh)
    c_arc = data['c_arc'] # Consumo de energia en cada arco (kWh)

    p_on_route   = data['p_on_route'] # Capacidad de los cargadores en ruta (kWh)
    p_depo_cap  = data['p_depo'] # Capacidad de los cargadores en el deposito (kWh)

    theta_on_route = data['theta_on_route'] # Eficiencia de los cargadores en ruta
    theta_depo     = data['theta_depo'] # Eficiencia de los cargadores en el deposito

    gamma = data['gamma'] # Divisor para el promedio de potencia (|Φ_z| en el paper)

    psi_on  = data['psi_on'] # Precio de energia en horas peak ($/kWh)
    psi_off = data['psi_off'] # Precio de energia fuera de  horas peak ($/kWh)

    pi_on  = data['pi_on'] # Precio de los cargos por demanda en horas peak ($/kW)
    pi_off = data['pi_off'] # Precio de los cargos por demanda fuera de horas peak ($/kW)

    omega = data['omega'] # Peso del cargo por demanda (depende de cuando de cobran estos cargos por demanda)

    # MODELO
    model = Model("Bus_Scheduling")

    # VARIABLES
    print("Creando variables...")
    # Energia en los depositos (kWh)
    e_depo_arr = model.addVars(
        [(d_k[k],k,m) for k in K for m in M[k]],
        lb=0.0, vtype=GRB.CONTINUOUS, name="e_depo_arr"
    )
    e_depo_dep = model.addVars(
        [(d_k[k],k,m) for k in K for m in M[k]],
        lb=0.0, vtype=GRB.CONTINUOUS, name="e_depo_dep"
    )

    # Energia en los terminales (kWh)
    e_term_arr = model.addVars(
        [(i,k,m,l) for k in K for m in M[k] for l in L_mk[(m,k)] for i in N_k[k]],
        lb=0.0, vtype=GRB.CONTINUOUS, name="e_term_arr"
    )
    e_term_dep = model.addVars(
        [(i,k,m,l) for k in K for m in M[k] for l in L_mk[(m,k)] for i in N_k[k]],
        lb=0.0, vtype=GRB.CONTINUOUS, name="e_term_dep"
    )

    # Potencia de carga (kW)
    p_term = model.addVars(
        [(i,k,m,phi) for k in K for m in M[k] for i in N_k[k] for phi in PHI],
        lb=0.0, vtype=GRB.CONTINUOUS, name="p_term"
    )
    p_depot = model.addVars(
        [(d_k[k],k,m,phi) for k in K for m in M[k] for phi in PHI],
        lb=0.0, vtype=GRB.CONTINUOUS, name="p_depot"
    )

    # Potencia promedio en periodos de demanda (kW)
    eta_term = model.addVars([(i,z) for i in N for z in Z], lb=0.0, vtype=GRB.CONTINUOUS, name="eta_term")
    eta_depo = model.addVars([(d,z) for d in D for z in Z], lb=0.0, vtype=GRB.CONTINUOUS, name="eta_depo")

    # Potencia peak
    sigma_term_on  = model.addVars(N, lb=0.0, vtype=GRB.CONTINUOUS, name="sigma_term_on")
    sigma_term_off = model.addVars(N, lb=0.0, vtype=GRB.CONTINUOUS, name="sigma_term_off")
    sigma_depo_on  = model.addVars(D, lb=0.0, vtype=GRB.CONTINUOUS, name="sigma_depo_on")
    sigma_depo_off = model.addVars(D, lb=0.0, vtype=GRB.CONTINUOUS, name="sigma_depo_off")

    # RESTRICCIONES
    print("Creando restricciones...")
    # R1: Energia inicial en el deposito
    model.addConstrs(
        (e_depo_dep[d_k[k],k,m] == epsilon_up * u_k[k] for k in K for m in M[k]),
        name="initial_soc_depo"
    )

    # R2: Consumo del deposito al terminal (primer viaje, l=1)
    model.addConstrs(
        (e_term_dep[o_k[k], k, m, min(L_mk[(m,k)])] ==
         e_depo_dep[d_k[k], k, m] - c_depo_to_term[(d_k[k], o_k[k], k, m)]
         for k in K for m in M[k]),
        name="soc_depo_to_term"
    )

    # R3: Consumo en cada arco (i,j) en la linea k
    model.addConstrs(
        (e_term_arr[j, k, m, l] ==
         e_term_dep[i, k, m, l] - c_arc[(i, j, k, m, l)]
         for k in K for m in M[k] for l in L_mk[(m,k)] for (i, j) in A_k[k]),
        name="soc_arc"
    )

    # R6: Consumo del terminal al deposito (ultimo viaje, l=max)
    model.addConstrs(
        (e_depo_arr[d_k[k], k, m] ==
         e_term_dep[o_k[k], k, m, max(L_mk[(m,k)])] - c_term_to_depo[(o_k[k], d_k[k], k, m)]
         for k in K for m in M[k]),
        name="soc_term_to_depo"
    )

    # R28: Energia cargada en terminal que no sea la inicial
    model.addConstrs(
        (e_term_dep[i, k, m, l] ==
         e_term_arr[i, k, m, l] +
         theta_on_route * quicksum(p_term[i, k, m, phi] * rho for phi in PHI_term.get((i, k, m, l), []))
         for k in K for m in M[k] for l in L_mk[(m,k)] for i in N_k[k] if i != o_k[k]),
        name="soc_not_o_term"
    )

    # R29: Energia cargada en el terminal inicial
    model.addConstrs(
        (e_term_dep[o_k[k], k, m, l] ==
         e_term_arr[o_k[k], k, m, l-1] +
         theta_on_route * quicksum(p_term[o_k[k], k, m, phi] * rho
                                   for phi in PHI_term.get((o_k[k], k, m, l), []))
         for k in K for m in M[k] for l in L_mk[(m,k)] if l >= 2),
        name="soc_o_term"
    )

    # R30: Energia carga en el deposito
    model.addConstrs(
        (e_depo_dep[d_k[k], k, m] ==
         e_depo_arr[d_k[k], k, m] +
         theta_depo * quicksum(p_depot[d_k[k], k, m, phi] * rho for phi in PHI_depo.get((k, m), []))
         for k in K for m in M[k]),
        name="soc_depo"
    )


    # R32/R33: Potencia de carga en terminales donde el bus no esta idling debe ser 0
    for k in K:
        for m in M[k]:
            for i in N_k[k]:
                allowed = set().union(*(set(PHI_term.get((i, k, m, l), [])) for l in L_mk[(m,k)]))
                for phi in PHI:
                    if phi not in allowed:
                        model.addConstr(p_term[i, k, m, phi] == 0.0,
                                        name=f"p_term_not_idle[{i},{k},{m},{phi}]")

    # R34: Consumo de potencia maxima en terminales
    model.addConstrs(
        (quicksum(p_term[i, k, m, phi] for m in M[k]) <= p_on_route[(i, k)]
         for k in K for i in N_k[k] for phi in PHI),
        name="p_term_on_route_limit"
    )

    # R36: Potencia de carga en depositos donde el bus no esta idling debe ser 0
    for k in K:
        for m in M[k]:
            allowed = set(PHI_depo.get((k, m), []))
            for phi in PHI:
                if phi not in allowed:
                    model.addConstr(p_depot[d_k[k], k, m, phi] == 0.0,
                                    name=f"p_depot_not_idle[{d_k[k]},{k},{m},{phi}]")

    # R37: Consumo de potencia maxima en depositos
    model.addConstrs(
        (quicksum(p_depot[d, k, m, phi] for k in K for m in M[k] if d in d_k[k]) <= p_depo_cap[d]
         for d in D for phi in PHI),
        name="p_depot_limit"
    )

    # R39: Potencia promedio por terminal en periodo de demanda z
    model.addConstrs(
        (eta_term[i, z] ==
         (1.0 / gamma) *
         quicksum(p_term[i, k, m, phi] * rho for phi in PHI_z[z] for k in K for m in M[k] if i in N_k[k])
         for i in N for z in Z),
        name="average_eta_term"
    )

    # R40: Potencia promedio por deposito en periodo de demanda z
    model.addConstrs(
        (eta_depo[d, z] ==
         (1.0 / gamma) *
         quicksum(p_depot[d_k[k], k, m, phi] * rho for phi in PHI_z[z] for k in K for m in M[k])
         for d in D for z in Z),
        name="average_eta_depo"
    )

    # R8-11: Cotas inferiores y superiored de las baterias 
    # Cota inferior
    model.addConstrs((e_depo_arr[d_k[k], k, m] >= epsilon_low * u_k[k]
                      for k in K for m in M[k]), name="lb_depo_arr")
    model.addConstrs((e_depo_dep[d_k[k], k, m] >= epsilon_low * u_k[k]
                      for k in K for m in M[k]), name="lb_depo_dep")
    model.addConstrs((e_term_arr[i, k, m, l] >= epsilon_low * u_k[k]
                      for k in K for m in M[k] for l in L_mk[(m,k)] for i in N_k[k]), name="lb_term_arr")
    model.addConstrs((e_term_dep[i, k, m, l] >= epsilon_low * u_k[k]
                      for k in K for m in M[k] for l in L_mk[(m,k)] for i in N_k[k]), name="lb_term_dep")

    # Cota superior
    model.addConstrs((e_depo_arr[d_k[k], k, m] <= epsilon_up * u_k[k]
                      for k in K for m in M[k]), name="ub_depo_arr")
    model.addConstrs((e_depo_dep[d_k[k], k, m] <= epsilon_up * u_k[k]
                      for k in K for m in M[k]), name="ub_depo_dep")
    model.addConstrs((e_term_arr[i, k, m, l] <= epsilon_up * u_k[k]
                      for k in K for m in M[k] for l in L_mk[(m,k)] for i in N_k[k]), name="ub_term_arr")
    model.addConstrs((e_term_dep[i, k, m, l] <= epsilon_up * u_k[k]
                      for k in K for m in M[k] for l in L_mk[(m,k)] for i in N_k[k]), name="ub_term_dep")

    #  R45-48: Demanda peak 
    model.addConstrs((sigma_term_on[i]  >= eta_term[i, z] for i in N for z in Z if z in Z_peak),  name="sigma_term_on")
    model.addConstrs((sigma_term_off[i] >= eta_term[i, z] for i in N for z in Z if z not in Z_peak), name="sigma_term_off")
    model.addConstrs((sigma_depo_on[d]  >= eta_depo[d, z] for d in D for z in Z if z in Z_peak),  name="sigma_depo_on")
    model.addConstrs((sigma_depo_off[d] >= eta_depo[d, z] for d in D for z in Z if z not in Z_peak), name="sigma_depo_off")

    # OBJECTIVE
    print("Creando funcion objetivo...")
    # Cargos por energia
    # En horas peak
    EC_on =  quicksum(
        psi_on * (
            quicksum(p_term[i, k, m, phi] * rho for phi in PHI_h[h] for i in N_k[k]) +
            quicksum(p_depot[d_k[k], k, m, phi] * rho for phi in PHI_h[h])
        )
        for k in K for m in M[k] for h in H if h in H_peak
    )
    # Fuera de horas peak
    EC_off = quicksum(
        psi_off * (
            quicksum(p_term[i, k, m, phi] * rho for phi in PHI_h[h] for i in N_k[k]) +
            quicksum(p_depot[d_k[k], k, m, phi] * rho for phi in PHI_h[h])
        )
        for k in K for m in M[k] for h in H if h not in H_peak
    )
    EC = EC_on + EC_off

    # Cargos por demanda
    DC = pi_on  * (quicksum(sigma_term_on[i]  for i in N) + quicksum(sigma_depo_on[d]  for d in D)) \
       + pi_off * (quicksum(sigma_term_off[i] for i in N) + quicksum(sigma_depo_off[d] for d in D))
    model.setObjective(EC + omega * DC, GRB.MINIMIZE)



    print("Modelo listo!")
    return model


def load_data():
    """ Se carga una instancia de datos de prueba
        para resolver el problema y verificar su funcionamiento
    """
    N = ["A", "B", "C", "D"]
    D     = ["D1"]
    K     = ["L1", "L2"]

    N_k   = {
        "L1": ["A", "B"], 
        "L2": ["C", "D"]
        }
    A_k   = {
        "L1": [("A","B"), ("B","A")],
        "L2": [("C","D"), ("D","C")]
        }
    M     = {
        "L1": ["B1", "B2"],
        "L2": ["B3", "B4"]
        }
    L_mk  = {
        ("B1","L1"): [1, 2],
        ("B2","L1"): [1, 2],
        ("B3","L2"): [1, 2],
        ("B4","L2"): [1, 2]
        }

    o_k   = {
        "L1": "A",
        "L2": "C"
        }      
    d_k   = {
        "L1": ["D1"],
        "L2": ["D1"]
        }                  

    # Parametros de tiempo
    HOURS = 24
    RES_MIN = 60          # Resolucion en minutos
    H = list(range(HOURS))   
    SLOTS_PER_HOUR = 60 // RES_MIN
    T = HOURS * SLOTS_PER_HOUR

    # Periodos de tiempo
    PHI = list(range(1, T + 1))

    # Periodos de tiempo pro hora
    PHI_h = {
        h: list(range(h * SLOTS_PER_HOUR + 1, (h + 1) * SLOTS_PER_HOUR + 1))
        for h in H
    }

    # Periodos de demanda (se ocupan las horas)
    Z = H[:]
    PHI_z = {z: PHI_h[z] for z in Z}

    # Horas peak
    ON_PEAK_HOURS = {4}
    H_peak = list(ON_PEAK_HOURS)
    Z_peak = list(ON_PEAK_HOURS)   

    # Peso del paso de tiempo y divisor para el promedio de potencia
    rho = 1.0 / SLOTS_PER_HOUR
    gamma = SLOTS_PER_HOUR

    # Tener mucho cuidado con las ventanas de idling
    # Si el gasto de energia entre nodos es muy alto (inluso de 10 kWh)
    # Este se tiene que cargar en cada nodo en todas las ventanas posibles

    # Ventanas de idling en terminales
    PHI_term = {
    ("B", "L1", "B1", 1): PHI_h[3],
    ("B", "L1", "B1", 2): PHI_h[6],
    ("B", "L1", "B2", 1): PHI_h[3],
    ("B", "L1", "B2", 2): PHI_h[6],
    ("D", "L2", "B3", 2): PHI_h[6],
    ("D", "L2", "B4", 2): PHI_h[6]
    }

    # Ventanas de idling en depositos
    PHI_depo = {
        ("L1", "B1"): PHI_h[7],
        ("L1", "B2"): PHI_h[7],
        ("L2", "B3"): PHI_h[7],
        ("L2", "B4"): PHI_h[7],
    }

    # Parametros de las baterias
    u_k = {
        "L1": 500.0,
        "L2": 600.0
        }
    epsilon_low, epsilon_up = 0.2, 0.9

    # Consumo de energia (kWh)

    # Consumo del deposito al terminal y viceversa
    c_depo_to_term = { 
        ("D1","A","L1","B1"): 5.0,
        ("D1","A","L1","B2"): 5.0,
        ("D1","C","L2","B3"): 7.0,
        ("D1","C","L2","B4"): 7.0
        }
    c_term_to_depo = {
        ("A","D1","L1","B1"): 5.0,
        ("A","D1","L1","B2"): 5.0,
        ("C","D1","L2","B3"): 7.0,
        ("C","D1","L2","B4"): 7.0,

        }
    
    # Consumo en cada arco
    c_arc = {
        ("A","B","L1","B1",1): 150.0,
        ("B","A","L1","B1",1): 150.0,
        ("A","B","L1","B1",2): 150.0,
        ("B","A","L1","B1",2): 150.0,
        ("A","B","L1","B2",1): 150.0,
        ("B","A","L1","B2",1): 150.0,
        ("A","B","L1","B2",2): 150.0,
        ("B","A","L1","B2",2): 150.0,
        ("C","D","L2","B3",1): 20.0,
        ("D","C","L2","B3",1): 20.0,
        ("C","D","L2","B3",2): 20.0,
        ("D","C","L2","B3",2): 20.0,
        ("C","D","L2","B4",1): 20.0,
        ("D","C","L2","B4",1): 20.0,
        ("C","D","L2","B4",2): 20.0,
        ("D","C","L2","B4",2): 20.0
        }

    # Capacidad de los cargadores
    p_on_route  = {("A","L1"): 500.0,
                   ("B","L1"): 500.0,
                   ("C","L2"): 500.0, 
                   ("D","L2"): 500.0
                   }
    
    p_depo_cap  = {"D1": 1500.0}

    # Eficiencia de los cargadores
    theta_on_route = 0.9
    theta_depo     = 0.9

    # Precios
    psi_on, psi_off = 2.0, 1.0   # $/kWh
    pi_on,  pi_off  = 15.40 * 30, 2.0 * 30          # $/kW
    omega = 1.0/30.0

    data = dict(
        N=N, D=D, K=K, N_k=N_k, A_k=A_k, M=M, L=L_mk,
        PHI=PHI, Z=Z, H=H, Z_peak=Z_peak, H_peak=H_peak, PHI_h=PHI_h, PHI_z=PHI_z,
        PHI_term=PHI_term, PHI_depo=PHI_depo,
        d_k=d_k, o_k=o_k, rho=rho, u_k=u_k, epsilon_up=epsilon_up, epsilon_low=epsilon_low,
        c_depo_to_term=c_depo_to_term, c_term_to_depo=c_term_to_depo, c_arc=c_arc,
        p_on_route=p_on_route, p_depo=p_depo_cap,
        theta_on_route=theta_on_route, theta_depo=theta_depo,
        gamma=len(PHI_z[1]), 
        psi_on=psi_on, psi_off=psi_off, pi_on=pi_on, pi_off=pi_off, omega=omega
    )
    return data


if __name__ == "__main__":

    data = generate_data(num_lineas = 1, num_buses = 2, num_vueltas = 12, hora_inicio = 5, stochastic=False)
    model = build_model(data)
    model.Params.OutputFlag = 1
    model.optimize()
    print("Resolviendo modelo...")

    exec_time = datetime.now().strftime("%m-%d_%H-%M-%S")
    export_parameters(data, f"parameters/params_det_{exec_time}.json")
    logs_base = "logs"
    paths = [os.path.join(logs_base, "constraints"), os.path.join(logs_base, "sols"), os.path.join(logs_base, "ilps")]
    for p in paths:
        os.makedirs(p, exist_ok=True)

    if model.status == GRB.OPTIMAL:
        print(f"Valor objetivo = {model.objVal:.6f}")

        constraints_file = os.path.join(logs_base, "constraints", f"constraints_{exec_time}.csv")
        with open(constraints_file, "w") as f:
            f.write("Constraint,Slack\n")
            for constr in model.getConstrs():
                f.write(f"{constr.ConstrName},{constr.Slack}\n")

        model.write(os.path.join(logs_base, "sols", f"solucion_det_{exec_time}.sol"))

    elif model.status == GRB.INFEASIBLE:
        print("Modelo infactible")
        model.computeIIS()
        model.write(os.path.join(logs_base, "ilps", f"model_{exec_time}.ilp"))
    else:
        print("Modelo fallo con estado:", model.status)
