import numpy as np
import random

def generate_data(num_lineas = 2, num_buses = 4, num_vueltas = 12, hora_inicio = 5, stochastic = True):
    """Genera una instancia SIMPLE con escenarios estocásticos"""
    
    # Configuracion Básica (1 Linea, 2 Buses, 1 Deposito, 2 Terminales)
    K = [f"L{i + 1}" for i in range(num_lineas)] # Solo una linea
    N = [f"{linea}-A" for linea in K] + [f"{linea}-B" for linea in K]  # 2 nodos por linea
    D = [f"D{i + 1}" for i in range(num_lineas // 3 + 1)] # 1 deposito cada 5 lineas
    
    
    N_k = {K[i]: [N[i],N[i + num_lineas]] for i in range(num_lineas)}
    d_k = {K[i]: D[i // 3] for i in range(num_lineas)} # 1 deposito cada 3 lineas
    o_k = {K[i]: N[i] for i in range(num_lineas)} #Parten siempre en el terminal A
    A_k = {K[i]: [(N[i], N[i + num_lineas]), (N[i + num_lineas], N[i])] for i in range(num_lineas)} # Ida y vuelta
    M  = {K[i]: [f"B{j + 1}" for j in range(num_buses)] for i in range(num_lineas)} # 2 buses por linea
    L_mk = {(M[K[i]][j], K[i]) : [v for v in range(num_vueltas)] for j in range(num_buses) for i in range(num_lineas)} # 2 vueltas cada uno
    
    # Tiempo
    HOURS = 24
    RES_MIN = 1 # Bloques de 1 hora para que sea rapido
    SLOTS_PER_HOUR = 60 // RES_MIN
    T = HOURS * SLOTS_PER_HOUR
    PHI = list(range(1, T + 1))
    H = list(range(HOURS))
    PHI_h = {h: list(range(h*SLOTS_PER_HOUR+1, (h+1)*SLOTS_PER_HOUR+1)) for h in H}
    Z = H[:]
    PHI_z = {z: PHI_h[z] for z in Z}
    
    # Horas punta (ej. tarde)
    H_peak = {7, 8 , 9 , 10 , 18, 19, 20, 21}
    Z_peak = list(H_peak)
    
    rho = 1.0 / SLOTS_PER_HOUR
    gamma = SLOTS_PER_HOUR
    
    # Ventanas de carga (Simplificadas para que calcen)
    # Asumimos que B1 llega a B a las 10am y a las 14pm, etc.
    PHI_term = {
        #("B", "L1", "B1", 1): PHI_h[10],
        (N_k[K[i]][i], K[i], M[K[i]][j], v + 1): PHI_h[hora_inicio + v] for i in range(num_lineas) for j in range(num_buses) for v in range(num_vueltas)
    
    }
    PHI_depo = {
        # ("L1", "B1"): PHI_h[23] + PHI_h[0] + PHI_h[1], # Carga nocturna
        (K[i], M[K[i]][j]): PHI_h[23] + PHI_h[0] + PHI_h[1] + PHI_h[2] + PHI_h[3] for i in range(num_lineas) for j in range(num_buses)
    }
    
    # Parametros Deterministas
    u_k = {K[i]: 350.0 for i in range(num_lineas)} # Bateria chica para forzar carga
    epsilon_up, epsilon_low = 0.95, 0.15
    p_on_route = {(N_k[k][i], k): 1500.0 for i in range(2) for k in K} # Capacidad de carga en ruta
    p_depo = {D[i]: 500.0 for i in range(len(D))} # Capacidad de carga en deposito
    theta_on_route, theta_depo = 0.95, 0.95
    psi_on, psi_off = 120.0, 60.0  # Precios Energia
    pi_on, pi_off = 3000.0, 1000.0  # Precios Demanda
    omega = 1.0/30.0

    # --- GENERACIÓN ESTOCÁSTICA ---
    NUM_SCENARIOS = 5 # 5 Escenarios
    SCENARIOS = list(range(NUM_SCENARIOS))
    PROBS = {s: 1.0/NUM_SCENARIOS for s in SCENARIOS} # Equiprobables
    
    # Consumo Base
    base_c_arc = 38.0 # kWh por tramo
    base_c_depo = 5.0
    
    # Diccionarios estocásticos
    c_depo_to_term = {}
    c_term_to_depo = {}
    c_arc = {}
    
    if stochastic:
        for s in SCENARIOS:
            # Factor de variabilidad (Normal centrada en 1.0, desv std 0.2)
            # Algunos escenarios consumiran 20% más, otros menos.
            factor = np.random.normal(1.0, 0.15) 
            factor = max(1.3, factor) # Evitar negativos
            
            # Llenar diccionarios
            # Deposito -> Terminal
            for k in K:
                for m in M[k]:
                    d = d_k[k]
                    o = o_k[k]
                    c_depo_to_term[(d,o,k,m,s)] = base_c_depo * factor
                    c_term_to_depo[(o,d,k,m,s)] = base_c_depo * factor
                    
                    # Arcos
                    for l in L_mk[(m,k)]:
                        for (i,j) in A_k[k]:
                            # Variabilidad extra por arco
                            local_factor = np.random.normal(factor, 0.05)
                            c_arc[(i,j,k,m,l,s)] = base_c_arc * local_factor
    else:
        for k in K:
            for m in M[k]:
                d = d_k[k]
                o = o_k[k]
                c_depo_to_term[(d,o,k,m)] = base_c_depo
                c_term_to_depo[(o,d,k,m)] = base_c_depo
                
                # Arcos
                for l in L_mk[(m,k)]:
                    for (i,j) in A_k[k]:
                        c_arc[(i,j,k,m,l)] = base_c_arc
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
        'c_depo_to_term': c_depo_to_term,
        'c_term_to_depo': c_term_to_depo,
        'c_arc': c_arc
    }
    return data

