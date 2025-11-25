import numpy as np
import math

def generate_data_224():
    """
    Genera una instancia basada en la LINEA 224 (Santiago, Red Movilidad).
    Datos: Buses Zhongtong LCK6122EVG, Batería ~350kWh.
    Ciclo: ~180 min.
    """
    
    # --- 1. CONFIGURACIÓN GENERAL ---
    num_lineas = 1
    # Calculo aproximado de flota necesaria: Ciclo 180 min / Headway min (7 min) = ~26 buses activos + reservas.
    num_buses = 30 
    
    K = ["224"] 
    # Nodos: Terminal A (Inicio), Terminal B (Retorno)
    N = ["224-A", "224-B"] 
    D = ["Depot-224"]
    
    # Topología
    N_k = {"224": ["224-A", "224-B"]}
    d_k = {"224": "Depot-224"}
    o_k = {"224": "224-A"} # Asumimos inicio en Terminal A
    
    # Arcos: Ida (A->B) y Vuelta (B->A)
    A_k = {"224": [("224-A", "224-B"), ("224-B", "224-A")]}
    
    M = {"224": [f"B{j + 1}" for j in range(num_buses)]}
    
    # --- 2. TIEMPO Y RESOLUCIÓN ---
    HOURS = 24
    RES_MIN = 1 # Resolución de 1 minuto es necesaria para cumplir los headways de 7, 8, 13 min.
    SLOTS_PER_HOUR = 60 // RES_MIN
    T = HOURS * SLOTS_PER_HOUR
    
    # Conjuntos de tiempo
    PHI = list(range(1, T + 1))
    H = list(range(HOURS))
    Z = H[:]

    H_peak = {7, 8 , 9 , 10 , 18, 19, 20, 21}
    Z_peak = list(H_peak)

    rho = 1.0 / SLOTS_PER_HOUR
    gamma = SLOTS_PER_HOUR

    # Mapeo de Hora (0..23) a sus slots de minutos
    PHI_h = {h: list(range(h*SLOTS_PER_HOUR + 1, (h+1)*SLOTS_PER_HOUR + 1)) for h in H}
    PHI_z = {z: PHI_h[z] for z in Z}
    # --- 3. GENERACIÓN DE ITINERARIO (SALIDAS) SEGÚN HEADWAY ---
    # Definición de tramos horarios segun la info provista:
    # Formato: (HoraInicio, MinInicio, HoraFin, MinFin, Headway_minutos)
    schedule_blocks = [
        (5, 30, 6, 30, 13),  # Valle temprano: 5:30-6:30, cada 13 min
        (6, 30, 9, 30, 7),   # Punta mañana: 6:30-9:30, cada 7 min
        (9, 30, 17, 0, 13),  # Valle: 9:30-17:00, cada 13 min
        (17, 0, 20, 30, 8),  # Punta tarde: 17:00-20:30, cada 8 min
        (20, 30, 23, 30, 15) # Noche: 20:30-23:30, cada 15 min
    ]
    
    departure_times = []
    
    for start_h, start_m, end_h, end_m, headway in schedule_blocks:
        # Convertir a minutos absolutos desde media noche
        start_abs = start_h * 60 + start_m
        end_abs = end_h * 60 + end_m
        
        # Generar salidas
        current_t = start_abs
        while current_t < end_abs:
            departure_times.append(current_t)
            current_t += headway

    # --- 4. ASIGNACIÓN DE BUSES (Greedy) ---
    # Asumimos ciclo de 180 minutos. Un bus sale, y vuelve a estar disponible en t + 180.
    CYCLE_TIME = 180 
    
    # Estructuras para guardar las vueltas
    # L_mk: Diccionario de vueltas por bus. Key: (Bus, Linea) -> Lista de indices de vueltas [0, 1, 2...]
    L_mk = {(m, "224"): [] for m in M["224"]}
    
    # PHI_term: Cuándo llega el bus al terminal para cargar.
    # Key: (Nodo, Linea, Bus, Vuelta_idx) -> Lista de tiempos (slots)
    PHI_term = {}
    
    # Estado de los buses: minuto en que quedan libres
    bus_free_time = {m: 0 for m in M["224"]}
    
    # Contador de vueltas por bus
    lap_counters = {m: 1 for m in M["224"]} 
    
    for dep_time in departure_times:
        # Buscar el primer bus disponible
        assigned_bus = None
        
        # Ordenamos buses para priorizar el uso uniforme o el primero disponible (Round Robin simplificado por disponibilidad)
        # Buscamos bus tal que bus_free_time[bus] <= dep_time
        # Preferimos el que lleva más tiempo parado para dar oportunidad de carga, 
        # o simplemente el primero que cumpla.
        
        candidates = [m for m in M["224"] if bus_free_time[m] <= dep_time]
        
        if not candidates:
            # Si no hay buses (falta flota), tomamos el que se libere más pronto (retraso) 
            # o en este caso, asumimos que 'num_buses' es suficiente.
            # Para robustez, tomamos el de menor free_time aunque sea > dep_time (retraso forzado)
            assigned_bus = min(M["224"], key=lambda x: bus_free_time[x])
            # Ajustamos el dep_time real al momento que se libera
            real_dep_time = bus_free_time[assigned_bus]
        else:
            assigned_bus = candidates[0] # Tomamos el primero disponible
            real_dep_time = dep_time
            
        # Registrar la vuelta
        l_idx = lap_counters[assigned_bus]
        L_mk[(assigned_bus, "224")].append(l_idx)
        
        # Calcular llegada (Fin del ciclo o llegada al terminal opuesto). 
        # Asumiendo carga de oportunidad en las cabeceras.
        # Si el ciclo es 180 min (Ida + Vuelta), llega al terminal de inicio en t + 180.
        # Asumiremos ventana de carga al completar la vuelta.
        arrival_time = real_dep_time + CYCLE_TIME
        
        # Convertir minuto llegada a Slot (1-based)
        arrival_slot = arrival_time + 1 
        
        # Ventana de carga: Asumimos que puede cargar hasta su proxima salida
        # Pero como no sabemos su proxima salida aun, damos una ventana estandar (ej. 20 min) o hasta fin del horizonte
        # Simplificación: Ventana de 30 minutos post-llegada para carga oportunidad
        charging_window = list(range(arrival_slot, arrival_slot + 30))
        
        # Filtrar slots que se pasen del día
        charging_window = [t for t in charging_window if t <= T]
        
        if charging_window:
            # Nodo de llegada: Asumimos que vuelve al origen (224-A) tras el ciclo
            PHI_term[("224-A", "224", assigned_bus, l_idx)] = charging_window
            
        # Actualizar estado del bus
        bus_free_time[assigned_bus] = arrival_time
        lap_counters[assigned_bus] += 1

    # --- 5. VENTANAS DE CARGA EN DEPÓSITO (NOCTURNA) ---
    # Ventana principal: 23:00 (Hora 23) a 05:00 (Hora 5)
    # Slots correspondientes
    night_slots = []
    # 23:00 a 24:00
    night_slots.extend(PHI_h[23])
    # 00:00 a 05:00
    for h in range(0, 5):
        night_slots.extend(PHI_h[h])
        
    PHI_depo = {}
    for m in M["224"]:
        PHI_depo[("224", m)] = night_slots

    # --- 6. PARÁMETROS FÍSICOS (Zhongtong & Red) ---
    
    # Batería 350 kWh (Opción 3 Zhongtong)
    u_k = {"224": 350.0} 
    
    epsilon_up, epsilon_low = 0.95, 0.15 # DoD usual
    
    # Potencias de carga (kW)
    # Terminal (Red): 150 kW (Carga rápida DC)
    p_on_route = {}
    for m in M["224"]:
        # Asumimos que puede cargar en el terminal A al terminar vuelta
        p_on_route[("224-A", "224")] = 150.0
        p_on_route[("224-B", "224")] = 150.0 # Si hubiera carga en el otro extremo
        
    # Deposito: 50 kW (Carga lenta/nocturna)
    p_depo = {d: 50.0 for d in D}
    
    theta_on_route, theta_depo = 0.95, 0.95 # Eficiencia cargador
    
    # Precios (Ejemplo genérico, ajustar según tarifa chilena real si se tiene)
    psi_on, psi_off = 120.0, 60.0  # Precio Energía (CLP/kWh aprox punta/valle)
    pi_on, pi_off = 3000.0, 1000.0 # Precio Potencia
    omega = 1.0/30.0 # Factor de amortización mensual aproximado

    # --- 7. ESTOCASTICIDAD (Consumo) ---
    NUM_SCENARIOS = 5
    SCENARIOS = list(range(NUM_SCENARIOS))
    PROBS = {s: 1.0/NUM_SCENARIOS for s in SCENARIOS}
    
    # Consumo Energético
    # Bus de 12m consume aprox 1.0 - 1.2 kWh/km.
    # Velocidad promedio Santiago ~18-20 km/h. 
    # En 3 horas (Ciclo), recorre ~54-60 km.
    # Consumo total ciclo ~ 60km * 1.2 = 72 kWh.
    # Como tenemos 2 arcos (Ida y Vuelta), cada arco consume ~36 kWh.
    
    base_c_arc = 38.0 # kWh por sentido (ajustado para dar ~76 kWh por ciclo completo)
    base_c_depo = 5.0 # Consumo movimiento interno depot
    
    c_depo_to_term_scen = {}
    c_term_to_depo_scen = {}
    c_arc_scen = {}
    
    for s in SCENARIOS:
        factor = np.random.normal(1.0, 0.15) # Variabilidad 15%
        factor = max(0.8, min(1.3, factor)) # Cotas realistas
        
        for k in K:
            for m in M[k]:
                # Consumo Trayectos muertos (Depot <-> Terminal)
                c_depo_to_term_scen[(d_k[k], o_k[k], k, m, s)] = base_c_depo * factor
                c_term_to_depo_scen[(o_k[k], d_k[k], k, m, s)] = base_c_depo * factor
                
                # Consumo en ruta (Ida y Vuelta)
                # Iteramos sobre las vueltas asignadas a ese bus
                for l_idx in L_mk[(m, k)]:
                    for (i, j) in A_k[k]:
                        # Factor local por arco (tráfico especifico)
                        local_factor = np.random.normal(factor, 0.05)
                        c_arc_scen[(i, j, k, m, l_idx, s)] = base_c_arc * local_factor

    # Empaquetar datos
    data = {
        'N': N, 'D': D, 'K': K, 'N_k': N_k, 'd_k': d_k, 'o_k': o_k, 'A_k': A_k, 'M': M, 'L': L_mk,
        'PHI': PHI, 'Z': Z,'H': H, 'PHI_h': PHI_h,'Z_peak': Z_peak, 'H_peak': H_peak, 'PHI_h': PHI_h, 'PHI_z': PHI_z,
        'PHI_term': PHI_term, 'PHI_depo': PHI_depo,'rho': rho, 'gamma': gamma,
        'u_k': u_k, 'epsilon_up': epsilon_up, 'epsilon_low': epsilon_low,
        'p_on_route': p_on_route, 'p_depo': p_depo, 'theta_on_route': theta_on_route, 'theta_depo': theta_depo,
        'psi_on': psi_on, 'psi_off': psi_off, 'pi_on': pi_on, 'pi_off': pi_off, 'omega': omega,
        'SCENARIOS': SCENARIOS, 'PROBS': PROBS,
        'c_depo_to_term_scen': c_depo_to_term_scen,
        'c_term_to_depo_scen': c_term_to_depo_scen,
        'c_arc_scen': c_arc_scen
    }
    
    return data