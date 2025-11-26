from sol_parser import sol_parser
import numpy as np
import matplotlib.pyplot as plt

def plot_stochastic_e(sol_data):

    e_depo_arr_data = {key: sol_data['e_depo_arr'][key] for key in sol_data['e_depo_arr']}
    e_depo_dep_data = {key: sol_data['e_depo_dep'][key] for key in sol_data['e_depo_dep']}
    e_term_arr_data = {key: sol_data['e_term_arr'][key] for key in sol_data['e_term_arr']}
    e_term_dep_data = {key: sol_data['e_term_dep'][key] for key in sol_data['e_term_dep']}

    # Build easier-to-query maps: index by meaningful fields (dropping unknown first field for depo entries)
    depo_dep_map = {}
    depo_arr_map = {}
    term_arr_map = {}
    term_dep_map = {}

    for k, v in e_depo_dep_data.items():
        # assumed key structure: (unknown0, time, bus, scenario)
        depo_dep_map[(k[1], k[2], k[3])] = v
    for k, v in e_depo_arr_data.items():
        depo_arr_map[(k[1], k[2], k[3])] = v
    for k, v in e_term_arr_data.items():
        # assumed key structure: (term, time, bus, loop, scenario)
        term_arr_map[(k[0], k[1], k[2], k[3], k[4])] = v
    for k, v in e_term_dep_data.items():
        term_dep_map[(k[0], k[1], k[2], k[3], k[4])] = v

    buses = sorted(set([key[2] for key in e_depo_dep_data.keys()]), key=lambda x: (int(x) if str(x).isdigit() else x))
    scenarios = sorted(set([key[3] for key in e_depo_dep_data.keys()]), key=lambda s: (int(s) if str(s).isdigit() else s))

    for bus in buses:
        # collect ordered time steps for this bus
        times = sorted(set([k[1] for k in e_depo_dep_data.keys() if k[2] == bus]),
                       key=lambda t: (int(t) if str(t).isdigit() else t))

        # build a consistent sequence of stages (labels and retrieval instructions) independent of scenario
        stage_instructions = []
        x_labels = []
        for time in times:
            # depot departure
            stage_instructions.append(('depo_dep', time))
            x_labels.append(f'e_depo_dep_t{time}')
            # terminal entries/exits for both terms and loops
            for term in ['L1-A', 'L1-B']:
                for loop in range(0, 4):
                    loop_str = str(loop)
                    stage_instructions.append(('term_arr', time, term, loop_str))
                    x_labels.append(f'e_term_arr_{term}_loop_{loop}')
                    stage_instructions.append(('term_dep', time, term, loop_str))
                    x_labels.append(f'e_term_dep_{term}_loop_{loop}')
            # depot arrival
            stage_instructions.append(('depo_arr', time))
            x_labels.append(f'e_depo_arr_t{time}')

        plt.figure(figsize=(10, 6))

        # plot each scenario on the same axes
        for scenario in scenarios:
            y_values = []
            for instr in stage_instructions:
                if instr[0] == 'depo_dep':
                    _, time = instr
                    y = depo_dep_map.get((time, bus, scenario), 0)
                elif instr[0] == 'depo_arr':
                    _, time = instr
                    y = depo_arr_map.get((time, bus, scenario), 0)
                elif instr[0] == 'term_arr':
                    _, time, term, loop_str = instr
                    y = term_arr_map.get((term, time, bus, loop_str, scenario), 0)
                elif instr[0] == 'term_dep':
                    _, time, term, loop_str = instr
                    y = term_dep_map.get((term, time, bus, loop_str, scenario), 0)
                else:
                    y = 0
                y_values.append(y)

            # label scenario nicely (try numeric +1 if possible)
            try:
                scen_label = f'Escenario {int(scenario) + 1}'
            except Exception:
                scen_label = f'Escenario {scenario}'
            plt.plot(x_labels, np.array(y_values) * (100.0/350.0), marker='o', label=scen_label)

        plt.xlabel("Etapa del día")
        plt.ylabel("% Energía")
        plt.title(f"Energía para el Bus {bus} (todos los escenarios)")
        plt.xticks(rotation=90)
        plt.legend()
        plt.tight_layout()
        plt.show()

def plot_det_e(sol_data):

    e_depo_arr_data = {key: sol_data['e_depo_arr'][key] for key in sol_data['e_depo_arr']}
    e_depo_dep_data = {key: sol_data['e_depo_dep'][key] for key in sol_data['e_depo_dep']}
    e_term_arr_data = {key: sol_data['e_term_arr'][key] for key in sol_data['e_term_arr']}
    e_term_dep_data = {key: sol_data['e_term_dep'][key] for key in sol_data['e_term_dep']}

    buses = set([key[2] for key in e_depo_dep_data.keys()])

    for bus in buses:
        x_values = []
        y_values = []
        for key in e_depo_dep_data.keys():
            if key[2] == bus:
                depot_value = e_depo_dep_data[key]
                x_values.append(f'e_depo_dep_{key[2]}')
                y_values.append(depot_value)
                for term in ['L1-A', 'L1-B']:
                    for loop in range(0, 4):
                        arr_key = (term, key[1], key[2], str(loop))
                        dep_key = (term, key[1], key[2], str(loop))

                        x_values.append(f'e_term_arr_{arr_key[0]}_loop_{loop}')
                        y_values.append(e_term_arr_data[arr_key])
                        x_values.append(f'e_term_dep_{dep_key[0]}_loop_{loop}')
                        y_values.append(e_term_dep_data[dep_key])

                depo_arr_value = e_depo_arr_data[key]
                x_values.append(f'e_depo_arr_{key[2]}')
                y_values.append(depo_arr_value)

        plt.figure(figsize=(10, 6))
        plt.plot(x_values , np.array(y_values) * (100.0/350.0), marker='o', color='b', label="Energy")
        plt.xlabel("Etapa del día")
        plt.ylabel("% Energía")
        plt.title(f"Energía para el Bus {bus}")
        plt.xticks(rotation=90)
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    sol_data = sol_parser('logs/sols/solucion_estoca_11-25_21-27-25.sol')
    plot_stochastic_e(sol_data)
