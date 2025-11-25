from sol_parser import sol_parser

import matplotlib.pyplot as plt

def plot_e(sol_data):

    e_depo_arr_data = {key: sol_data['e_depo_arr'][key] for key in sol_data['e_depo_arr']}
    e_depo_dep_data = {key: sol_data['e_depo_dep'][key] for key in sol_data['e_depo_dep']}
    e_term_arr_data = {key: sol_data['e_term_arr'][key] for key in sol_data['e_term_arr']}
    e_term_dep_data = {key: sol_data['e_term_dep'][key] for key in sol_data['e_term_dep']}

    buses = set([key[2] for key in e_depo_dep_data.keys()])
    scenarios = set([key[3] for key in e_depo_dep_data.keys()])

    for bus in buses:
        for scenario in scenarios:
            x_values = []
            y_values = []
            for key in e_depo_dep_data.keys():
                if key[2] == bus and key[3] == scenario:
                    depot_value = e_depo_dep_data[key]
                    x_values.append(f'e_depo_dep_{key[2]}_{key[3]}')
                    y_values.append(depot_value)
                    for term in ['L1-A', 'L1-B']:
                        for loop in range(0, 4):
                            arr_key = (term, key[1], key[2], str(loop), key[3])
                            dep_key = (term, key[1], key[2], str(loop), key[3])

                            x_values.append(f'e_term_arr_{arr_key[0]}_{arr_key[3]}')
                            y_values.append(e_term_arr_data[arr_key])
                            x_values.append(f'e_term_dep_{dep_key[0]}_{dep_key[3]}')
                            y_values.append(e_term_dep_data[dep_key])

                    depo_arr_value = e_depo_arr_data[key]
                    x_values.append(f'e_depo_arr_{key[2]}_{key[3]}')
                    y_values.append(depo_arr_value)

            plt.figure(figsize=(10, 6))
            plt.plot(x_values, y_values, marker='o', color='b', label="Energy")
            plt.xlabel("Etapa del día")
            plt.ylabel("Energía")
            plt.title(f"Energía para el Bus {bus} y escenario {scenario + 1}")
            plt.xticks(rotation=90)
            plt.tight_layout()
            plt.show()

if __name__ == "__main__":
    sol_data = sol_parser('logs/sols/solucion_estoca_11-25_15-31.sol')
    plot_e(sol_data)
