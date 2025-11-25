from matplotlib import pyplot as plt
import numpy as np
from sol_parser import sol_parser

def plot_stochastic_p(sol_data):

    p_term_data = sol_data["p_term"]

    p_term_by_term_bus_scen = {}

    terms = set()
    buses = set()
    scenarios = set()

    for key, value in p_term_data.items():
        term = key[0]
        bus = key[2]
        scen = key[4]
        terms.add(term)
        buses.add(bus)
        scenarios.add(scen)
        if (term, bus, scen) not in p_term_by_term_bus_scen.keys():
            p_term_by_term_bus_scen[(term, bus, scen)] =[]
        p_term_by_term_bus_scen[(term, bus, scen)].append(float(value))

    for term in terms:
        for bus in buses:
            plt.figure()
            for scen in scenarios:
                if sum(p_term_by_term_bus_scen[(term, bus, scen)]) >= 1.0:
                    plt.plot(p_term_by_term_bus_scen[(term, bus, scen)], label=f'Scenario {scen}')
                    plt.title(f'Potencia en terminal {term} - Bus {bus}')
                    plt.xlabel('Tiempo')
                    plt.ylabel('Potencia [KWh]')
                    plt.legend()
                    plt.show()
            plt.close()
    p_depo_data = sol_data["p_depot"]

    p_depo_by_depo_bus_scen = {}

    depos = set()
    buses = set()
    scenarios = set()

    for key, value in p_depo_data.items():
        depo = key[0]
        bus = key[2]
        scen = key[4]
        depos.add(depo)
        buses.add(bus)
        scenarios.add(scen)
        if (depo, bus, scen) not in p_depo_by_depo_bus_scen.keys():
            p_depo_by_depo_bus_scen[(depo, bus, scen)] =[]
        p_depo_by_depo_bus_scen[(depo, bus, scen)].append(float(value))

    for depo in depos:
        for bus in buses:
            plt.figure()
            for scen in scenarios:
                if sum(p_depo_by_depo_bus_scen[(depo, bus, scen)]) >= 1.0:
                    plt.plot(p_depo_by_depo_bus_scen[(depo, bus, scen)], label=f'Scenario {scen}')
                    plt.title(f'Potencia en deposito {depo} - Bus {bus}')
                    plt.xlabel('Tiempo')
                    plt.ylabel('Potencia [KWh]')
                    plt.legend()
                    plt.show()
            plt.close()


if __name__ == "__main__":
    data = sol_parser('logs/sols/solucion_estoca_11-25_15-31.sol')
    plot_stochastic_p(data)



