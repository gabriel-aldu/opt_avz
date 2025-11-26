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
        if (term, bus) not in p_term_by_term_bus_scen.keys():
            p_term_by_term_bus_scen[(term, bus)] =[]
        p_term_by_term_bus_scen[(term, bus)].append(float(value))

    for term in terms:
        for bus in buses:
            plt.figure()
            for scen in scenarios:
                if sum(p_term_by_term_bus_scen[(term, bus)]) >= 1.0:
                    plt.plot(p_term_by_term_bus_scen[(term, bus)], label=f'Scenario {scen}')
                    plt.title(f'Potencia en terminal {term} - Bus {bus}')
                    plt.xlabel('Tiempo')
                    plt.ylabel('Potencia [KWh]')
                    plt.legend()
                    plt.show()
            plt.close()
    p_depo_data = sol_data["p_depot"]

    p_depo_by_depo_bus = {}

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
        if (depo, bus) not in p_depo_by_depo_bus.keys():
            p_depo_by_depo_bus[(depo, bus)] =[]
        p_depo_by_depo_bus[(depo, bus)].append(float(value))

    for depo in depos:
        for bus in buses:
            plt.figure()
            for scen in scenarios:
                if sum(p_depo_by_depo_bus[(depo, bus)]) >= 1.0:
                    plt.plot(p_depo_by_depo_bus[(depo, bus)], label=f'Scenario {scen}')
                    plt.title(f'Potencia en deposito {depo} - Bus {bus}')
                    plt.xlabel('Tiempo')
                    plt.ylabel('Potencia [KWh]')
                    plt.legend()
                    plt.show()
            plt.close()

def plot_det_p(sol_data):

    p_term_data = sol_data["p_term"]

    p_term_by_term_bus = {}

    terms = set()
    buses = set()
    scenarios = set()

    for key, value in p_term_data.items():
        term = key[0]
        bus = key[2]
        terms.add(term)
        buses.add(bus)
        if (term, bus) not in p_term_by_term_bus.keys():
            p_term_by_term_bus[(term, bus)] =[]
        p_term_by_term_bus[(term, bus)].append(float(value))

    for term in terms:
        for bus in buses:
            plt.figure()
            if sum(p_term_by_term_bus[(term, bus)]) >= 1.0:
                plt.plot(p_term_by_term_bus[(term, bus)])
                plt.title(f'Potencia en terminal {term} - Bus {bus}')
                plt.xlabel('Tiempo')
                plt.ylabel('Potencia [KWh]')
                plt.legend()
                plt.show()
            plt.close()
    p_depo_data = sol_data["p_depot"]

    p_depo_by_depo_bus = {}

    depos = set()
    buses = set()


    for key, value in p_depo_data.items():
        depo = key[0]
        bus = key[2]
        depos.add(depo)
        buses.add(bus)
        if (depo, bus) not in p_depo_by_depo_bus.keys():
            p_depo_by_depo_bus[(depo, bus)] =[]
        p_depo_by_depo_bus[(depo, bus)].append(float(value))

    for depo in depos:
        for bus in buses:
            plt.figure()

            if sum(p_depo_by_depo_bus[(depo, bus)]) >= 1.0:
                plt.plot(p_depo_by_depo_bus[(depo, bus)])
                plt.title(f'Potencia en deposito {depo} - Bus {bus}')
                plt.xlabel('Tiempo')
                plt.ylabel('Potencia [KWh]')
                plt.legend()
                plt.show()
            plt.close()


if __name__ == "__main__":
    data = sol_parser('logs/sols/solucion_estoca_11-25_21-27-25.sol')
    plot_stochastic_p(data)



