

def sol_parser(filepath):

    sol_data = {}
    with open(filepath, 'r') as f:
        lines = f.readlines()

    lines.pop(0) # quitar las dos primeras lineas
    lines.pop(0)

    for line in lines:
        var_name = line[:line.index('[')]
        var_key = line[line.index('[')+1:line.index(']')]
        var_key = tuple(var_key.split(',')) if ',' in var_key else var_key
        var_value = float(line[line.index(' ')+1:].strip())
        if var_name not in sol_data:
            sol_data[var_name] = {}
        
        sol_data[var_name][var_key] = var_value
    
    return sol_data