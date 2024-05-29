import random
import time
import cplex
import csv


def validate_ranges(parameter: float):
    if parameter <= 0.0 or parameter > 1.0:
        return 0.5
    return parameter


def calculate_collision_probability(maneuvers: int, base_probability: float, indices: list):
    probabilities = []

    for variable in indices:
        probabilities.append(int(round(base_probability * 2 ** ((1 - maneuvers) / sum(variable)), 2) * 100))

    return probabilities


def generate_conflict_matrix(units: int, maneuvers: int, collision_base_probability: float, fill: float):
    # calculate square matrix dimension
    matrix_size = units * maneuvers

    # initialize matrix
    collision_matrix = [[0] * units * maneuvers for _ in range(matrix_size)]

    # calculate maximum number of collisions in upper triangular matrix
    number_of_possible_collisions = (matrix_size * (units - 1) * maneuvers) / 2

    # create indices for each binary variable in upper triangular matrix
    indices = [(i, j) for i in range(matrix_size) for j in range(i + 1, matrix_size) if
               (i // maneuvers) != (j // maneuvers)]

    # calculate probability for each binary variable
    probability = calculate_collision_probability(maneuvers, validate_ranges(collision_base_probability), indices)

    # select fill % of upper triangular matrix
    conflicts = random.choices(population=indices, weights=probability,
                               k=int(number_of_possible_collisions * validate_ranges(fill)))

    for collision in conflicts:
        collision_matrix[collision[0]][collision[1]] = 1
        collision_matrix[collision[1]][collision[0]] = 1

    return collision_matrix


def get_cuts_by_type(model: cplex.Cplex):
    cuts = [model.solution.MIP.cut_type.clique, model.solution.MIP.cut_type.cover,
            model.solution.MIP.cut_type.disjunctive,
            model.solution.MIP.cut_type.flow_cover, model.solution.MIP.cut_type.fractional,
            model.solution.MIP.cut_type.GUB_cover,
            model.solution.MIP.cut_type.zero_half, model.solution.MIP.cut_type.lift_and_project]

    cuts_names = ["Clique", "Cover", "Disjunctive", "Flow cover", "Gomory", "GUB", "zero-half", "lift and projects"]

    cuts_count_by_type = {}

    for i in range(len(cuts)):
        cuts_count_by_type[cuts_names[i]] = model.solution.MIP.get_num_cuts(cuts[i])

    return cuts_count_by_type


def solve_conflict_matrix(units: int, maneuvers: int, collision_matrix):
    start = time.time()

    # Initialize model
    model = cplex.Cplex()

    # Parametrize display
    parameter_set = model.create_parameter_set()
    parameter_set.add(model.parameters.mip.display, 3)
    model.set_parameter_set(parameter_set)

    # Direction of optimization
    model.objective.set_sense(model.objective.sense.minimize)

    # Add variables - for Common Edition max 1000
    variables = []
    for i in range(units):
        for j in range(maneuvers):
            var_name = f"x_{i}_{j}"
            variables.append(var_name)
            model.variables.add(obj=[j + 1], lb=[0], ub=[1], types=[model.variables.type.binary], names=[var_name])

    # Add constraints - for Common Edition max 1000
    # Limit to only one maneuver per plane
    for i in range(units):
        constraint = [
            cplex.SparsePair(ind=[variables[i * maneuvers + j] for j in range(maneuvers)], val=[1] * maneuvers)]
        model.linear_constraints.add(lin_expr=constraint, senses=["E"], rhs=[1])

    # Avoid collisions
    for i in range(units):
        for j in range(maneuvers):
            for k in range(i + 1, units):
                for l in range(maneuvers):
                    if collision_matrix[i * maneuvers + j][k * maneuvers + l] == 1:
                        constraint = [cplex.SparsePair(ind=[variables[i * maneuvers + j], variables[k * maneuvers + l]],
                                                       val=[1, 1])]
                        model.linear_constraints.add(lin_expr=constraint, senses=["L"], rhs=[1])

    # find solution
    model.solve()

    elapsed_time = time.time() - start

    solution_type = None
    solutions = 1
    iterations = 1
    explored_nodes = 1
    cuts = {}

    try:
        model.solution.get_objective_value()
        solution_type = model.solution.get_status_string()
        solutions = model.solution.pool.get_num()
        iterations = model.solution.progress.get_num_iterations()
        explored_nodes = model.solution.progress.get_num_nodes_processed()
        cuts = get_cuts_by_type(model)

    except cplex.exceptions.errors.CplexSolverError:
        if cplex.exceptions.errors.CplexSolverError == 1217:
            pass  # infeasible solution
        solution_type = model.solution.get_status_string()
        solutions = model.solution.pool.get_num()
        iterations = model.solution.progress.get_num_iterations()
        explored_nodes = model.solution.progress.get_num_nodes_processed()
        cuts = get_cuts_by_type(model)
    except cplex.exceptions.errors.CplexError:
        if cplex.exceptions.errors.CplexError == 1719:
            print(model.conflict.get_groups())

    return elapsed_time, solution_type, solutions, iterations, explored_nodes, cuts


def process_data(data: dict):
    min_time = 10000000000000.0
    max_time = -1.0
    solution_sums_by_solution_type = {}

    for i in range(10):
        if data["Solution type"][i] is None:
            continue
        else:
            if data["Solution type"][i] not in solution_sums_by_solution_type:
                solution_sums_by_solution_type[data["Solution type"][i]] = {}
                solution_sums_by_solution_type[data["Solution type"][i]]["Units"] = data["Units"][i]
                solution_sums_by_solution_type[data["Solution type"][i]]["Maneuvers"] = data["Maneuvers"][i]
                solution_sums_by_solution_type[data["Solution type"][i]]["Probability"] = data["Probability"][i]
                solution_sums_by_solution_type[data["Solution type"][i]]["Fill"] = data["Fill"][i]
                solution_sums_by_solution_type[data["Solution type"][i]]["Avg time"] = data["Time"][i]
                solution_sums_by_solution_type[data["Solution type"][i]]["Min Time"] = data["Time"][i]
                solution_sums_by_solution_type[data["Solution type"][i]]["Max Time"] = data["Time"][i]
                solution_sums_by_solution_type[data["Solution type"][i]]["Solution type"] = data["Solution type"][i]
                solution_sums_by_solution_type[data["Solution type"][i]]["Avg solution"] = data["Solution"][i]
                solution_sums_by_solution_type[data["Solution type"][i]]["Avg iterations"] = data["Iterations"][i]
                solution_sums_by_solution_type[data["Solution type"][i]]["Avg nodes"] = data["Nodes"][i]
                solution_sums_by_solution_type[data["Solution type"][i]]["Avg cuts"] = {}
                for cut in data["Cuts"][i]:
                    solution_sums_by_solution_type[data["Solution type"][i]]["Avg cuts"][cut] = data["Cuts"][i][cut]
                solution_sums_by_solution_type[data["Solution type"][i]]["Total avg cuts"] = sum(data["Cuts"][i].values())
                solution_sums_by_solution_type[data["Solution type"][i]]["Counter"] = 1
            else:
                solution_sums_by_solution_type[data["Solution type"][i]]["Units"] += data["Units"][i]
                solution_sums_by_solution_type[data["Solution type"][i]]["Maneuvers"] += data["Maneuvers"][i]
                solution_sums_by_solution_type[data["Solution type"][i]]["Probability"] += data["Probability"][i]
                solution_sums_by_solution_type[data["Solution type"][i]]["Fill"] += data["Fill"][i]
                solution_sums_by_solution_type[data["Solution type"][i]]["Avg time"] += data["Time"][i]
                if data["Time"][i] < solution_sums_by_solution_type[data["Solution type"][i]]["Min Time"]:
                    solution_sums_by_solution_type[data["Solution type"][i]]["Min Time"] = data["Time"][i]
                if data["Time"][i] > solution_sums_by_solution_type[data["Solution type"][i]]["Max Time"]:
                    solution_sums_by_solution_type[data["Solution type"][i]]["Max Time"] = data["Time"][i]
                solution_sums_by_solution_type[data["Solution type"][i]]["Avg solution"] += data["Solution"][i]
                solution_sums_by_solution_type[data["Solution type"][i]]["Avg iterations"] += data["Iterations"][i]
                solution_sums_by_solution_type[data["Solution type"][i]]["Avg nodes"] += data["Nodes"][i]
                for cut in solution_sums_by_solution_type[data["Solution type"][i]]["Avg cuts"]:
                    solution_sums_by_solution_type[data["Solution type"][i]]["Avg cuts"][cut] += data["Cuts"][i][cut]
                solution_sums_by_solution_type[data["Solution type"][i]]["Total avg cuts"] += sum(data["Cuts"][i].values())
                solution_sums_by_solution_type[data["Solution type"][i]]["Counter"] += 1

    for sol_type in solution_sums_by_solution_type:
        for field in solution_sums_by_solution_type[sol_type]:
            if field in ("Min Time", "Max Time", "Counter", "Solution type"):
                pass
            elif field == "Avg cuts":
                for cut in solution_sums_by_solution_type[sol_type][field]:
                    solution_sums_by_solution_type[sol_type][field][cut] /= solution_sums_by_solution_type[sol_type]["Counter"]
            else:
                solution_sums_by_solution_type[sol_type][field] /= solution_sums_by_solution_type[sol_type]["Counter"]
        del solution_sums_by_solution_type[sol_type]["Counter"]

    print(solution_sums_by_solution_type)

    return solution_sums_by_solution_type


def run_optimization(units: int, maneuvers: int, collision_base_probability: float, fill: float):
    table_data = {"Units": [], "Maneuvers": [], "Probability": [], "Fill": [], "Time": [], "Solution type": [],
                  "Solution": [], "Iterations": [], "Nodes": [], "Cuts": []}

    final_data =[]

    for _ in range(10):
        matrix = generate_conflict_matrix(units, maneuvers, collision_base_probability, fill)
        elapsed_time, sol_type, solution, iterations, nodes, cuts = solve_conflict_matrix(units, maneuvers, matrix)
        data = [units, maneuvers, collision_base_probability, fill, elapsed_time, sol_type, solution, iterations, nodes,
                cuts]
        for i, key in enumerate(table_data):
            table_data[key].append(data[i])

    processed_data = process_data(table_data)
    for key in processed_data:
        final_data.append(flatten_dict(processed_data[key]))

    return final_data


def flatten_dict(d):
    flattened = {}
    for key, value in d.items():
        if isinstance(value, dict):
            for sub_key, sub_value in value.items():
                flattened[f"{key} {sub_key}"] = sub_value
        else:
            flattened[key] = value
    return flattened


def write_to_file(data: list[dict]):
    headers = data[0][0].keys()
    file_path = "aircrafts_data.csv"

    with open(file_path, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=headers)

        writer.writeheader()

        for instance in data:
            for row in instance:
                writer.writerow(flatten_dict(row))

    print("Table data has been successfully saved to", file_path)


if __name__ == "__main__":
    planes = [10, 20, 30, 40]
    moves = [3, 5, 7, 9]
    prob = [0.6, 0.8, 1.0]
    fill_percentage = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
    calculated_solutions = []

    for plane in planes:
        for move in moves:
            for proba in prob:
                for fill_per in fill_percentage:
                    calculated_solutions.append(run_optimization(plane, move, proba, fill_per))
    write_to_file(calculated_solutions)
