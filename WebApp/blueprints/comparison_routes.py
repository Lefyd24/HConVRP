import time
from flask import Blueprint, jsonify, render_template, request, current_app
import os
import yaml
from solver_modules.models import *
from solver_modules.helpers import create_node_matrix, get_directory_structure
from solver_modules.solution_handling import SolutionLoader,  SolutionsComparator
import networkx as nx

comparison_bp = Blueprint('comparison_bp', __name__)

@comparison_bp.route('/compare', methods=['GET'])
def comparison_index():
    solutions_base_path = current_app.config['SOLUTION_PATH']
    solutions = get_directory_structure(solutions_base_path)    
    return render_template('compare.html', solutions=solutions)


@comparison_bp.route('/compare_solutions', methods=['POST'])
def compare_solutions():
    solution_files = request.get_json().get('files')
    solutions_base_path = current_app.config['SOLUTION_PATH']
    
    loader = SolutionLoader()
    
    solutions = {}
    for sol in solution_files:
        file_path = sol.lstrip("/\\")
        full_path = os.path.normpath(os.path.join(solutions_base_path, file_path))
        with open(full_path, 'r') as file:
            solution = yaml.load(file, Loader=yaml.FullLoader)
            file_name = os.path.basename(file_path)
            solutions[file_name] = solution
    
    comparator = SolutionsComparator(solutions=solutions)
    results = comparator.compare()
    
    return jsonify({'results': results})