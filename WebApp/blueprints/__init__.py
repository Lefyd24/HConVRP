from flask import Blueprint

# Importing all the blueprints
from .basic_routes import basic_bp
from .dataset_routes import dataset_bp
from .solver_routes import solver_bp
from .evaluator_routes import evaluator_bp
from .comparison_routes import comparison_bp
