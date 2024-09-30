from flask import Flask
from blueprints import basic_bp, dataset_bp, solver_bp
from blueprints.extensions import socketio  # Import the initialized socketio
import warnings
import os

warnings.filterwarnings('ignore')

app = Flask(__name__)
app.config['SECRET_KEY'] = 'hconvrpsolver'
app.config['TEMPLATES_AUTO_RELOAD'] = True
app.config['DATASET_PATH'] = os.path.join(app.root_path, 'datasets/')
app.config['SOLUTION_PATH'] = os.path.join(app.root_path, 'solutions/')

# Initialize SocketIO with the app
socketio.init_app(app)

# Registering Blueprints
app.register_blueprint(basic_bp)
app.register_blueprint(dataset_bp)
app.register_blueprint(solver_bp)

if __name__ == '__main__':
    socketio.run(app, debug=True, port=5050)
