from flask import Blueprint, render_template, current_app
import os

basic_bp = Blueprint('basic_bp', __name__)

def list_files_recursive(dataset, path='.'):
    for entry in os.listdir(path):
        if entry == '.DS_Store':
            continue
        full_path = os.path.join(path, entry)
        if os.path.isdir(full_path):
            list_files_recursive(dataset, full_path)
        else:
            dataset[f"{path} - {entry}"] = full_path

@basic_bp.route('/')
def index():
    dataset_path = current_app.config['DATASET_PATH']
    datasets = {}
    list_files_recursive(datasets, dataset_path)
    return render_template('index.html', datasets=datasets)

@basic_bp.route('/documentation')
def documentation():
    return render_template('documentation.html')

@basic_bp.route('/contact')
def contact():
    return render_template('contact.html')
