import json
import os
from jsonschema import validate, ValidationError

ROOT = os.path.dirname(os.path.dirname(__file__))
# contracts and fixtures live under the feature directory (ROOT)
CONTRACTS_DIR = os.path.join(ROOT, 'contracts')
FIXTURES_DIR = os.path.join(ROOT, 'tests', 'fixtures') if os.path.basename(os.path.dirname(
    __file__)) != 'tests' else os.path.join(os.path.dirname(__file__), 'fixtures')
# Simplify: if tests are under <feature>/tests, use that fixtures path
if not os.path.isdir(CONTRACTS_DIR):
    # fallback to project-level contracts (if any)
    CONTRACTS_DIR = os.path.abspath(os.path.join(ROOT, '..', 'contracts'))


def load_json(path):
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)


def test_posegraph_schema_valid():
    schema_path = os.path.join(CONTRACTS_DIR, 'posegraph.schema.json')
    data_path = os.path.join(FIXTURES_DIR, 'posegraph_small.json')
    schema = load_json(schema_path)
    data = load_json(data_path)
    try:
        validate(instance=data, schema=schema)
    except ValidationError as e:
        raise AssertionError(
            f'PoseGraph fixture does not match schema: {e.message}')


def test_gnss_schema_valid():
    schema_path = os.path.join(CONTRACTS_DIR, 'gnss.schema.json')
    data_path = os.path.join(FIXTURES_DIR, 'gnss_small.json')
    schema = load_json(schema_path)
    data = load_json(data_path)
    try:
        # allow fixture to be a single object or a list of observations
        if isinstance(data, list):
            for i, item in enumerate(data):
                validate(instance=item, schema=schema)
        else:
            validate(instance=data, schema=schema)
    except ValidationError as e:
        raise AssertionError(
            f'GNSS fixture does not match schema: {e.message}')
