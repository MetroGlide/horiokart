import json
import os
import pytest
from jsonschema import validate, RefResolver


SCHEMA_PATH = os.path.abspath(os.path.join(os.path.dirname(
    __file__), '..', '..', 'contracts', 'posegraph.schema.json'))


def load_schema():
    with open(SCHEMA_PATH, 'r') as f:
        schema = json.load(f)
    return schema


def sample_posegraph():
    # Minimal valid sample matching expected fields
    return {
        "nodes": [
            {"id": 0, "state_id": 0, "pose": [
                0.0, 0.0, 0.0], "timestamp": 0.0},
            {"id": 1, "state_id": 1, "pose": [1.0, 0.0, 0.0], "timestamp": 1.0}
        ],
        "edges": [
            {"from": 0, "to": 1, "transform": [1.0, 0.0, 0.0], "info": [
                [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]}
        ]
    }


def test_karto_export_schema_passes():
    if not os.path.exists(SCHEMA_PATH):
        pytest.skip(
            f"Schema not found at {SCHEMA_PATH}; skip export schema validation")
    schema = load_schema()
    pg = sample_posegraph()
    resolver = RefResolver(base_uri='file://' +
                           os.path.dirname(SCHEMA_PATH) + '/', referrer=schema)
    validate(instance=pg, schema=schema, resolver=resolver)
