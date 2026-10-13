"""The launch helper READS the planner registry (vNext M2.1, ADR-0021).

trainit_moveit_config.py takes the pipelines move_group loads, their parameter files and the
default pipeline from config/planner_profiles.yaml -- so they are typed nowhere else. Its pinned
values (the pipeline set of TMR 1.7.0) are test_trainit_moveit_config.py; this file proves they
COME FROM the registry, that the registry's loaded plugins are declared dependencies of this
package, and that a registry of another schema is refused instead of half-read.
"""
import inspect
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from trainit_motion_runtime import trainit_moveit_config as helper

SHARE = Path(get_package_share_directory('trainit_motion_runtime'))
REGISTRY = yaml.safe_load((SHARE / 'config' / 'planner_profiles.yaml').read_bytes())

# ADR-0019 Decision 7: MoveIt's in-tree planners are Community. Only these plugins may back a
# pipeline of this (Community) registry; a candidate or Pro planner enters through an extension
# point, never as an entry here (ADR-0021 Decision 7). TSA's tests hold the same list.
COMMUNITY_MOVEIT_PLANNERS = {'moveit_planners_ompl', 'pilz_industrial_motion_planner',
                             'moveit_planners_chomp', 'moveit_planners_stomp'}


def test_the_helper_reads_the_installed_registry():
    assert Path(helper.planner_registry_path()) == SHARE / 'config' / 'planner_profiles.yaml'
    assert helper.planner_registry() == REGISTRY


def test_the_pipeline_set_is_the_registrys():
    pipelines = REGISTRY['pipelines']
    assert tuple(helper.DEFAULT_PIPELINES) == tuple(p for p, s in pipelines.items() if s['loaded'])
    assert helper.PIPELINE_FILES == {p: s['parameters'] for p, s in pipelines.items()
                                     if s['parameters']}
    assert helper.DEFAULT_PIPELINE == REGISTRY['default_pipeline']
    sig = inspect.signature(helper.build_move_group_params)
    assert sig.parameters['default_pipeline'].default == REGISTRY['default_pipeline']


def test_every_loaded_plugin_is_a_declared_dependency():
    root = ET.parse(SHARE / 'package.xml').getroot()
    declared = {e.text.strip() for tag in ('exec_depend', 'depend') for e in root.findall(tag)}
    for name, spec in REGISTRY['pipelines'].items():
        if spec['loaded']:
            assert spec['plugin_package'] in declared, \
                f'pipeline {name} is loaded but {spec["plugin_package"]} is not a dependency'


def test_every_pipeline_is_a_community_moveit_planner():
    for name, spec in REGISTRY['pipelines'].items():
        assert spec['plugin_package'] in COMMUNITY_MOVEIT_PLANNERS, \
            f'pipeline {name}: {spec["plugin_package"]} is not a Community MoveIt planner'


def test_a_registry_of_another_schema_is_refused(tmp_path, monkeypatch):
    other = dict(REGISTRY, schema_version=2)
    path = tmp_path / 'planner_profiles.yaml'
    path.write_text(yaml.safe_dump(other))
    monkeypatch.setattr(helper, 'planner_registry_path', lambda: str(path))
    with pytest.raises(RuntimeError, match='schema_version 2'):
        helper.planner_registry()
