"""The launch helper's pipeline set, pinned (vNext M2.1).

trainit_moveit_config.py is the ONE place the move_group planning_pipelines list is written, and
every bundle's launch files call it by name (build_move_group_params, available_pipeline_names), so
its constants and its keyword API are a contract: the golden, the reference app and the ROS2ML
bundles depend on them, and no byte gate of a generated bundle sees them.

These pins were written and run against TMR 1.7.0, BEFORE the planner registry: the registry
changes who KNOWS the pipeline set, never what move_group loads. Changing the set on purpose edits
these pins in the same commit, with the reason.
"""
import inspect
import os
import re
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory

from trainit_motion_runtime import trainit_moveit_config as helper

LOADED = ('ompl', 'pilz_industrial_motion_planner', 'chomp')
FILES = {
    'ompl': 'ompl_planning.yaml',
    'pilz_industrial_motion_planner': 'pilz_industrial_motion_planner_planning.yaml',
    'chomp': 'chomp_planning.yaml',
}


def test_the_loaded_pipeline_set_and_its_files_are_unchanged():
    assert tuple(helper.DEFAULT_PIPELINES) == LOADED
    assert dict(helper.PIPELINE_FILES) == FILES
    assert helper.available_pipeline_names() == list(LOADED)


def test_the_keyword_api_bundles_call_is_unchanged():
    sig = inspect.signature(helper.build_move_group_params)
    assert list(sig.parameters) == ['robot_name', 'moveit_config_package', 'use_isaac', 'backend',
                                    'pipelines', 'default_pipeline', 'trajectory_execution']
    assert sig.parameters['default_pipeline'].default == 'ompl'
    assert tuple(sig.parameters['pipelines'].default) == LOADED
    assert tuple(inspect.signature(helper.available_pipeline_names)
                 .parameters['pipelines'].default) == LOADED


def test_every_loaded_pipeline_has_its_parameters():
    params = helper.generic_pipeline_params()
    assert list(params) == list(LOADED)
    for name, p in params.items():
        assert isinstance(p.get('planning_plugin'), str) and p['planning_plugin'], name


@pytest.mark.parametrize('name', ['stomp', 'cumotion', 'curobo'])
def test_a_pipeline_that_is_not_shipped_is_refused(name):
    """stomp: declared by TMR, never shipped on Humble. cumotion/curobo: candidate providers
    (ADR-0019), never a pipeline of this framework."""
    assert name not in helper.PIPELINE_FILES
    with pytest.raises(ValueError, match=f"unknown generic pipeline '{name}'"):
        helper.generic_pipeline_params((name,))


def test_the_shipped_pipeline_files_are_exactly_the_declared_ones():
    shipped = Path(get_package_share_directory('trainit_motion_runtime')) / 'config' / 'pipelines'
    assert sorted(f.name for f in shipped.glob('*.yaml')) == sorted(FILES.values())


def test_the_helper_takes_nothing_from_the_environment():
    """The pipeline set is data of this package, never of the shell that launches a bundle."""
    src = Path(helper.__file__).read_text()
    assert not re.search(r'\bos\.environ\b|\bgetenv\b', src), \
        'trainit_moveit_config.py must not read the environment'
    assert os.path.isfile(helper.__file__)
