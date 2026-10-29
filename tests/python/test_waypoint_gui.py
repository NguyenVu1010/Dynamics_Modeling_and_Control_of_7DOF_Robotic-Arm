"""Waypoint GUI package and launch (feature waypoint-gui, design 6.5).

No ROS, no display: files are parsed, and sim/launch/sim.launch.py is run with the launch
modules, ament_index and xacro replaced by fakes (same technique as
test_sim_launch_realism.py). Pinned:
- ros2_gui/ is the colcon package cobot_gui; its plugin_description.xml exports
  cobot_gui/WaypointPanel (type cobot_gui::WaypointPanel) with base class rviz_common::Panel,
  exported to rviz_common from CMake; the C++ unit test test/test_waypoint_table.cpp is an
  ament gtest;
- sim/rviz/waypoints.rviz holds the panel, RobotModel, TF, InteractiveMarkers and two Path
  displays: the preview (.../preview_path) and the loaded goal (/cobot_controller/reference_path);
- sim.launch.py rviz:=true starts rviz2 with that config and gives the controller
  command_source external unless command_source is set; rviz false (default) has no rviz2;
- sim.sh builds ros2_gui and has a `test` command (colcon test of cobot_gui) that the gazebo
  CI script calls; the ROS image installs RViz2 and interactive markers.
"""

import importlib.util
import sys
import types
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
GUI = REPO / "ros2_gui"
PLUGIN_XML = GUI / "plugin_description.xml"
RVIZ = REPO / "sim/rviz/waypoints.rviz"
LAUNCH = REPO / "sim/launch/sim.launch.py"
SIM_SH = REPO / "tools/wsl/sim.sh"
GAZEBO_CI = REPO / "tools/ci/gazebo_ci.sh"
ROS_DOCKERFILE = REPO / "docker/ros.Dockerfile"
README = REPO / "README.md"
SHARES = {"cobot_sim": REPO / "sim", "cobot_ros2": REPO, "cobot_gui": GUI}
CONTROLLER = "cobot_controller"
PANEL = "cobot_gui/WaypointPanel"
REFERENCE_TOPIC = "/cobot_controller/reference_path"


# ---------------------------------------------------------------- package and plugin


def test_gui_package_is_cobot_gui():
    root = ET.fromstring((GUI / "package.xml").read_text(encoding="utf-8"))
    assert root.findtext("name") == "cobot_gui"
    deps = {e.text for e in root if e.tag.endswith("depend")}
    for dep in ("rviz_common", "interactive_markers", "cobot_interfaces", "pluginlib"):
        assert dep in deps, f"package.xml: missing dependency {dep}"
    build_types = [e.text for e in root.iter("build_type")]
    assert build_types == ["ament_cmake"], build_types


def test_plugin_description_exports_the_panel():
    root = ET.fromstring(PLUGIN_XML.read_text(encoding="utf-8"))
    classes = [c for c in root.iter("class") if c.get("name") == PANEL]
    assert len(classes) == 1, f"{PLUGIN_XML.name}: one <class name={PANEL!r}> expected"
    cls = classes[0]
    assert cls.get("type") == "cobot_gui::WaypointPanel"
    assert cls.get("base_class_type") == "rviz_common::Panel"
    libraries = [lib for lib in root.iter("library") if cls in list(lib)]
    assert len(libraries) == 1 and libraries[0].get("path"), "class must sit in a <library path>"


def test_cmake_exports_the_plugin_and_builds_the_unit_test():
    cmake = (GUI / "CMakeLists.txt").read_text(encoding="utf-8")
    flat = " ".join(cmake.split())
    assert "pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)" in flat
    assert "ament_add_gtest" in cmake
    assert "test/test_waypoint_table.cpp" in cmake
    assert (GUI / "test/test_waypoint_table.cpp").is_file()


def test_waypoint_table_header_has_no_qt():
    headers = list((GUI / "include").rglob("waypoint_table.hpp"))
    assert len(headers) == 1, headers
    text = headers[0].read_text(encoding="utf-8")
    assert "#include <Q" not in text and '#include "Q' not in text, "data model must not use Qt"
    assert "class WaypointTable" in text


# ---------------------------------------------------------------- rviz config


def walk(node):
    """Every dict nested anywhere in a parsed YAML document."""
    if isinstance(node, dict):
        yield node
        for v in node.values():
            yield from walk(v)
    elif isinstance(node, list):
        for v in node:
            yield from walk(v)


def topic_of(display):
    topic = display.get("Topic")
    if isinstance(topic, dict):
        topic = topic.get("Value")
    return topic


@pytest.fixture(scope="module")
def rviz_config():
    return yaml.safe_load(RVIZ.read_text(encoding="utf-8"))


def test_rviz_config_docks_the_panel(rviz_config):
    panels = (rviz_config.get("Panels") or []) if isinstance(rviz_config, dict) else []
    assert any(p.get("Class") == PANEL for p in panels), f"Panels must contain {PANEL}"


def test_rviz_config_has_the_waypoint_displays(rviz_config):
    classes = {d.get("Class") for d in walk(rviz_config)}
    for cls in (
        "rviz_default_plugins/RobotModel",
        "rviz_default_plugins/TF",
        "rviz_default_plugins/InteractiveMarkers",
    ):
        assert cls in classes, f"display {cls} missing"


def test_rviz_config_draws_preview_and_reference_paths(rviz_config):
    paths = [d for d in walk(rviz_config) if d.get("Class") == "rviz_default_plugins/Path"]
    topics = [topic_of(d) for d in paths]
    assert len(paths) == 2, f"two Path displays expected, got topics {topics}"
    assert REFERENCE_TOPIC in topics, topics
    preview = [t for t in topics if t != REFERENCE_TOPIC]
    assert len(preview) == 1 and str(preview[0]).endswith("/preview_path"), topics


def test_rviz_config_fixed_frame_is_base_link(rviz_config):
    frames = [d["Fixed Frame"] for d in walk(rviz_config) if "Fixed Frame" in d]
    assert frames == ["base_link"], frames


def test_sim_package_installs_the_rviz_config():
    cmake = (REPO / "sim/CMakeLists.txt").read_text(encoding="utf-8")
    install = [line for line in cmake.splitlines() if "install(DIRECTORY" in line]
    assert any("rviz" in line.split() for line in install), "sim/CMakeLists.txt must install rviz/"


# ---------------------------------------------------------------- launch


class Recorded:
    """Stand-in for any launch entity: keeps its constructor arguments."""

    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


class FakeModule(types.ModuleType):
    """Module whose unknown attributes are Recorded subclasses (permissive launch API)."""

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        cls = type(name, (Recorded,), {})
        setattr(self, name, cls)
        return cls


class LaunchConfiguration(Recorded):
    def __init__(self, name, default=None, **kwargs):
        super().__init__(name, default=default, **kwargs)
        self.name = name
        self.default = default

    def perform(self, context):
        value = context.launch_configurations.get(self.name, self.default)
        assert value is not None, f"launch configuration {self.name} not set"
        return value


class DeclareLaunchArgument(Recorded):
    def __init__(self, name, default_value=None, **kwargs):
        super().__init__(name, default_value=default_value, **kwargs)
        self.name = name
        self.default_value = default_value


class OpaqueFunction(Recorded):
    def __init__(self, function, args=None, kwargs=None, **rest):
        super().__init__(function=function, **rest)
        self.function = function
        self.fn_args = args or []
        self.fn_kwargs = kwargs or {}


def _text(value, context):
    if hasattr(value, "perform"):
        return value.perform(context)
    if isinstance(value, list | tuple):
        return "".join(_text(v, context) for v in value)
    return str(value)


class IfCondition(Recorded):
    def __init__(self, predicate, **kwargs):
        super().__init__(predicate, **kwargs)
        self.predicate = predicate

    def evaluate(self, context):
        return _text(self.predicate, context).strip().lower() in ("true", "1")


class UnlessCondition(IfCondition):
    def evaluate(self, context):
        return not super().evaluate(context)


class LaunchDescription(Recorded):
    def __init__(self, initial_entities=None, **kwargs):
        super().__init__(initial_entities, **kwargs)
        self.entities = list(initial_entities or [])

    def add_action(self, action):
        self.entities.append(action)


class FakeContext:
    def __init__(self, configurations):
        self.launch_configurations = dict(configurations)

    def perform_substitution(self, substitution):
        return substitution.perform(self)


class XacroDoc:
    def toxml(self):
        return "<robot/>"


def share_directory(package):
    assert package in SHARES, f"unexpected package {package}"
    return str(SHARES[package])


def fake_modules():
    names = [
        "launch",
        "launch.actions",
        "launch.conditions",
        "launch.event_handlers",
        "launch.substitutions",
        "launch_ros",
        "launch_ros.actions",
        "launch_ros.substitutions",
        "ament_index_python",
        "ament_index_python.packages",
        "xacro",
    ]
    mods = {name: FakeModule(name) for name in names}
    for name, mod in mods.items():
        if "." in name:
            parent, child = name.rsplit(".", 1)
            setattr(mods[parent], child, mod)
    mods["launch"].LaunchDescription = LaunchDescription
    mods["launch.actions"].DeclareLaunchArgument = DeclareLaunchArgument
    mods["launch.actions"].OpaqueFunction = OpaqueFunction
    mods["launch.conditions"].IfCondition = IfCondition
    mods["launch.conditions"].UnlessCondition = UnlessCondition
    mods["launch.substitutions"].LaunchConfiguration = LaunchConfiguration
    mods["ament_index_python.packages"].get_package_share_directory = share_directory
    mods["xacro"].process_file = lambda path, mappings=None, **kwargs: XacroDoc()
    return mods


class Launched:
    def __init__(self, declared, actions):
        self.declared = declared
        self.actions = actions

    def rviz_nodes(self):
        return [
            a
            for a in self.actions
            if isinstance(a, Recorded)
            and (a.kwargs.get("package") == "rviz2" or a.kwargs.get("executable") == "rviz2")
        ]

    def controller_params(self):
        """ros__parameters handed to the cobot_controller spawner ({} without a param file)."""
        spawners = [
            a
            for a in self.actions
            if isinstance(a, Recorded)
            and a.kwargs.get("executable") == "spawner"
            and CONTROLLER in list(a.kwargs.get("arguments", []))
        ]
        assert len(spawners) == 1, "one spawner for cobot_controller expected"
        args = list(spawners[0].kwargs["arguments"])
        if "--param-file" not in args:
            return {}
        path = Path(args[args.index("--param-file") + 1])
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        path.unlink(missing_ok=True)
        return data[CONTROLLER]["ros__parameters"]


def _active(entity, context):
    condition = entity.kwargs.get("condition") if isinstance(entity, Recorded) else None
    return condition is None or condition.evaluate(context)


@pytest.fixture
def run_launch(monkeypatch):
    def run(**overrides):
        for name, mod in fake_modules().items():
            monkeypatch.setitem(sys.modules, name, mod)
        spec = importlib.util.spec_from_file_location("sim_launch_gui_under_test", LAUNCH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        desc = module.generate_launch_description()
        declared = {
            e.name: e.default_value for e in desc.entities if isinstance(e, DeclareLaunchArgument)
        }
        configs = {k: v for k, v in declared.items() if isinstance(v, str)}
        configs |= {"controller": CONTROLLER} | overrides
        context = FakeContext(configs)
        actions = []
        for e in desc.entities:
            if isinstance(e, OpaqueFunction):
                actions += [
                    a
                    for a in (e.function(context, *e.fn_args, **e.fn_kwargs) or [])
                    if _active(a, context)
                ]
            elif not isinstance(e, DeclareLaunchArgument) and _active(e, context):
                actions.append(e)
        return Launched(declared, actions)

    return run


def rviz_config_arg(node):
    args = [str(a) for a in node.kwargs.get("arguments", [])]
    for flag in ("-d", "--display-config"):
        if flag in args:
            return Path(args[args.index(flag) + 1])
    raise AssertionError(f"rviz2 node without -d CONFIG: {args}")


def test_rviz_argument_defaults_to_false(run_launch):
    assert run_launch().declared.get("rviz") == "false"


def test_rviz_false_starts_no_rviz(run_launch):
    launched = run_launch()
    assert launched.rviz_nodes() == []
    assert launched.controller_params().get("command_source", "") == ""


def test_rviz_true_starts_rviz_with_the_waypoint_config(run_launch):
    nodes = run_launch(rviz="true").rviz_nodes()
    assert len(nodes) == 1, "one rviz2 node expected"
    assert rviz_config_arg(nodes[0]).resolve() == RVIZ.resolve()


def test_rviz_true_puts_the_controller_in_external_mode(run_launch):
    assert run_launch(rviz="true").controller_params()["command_source"] == "external"


def test_explicit_command_source_wins_over_rviz(run_launch):
    params = run_launch(rviz="true", command_source="config").controller_params()
    assert params["command_source"] == "config"


def test_rviz_keeps_the_other_controller_overrides(run_launch):
    params = run_launch(rviz="true", seed="7").controller_params()
    assert params["command_source"] == "external"
    assert params["seed"] == 7


# ---------------------------------------------------------------- build, CI, docs


def test_sim_sh_builds_the_gui_and_runs_its_tests():
    text = SIM_SH.read_text(encoding="utf-8")
    assert "ros2_gui" in text, "sim.sh must add ros2_gui to the colcon paths"
    test_case = text.split("\n  test)", 1)
    assert len(test_case) == 2, "sim.sh needs a `test)` command"
    body = test_case[1].split(";;", 1)[0]
    flat = " ".join(body.split())
    assert "colcon" in flat and " test " in f" {flat} ", body
    assert "--packages-select cobot_gui" in flat, body
    assert "test-result" in flat, "the test command must fail when a test fails"


def test_gazebo_ci_runs_the_gui_tests():
    text = GAZEBO_CI.read_text(encoding="utf-8")
    assert any(
        "sim.sh" in line and line.rstrip().split()[-1] == "test" for line in text.splitlines()
    ), "gazebo_ci.sh must call sim.sh test"


def test_ros_image_has_rviz_and_interactive_markers():
    text = ROS_DOCKERFILE.read_text(encoding="utf-8")
    # rviz2 brings rviz_common and rviz_default_plugins; interactive_markers is used directly.
    for pkg in ("ros-jazzy-rviz2", "ros-jazzy-interactive-markers"):
        assert pkg in text, f"docker/ros.Dockerfile must install {pkg}"


def test_readme_documents_the_waypoint_gui():
    text = README.read_text(encoding="utf-8")
    heads = [line for line in text.splitlines() if line.startswith("#") and "Waypoint GUI" in line]
    assert len(heads) == 1, "README needs one 'Waypoint GUI' section"
    section = text.split(heads[0], 1)[1]
    nxt = [i for i, line in enumerate(section.splitlines()) if line.startswith("## ")]
    body = "\n".join(section.splitlines()[: nxt[0]] if nxt else section.splitlines())
    assert "rviz:=true" in body
    for word in ("add", "preview", "run", "stop", "reset"):
        assert word in body.lower(), f"README Waypoint GUI section must describe {word}"
