"""sim/launch/sim.launch.py, Gazebo realism (feature gazebo-realism).

The launch is imported with the ROS launch modules, ament_index and xacro replaced by fakes
(no ROS and no Gazebo needed), then generate_launch_description() and its OpaqueFunction are
run against a fake launch context. Pinned:
- imperfections_path defaults to config/imperfections/realistic.yaml;
- a `seed` argument (default empty = the profile's own seed) reaches the controller as an int;
- the friction of the chosen profile becomes per-joint xacro args joint_<i>_friction (Coulomb,
  N m) and joint_<i>_damping (viscous, N m s/rad), so DART applies friction in the physics;
- the xacro declares those args with default 0 and, when ROS xacro is installed, writes them
  into one <dynamics friction=.. damping=..> per arm joint.
"""

import importlib.util
import sys
import types
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
LAUNCH = REPO / "sim/launch/sim.launch.py"
XACRO = REPO / "sim/urdf/cobot30_5.urdf.xacro"
REALISTIC = REPO / "config/imperfections/realistic.yaml"
IDEAL = REPO / "config/imperfections/ideal.yaml"
PESSIMISTIC = REPO / "config/imperfections/pessimistic.yaml"
SHARES = {"cobot_sim": REPO / "sim", "cobot_ros2": REPO}  # installed share = source layout
N = 7
CONTROLLER = "cobot_controller"


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


def fake_modules(xacro_calls):
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
    mods["launch.substitutions"].LaunchConfiguration = LaunchConfiguration
    mods["ament_index_python.packages"].get_package_share_directory = share_directory

    def process_file(path, mappings=None, **kwargs):
        xacro_calls.append({"path": str(path), "mappings": dict(mappings or {})})
        return XacroDoc()

    mods["xacro"].process_file = process_file
    return mods


class Launched:
    """Result of running the launch: declared defaults, xacro calls, controller parameters."""

    def __init__(self, declared, xacro_calls, actions):
        self.declared = declared
        self.xacro_calls = xacro_calls
        self.actions = actions

    @property
    def mappings(self):
        assert len(self.xacro_calls) == 1, self.xacro_calls
        return self.xacro_calls[0]["mappings"]

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


@pytest.fixture
def run_launch(monkeypatch):
    def run(**overrides):
        xacro_calls = []
        for name, mod in fake_modules(xacro_calls).items():
            monkeypatch.setitem(sys.modules, name, mod)
        spec = importlib.util.spec_from_file_location("sim_launch_under_test", LAUNCH)
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
                actions += list(e.function(context, *e.fn_args, **e.fn_kwargs) or [])
        launched = Launched(declared, xacro_calls, actions)
        return launched

    return run


def per_joint(value):
    """A scalar applies to all joints; a list sets each joint (imperfection schema)."""
    values = [float(v) for v in value] if isinstance(value, list) else [float(value)] * N
    assert len(values) == N
    return values


def profile_friction(path):
    friction = yaml.safe_load(path.read_text(encoding="utf-8"))["imperfections"]["friction"]
    return per_joint(friction["coulomb_nm"]), per_joint(friction["viscous_nm_s_rad"])


def xacro_friction(mappings):
    coulomb = [float(mappings[f"joint_{i}_friction"]) for i in range(1, N + 1)]
    viscous = [float(mappings[f"joint_{i}_damping"]) for i in range(1, N + 1)]
    return coulomb, viscous


# ---------------------------------------------------------------- defaults


def test_imperfections_default_is_realistic(run_launch):
    default = run_launch().declared["imperfections_path"]
    assert isinstance(default, str), "imperfections_path default must be a plain path"
    assert Path(default).as_posix().endswith("config/imperfections/realistic.yaml")


def test_controller_gets_the_realistic_profile_without_arguments(run_launch):
    # Either the launch passes the realistic file as an absolute path, or it passes none and
    # the controller's own default (also realistic.yaml) applies.
    params = run_launch().controller_params()
    if "imperfections_path" in params:
        path = Path(params["imperfections_path"])
        assert path.is_absolute(), path
        assert path.read_bytes() == REALISTIC.read_bytes()


def test_explicit_imperfections_path_reaches_controller(run_launch):
    params = run_launch(imperfections_path=str(PESSIMISTIC)).controller_params()
    assert Path(params["imperfections_path"]) == PESSIMISTIC


def test_seed_argument_defaults_to_the_profile_seed(run_launch):
    launched = run_launch()
    assert launched.declared.get("seed") == "", "seed launch arg, default empty (from file)"
    assert "seed" not in launched.controller_params()


def test_seed_argument_reaches_controller_as_int(run_launch):
    seed = run_launch(seed="7").controller_params()["seed"]
    assert type(seed) is int and seed == 7


# ---------------------------------------------------------------- friction into the physics


def test_realistic_friction_becomes_xacro_args(run_launch):
    # realistic.yaml: coulomb_nm [1.7 x4, 0.24 x3], viscous_nm_s_rad [0.5 x4, 0.1 x3].
    coulomb, viscous = xacro_friction(run_launch(imperfections_path=str(REALISTIC)).mappings)
    assert coulomb == pytest.approx([1.7, 1.7, 1.7, 1.7, 0.24, 0.24, 0.24], rel=1e-9)
    assert viscous == pytest.approx([0.5, 0.5, 0.5, 0.5, 0.1, 0.1, 0.1], rel=1e-9)


def test_default_profile_friction_becomes_xacro_args(run_launch):
    coulomb, viscous = xacro_friction(run_launch().mappings)
    expected_c, expected_v = profile_friction(REALISTIC)
    assert coulomb == pytest.approx(expected_c, rel=1e-9)
    assert viscous == pytest.approx(expected_v, rel=1e-9)


@pytest.mark.parametrize("profile", [IDEAL, PESSIMISTIC], ids=["ideal", "pessimistic"])
def test_profile_friction_becomes_xacro_args(run_launch, profile):
    # ideal: scalars 0.0 -> all zero; pessimistic: [3.5 x4, 0.5 x3] and [1.5 x4, 0.3 x3].
    coulomb, viscous = xacro_friction(run_launch(imperfections_path=str(profile)).mappings)
    expected_c, expected_v = profile_friction(profile)
    assert coulomb == pytest.approx(expected_c, rel=1e-9, abs=0.0)
    assert viscous == pytest.approx(expected_v, rel=1e-9, abs=0.0)


def test_scalar_friction_applies_to_every_joint(run_launch, tmp_path):
    data = yaml.safe_load(REALISTIC.read_text(encoding="utf-8"))
    data["imperfections"]["friction"]["coulomb_nm"] = 0.3
    data["imperfections"]["friction"]["viscous_nm_s_rad"] = 0.05
    custom = tmp_path / "custom.yaml"
    custom.write_text(yaml.safe_dump(data), encoding="utf-8")
    coulomb, viscous = xacro_friction(run_launch(imperfections_path=str(custom)).mappings)
    assert coulomb == pytest.approx([0.3] * N, rel=1e-9)
    assert viscous == pytest.approx([0.05] * N, rel=1e-9)


def test_xacro_still_gets_controllers_yaml(run_launch):
    launched = run_launch(imperfections_path=str(REALISTIC))
    assert Path(launched.xacro_calls[0]["path"]) == XACRO
    assert Path(launched.mappings["controllers_yaml"]) == REPO / "sim/config/controllers.yaml"


def test_xacro_declares_friction_args_with_zero_default():
    root = ET.fromstring(XACRO.read_text(encoding="utf-8"))
    ns = "{http://www.ros.org/wiki/xacro}"
    defaults = {a.get("name"): a.get("default") for a in root.iter(f"{ns}arg")}
    for i in range(1, N + 1):
        for kind in ("friction", "damping"):
            name = f"joint_{i}_{kind}"
            assert name in defaults, f"xacro:arg {name} missing"
            assert float(defaults[name]) == 0.0


def ros_xacro():
    """ROS xacro module when a ROS install is present (WSL dev box, gazebo CI image)."""
    for site in sorted(Path("/opt/ros").glob("*/lib/python3*/site-packages")):
        if str(site) not in sys.path:
            sys.path.append(str(site))
    return pytest.importorskip("xacro")


def joint_dynamics(urdf_xml):
    root = ET.fromstring(urdf_xml)
    out = {}
    for i in range(1, N + 1):
        joints = [j for j in root.findall("joint") if j.get("name") == f"joint_{i}"]
        assert len(joints) == 1, f"joint_{i} defined {len(joints)} times"
        dyn = joints[0].findall("dynamics")
        assert len(dyn) == 1, f"joint_{i}: {len(dyn)} <dynamics> elements"
        out[i] = (float(dyn[0].get("friction")), float(dyn[0].get("damping")))
    return out


def test_processed_xacro_carries_profile_friction():
    xacro = ros_xacro()
    coulomb, viscous = profile_friction(REALISTIC)
    mappings = {"controllers_yaml": ""}
    for i in range(1, N + 1):
        mappings[f"joint_{i}_friction"] = repr(coulomb[i - 1])
        mappings[f"joint_{i}_damping"] = repr(viscous[i - 1])
    dyn = joint_dynamics(xacro.process_file(str(XACRO), mappings=mappings).toxml())
    for i in range(1, N + 1):
        assert dyn[i] == pytest.approx((coulomb[i - 1], viscous[i - 1]), rel=1e-12)


def test_processed_xacro_defaults_to_no_friction():
    xacro = ros_xacro()
    dyn = joint_dynamics(xacro.process_file(str(XACRO), mappings={}).toxml())
    assert all(dyn[i] == (0.0, 0.0) for i in range(1, N + 1))
