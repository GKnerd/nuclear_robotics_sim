"""Expand a MuJoCo scene template with a YAML layout config.

The static scene (materials, robot <include>, lights, floor) lives in a normal
MuJoCo XML file you can edit by hand (e.g. scenes/radiation_room.xml). This
module loads that file as an XML tree, then injects the room size + walls +
canisters described by a YAML config -- so layouts/room size/materials vary per
run without touching the XML.

Used by hello_stretch_sim_bringup.launch.py:
    write_scene_from_config(config_yaml, template_xml, out_xml)

Config schema (see config/radiation_room.yaml):

    room:
      half_x: 15.0          # floor is 2*half_x wide (X); walls sit at +/-half_x
      half_y: 15.0          # floor is 2*half_y deep  (Y); walls sit at +/-half_y
      wall_height: 3.0
      wall_thickness: 0.2
    canister:
      radius: 0.89          # outer radius R_out (D = 1.78 m)
      half_height: 2.09     # H/2 (H = 4.18 m)
      material: G4_STAINLESS-STEEL   # default material for every canister
    canisters:
      - {x: -6, y: -5}                          # upright; z defaults to half_height
      - {x:  6, y:  5, roll: 90, z: 0.89}       # tipped onto its side (z = radius)
      - {x:  0, y:  0, material: G4_Pb}         # per-canister material override

Per-canister: x, y required. z defaults to half_height (upright, base on floor).
roll/pitch/yaw are in DEGREES. `material` defaults to canister.material.
Any material name referenced but not already defined in the template's <asset>
is added with a neutral-grey placeholder (the name is what the Geant4 bridge
maps on, so the colour is cosmetic).
"""

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Iterable, Union, Optional

import yaml
from scipy.spatial.transform import Rotation as R

DEFAULT_CANISTER_MATERIAL = "G4_STAINLESS-STEEL"
WALL_MATERIAL = "G4_CONCRETE"

# This module is imported from its symlink-installed copy (share/.../utils); resolve()
# follows the symlink back to the SRC tree, so the generated scene is written under src/
# (persists across rebuilds, reusable later) instead of the ephemeral install/share dir.
# Stable name, overwritten each run -> always the most recent generated scene.
PACKAGE_SRC_DIR = Path(__file__).resolve().parent.parent
LATEST_SCENE = PACKAGE_SRC_DIR / "generated" / "radiation_room_latest.xml"


def load_xml(scene_xml: Union[Path, str]) -> ET.ElementTree:
    """Load and parse a MuJoCo scene XML file into an ElementTree."""
    return ET.parse(scene_xml)


def load_yaml(file_path: Union[Path, str]) -> Dict:
    """Load and parse a YAML file into a python config dict."""
    with open(file_path, "r") as file:
        return yaml.safe_load(file) or {}


def euler_deg_to_quat(roll, pitch, yaw):
    """XYZ euler angles in degrees -> MuJoCo quaternion (w, x, y, z).

    scipy's Rotation.as_quat() is scalar-last (x, y, z, w); MuJoCo wants
    scalar-first (w, x, y, z), so we reorder.
    """
    x, y, z, w = R.from_euler("xyz", [roll, pitch, yaw], degrees=True).as_quat()
    return (w, x, y, z)


def ensure_materials(root: ET.Element, names: Iterable[str]) -> None:
    """Make sure each material name exists in <asset> (MuJoCo errors otherwise).

    Names referenced from the config but missing from the template get a
    neutral-grey placeholder so the scene still compiles.
    """
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    existing = {m.get("name") for m in asset.findall("material")}
    for name in names:
        if name not in existing:
            ET.SubElement(asset, "material", {"name": name, "rgba": "0.6 0.6 0.6 1"})


def modify_scene_xml(config: Dict, tree: ET.ElementTree) -> ET.ElementTree:
    """Inject room size, walls and canisters from `config` into a parsed scene.

    Operates on a loaded scene template (see load_xml). Idempotent: any geoms
    named ``wall_*`` or ``canister_*`` already present are removed first, so the
    same template can be re-expanded with a different config.
    """
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("scene template has no <worldbody> element")

    room = config.get("room", {})
    hx = float(room.get("half_x", 10.0))
    hy = float(room.get("half_y", 10.0))
    wh = float(room.get("wall_height", 3.0))
    wt = float(room.get("wall_thickness", 0.2))

    can = config.get("canister", {})
    radius = float(can.get("radius", 0.89))
    half_h = float(can.get("half_height", 2.09))
    default_material = can.get("material", DEFAULT_CANISTER_MATERIAL)

    # Resize the floor to the room (the geom named "floor" must stay; robot needs it).
    floor = worldbody.find("geom[@name='floor']")
    if floor is not None:
        floor.set("size", f"{hx} {hy} .05")

    # Drop previously-generated walls/canisters so re-expansion is clean.
    for geom in list(worldbody.findall("geom")):
        name = geom.get("name", "")
        if name.startswith("wall_") or name.startswith("canister_"):
            worldbody.remove(geom)

    # Perimeter walls: concrete boxes on the floor edges.
    used_materials = {WALL_MATERIAL}
    wz = wh / 2.0
    ht = wt / 2.0
    for name, pos, size in (
        ("wall_north", f"0 {hy} {wz}", f"{hx} {ht} {wz}"),
        ("wall_south", f"0 {-hy} {wz}", f"{hx} {ht} {wz}"),
        ("wall_east", f"{hx} 0 {wz}", f"{ht} {hy} {wz}"),
        ("wall_west", f"{-hx} 0 {wz}", f"{ht} {hy} {wz}"),
    ):
        ET.SubElement(
            worldbody,
            "geom",
            {"name": name, "type": "box", "pos": pos, "size": size, "material": WALL_MATERIAL},
        )

    # Interior / maze walls from config: a box at (x, y) on the floor, `length`
    # along its local X, `thickness` along Y, `height` tall, rotated by `yaw` (deg).
    for i, wcfg in enumerate(config.get("walls", []), start=1):
        wx = float(wcfg.get("x", 0.0))
        wy = float(wcfg.get("y", 0.0))
        length = float(wcfg.get("length", 1.0))
        thickness = float(wcfg.get("thickness", wt))
        height = float(wcfg.get("height", wh))
        material = wcfg.get("material", WALL_MATERIAL)
        used_materials.add(material)
        w, qx, qy, qz = euler_deg_to_quat(0.0, 0.0, float(wcfg.get("yaw", 0.0)))
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": f"wall_inner_{i}",
                "type": "box",
                "size": f"{length / 2} {thickness / 2} {height / 2}",
                "pos": f"{wx} {wy} {height / 2}",
                "quat": f"{w:.6f} {qx:.6f} {qy:.6f} {qz:.6f}",
                "material": material,
            },
        )

    # Canisters: closed solid cylinders; material per-canister or canister default.
    for i, c in enumerate(config.get("canisters", []), start=1):
        x = float(c.get("x", 0.0))
        y = float(c.get("y", 0.0))
        z = float(c["z"]) if c.get("z") is not None else half_h  # upright default
        material = c.get("material", default_material)
        used_materials.add(material)
        w, qx, qy, qz = euler_deg_to_quat(
            float(c.get("roll", 0.0)), float(c.get("pitch", 0.0)), float(c.get("yaw", 0.0))
        )
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": f"canister_{i}",
                "type": "cylinder",
                "size": f"{radius} {half_h}",
                "pos": f"{x} {y} {z}",
                "quat": f"{w:.6f} {qx:.6f} {qy:.6f} {qz:.6f}",
                "material": material,
            },
        )

    # Auto-define any referenced material not already in the template's <asset>.
    ensure_materials(root, used_materials)
    return tree


def write_scene_from_config(
    config_path: Union[Path, str],
    template_path: Union[Path, str],
    out_path: Optional[Union[Path, str]] = None,
) -> str:
    """Load `template_path`, inject `config_path`'s layout, write to `out_path`."""
    config = load_yaml(Path(config_path))
    tree = load_xml(Path(template_path))
    modify_scene_xml(config=config, tree=tree)
    if not out_path:
        LATEST_SCENE.parent.mkdir(parents=True, exist_ok=True)
        out_path = LATEST_SCENE

    ET.indent(tree, space="  ")  # pretty-print: one element per line, consistent indent
    tree.write(str(out_path), encoding="unicode")
    return str(out_path)
