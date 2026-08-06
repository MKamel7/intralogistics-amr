#!/usr/bin/env python3
"""Import the AWS RoboMaker small-warehouse assets into a Gazebo Harmonic model set.

The AWS package targets Gazebo Classic. Its meshes are COLLADA with the materials
baked into the DAE, so they render fine in Harmonic; only two things stop the
models loading:

  1. Every mesh uri is 'file://models/<model>/meshes/<mesh>.DAE', a path relative
     to the Classic model database root. Harmonic resolves 'model://<model>/...'
     against GZ_SIM_RESOURCE_PATH instead.
  2. The model names collide with the copies still on disk under the AWS source
     tree, and Harmonic finds those first, so a rewritten copy under the same
     name is silently shadowed by the original (this cost an hour to diagnose:
     the parser kept reporting the OLD uri from a model.sdf we were not editing).

So every model is re-emitted under a 'wf_' prefix with model:// uris. The world
is regenerated from the AWS world's include list, keeping the original layout
poses, in SDF 1.10 with Harmonic system plugins.

Usage:
    python3 tools/import_aws_warehouse.py \
        --src ~/ros2_ws/src/aws-robomaker-small-warehouse-world \
        --dst .

Re-runnable: it clears and rewrites the generated model set each time.
"""

import argparse
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

# AWS model name -> our name. Prefixed so nothing can shadow these, and shortened
# so the world file stays readable.
PREFIX = 'wf_'


def our_name(aws_name: str) -> str:
    """aws_robomaker_warehouse_ShelfF_01 -> wf_shelf_f_01."""
    stem = re.sub(r'^aws_robomaker_warehouse_', '', aws_name)
    # ShelfF_01 -> Shelf_F_01, ClutteringA_01 -> Cluttering_A_01
    stem = re.sub(r'([a-z])([A-Z])', r'\1_\2', stem)
    stem = re.sub(r'([A-Za-z])([A-Z])(?=_?\d)', r'\1_\2', stem)
    return PREFIX + stem.lower()


INERTIA_KEYS = ('ixx', 'iyy', 'izz')


def repair_inertia(sdf: str, name: str, static: bool):
    """Make the inertia tensor satisfy the triangle inequality.

    Two AWS models ship tensors that are not physically realisable:
    Ground_B has ixx+izz < iyy, and Roof_B has ixx+iyy < izz (by 0.7 percent).
    Gazebo Classic never validated this. Harmonic rejects the whole world with
    'A link named link has invalid inertia', which is the correct behaviour.

    Every AWS scenery model is static, so its inertia is never integrated and
    the repair changes no observable behaviour. We set the offending diagonal
    entries to the largest of the three, which trivially satisfies
    i_a + i_b >= i_c and keeps the magnitude in the right range. A non-static
    model with a bad tensor is a real modelling error, so that is raised
    instead of silently patched.

    Returns (sdf, repaired: bool).
    """
    vals = {}
    for k in INERTIA_KEYS:
        m = re.search(rf'<{k}>\s*([^<]+?)\s*</{k}>', sdf)
        if not m:
            return sdf, False
        try:
            vals[k] = float(m.group(1))
        except ValueError:
            return sdf, False

    ixx, iyy, izz = (vals[k] for k in INERTIA_KEYS)
    ok = (min(ixx, iyy, izz) > 0
          and ixx + iyy >= izz and ixx + izz >= iyy and iyy + izz >= ixx)
    if ok:
        return sdf, False
    if not static:
        raise SystemExit(
            f'{name}: non-static model has an unrealisable inertia tensor '
            f'({ixx}, {iyy}, {izz}); fix the source model rather than importing it')

    big = max(ixx, iyy, izz)
    for k in INERTIA_KEYS:
        sdf = re.sub(rf'<{k}>\s*[^<]+?\s*</{k}>', f'<{k}>{big:.6f}</{k}>',
                     sdf, count=1)
    return sdf, True


MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <description>
    Imported from {aws_name} (AWS RoboMaker small warehouse world, Apache 2.0).
    Mesh uris rewritten from Classic 'file://models/...' to 'model://...' for
    Gazebo Harmonic. Geometry and textures are unmodified.
  </description>
</model>
"""


def convert_model(src_model: Path, dst_models: Path):
    """Copy one AWS model into the Harmonic model set.

    Returns (new_name, inertia_repaired).
    """
    aws_name = src_model.name
    name = our_name(aws_name)
    dst = dst_models / name
    dst.mkdir(parents=True, exist_ok=True)

    for sub in ('meshes', 'materials'):
        if (src_model / sub).is_dir():
            shutil.copytree(src_model / sub, dst / sub, dirs_exist_ok=True)

    sdf = (src_model / 'model.sdf').read_text()
    # file://models/<aws_name>/  ->  model://<name>/
    sdf = sdf.replace(f'file://models/{aws_name}/', f'model://{name}/')
    # The model element itself must carry the new name or includes resolve to a
    # model whose internal name disagrees with its directory.
    sdf = re.sub(r'(<model\s+name=)"[^"]*"', rf'\1"{name}"', sdf, count=1)
    sdf = sdf.replace('<sdf version="1.6">', '<sdf version="1.10">')

    leftover = re.findall(r'file://[^<"\s]+', sdf)
    if leftover:
        raise SystemExit(f'{aws_name}: unrewritten uri(s) {leftover}')

    static = re.search(r'<static>\s*(1|true)\s*</static>', sdf) is not None
    sdf, repaired = repair_inertia(sdf, name, static)

    (dst / 'model.sdf').write_text(sdf)
    (dst / 'model.config').write_text(
        MODEL_CONFIG.format(name=name, aws_name=aws_name))
    return name, repaired


WORLD_HEADER = """<?xml version="1.0"?>
<!-- Generated by tools/import_aws_warehouse.py. Do not hand-edit the imported
     layout below; edit the generator, or add fleet-specific models (conveyor
     stations, chargers, KLT boxes, pedestrians) in a separate include file so a
     re-import does not clobber them. -->
<sdf version="1.10">
  <world name="{world_name}">

    <physics name="default" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-contact-system"
            name="gz::sim::systems::Contact"/>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.78 0.82 1</background>
      <shadows>false</shadows>
    </scene>

    <!-- Shadows off on purpose: this runs on an Iris Xe iGPU and shadow passes
         cost more real-time factor than they buy in a warehouse with flat
         overhead lighting. -->
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
      <cast_shadows>false</cast_shadows>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <material>
            <ambient>0.4 0.4 0.42 1</ambient>
            <diffuse>0.5 0.5 0.52 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

"""

WORLD_FOOTER = """
  </world>
</sdf>
"""


def parse_layout(world_path: Path):
    """Pull (aws_model_name, pose) out of the AWS world, in file order.

    The AWS world wraps each include in an outer <model> that carries the pose,
    which is legal SDF but unusual; the pose we want is that outer one.
    """
    root = ET.parse(world_path).getroot()
    world = root.find('world')
    items = []
    for model in world.findall('model'):
        inc = model.find('include')
        if inc is None:
            continue
        uri = (inc.findtext('uri') or '').strip()
        if not uri.startswith('model://'):
            continue
        pose = (model.findtext('pose') or '0 0 0 0 0 0').strip()
        items.append((model.get('name'), uri[len('model://'):], pose))
    return items


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', required=True, type=Path,
                    help='aws-robomaker-small-warehouse-world checkout')
    ap.add_argument('--dst', required=True, type=Path,
                    help='package root to write models/ and worlds/ into')
    ap.add_argument('--world', default='no_roof_small_warehouse',
                    help='which AWS world to take the layout from')
    ap.add_argument('--world-name', default='warehouse',
                    help='name of the generated world')
    args = ap.parse_args()

    src_models = args.src.expanduser() / 'models'
    src_world = (args.src.expanduser() / 'worlds' / args.world /
                 f'{args.world}.world')
    if not src_models.is_dir() or not src_world.is_file():
        sys.exit(f'source not found under {args.src}')

    dst_models = args.dst.expanduser() / 'models'
    dst_worlds = args.dst.expanduser() / 'worlds'
    dst_worlds.mkdir(parents=True, exist_ok=True)

    # Clear only our generated models, never anything hand-authored alongside.
    if dst_models.is_dir():
        for d in sorted(dst_models.glob(f'{PREFIX}*')):
            shutil.rmtree(d)
    dst_models.mkdir(parents=True, exist_ok=True)

    mapping, repaired = {}, []
    for src_model in sorted(p for p in src_models.iterdir() if p.is_dir()):
        name, fixed = convert_model(src_model, dst_models)
        mapping[src_model.name] = name
        if fixed:
            repaired.append(name)
    print(f'converted {len(mapping)} models -> {dst_models}')
    if repaired:
        print(f'repaired unrealisable inertia on {len(repaired)} static '
              f'model(s): {", ".join(repaired)}')

    layout = parse_layout(src_world)
    body = []
    for inst_name, aws_name, pose in layout:
        if aws_name not in mapping:
            print(f'  skip {inst_name}: no model {aws_name}', file=sys.stderr)
            continue
        inst = re.sub(r'^aws_robomaker_warehouse_', PREFIX, inst_name).lower()
        body.append(f'    <include>\n'
                    f'      <name>{inst}</name>\n'
                    f'      <uri>model://{mapping[aws_name]}</uri>\n'
                    f'      <pose>{pose}</pose>\n'
                    f'    </include>\n')

    out = dst_worlds / f'{args.world_name}.sdf'
    out.write_text(WORLD_HEADER.format(world_name=args.world_name)
                   + ''.join(body) + WORLD_FOOTER)
    print(f'wrote {out} with {len(body)} model instances')


if __name__ == '__main__':
    main()
