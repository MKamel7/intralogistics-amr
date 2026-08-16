"""The carried load, and where its mass ends up.

WHY A MASS AND NOT A PICK AND PLACE

Gazebo's DetachableJoint starts attached and gz-sim 8.11 carries no
`suppress_initial_attach`, so a joint cannot be created where a box was set
down and re-made when the vehicle comes back for the next one. Modelling the
carried mass directly measures the thing that actually matters, which is what
the rated payload does to the braking distance every protective field is sized
from, without pretending to a transfer mechanism that is not simulated.

WHY IT IS ITS OWN LINK

Adding the mass to `body_mass` would put 100 kg at the chassis centroid, which
is 190 mm below the plate it actually sits on and centred in a box 590 mm long.
The centre of mass height is the part that matters under braking, so the load
gets its own inertia at its own place.
"""

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
XACRO = PKG / 'urdf' / 'amr.urdf.xacro'
SPECS = PKG / 'config' / 'platforms'


def build(platform, payload_kg):
    r = subprocess.run(
        ['xacro', str(XACRO), f'platform:={platform}',
         f'payload_kg:={payload_kg}'],
        capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stderr[:2000]
    return ET.fromstring(r.stdout)


def masses(root):
    return {link.get('name'): float(link.find('inertial/mass').get('value'))
            for link in root.findall('link')
            if link.find('inertial/mass') is not None}


def spec(platform):
    return yaml.safe_load((SPECS / f'{platform}.yaml').read_text())['values']


PLATFORMS = sorted(p.stem for p in SPECS.glob('*.yaml'))


@pytest.mark.parametrize('platform', PLATFORMS)
def test_an_empty_vehicle_is_unchanged(platform):
    """Every figure measured without a load must stay comparable.

    If the payload link existed at zero mass it would still add a link, an
    inertia and a joint, and a difference measured against earlier runs would
    be a difference in the model rather than in the load.
    """
    root = build(platform, 0.0)
    assert 'payload' not in masses(root), (
        'an empty vehicle carries a payload link, so it is not the same '
        'vehicle every previous measurement was taken on')


@pytest.mark.parametrize('platform', PLATFORMS)
def test_the_rated_payload_adds_exactly_its_mass(platform):
    v = spec(platform)
    empty = sum(masses(build(platform, 0.0)).values())
    laden = sum(masses(build(platform, v['max_payload'])).values())
    assert laden - empty == pytest.approx(v['max_payload'], abs=1e-6), (
        f'{platform} gains {laden - empty:.3f} kg carrying a '
        f'{v["max_payload"]} kg load')


@pytest.mark.parametrize('platform', PLATFORMS)
def test_the_load_sits_on_the_plate_not_in_the_chassis(platform):
    """The centre of mass height is what changes braking behaviour.

    Folding the load into body_mass would place it at the chassis centroid,
    which on the MP-400 is 190 mm below the surface it rests on.
    """
    v = spec(platform)
    root = build(platform, v['max_payload'])
    joint = next(j for j in root.findall('joint')
                 if j.get('name') == 'payload_joint')
    z = float(joint.find('origin').get('xyz').split()[2])
    assert z == pytest.approx(v['top_plate_height'], abs=1e-9), (
        f'the payload joint sits at {z} m, not on the {v["top_plate_height"]} '
        f'm top plate')
    assert joint.get('type') == 'fixed'


@pytest.mark.parametrize('platform', PLATFORMS)
def test_the_load_raises_the_centre_of_mass(platform):
    """Stated as behaviour, because it is the reason for all of the above.

    The height has to be resolved through the JOINT TREE. The first version of
    this test summed each link's inertial origin as if every link frame were
    base_link, which reads the payload's 0.100 m local offset as 0.100 m above
    the ground instead of 0.481 m, and duly reported that carrying 100 kg on
    the roof LOWERS the centre of mass. The model was right and the test was
    wrong, which is worth recording because the number it produced was
    plausible enough to have been believed.

    Translations only. Every joint from base_link to a mass bearing link on
    this vehicle is fixed or a wheel axis with no vertical offset from its
    rotation, so ignoring rotation does not move a z height here. Stated
    because it would stop being true on a vehicle with a mast or an arm.
    """
    v = spec(platform)

    def com_z(root):
        origin = {}
        for j in root.findall('joint'):
            o = j.find('origin')
            xyz = (o.get('xyz') if o is not None else '0 0 0') or '0 0 0'
            origin[j.find('child').get('link')] = (
                j.find('parent').get('link'), float(xyz.split()[2]))

        def height(link):
            z = 0.0
            seen = set()
            while link in origin:
                assert link not in seen, f'cycle in the joint tree at {link}'
                seen.add(link)
                parent, dz = origin[link]
                z += dz
                link = parent
            return z

        total = moment = 0.0
        for link in root.findall('link'):
            m = link.find('inertial/mass')
            if m is None:
                continue
            mass = float(m.get('value'))
            o = link.find('inertial/origin')
            local = float(o.get('xyz').split()[2]) if o is not None else 0.0
            total += mass
            moment += mass * (height(link.get('name')) + local)
        return moment / total

    empty = com_z(build(platform, 0.0))
    laden = com_z(build(platform, v['max_payload']))
    assert laden > empty, (
        f'{platform} centre of mass does not rise under load: '
        f'{empty:.4f} -> {laden:.4f} m')
    # And it must rise by a sane amount rather than by a rounding error.
    assert laden - empty > 0.05, (
        f'{platform} centre of mass rises only {(laden - empty) * 1000:.0f} mm '
        f'carrying {v["max_payload"]} kg, which suggests the load is not where '
        f'the joint says it is')


@pytest.mark.parametrize('platform', PLATFORMS)
def test_the_load_is_not_visible_to_the_vehicles_own_scanners(platform):
    """No collision geometry, deliberately.

    The scan plane sits at scanner_mount_height and the load sits on the top
    plate, far above it, so the vehicle cannot see its own cargo. Giving the
    load collision would let it catch on racking the protective fields were
    never sized to clear, which measures the model rather than the vehicle.
    """
    v = spec(platform)
    root = build(platform, v['max_payload'])
    payload = next(link for link in root.findall('link')
                   if link.get('name') == 'payload')
    assert payload.find('collision') is None, (
        'the payload has collision geometry, so it can snag on structure the '
        'protective fields do not cover')
    assert v['top_plate_height'] > v['scanner_mount_height'], (
        'the top plate is at or below the scan plane, so a load would occlude '
        'the vehicle\'s own scanners and this test is no longer sufficient')


@pytest.mark.parametrize('platform', PLATFORMS)
def test_a_negative_payload_is_not_silently_carried(platform):
    """Guarding the one input that could quietly reduce the vehicle's mass."""
    root = build(platform, -50.0)
    assert 'payload' not in masses(root), (
        'a negative payload built a link, which would subtract mass from the '
        'vehicle and flatter every braking figure')
