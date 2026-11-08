"""Split the joined Spyder STL into links and write spyder.urdf.

Spyder parts/main.stl keeps every CAD body as its own closed shell, so shells are
identified by their bounding box in a leg frame (spyder.json) rather than edited.
All six legs are the same assembly, which is verified, so they share one mesh set.
Generated STL vertices are metres in their owning URDF link frame.

Frames: body is CAD axes with its origin at the robot's XY centre, CAD Z=0.
Each leg frame sits on the yaw axis at hip-axis height, X out along the leg, Z up.
"""
from pathlib import Path
import hashlib
import json
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import ConvexHull, cKDTree
import trimesh

from build_assembly import numbers, visual_link

HERE = Path(__file__).resolve().parent
MESHES = HERE / 'meshes' / 'spyder'
LEG_LINKS = ('base', 'coxa', 'femur', 'tibia')


def read_stl(path):
    """Return binary STL triangles as an (n, 3, 3) array in file order."""
    data = Path(path).read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84 + 50 * count:
        raise ValueError(f'{Path(path).name} is not a binary STL with the expected layout')
    layout = np.dtype([('normal', '<3f4'), ('vertices', '<9f4'), ('attribute', '<u2')])
    return np.frombuffer(data, layout, count, 84)['vertices'].reshape(-1, 3, 3).astype(float)


def write_stl(path, triangles):
    triangles = np.asarray(triangles, dtype=float)
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-20)
    layout = np.dtype([('normal', '<3f4'), ('vertices', '<9f4'), ('attribute', '<u2')])
    records = np.zeros(len(triangles), layout)
    records['normal'] = normals
    records['vertices'] = triangles.reshape(-1, 9)
    Path(path).write_bytes(b'build_spyder.py'.ljust(80) + struct.pack('<I', len(triangles)) + records.tobytes())


def split_shells(triangles):
    """Return one triangle-index array per connected shell, largest first."""
    vertices, inverse = np.unique(np.round(triangles.reshape(-1, 3), 3), axis=0, return_inverse=True)
    faces = inverse.ravel().reshape(-1, 3)
    edges = np.r_[faces[:, [0, 1]], faces[:, [1, 2]]]
    graph = coo_matrix((np.ones(len(edges), bool), (edges[:, 0], edges[:, 1])),
                       shape=(len(vertices), len(vertices)))
    labels = connected_components(graph, directed=False)[1][faces[:, 0]]
    order = np.argsort(labels, kind='stable')
    shells = np.split(order, np.cumsum(np.bincount(labels))[:-1])
    return sorted(shells, key=len, reverse=True)


def heading_rotation(degrees):
    c, s = np.cos(np.radians(degrees)), np.sin(np.radians(degrees))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def to_frame(points, origin, rotation):
    return (points - origin) @ rotation


def shaft_centre(points, axis):
    """Centre of the output spline: the 0.6 mm of a servo furthest along its +axis."""
    top = points[points[:, axis] > points[:, axis].max() - .6]
    return (top.min(0) + top.max(0)) / 2


def find_leg_frames(config, shells, triangles, centre):
    """Locate each leg's yaw axis from the upright servo nearest to its heading."""
    size = np.array(config['servo_size_mm'])
    frames = []
    claimed = set()
    points = [triangles[shell].reshape(-1, 3) for shell in shells]
    for leg in config['legs']:
        rotation = heading_rotation(leg['heading_deg'])
        best = None
        for index, shell_points in enumerate(points):
            if index in claimed or len(shells[index]) < 1000:
                continue
            local = to_frame(shell_points, centre, rotation)
            if not np.allclose(np.ptp(local, axis=0), size, atol=.05):
                continue
            middle = (local.min(0) + local.max(0)) / 2
            bearing = abs(np.arctan2(middle[1], middle[0]))
            if best is None or bearing < best[0]:
                best = (bearing, index, local)
        if best is None:
            raise ValueError(f"No upright yaw servo found for leg {leg['name']}")
        claimed.add(best[1])
        spline = shaft_centre(best[2], 2)
        frames.append({'rotation': rotation, 'xy': centre + rotation @ np.array([spline[0], spline[1], 0])})
    return frames


def identify_parts(config, shells, triangles, frames):
    """Assign each shell to (leg, part), a servo's small hardware, or the body."""
    parts = config['leg_parts']
    bounds = np.array([[part['low_mm'], part['high_mm']] for part in parts])
    tolerance = config['match_tolerance_mm']
    assigned, owners = {}, {}
    for leg_index, frame in enumerate(frames):
        for shell_index, shell in enumerate(shells):
            if shell_index in owners:
                continue
            local = to_frame(triangles[shell].reshape(-1, 3), frame['origin'], frame['rotation'])
            box = np.array([local.min(0), local.max(0)])
            match = np.flatnonzero(np.all(np.abs(bounds - box) < tolerance, axis=(1, 2)))
            if len(match) == 1:
                key = (leg_index, parts[match[0]]['name'])
                if key in assigned:
                    raise ValueError(f'Two shells match {key}')
                assigned[key] = shell_index
                owners[shell_index] = key
    missing = [(i, p['name']) for i in range(len(frames)) for p in parts if (i, p['name']) not in assigned]
    if missing:
        raise ValueError(f'Shells not found: {missing[:6]}')

    servos = [part for part in parts if part.get('servo')]
    hardware = {(i, s['name']): [] for i in range(len(frames)) for s in servos}
    body = []
    for shell_index, shell in enumerate(shells):
        if shell_index in owners:
            continue
        shell_points = triangles[shell].reshape(-1, 3)
        if np.ptp(shell_points, axis=0).max() > config['small_shell_max_mm']:
            body.append(shell_index)
            continue
        placed = False
        for leg_index, frame in enumerate(frames):
            local = to_frame(shell_points, frame['origin'], frame['rotation'])
            for servo in servos:
                low, high = np.array(servo['low_mm']) - .5, np.array(servo['high_mm']) + .5
                if np.all(local >= low) and np.all(local <= high):
                    hardware[(leg_index, servo['name'])].append(shell_index)
                    placed = True
                    break
            if placed:
                break
        if not placed:
            raise ValueError(f'Small shell {shell_index} is not on a servo')
    expected = config['small_shells_per_servo']
    for key, members in hardware.items():
        if len(members) != expected:
            raise ValueError(f'{key} has {len(members)} small shells, expected {expected}')
    if len(body) != config['body_shell_count']:
        raise ValueError(f"Found {len(body)} body shells, expected {config['body_shell_count']}")
    return assigned, hardware, body


def mass_properties(triangles, mass=None, density=None):
    """Mass, centre and inertia about that centre for one closed shell (metres)."""
    faces = np.arange(len(triangles) * 3).reshape(-1, 3)
    mesh = trimesh.Trimesh(np.round(triangles.reshape(-1, 3), 8), faces, process=True)
    if mesh.volume < 0:
        mesh.invert()
    volume = float(mesh.volume)
    if volume <= 0:
        raise ValueError('Shell has no enclosed volume')
    if mass is None:
        mass = volume * density
    return mass, np.array(mesh.center_mass), np.array(mesh.moment_inertia) * mass / volume


def combine_inertia(items):
    total = sum(item[0] for item in items)
    com = sum(item[0] * item[1] for item in items) / total
    inertia = np.zeros((3, 3))
    for mass, centre, tensor in items:
        d = centre - com
        inertia += tensor + mass * (d @ d * np.eye(3) - np.outer(d, d))
    return total, com, inertia


def add_inertial(link, props):
    mass, com, inertia = props
    if np.linalg.eigvalsh(inertia).min() <= 0:
        raise ValueError(f"{link.get('name')} inertia is not positive definite")
    element = ET.SubElement(link, 'inertial')
    ET.SubElement(element, 'origin', xyz=numbers(com), rpy='0 0 0')
    ET.SubElement(element, 'mass', value=f'{mass:.8g}')
    ET.SubElement(element, 'inertia', ixx=f'{inertia[0, 0]:.8g}', iyy=f'{inertia[1, 1]:.8g}',
                  izz=f'{inertia[2, 2]:.8g}', ixy=f'{inertia[0, 1]:.8g}', ixz=f'{inertia[0, 2]:.8g}',
                  iyz=f'{inertia[1, 2]:.8g}')


def add_link(robot, name, props, collision):
    link = ET.SubElement(robot, 'link', name=name)
    # Real part meshes live on fixed children to preserve GUI colours.
    hidden = ET.SubElement(link, 'visual', name='invisible_frame')
    ET.SubElement(ET.SubElement(hidden, 'geometry'), 'sphere', radius='0.000000001')
    ET.SubElement(ET.SubElement(hidden, 'material', name=name + '_invisible'), 'color', rgba='0 0 0 0')
    add_inertial(link, props)
    element = ET.SubElement(link, 'collision', name=name + '_hull')
    ET.SubElement(element, 'origin', xyz='0 0 0', rpy='0 0 0')
    ET.SubElement(ET.SubElement(element, 'geometry'), 'mesh', filename=collision, scale='1 1 1')
    return link


def add_joint(robot, name, kind, parent, child, xyz, rpy=(0, 0, 0), axis=None, limits=None, servo=None):
    joint = ET.SubElement(robot, 'joint', name=name, type=kind)
    ET.SubElement(joint, 'parent', link=parent)
    ET.SubElement(joint, 'child', link=child)
    ET.SubElement(joint, 'origin', xyz=numbers(xyz), rpy=numbers(rpy))
    if kind == 'revolute':
        ET.SubElement(joint, 'axis', xyz=numbers(axis))
        lower, upper = np.radians(limits)
        ET.SubElement(joint, 'limit', lower=f'{lower:.10g}', upper=f'{upper:.10g}',
                      effort=str(servo['torque_nm']), velocity=str(servo['speed_rad_s']))
        ET.SubElement(joint, 'dynamics', damping=str(servo['damping']), friction=str(servo['friction']))


def write_hull(name, triangles):
    points = np.unique(np.round(triangles.reshape(-1, 3), 6), axis=0)
    hull = ConvexHull(points)
    write_stl(MESHES / f'{name}_hull.stl', points[hull.simplices])
    return f'meshes/spyder/{name}_hull.stl'


def congruence_error(a, b):
    """Largest nearest-vertex distance between two rigid point sets, either way."""
    a, b = np.unique(np.round(a, 4), axis=0), np.unique(np.round(b, 4), axis=0)
    return max(cKDTree(b).query(a)[0].max(), cKDTree(a).query(b)[0].max())


def build():
    config = json.loads((HERE / 'spyder.json').read_text(encoding='utf-8'))
    source = (HERE / config['source']).resolve()
    triangles = read_stl(source)
    shells = split_shells(triangles)
    bounds = triangles.min((0, 1)), triangles.max((0, 1))
    centre = np.array([(bounds[0][0] + bounds[1][0]) / 2, (bounds[0][1] + bounds[1][1]) / 2, 0])
    frames = find_leg_frames(config, shells, triangles, centre)
    for frame in frames:
        frame['origin'] = frame['xy'] + np.array([0, 0, config['hip_height_mm']])
    assigned, hardware, body_shells = identify_parts(config, shells, triangles, frames)

    # Joint axes come from the servo output splines of the first leg.
    def leg_points(leg_index, part):
        frame = frames[leg_index]
        return to_frame(triangles[shells[assigned[(leg_index, part)]]].reshape(-1, 3), frame['origin'], frame['rotation'])

    hip = shaft_centre(leg_points(0, 'hip_servo'), 1)
    knee = shaft_centre(leg_points(0, 'knee_servo'), 1)
    if abs(hip[2]) > .01:
        raise ValueError(f"hip_height_mm should be {config['hip_height_mm'] + hip[2]:.4f}")
    hip_origin = np.array([hip[0], 0, 0])
    knee_origin = np.array([knee[0] - hip[0], 0, knee[2] - hip[2]])
    link_origin = {'base': np.zeros(3), 'coxa': np.zeros(3), 'femur': hip_origin, 'tibia': hip_origin + knee_origin}

    # Group shells by link and appearance in the first leg; check the others match.
    groups = {}
    for part in config['leg_parts']:
        groups.setdefault(part['group'], []).append(part)
    leg_meshes, leg_mass = {}, {link: [] for link in LEG_LINKS}
    report_congruence = {}
    servo_mass = config['mass']['servo_kg']
    density = config['mass']['printed_density_kg_m3']
    for group, members in groups.items():
        link = config['groups'][group]['link']
        offset = link_origin[link]
        collected = [[] for _ in frames]
        for part in members:
            for leg_index, frame in enumerate(frames):
                indices = [assigned[(leg_index, part['name'])]]
                if part.get('servo'):
                    indices += hardware[(leg_index, part['name'])]
                for index in indices:
                    local = to_frame(triangles[shells[index]].reshape(-1, 3), frame['origin'], frame['rotation'])
                    collected[leg_index].append(local - offset)
            main = to_frame(triangles[shells[assigned[(0, part['name'])]]], frames[0]['origin'], frames[0]['rotation']) - offset
            leg_mass[link].append(mass_properties(main / 1000, mass=servo_mass if part.get('servo') else None,
                                                  density=density))
        points = [np.vstack(c) for c in collected]
        report_congruence[group] = round(max(congruence_error(points[0], other) for other in points[1:]), 4)
        if report_congruence[group] > config['leg_congruence_tolerance_mm']:
            raise ValueError(f'Legs differ in {group} by {report_congruence[group]} mm')
        leg_meshes[group] = points[0].reshape(-1, 3, 3) / 1000

    MESHES.mkdir(parents=True, exist_ok=True)
    for group, mesh in leg_meshes.items():
        write_stl(MESHES / f'{group}.stl', mesh)
    body_triangles = np.vstack([triangles[shells[i]] for i in body_shells])
    body_mesh = (body_triangles - centre) / 1000
    write_stl(MESHES / 'body.stl', body_mesh)

    robot = ET.Element('robot', name='spyder')
    body_mass = [mass_properties((triangles[shells[i]] - centre) / 1000, density=density) for i in body_shells]
    extra = config['mass']['electronics_kg']
    if extra > 0:
        body_mass.append((extra, np.array(config['mass']['electronics_centre_mm']) / 1000, np.eye(3) * extra * 1e-3))
    body_props = combine_inertia(body_mass)
    add_link(robot, 'body', body_props, write_hull('body', body_mesh))
    visual_link(robot, 'body', 'body_shell', config['body_color'], 'meshes/spyder/body.stl')

    tibia_points = np.vstack([leg_meshes[g] for g in groups if config['groups'][g]['link'] == 'tibia']).reshape(-1, 3)
    tip_low = tibia_points[tibia_points[:, 2] < tibia_points[:, 2].min() + 3e-4]
    foot = np.array([(tip_low[:, 0].min() + tip_low[:, 0].max()) / 2,
                     (tip_low[:, 1].min() + tip_low[:, 1].max()) / 2, tibia_points[:, 2].min()])

    hulls = {}
    for link in LEG_LINKS:
        hulls[link] = write_hull(link, np.vstack([leg_meshes[g] for g in groups if config['groups'][g]['link'] == link]))
    link_props = {link: combine_inertia(leg_mass[link]) for link in LEG_LINKS}
    origins = {'femur': hip_origin / 1000, 'tibia': knee_origin / 1000}
    servo = config['servo']
    limits = config['joint_limits_deg']
    for index, leg in enumerate(config['legs']):
        prefix = leg['name']
        frame = frames[index]
        names = {link: f'{prefix}_{link}' for link in LEG_LINKS}
        for link in LEG_LINKS:
            add_link(robot, names[link], link_props[link], hulls[link])
        for group, settings in config['groups'].items():
            visual_link(robot, names[settings['link']], f'{prefix}_{group}', settings['color'],
                        f'meshes/spyder/{group}.stl')
        mount = (frame['origin'] - centre) / 1000
        add_joint(robot, f'{prefix}_mount', 'fixed', 'body', names['base'], mount,
                  rpy=(0, 0, np.radians(leg['heading_deg'])))
        add_joint(robot, f'{prefix}_coxa_yaw', 'revolute', names['base'], names['coxa'], np.zeros(3),
                  axis=(0, 0, 1), limits=limits['coxa_yaw'], servo=servo)
        add_joint(robot, f'{prefix}_femur_pitch', 'revolute', names['coxa'], names['femur'], origins['femur'],
                  axis=(0, 1, 0), limits=limits['femur_pitch'], servo=servo)
        add_joint(robot, f'{prefix}_tibia_pitch', 'revolute', names['femur'], names['tibia'], origins['tibia'],
                  axis=(0, 1, 0), limits=limits['tibia_pitch'], servo=servo)
        tip = ET.SubElement(robot, 'link', name=f'{prefix}_foot_tip')
        # A small sphere is the ground contact: one stable point per foot.
        radius = config['foot_contact_radius_mm'] / 1000
        inertial = ET.SubElement(tip, 'inertial')
        ET.SubElement(inertial, 'mass', value='0.001')
        ET.SubElement(inertial, 'inertia', ixx='1e-7', iyy='1e-7', izz='1e-7', ixy='0', ixz='0', iyz='0')
        contact = ET.SubElement(tip, 'collision', name=f'{prefix}_foot_contact')
        ET.SubElement(contact, 'origin', xyz=numbers((0, 0, radius)), rpy='0 0 0')
        ET.SubElement(ET.SubElement(contact, 'geometry'), 'sphere', radius=str(radius))
        add_joint(robot, f'{prefix}_foot_tip_fixed', 'fixed', names['tibia'], f'{prefix}_foot_tip', foot)

    ET.indent(robot, space='  ')
    ET.ElementTree(robot).write(HERE / 'spyder.urdf', encoding='utf-8', xml_declaration=True)

    total = body_props[0] + len(frames) * sum(link_props[link][0] for link in LEG_LINKS)
    report = {
        'units': 'metres, kilograms',
        'source': config['source'],
        'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'source_triangles': len(triangles),
        'source_shells': len(shells),
        'body_shells': len(body_shells),
        'leg_congruence_mm': report_congruence,
        'lengths_mm': {'yaw_to_hip': float(hip[0]), 'hip_to_knee': float(knee[0] - hip[0]),
                       'knee_above_hip': float(knee[2] - hip[2])},
        'foot_tip_in_tibia_mm': (foot * 1000).round(4).tolist(),
        'mass_kg': {'total': round(total, 4), 'body': round(body_props[0], 4),
                    **{link: round(link_props[link][0], 4) for link in LEG_LINKS}},
        'legs': [{'name': leg['name'], 'label': leg['label'], 'heading_deg': leg['heading_deg'],
                  'mount_in_body_mm': ((frame['origin'] - centre)).round(4).tolist()}
                 for leg, frame in zip(config['legs'], frames)],
    }
    (HERE / 'spyder_report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(f"Built spyder.urdf: {len(shells)} shells into {len(frames) * 4 + 1} links, "
          f"{len(frames) * 3} revolute joints, estimated mass {total:.3f} kg.")
    return report


if __name__ == '__main__':
    build()
