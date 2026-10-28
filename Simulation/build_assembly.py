"""Rebuild the fitted meshes and URDF without changing the supplied STL files.

All fitting coordinates in assembly.json are millimetres. Generated STL vertices
are metres in their owning URDF link frame. Run with the project .venv Python.
"""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent


def numbers(values):
    return ' '.join(f'{float(v):.10g}' for v in values)


def origin(parent, xyz):
    return ET.SubElement(parent, 'origin', xyz=numbers(xyz), rpy='0 0 0')


def visual_link(robot, parent, name, color, filename):
    """One mesh per fixed child preserves per-part colours in Bullet's GUI."""
    link = ET.SubElement(robot, 'link', name=name + '_visual')
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass', value='0')
    ET.SubElement(inertial, 'inertia', ixx='0', iyy='0', izz='0', ixy='0', ixz='0', iyz='0')
    visual = ET.SubElement(link, 'visual', name=name)
    geometry = ET.SubElement(visual, 'geometry')
    ET.SubElement(geometry, 'mesh', filename=filename, scale='1 1 1')
    material = ET.SubElement(visual, 'material', name=name + '_color')
    ET.SubElement(material, 'color', rgba=numbers(color))
    joint = ET.SubElement(robot, 'joint', name=name + '_attachment', type='fixed')
    ET.SubElement(joint, 'parent', link=parent)
    ET.SubElement(joint, 'child', link=name + '_visual')
    origin(joint, [0, 0, 0])


def adapter_mesh(spec):
    """Explicit illustrative support hardware, separate from imported CAD."""
    pieces = []
    for shape in spec['shapes']:
        if shape['type'] == 'annulus':
            mesh = trimesh.creation.annulus(r_min=shape['inner_radius_mm'],
                                           r_max=shape['outer_radius_mm'],
                                           height=shape['height_mm'], sections=64)
        elif shape['type'] == 'box':
            mesh = trimesh.creation.box(extents=shape['size_mm'])
        elif shape['type'] == 'cylinder':
            mesh = trimesh.creation.cylinder(radius=shape['radius_mm'],
                                            height=shape['height_mm'],
                                            sections=shape.get('sections',32))
        else:
            raise ValueError(f"Unsupported adapter shape: {shape['type']}")
        mesh.apply_transform(trimesh.geometry.align_vectors([0,0,1],shape.get('axis',[0,0,1])))
        mesh.apply_translation(shape['center_mm'])
        pieces.append(mesh)
    result = trimesh.util.concatenate(pieces)
    result.apply_scale(.001)
    return result


def build():
    config = json.loads((HERE / 'assembly.json').read_text(encoding='utf-8'))
    output = HERE / 'meshes' / 'assembled'
    output.mkdir(exist_ok=True)
    robot = ET.Element('robot', name='assembled_single_leg')
    links = {}
    report = {'units': 'metres', 'parts': [], 'sources': {}, 'assumptions': config['assumptions']}
    for name, settings in config['links'].items():
        link = ET.SubElement(robot, 'link', name=name)
        links[name] = link
        # Bullet otherwise displays collision boxes when a link has no visual.
        # Real part meshes live on fixed children to preserve GUI colours.
        hidden = ET.SubElement(link, 'visual', name='invisible_frame')
        ET.SubElement(ET.SubElement(hidden, 'geometry'), 'sphere', radius='0.000000001')
        ET.SubElement(ET.SubElement(hidden, 'material', name=name+'_invisible'),
                      'color', rgba='0 0 0 0')
        inertial = ET.SubElement(link, 'inertial')
        origin(inertial, np.array(settings['com_mm']) / 1000)
        mass = settings['mass_kg']
        ET.SubElement(inertial, 'mass', value=str(mass))
        x, y, z = np.array(settings['inertia_box_mm']) / 1000
        ET.SubElement(inertial, 'inertia', ixx=str(mass*(y*y+z*z)/12),
                      iyy=str(mass*(x*x+z*z)/12), izz=str(mass*(x*x+y*y)/12),
                      ixy='0', ixz='0', iyz='0')
        for box in settings.get('collision_boxes', []):
            collision = ET.SubElement(link, 'collision')
            origin(collision, np.array(box['center_mm']) / 1000)
            geometry = ET.SubElement(collision, 'geometry')
            ET.SubElement(geometry, 'box', size=numbers(np.array(box['size_mm']) / 1000))
    for part in config['parts']:
        source = HERE / part['source']
        report['sources'][part['source']] = hashlib.sha256(source.read_bytes()).hexdigest()
        mesh = trimesh.load_mesh(source, process=False)
        # A combined export can contain several distant rigid components. Select
        # triangle centroids in source coordinates, retaining all tiny servo details.
        if 'source_region' in part:
            low, high = np.array(part['source_region'])
            centers = mesh.triangles_center
            selected = np.all((centers >= low) & (centers <= high), axis=1)
            if not selected.any():
                raise ValueError(f"Empty source region for {part['name']}")
            mesh = trimesh.Trimesh(vertices=mesh.vertices.copy(), faces=mesh.faces[selected], process=False)
            mesh.remove_unreferenced_vertices()
        rotation = np.array(part['rotation'], dtype=float)
        scale = np.array(part['scale'], dtype=float)
        translation = np.array(part['translation_mm'], dtype=float)
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7) or np.linalg.det(rotation) < .999:
            raise ValueError(f"Invalid rotation for {part['name']}")
        if not np.all(np.isfinite(scale)) or np.any(scale <= 0):
            raise ValueError(f"Invalid scale for {part['name']}")
        mesh.vertices = ((mesh.vertices * scale) @ rotation.T + translation) / 1000
        mesh.export(output / (part['name'] + '.stl'))
        visual_link(robot, part['link'], part['name'], part['color'],
                    f"meshes/assembled/{part['name']}.stl")
        report['parts'].append({'name': part['name'], 'source': part['source'],
                                'link': part['link'], 'triangles': len(mesh.faces),
                                'bounds_m': mesh.bounds.tolist()})
    report['generated_supports'] = []
    for part in config.get('generated_supports', []):
        mesh = adapter_mesh(part)
        mesh.export(output / (part['name'] + '.stl'))
        visual_link(robot, part['link'], part['name'], part['color'],
                    f"meshes/assembled/{part['name']}.stl")
        report['generated_supports'].append({'name': part['name'], 'link': part['link'],
                                             'bounds_m': mesh.bounds.tolist(), 'note': part['note']})
    for setting in config['joints']:
        joint = ET.SubElement(robot, 'joint', name=setting['name'], type='revolute')
        ET.SubElement(joint, 'parent', link=setting['parent'])
        ET.SubElement(joint, 'child', link=setting['child'])
        origin(joint, np.array(setting['origin_mm']) / 1000)
        ET.SubElement(joint, 'axis', xyz=numbers(setting['axis']))
        lower, upper = np.deg2rad(setting['limits_deg'])
        ET.SubElement(joint, 'limit', lower=str(lower), upper=str(upper), effort='0.5', velocity='2.0')
        ET.SubElement(joint, 'dynamics', damping='0.05', friction='0')
    tip = ET.SubElement(robot, 'link', name='foot_tip')
    inertial = ET.SubElement(tip, 'inertial')
    ET.SubElement(inertial, 'mass', value='0')
    ET.SubElement(inertial, 'inertia', ixx='0', iyy='0', izz='0', ixy='0', ixz='0', iyz='0')
    joint = ET.SubElement(robot, 'joint', name='foot_tip_fixed', type='fixed')
    ET.SubElement(joint, 'parent', link='tibia_link')
    ET.SubElement(joint, 'child', link='foot_tip')
    origin(joint, np.array(config['foot_contact_mm']) / 1000)
    ET.indent(robot, space='  ')
    ET.ElementTree(robot).write(HERE / 'leg_assembled.urdf', encoding='utf-8', xml_declaration=True)
    (HERE / 'assembly_report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(f"Built {len(config['parts'])} fitted parts from {len(report['sources'])} source STLs, "
          f"{len(report['generated_supports'])} illustrative supports; 3 revolute joints.")


if __name__ == '__main__':
    build()
