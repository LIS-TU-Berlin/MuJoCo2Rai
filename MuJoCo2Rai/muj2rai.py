import os
import rowan
import numpy as np
import xml.etree.ElementTree as ET
from utils import pose_matrix_to_7d, remove_empty_subdirectories
import trimesh
from tqdm import tqdm
from PIL import Image
from mesh_helper import *


muj2rai_joint_map = {
    "1 0 0": "hingeX",
    "0 1 0": "hingeY",
    "0 0 1": "hingeZ",
}


def invert_floats(input_string):
    numbers = input_string.split()
    inverted_numbers = [str(-1 * float(num)) for num in numbers]
    output_string = " ".join(inverted_numbers)
    return output_string


def load_model_dicts(root, out_path: str, root_path: str, materials: dict[str]) -> dict:
    models = {}
    meshes = root.findall(".//mesh")
    for mesh in meshes:
        name = mesh.attrib.get("name", "")
        path = mesh.attrib.get("file", "")
        material_name = mesh.attrib.get("material", "")
        path = os.path.join(root_path, path)
        texture_path = materials[material_name]

        ply_path = f"{name}.ply"
        ply_success = obj2ply(path, os.path.join(out_path, ply_path), texture_path=texture_path)

        if ply_success:
            models[name] = ply_path
        else:
            print(f"Error when tranforming {path} to ply")
            return None
        
    return models


def obj2ply(obj_file: str, ply_out: str, scale: float=1.0, texture_path: str="none", toH5: bool = True) -> bool:

    tri_obj = trimesh.load(obj_file)
    if hasattr(tri_obj.visual, 'to_color'):

        if texture_path == "none":
            vertex_colors_visual = tri_obj.visual.to_color()
            tri_obj.visual = vertex_colors_visual
        elif texture_path:
            try:
                texture_image = Image.open(texture_path)
                
                texture = trimesh.visual.TextureVisuals(image=texture_image, uv=tri_obj.visual.uv)
                tri_obj.visual = texture
    
                
                texture = Image.open(texture_path)
                texture = np.array(texture).astype(float) / 255.0
                
                if texture.shape[-1] == 3:
                    alpha = np.ones((texture.shape[0], texture.shape[1], 1))
                    texture = np.concatenate([texture, alpha], axis=-1)
                
                if not hasattr(tri_obj.visual, 'uv'):
                    raise ValueError("Mesh does not have UV coordinates!")
                
                uv_coords = tri_obj.visual.uv
                
                uv_coords_copy = uv_coords.copy()
                uv_coords_copy[:, 1] = 1 - uv_coords_copy[:, 1]
                
                pixel_coords = np.zeros_like(uv_coords_copy)
                pixel_coords[:, 0] = uv_coords_copy[:, 0] * (texture.shape[1] - 1)
                pixel_coords[:, 1] = uv_coords_copy[:, 1] * (texture.shape[0] - 1)
                pixel_coords = pixel_coords.astype(int)
                
                vertex_colors = texture[pixel_coords[:, 1], pixel_coords[:, 0]]
                
                tri_obj.visual = trimesh.visual.ColorVisuals(
                    mesh=tri_obj,
                    vertex_colors=vertex_colors
                )
                
                
            except:
                print(texture_path, "is not a path to a texture or could not been applied vertex wise to the mesh.")
        if scale != 1.0:
            print(scale)
            scaling_mat = scale * np.eye(4)
            scaling_mat[-1, -1] = 1.0
            tri_obj.apply_transform(scaling_mat)

        tri_obj.export(ply_out)
        print(f"Converted {obj_file}")
        
        if toH5:
            M = MeshHelper(ply_out)
            transform_mat = M.transformInertia()
            M.createPoints()
            M.createDecomposition()
            M.export_h5(True)
            return True, M.mesh.center_mass.tolist(), np.diagonal(M.mesh.moment_inertia).tolist(), transform_mat
        
        return True

    else:
        print(f"Failed on {obj_file}")
        if toH5:
            return False, False, False, False
        return False

def get_h5_props(name: str, obj_path: str, texture_path: str, root_path: str, out_path: str, scaling: float=1.0) -> str:

    obj_path = os.path.join(root_path, obj_path)
    texture_path = os.path.join(root_path, texture_path)
    ply_path = f"{name}.ply"

    ply_success, mass, inertia, transform_mat = obj2ply(obj_path, os.path.join(out_path, ply_path), texture_path=texture_path, scale=scaling)
    if ply_success:
        return ply_path[:-3]+"h5", mass, inertia , transform_mat
    return None, None, None, None


def find_parent_body(child_name, root_tree):
    for parent_body in root_tree.findall(".//body"):
        child_body = parent_body.find(f"./body[@name='{child_name}']")
        if child_body is not None:
            parent_body = parent_body.attrib.get("name", "")
            if parent_body:
                return parent_body
            return None

    return None


def relative_rotation(vec1, vec2):
    # Normalize the input vectors
    v1 = vec1 / np.linalg.norm(vec1)
    v2 = vec2 / np.linalg.norm(vec2)
    
    # Compute the cross product and the dot product
    cross = np.cross(v1, v2)
    dot = np.dot(v1, v2)
    
    # Handle edge cases
    if np.allclose(v1, v2):
        # Vectors are aligned; no rotation needed
        return np.array([1, 0, 0, 0])  # Identity quaternion
    elif np.allclose(v1, -v2):
        # Vectors are opposite; 180-degree rotation
        # Choose an arbitrary orthogonal vector for the axis
        axis = np.array([1, 0, 0]) if not np.allclose(v1, [1, 0, 0]) else np.array([0, 1, 0])
        return rowan.from_axis_angle(axis, np.pi)
    
    # Compute the quaternion
    q = np.array([1 + dot, *cross])
    return rowan.normalize(q)


def get_models_mats(root_tree, root_path: str):
    materials = {}
    textures = {}

    texs = root_tree.findall(".//texture")
    for tex in texs:
        tex_name = tex.attrib.get("name", "")
        file_path = tex.attrib.get("file", "")
        textures[tex_name] = file_path

    maters = root_tree.findall(".//material")
    for mater in maters:
        mater_name = mater.attrib.get("name", "")
        tex_name = mater.attrib.get("texture", "")
        if tex_name:
            materials[mater_name] = textures[tex_name]
        else:
            materials[mater_name] = ""

    models = {}
    meshes = root_tree.findall(".//mesh")
    for mesh in meshes:
        name = mesh.attrib.get("name", "")
        path = mesh.attrib.get("file", "")
        models[name] = path

    return models, materials


def convert_obj_to_ply(root_path="aigen_objs", output_base="./rai_plys"):
    """ Converts all .obj files (excluding 'collision' ones) in the directory to .ply and .h5 format. """
    
    for root, dirs, files in tqdm(os.walk(root_path)):
        for file in files:
            if file.endswith('.obj') and "collision" not in file:
                ply_dir = os.path.join(f"{output_base}/{root}".replace("/visual", ""))
                os.makedirs(ply_dir, exist_ok=True)

                ply_path = os.path.join(ply_dir, file.replace(".obj", ".ply"))
                obj_path = os.path.join(root, file)

                try:
                    obj2ply(obj_path, ply_path, scale=0.2)
                except Exception as e:
                    print(f"Failed to convert {obj_path}: {e}")


def process_model_xml(root_path="fixtures", output_base="./rai_jointed"):
    """
    Processes all mujoco 'model.xml' files in the given root directory,
    extracts joint and geometry data, and generates corresponding RaiSim scene descriptions.

    Args:
        root_path (str): The root directory to search for model.xml files.
        output_base (str): The base directory where processed files will be stored.

    Returns:
        None
    """
    total = 0
    successful = 0

    for root, dirs, files in os.walk(root_path):
        for file in files:
            if file == "model.xml":
                total += 1
                file_path = os.path.join(root, file)
                tree = ET.parse(file_path)
                root_tree = tree.getroot()

                out_path = os.path.join(output_base, root)
                os.makedirs(out_path, exist_ok=True)

                # Get texture and model paths
                models, materials = get_models_mats(root_tree, root_path)
                if models is None:
                    continue

                lines = []
                for body in root_tree.findall(".//body"):
                    body_name = body.attrib.get("name", "")
                    if not body_name:
                        continue

                    joints = body.findall("./joint")
                    parent_body = find_parent_body(body_name, root_tree)
                    
                    if joints:
                        joint_type = joints[0].attrib.get("axis", "")
                        joint_pos = joints[0].attrib.get("pos", "")
                        joint_limits = joints[0].attrib.get("range", "").replace(" ", ", ")
                        pre = f"{body_name}_joint_pre"
                        lines.append(f"""{pre} ({parent_body}) {{Q: "t({joint_pos})"}}\n""")
                        
                        splited_joint_params = joint_type.split()
                        if len(splited_joint_params) == 3:
                            if joint_type in muj2rai_joint_map:
                                rai_joint = muj2rai_joint_map[joint_type]
                            else:
                                vec1 = np.array([0., 0., 1.])
                                vec2 = np.array(list(map(float, splited_joint_params)))
                                quat = relative_rotation(vec1, vec2)
                                new_pre = f"{pre}_special_joint"
                                lines.append(f"""{new_pre} ({pre}) {{quaternion: [{quat[0]}, {quat[1]}, {quat[2]}, {quat[3]}]}}\n""")
                                pre = new_pre
                                rai_joint = "hingeZ"
                        else:
                            rai_joint = "hingeX"
                            joint_limits = "-1, 1"

                        if not joint_limits:
                            joint_limits = "-1, 1"

                        lines.append(f"""{body_name}_joint ({pre}) {{joint: {rai_joint}, ctrl_limits: [{joint_limits}, 12]}}\n""")
                        lines.append(f"""{body_name} ({body_name}_joint) {{Q: "t({invert_floats(joint_pos)})"}}\n""")

                    elif parent_body:
                        lines.append(f"""{body_name} ({parent_body}) {{}}\n""")
                    else:
                        lines.append(f"""{body_name} {{}}\n""")

                    for i, geom in enumerate(body.findall("./geom")):
                        if geom.attrib.get("type", "") == "mesh":
                            mesh_name = geom.attrib.get("mesh", "")
                            material_name = geom.attrib.get("material", "")

                            h5_path, mass, inertia, transform_mat = get_h5_props(
                                f"{body_name}_geom_{i}",
                                models[mesh_name],
                                materials[material_name],
                                root,
                                out_path
                            )
                            if transform_mat is not None:
                                pose_vec = pose_matrix_to_7d(transform_mat)

                            lines.append(f"""{body_name}_cvx{i} ({body_name}) {{mesh_decomp: <{h5_path}>, mass: {mass}, inertia: {inertia}, Q:{pose_vec.tolist()}, color:[0, 0, 0, 0], simulate}}\n""")
                            lines.append(f"""{body_name}_visual{i} ({body_name}_cvx{i}) {{shape: mesh, mesh: <{h5_path}>, contact: 0, simulate: False}}\n""")

                # Save to file
                g_path = os.path.join(out_path, "joint_scene.g")
                with open(g_path, "w") as file:
                    file.writelines(lines)

                print(f"Successfully created: {out_path}")
                successful += 1

    remove_empty_subdirectories(output_base)
    print(f"Processed {total} files, successfully converted {successful}.")

# Run when executed as a script
if __name__ == "__main__":
    process_model_xml()
