"""Procedurally build a classroom scene (and a small car) for ROS / Gazebo.

Usage (headless):
    blender --background --python blender/build_classroom_scene.py

The script is the single source of truth for the scene: it builds the
geometry in Blender, exports Wavefront OBJ meshes into the ``classroom_sim``
ROS package and writes the matching Gazebo ``model.sdf``/``model.config``
files.  Mesh files use the same layout constants as the collision boxes, so
the 2D lidar in Gazebo "sees" the furniture that is actually rendered.

Blender coordinates: +X points to the front wall (door side), +Y to the
window wall, +Z is up.  They are exported unchanged so that Gazebo (also
Z-up) can consume the meshes directly.
"""

import math
import os

import bpy
from mathutils import Vector


# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
PKG_ROOT = os.path.join(REPO_ROOT, "src", "classroom_sim")

CLASSROOM_MESH_DIR = os.path.join(PKG_ROOT, "models", "classroom", "meshes")
CLASSROOM_MODEL_DIR = os.path.join(PKG_ROOT, "models", "classroom")
CAR_MODEL_DIR = os.path.join(PKG_ROOT, "models", "classroom_car")
CAR_MESH_DIR = os.path.join(CAR_MODEL_DIR, "meshes")


# ---------------------------------------------------------------------------
# Layout constants (metres)
# ---------------------------------------------------------------------------
ROOM_L = 8.0            # size along X
ROOM_W = 6.0            # size along Y
ROOM_H = 3.0            # size along Z
WALL_T = 0.15

DESK_ROWS_X = (-1.0, 0.6, 2.2)
DESK_COLS_Y = (-1.4, 1.4)
DESK_SIZE = (0.60, 1.40, 0.75)      # X, Y, top height
DESK_TOP_T = 0.05
CHAIR_OFFSET_X = 0.55
CHAIR_OFFSET_Y = 0.35
CHAIR_SEAT_Z = 0.45

BLACKBOARD_X = -ROOM_L / 2.0
WINDOW_XS = (-2.0, 0.0, 2.0)
DOOR_Y = -2.4

CAR_SPAWN = (2.9, 2.55, 0.09)
CAR_YAW = math.pi


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------
groups = {}


def _link(obj, group, material):
    if material is not None:
        obj.data.materials.append(material)
    groups.setdefault(group, []).append(obj)
    return obj


def make_material(name, color, roughness=0.75, metallic=0.0, alpha=1.0):
    mat = bpy.data.materials.get(name)
    if mat is None:
        mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = next(n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    bsdf.inputs["Base Color"].default_value = (*color, alpha)
    bsdf.inputs["Roughness"].default_value = roughness
    bsdf.inputs["Metallic"].default_value = metallic
    if alpha < 1.0:
        try:
            bsdf.inputs["Alpha"].default_value = alpha
            mat.blend_method = "BLEND"
        except Exception:
            pass
    mat.diffuse_color = (*color, alpha)
    return mat


def add_box(name, size, loc, material, group, bevel=0.0):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=loc)
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = size
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
    if bevel > 0.0:
        mod = obj.modifiers.new("Bevel", "BEVEL")
        mod.width = bevel
        mod.segments = 2
        mod.limit_method = "ANGLE"
    return _link(obj, group, material)


def add_cylinder(name, radius, depth, loc, material, group, axis="Z"):
    if axis == "Y":
        rot = (math.radians(90.0), 0.0, 0.0)
    elif axis == "X":
        rot = (0.0, math.radians(90.0), 0.0)
    else:
        rot = (0.0, 0.0, 0.0)
    bpy.ops.mesh.primitive_cylinder_add(
        radius=radius, depth=depth, location=loc, rotation=rot, vertices=32)
    obj = bpy.context.active_object
    obj.name = name
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
    bpy.ops.object.shade_smooth()
    return _link(obj, group, material)


def export_group_stl(group, filename):
    """Export a group as a binary STL (RViz's mesh loader always handles STL)."""
    objects = [o for o in groups.get(group, []) if o.name in bpy.data.objects]
    if not objects:
        return
    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]
    bpy.ops.wm.stl_export(
        filepath=filename,
        export_selected_objects=True,
        apply_modifiers=True,
        forward_axis="Y",
        up_axis="Z",
        ascii_format=False,
    )


def export_group(group, filename):
    objects = [o for o in groups.get(group, []) if o.name in bpy.data.objects]
    if not objects:
        return
    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]
    bpy.ops.wm.obj_export(
        filepath=filename,
        export_selected_objects=True,
        forward_axis="Y",
        up_axis="Z",
        apply_modifiers=True,
        apply_transform=True,
        export_materials=True,
        export_triangulated_mesh=True,
        export_normals=True,
    )


# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
def build_materials():
    return {
        "floor": make_material("floor", (0.62, 0.56, 0.46), roughness=0.9),
        "wall": make_material("wall", (0.86, 0.86, 0.82), roughness=0.9),
        "ceiling": make_material("ceiling", (0.94, 0.94, 0.92), roughness=0.95),
        "blackboard": make_material(
            "blackboard", (0.05, 0.16, 0.11), roughness=0.55),
        "wood_trim": make_material(
            "wood_trim", (0.42, 0.26, 0.14), roughness=0.7),
        "door": make_material("door", (0.52, 0.34, 0.19), roughness=0.75),
        "window_frame": make_material(
            "window_frame", (0.90, 0.92, 0.94), roughness=0.5),
        "glass": make_material(
            "glass", (0.55, 0.76, 0.92), roughness=0.1, alpha=0.55),
        "light": make_material("light", (1.0, 0.98, 0.86), roughness=0.4),
        "desk_top": make_material(
            "desk_top", (0.72, 0.50, 0.29), roughness=0.6),
        "metal": make_material(
            "metal", (0.46, 0.49, 0.52), roughness=0.35, metallic=0.8),
        "chair": make_material("chair", (0.16, 0.42, 0.68), roughness=0.7),
        "cabinet": make_material(
            "cabinet", (0.53, 0.38, 0.23), roughness=0.7),
        "car_body": make_material(
            "car_body", (0.10, 0.35, 0.85), roughness=0.35, metallic=0.2),
        "car_glass": make_material(
            "car_glass", (0.05, 0.16, 0.24), roughness=0.1),
        "car_light": make_material(
            "car_light", (1.0, 0.86, 0.45), roughness=0.3),
        "tire": make_material("tire", (0.05, 0.05, 0.05), roughness=0.9),
        "hub": make_material(
            "hub", (0.70, 0.70, 0.72), roughness=0.4, metallic=0.9),
    }


# ---------------------------------------------------------------------------
# Classroom geometry
# ---------------------------------------------------------------------------
collisions = []


def add_collision(name, size, center, yaw=0.0):
    collisions.append((name, size, center, yaw))


def build_room(mats):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=(0, 0, -0.05))
    floor = bpy.context.active_object
    floor.name = "floor"
    floor.scale = (ROOM_L, ROOM_W, 0.10)
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
    _link(floor, "floor", mats["floor"])
    add_collision("floor_collision", (ROOM_L, ROOM_W, 0.10), (0, 0, -0.05))

    add_box("wall_back", (WALL_T, ROOM_W, ROOM_H),
            (BLACKBOARD_X - WALL_T / 2.0, 0, ROOM_H / 2.0),
            mats["wall"], "walls")
    add_box("wall_front", (WALL_T, ROOM_W, ROOM_H),
            (ROOM_L / 2.0 + WALL_T / 2.0, 0, ROOM_H / 2.0),
            mats["wall"], "walls")
    add_box("wall_left", (ROOM_L, WALL_T, ROOM_H),
            (0, -ROOM_W / 2.0 - WALL_T / 2.0, ROOM_H / 2.0),
            mats["wall"], "walls")
    add_box("wall_right", (ROOM_L, WALL_T, ROOM_H),
            (0, ROOM_W / 2.0 + WALL_T / 2.0, ROOM_H / 2.0),
            mats["wall"], "walls")

    add_collision("wall_back_collision", (WALL_T, ROOM_W, ROOM_H),
                  (BLACKBOARD_X - WALL_T / 2.0, 0, ROOM_H / 2.0))
    add_collision("wall_front_collision", (WALL_T, ROOM_W, ROOM_H),
                  (ROOM_L / 2.0 + WALL_T / 2.0, 0, ROOM_H / 2.0))
    add_collision("wall_left_collision", (ROOM_L, WALL_T, ROOM_H),
                  (0, -ROOM_W / 2.0 - WALL_T / 2.0, ROOM_H / 2.0))
    add_collision("wall_right_collision", (ROOM_L, WALL_T, ROOM_H),
                  (0, ROOM_W / 2.0 + WALL_T / 2.0, ROOM_H / 2.0))

    add_box("ceiling", (ROOM_L, ROOM_W, WALL_T),
            (0, 0, ROOM_H + WALL_T / 2.0), mats["ceiling"], "ceiling")
    add_collision("ceiling_collision", (ROOM_L, ROOM_W, WALL_T),
                  (0, 0, ROOM_H + WALL_T / 2.0))


def build_blackboard(mats):
    board_x = BLACKBOARD_X + 0.02
    add_box("blackboard", (0.04, 4.0, 1.2),
            (board_x, 0.0, 1.6), mats["blackboard"], "blackboard", bevel=0.01)
    add_collision("blackboard_collision", (0.06, 4.0, 1.2),
                  (board_x, 0.0, 1.6))

    frame_x = board_x + 0.03
    add_box("board_frame_top", (0.06, 4.16, 0.08),
            (frame_x, 0.0, 2.24), mats["wood_trim"], "wood_trim")
    add_box("board_frame_bottom", (0.06, 4.16, 0.08),
            (frame_x, 0.0, 0.96), mats["wood_trim"], "wood_trim")
    add_box("board_frame_left", (0.06, 0.08, 1.28),
            (frame_x, -2.08, 1.6), mats["wood_trim"], "wood_trim")
    add_box("board_frame_right", (0.06, 0.08, 1.28),
            (frame_x, 2.08, 1.6), mats["wood_trim"], "wood_trim")


def build_windows(mats):
    for index, x in enumerate(WINDOW_XS):
        add_box("window_glass_%d" % index, (1.40, 0.03, 1.20),
                (x, ROOM_W / 2.0 - 0.02, 1.70), mats["glass"], "glass")
        add_box("window_frame_top_%d" % index, (1.56, 0.08, 0.08),
                (x, ROOM_W / 2.0 - 0.05, 2.34), mats["window_frame"],
                "window_frame")
        add_box("window_frame_bottom_%d" % index, (1.56, 0.10, 0.08),
                (x, ROOM_W / 2.0 - 0.05, 1.06), mats["window_frame"],
                "window_frame")
        add_box("window_frame_left_%d" % index, (0.08, 0.08, 1.36),
                (x - 0.74, ROOM_W / 2.0 - 0.05, 1.70), mats["window_frame"],
                "window_frame")
        add_box("window_frame_right_%d" % index, (0.08, 0.08, 1.36),
                (x + 0.74, ROOM_W / 2.0 - 0.05, 1.70), mats["window_frame"],
                "window_frame")


def build_door(mats):
    door_x = ROOM_L / 2.0 - 0.04
    add_box("door", (0.05, 1.0, 2.10),
            (door_x, DOOR_Y, 1.05), mats["door"], "door", bevel=0.01)
    add_box("door_frame_left", (0.08, 0.08, 2.20),
            (door_x, DOOR_Y - 0.54, 1.10), mats["wood_trim"], "wood_trim")
    add_box("door_frame_right", (0.08, 0.08, 2.20),
            (door_x, DOOR_Y + 0.54, 1.10), mats["wood_trim"], "wood_trim")
    add_box("door_frame_top", (0.08, 1.16, 0.08),
            (door_x, DOOR_Y, 2.20), mats["wood_trim"], "wood_trim")


def build_ceiling_lights(mats):
    for ix, x in enumerate((-1.8, 1.8)):
        for iy, y in enumerate((-1.6, 1.6)):
            add_box("ceiling_light_%d_%d" % (ix, iy), (1.20, 0.40, 0.05),
                    (x, y, ROOM_H - 0.04), mats["light"], "lights")


def build_chair(cx, cy, mats, tag):
    add_box("chair_seat_%s" % tag, (0.38, 0.38, 0.04),
            (cx, cy, CHAIR_SEAT_Z), mats["chair"], "chair", bevel=0.01)
    add_box("chair_back_%s" % tag, (0.04, 0.38, 0.45),
            (cx + 0.19, cy, CHAIR_SEAT_Z + 0.225), mats["chair"], "chair",
            bevel=0.01)
    for dx in (-0.15, 0.15):
        for dy in (-0.15, 0.15):
            add_box("chair_leg_%s" % tag, (0.03, 0.03, CHAIR_SEAT_Z),
                    (cx + dx, cy + dy, CHAIR_SEAT_Z / 2.0),
                    mats["metal"], "metal")
    add_collision("chair_collision_%s" % tag, (0.42, 0.42, 0.90),
                  (cx + 0.02, cy, CHAIR_SEAT_Z))


def build_student_desks(mats):
    top_z = DESK_SIZE[2] - DESK_TOP_T / 2.0
    for ix, x in enumerate(DESK_ROWS_X):
        for iy, y in enumerate(DESK_COLS_Y):
            tag = "%d_%d" % (ix, iy)
            add_box("desk_top_%s" % tag, (DESK_SIZE[0], DESK_SIZE[1],
                    DESK_TOP_T), (x, y, top_z), mats["desk_top"],
                    "desk_top", bevel=0.01)
            for dx in (-0.25, 0.25):
                for dy in (-0.60, 0.60):
                    add_box("desk_leg_%s" % tag, (0.05, 0.05, 0.70),
                            (x + dx, y + dy, 0.35), mats["metal"], "metal")
            add_collision("desk_collision_%s" % tag,
                          (DESK_SIZE[0] + 0.02, DESK_SIZE[1] + 0.04, 0.76),
                          (x, y, 0.38))
            for jy, sign in enumerate((-1, 1)):
                build_chair(x + CHAIR_OFFSET_X,
                            y + sign * CHAIR_OFFSET_Y,
                            mats, "%s_%d" % (tag, jy))


def build_teacher_desk(mats):
    tx, ty = -2.9, -1.6
    add_box("teacher_desk_top", (0.80, 1.60, 0.06),
            (tx, ty, 0.72), mats["desk_top"], "desk_top", bevel=0.01)
    for dx in (-0.35, 0.35):
        for dy in (-0.70, 0.70):
            add_box("teacher_desk_leg", (0.05, 0.05, 0.70),
                    (tx + dx, ty + dy, 0.35), mats["metal"], "metal")
    add_collision("teacher_desk_collision", (0.85, 1.65, 0.80),
                  (tx, ty, 0.40))


def build_cabinet(mats):
    cx, cy = -3.70, 2.40
    add_box("cabinet", (0.45, 1.00, 1.00),
            (cx, cy, 0.50), mats["cabinet"], "cabinet", bevel=0.02)
    add_collision("cabinet_collision", (0.47, 1.02, 1.00), (cx, cy, 0.50))


# ---------------------------------------------------------------------------
# Car geometry (local frame = base_link, origin at chassis centre)
# ---------------------------------------------------------------------------
CAR_WHEEL_RADIUS = 0.06
CAR_WHEEL_WIDTH = 0.04
CAR_AXLE_X = 0.13
CAR_TRACK_Y = 0.17
CAR_WHEEL_Z = -0.03

car_preview_objects = []


def build_car(mats):
    body = []
    body.append(add_box("car_chassis", (0.42, 0.30, 0.10),
                        (0.0, 0.0, 0.0), mats["car_body"], "car_body",
                        bevel=0.02))
    body.append(add_box("car_cabin", (0.22, 0.24, 0.10),
                        (-0.03, 0.0, 0.10), mats["car_body"], "car_body",
                        bevel=0.03))
    body.append(add_box("car_bumper_front", (0.04, 0.30, 0.06),
                        (0.21, 0.0, -0.01), mats["car_body"], "car_body"))
    body.append(add_box("car_bumper_rear", (0.04, 0.30, 0.06),
                        (-0.21, 0.0, -0.01), mats["car_body"], "car_body"))

    add_box("car_windshield", (0.03, 0.20, 0.08),
            (0.085, 0.0, 0.10), mats["car_glass"], "car_glass",
            bevel=0.01)
    add_box("car_rear_window", (0.03, 0.20, 0.08),
            (-0.145, 0.0, 0.10), mats["car_glass"],
            "car_glass", bevel=0.01)
    for side in (-1, 1):
        add_box("car_side_window_%d" % side, (0.16, 0.02, 0.07),
                (-0.03, side * 0.12, 0.10), mats["car_glass"],
                "car_glass", bevel=0.005)

    lights = []
    for side in (-1, 1):
        lights.append(add_cylinder("car_headlight_%d" % side, 0.025, 0.02,
                                   (0.215, side * 0.09, 0.02),
                                   mats["car_light"], "car_light", axis="X"))
    lights.append(add_box("car_taillight", (0.02, 0.24, 0.03),
                          (-0.215, 0.0, 0.02), mats["car_light"],
                          "car_light"))

    car_preview_objects.extend(body)
    car_preview_objects.extend(lights)
    car_preview_objects.extend(groups["car_glass"])

    # One wheel centred at the link origin, exported for the URDF.  It is
    # deleted afterwards so the preview shows four wheels instead of five.
    wheel = add_cylinder("car_wheel_export", CAR_WHEEL_RADIUS,
                         CAR_WHEEL_WIDTH, (0.0, 0.0, 0.0),
                         mats["tire"], "wheel", axis="Y")
    hub = add_cylinder("car_hub_export", 0.028, CAR_WHEEL_WIDTH + 0.005,
                       (0.0, 0.0, 0.0), mats["hub"], "wheel", axis="Y")
    export_group("wheel", os.path.join(CAR_MESH_DIR, "wheel.obj"))
    export_group_stl("wheel", os.path.join(CAR_MESH_DIR, "wheel.stl"))
    for obj in (wheel, hub):
        bpy.data.objects.remove(obj, do_unlink=True)
    groups["wheel"] = []

    export_group("car_body", os.path.join(CAR_MESH_DIR, "car_body.obj"))
    export_group("car_glass", os.path.join(CAR_MESH_DIR, "car_glass.obj"))
    export_group("car_light", os.path.join(CAR_MESH_DIR, "car_lights.obj"))
    export_group_stl("car_body", os.path.join(CAR_MESH_DIR, "car_body.stl"))
    export_group_stl("car_glass", os.path.join(CAR_MESH_DIR, "car_glass.stl"))
    export_group_stl("car_light", os.path.join(CAR_MESH_DIR, "car_lights.stl"))

    # Four wheels for the .blend preview only (never exported).
    for axle in (-1, 1):
        for side in (-1, 1):
            loc = (axle * CAR_AXLE_X, side * CAR_TRACK_Y, CAR_WHEEL_Z)
            car_preview_objects.append(add_cylinder(
                "wheel_preview_%d_%d" % (axle, side), CAR_WHEEL_RADIUS,
                CAR_WHEEL_WIDTH, loc, mats["tire"], "wheel_preview",
                axis="Y"))
            car_preview_objects.append(add_cylinder(
                "hub_preview_%d_%d" % (axle, side), 0.028,
                CAR_WHEEL_WIDTH + 0.005, loc, mats["hub"], "wheel_preview",
                axis="Y"))


def park_car_preview():
    """Move the car out of the way of the exported mesh files for preview."""
    yaw = CAR_YAW
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    for obj in car_preview_objects:
        if obj.name not in bpy.data.objects:
            continue
        x, y, z = obj.location
        obj.location = (
            CAR_SPAWN[0] + x * cos_y - y * sin_y,
            CAR_SPAWN[1] + x * sin_y + y * cos_y,
            z + CAR_SPAWN[2],
        )
        obj.rotation_euler[2] += yaw


def setup_preview_scene():
    """Add a camera and lights so the saved .blend is ready to inspect."""
    scene = bpy.context.scene

    cam_data = bpy.data.cameras.new("PreviewCamera")
    cam_data.lens = 18.0
    cam = bpy.data.objects.new("PreviewCamera", cam_data)
    bpy.context.collection.objects.link(cam)
    cam.location = Vector((3.4, -2.5, 2.1))
    target = Vector((-1.2, 0.3, 0.7))
    cam.rotation_euler = (target - cam.location).to_track_quat(
        "-Z", "Y").to_euler()
    scene.camera = cam

    sun_data = bpy.data.lights.new("PreviewSun", "SUN")
    sun_data.energy = 3.0
    sun = bpy.data.objects.new("PreviewSun", sun_data)
    bpy.context.collection.objects.link(sun)
    sun.rotation_euler = (math.radians(35), math.radians(15),
                          math.radians(20))

    fill_data = bpy.data.lights.new("PreviewFill", "AREA")
    fill_data.energy = 400.0
    fill_data.size = 4.0
    fill = bpy.data.objects.new("PreviewFill", fill_data)
    bpy.context.collection.objects.link(fill)
    fill.location = (0.5, 0.0, 2.7)

    world = bpy.data.worlds.get("PreviewWorld")
    if world is None:
        world = bpy.data.worlds.new("PreviewWorld")
    world.use_nodes = True
    background = next(
        (n for n in world.node_tree.nodes if n.type == "BACKGROUND"), None)
    if background is not None:
        background.inputs[0].default_value = (0.75, 0.78, 0.85, 1.0)
        background.inputs[1].default_value = 0.6
    scene.world = world


# ---------------------------------------------------------------------------
# SDF writers
# ---------------------------------------------------------------------------
def _rgb(color, factor=1.0):
    return " ".join("%.3f" % min(1.0, c * factor) for c in color)


def _vec(values):
    return " ".join("%.4f" % value for value in values)


VISUALS = [
    ("floor", "floor.obj", (0.62, 0.56, 0.46), None, False),
    ("walls", "walls.obj", (0.86, 0.86, 0.82), None, False),
    ("ceiling", "ceiling.obj", (0.94, 0.94, 0.92), None, False),
    ("blackboard", "blackboard.obj", (0.05, 0.16, 0.11), None, False),
    ("wood_trim", "wood_trim.obj", (0.42, 0.26, 0.14), None, False),
    ("door", "door.obj", (0.52, 0.34, 0.19), None, False),
    ("window_frame", "window_frame.obj", (0.90, 0.92, 0.94), None, False),
    ("glass", "glass.obj", (0.55, 0.76, 0.92), 0.55, False),
    ("lights", "lights.obj", (1.00, 0.98, 0.86), None, True),
    ("desk_top", "desk_top.obj", (0.72, 0.50, 0.29), None, False),
    ("metal", "metal.obj", (0.46, 0.49, 0.52), None, False),
    ("chair", "chair.obj", (0.16, 0.42, 0.68), None, False),
    ("cabinet", "cabinet.obj", (0.53, 0.38, 0.23), None, False),
]


def _visual_xml(name, mesh, color, transparency, emissive):
    lines = [
        '        <visual name="%s">' % name,
        "          <geometry>",
        "            <mesh>",
        "              <uri>model://classroom/meshes/%s</uri>" % mesh,
        "            </mesh>",
        "          </geometry>",
        "          <material>",
        "            <ambient>%s 1</ambient>" % _rgb(color, 0.85),
        "            <diffuse>%s 1</diffuse>" % _rgb(color),
        "            <specular>0.1 0.1 0.1 1</specular>",
    ]
    if emissive:
        lines.append("            <emissive>%s 1</emissive>" % _rgb(color))
    lines.append("          </material>")
    if transparency is not None:
        lines.append("          <transparency>%.2f</transparency>"
                     % transparency)
    lines.append("        </visual>")
    return "\n".join(lines)


def _collision_xml(name, size, center, yaw):
    return "\n".join([
        '        <collision name="%s">' % name,
        "          <pose>%.3f %.3f %.3f 0 0 %.3f</pose>"
        % (center[0], center[1], center[2], yaw),
        "          <geometry>",
        "            <box><size>%s</size></box>" % _vec(size),
        "          </geometry>",
        "        </collision>",
    ])


def write_classroom_model():
    parts = [
        '<?xml version="1.0" ?>',
        '<sdf version="1.8">',
        '  <model name="classroom">',
        '    <static>true</static>',
        '    <link name="classroom_link">',
    ]
    parts.extend(_collision_xml(*c) for c in collisions)
    parts.extend(_visual_xml(*v) for v in VISUALS)
    parts.extend([
        '    </link>',
        '  </model>',
        '</sdf>',
        '',
    ])
    path = os.path.join(CLASSROOM_MODEL_DIR, "model.sdf")
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(parts))

    config = """<?xml version="1.0"?>
<model>
  <name>classroom</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author>
    <name>classroom_sim</name>
    <email>dev@example.com</email>
  </author>
  <description>Blender generated classroom (8 x 6 x 3 m).</description>
</model>
"""
    with open(os.path.join(CLASSROOM_MODEL_DIR, "model.config"), "w",
              encoding="utf-8") as handle:
        handle.write(config)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    for directory in (CLASSROOM_MESH_DIR, CAR_MESH_DIR):
        os.makedirs(directory, exist_ok=True)

    bpy.ops.wm.read_factory_settings(use_empty=True)
    groups.clear()
    collisions.clear()
    car_preview_objects.clear()

    mats = build_materials()
    build_room(mats)
    build_blackboard(mats)
    build_windows(mats)
    build_door(mats)
    build_ceiling_lights(mats)
    build_student_desks(mats)
    build_teacher_desk(mats)
    build_cabinet(mats)
    build_car(mats)

    for group, filename in (
        ("floor", "floor.obj"),
        ("walls", "walls.obj"),
        ("ceiling", "ceiling.obj"),
        ("blackboard", "blackboard.obj"),
        ("wood_trim", "wood_trim.obj"),
        ("door", "door.obj"),
        ("window_frame", "window_frame.obj"),
        ("glass", "glass.obj"),
        ("lights", "lights.obj"),
        ("desk_top", "desk_top.obj"),
        ("metal", "metal.obj"),
        ("chair", "chair.obj"),
        ("cabinet", "cabinet.obj"),
    ):
        export_group(group, os.path.join(CLASSROOM_MESH_DIR, filename))

    park_car_preview()
    setup_preview_scene()

    write_classroom_model()

    blend_path = os.path.join(SCRIPT_DIR, "classroom_scene.blend")
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=blend_path)

    print("Classroom meshes written to:", CLASSROOM_MESH_DIR)
    print("Car meshes written to:", CAR_MESH_DIR)
    print("Blender file written to:", blend_path)
    print("Collision boxes:", len(collisions))


if __name__ == "__main__":
    main()
