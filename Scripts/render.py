"""
Author: Qingning Lan
This script is used to render one image of the rover and the SPH fluid using Blender.

Input:
    - The position and rotation of the rover and the fluid at different time.
    - The obj file of the rover and the fluid.
    - The camera position and rotation.
    
"""
import bpy
import math
import random
import mathutils
import csv
import sys
import os

time = []

num_body = 14  # ignore the first body which is the base
num_link = 35  # ignore the first link which is the base
num_rover = 1
num_rock = 0
rock_scale = 0.3
num_frame = 382 # change this to the total number of frames


def point_at(obj, target, roll=0):
    """
    Rotate obj to look at target

    :arg obj: the object to be rotated. Usually the camera
    :arg target: the location (3-tuple or Vector) to be looked at
    :arg roll: The angle of rotation about the axis from obj to target in radians.
    """
    if not isinstance(target, mathutils.Vector):
        target = mathutils.Vector(target)
    loc = obj.location
    # direction points from the object to the target
    direction = target - loc

    quat = direction.to_track_quat("-Z", "Y")

    # /usr/share/blender/scripts/addons/add_advanced_objects_menu/arrange_on_curve.py
    quat = quat.to_matrix().to_4x4()
    rollMatrix = mathutils.Matrix.Rotation(roll, 4, "Z")

    # remember the current location, since assigning to obj.matrix_world changes it
    loc = loc.to_tuple()
    # obj.matrix_world = quat * rollMatrix
    # in blender 2.8 and above @ is used to multiply matrices
    # using * still works but results in unexpected behaviour!
    obj.matrix_world = quat @ rollMatrix
    obj.location = loc


def rot_pos(center, radius, angle=0):
    """
    return (x,y,z) points on a circle centered at (center) with a radius of (radius) with an angle of (angle)
    """
    # convert to radian
    angle_pi = angle / 180 * math.pi
    # generate point on circle
    return (
        center[0] + radius * math.sin(angle_pi),
        center[1] + radius * math.cos(angle_pi),
        center[2],
    )


# ============ specify the directory of csv, obj, image, and such
current_file_path = os.path.abspath(__file__)
current_directory = os.path.dirname(current_file_path)
home_root = os.path.dirname(current_directory)

# read the input data of the fluid, body, and link
data_in = "J:\\Chrono_main\\LRV_MLPEngine_AWD_new\\LRV_MLPEngine_AWD_new_rugged_front\\SPHTerrain\\particles\\"
file_in = "F:\\Chrono_project\\Models\\Lunar Rover Scene & Textures\\"
body_in = home_root + "\\body\\"
image_in_rot = home_root + "\\Animation\\Rendered_IMG_rot_cam\\"
image_in_fix = home_root + "\\Animation\\Rendered_IMG_fix_cam\\"

if not os.path.exists(image_in_rot):
    os.mkdir(image_in_rot)
if not os.path.exists(image_in_fix):
    os.mkdir(image_in_fix)

# ============ load position, velocity, rotation of each body at different time
dis = []
rot = []
for i in range(num_body * num_rover + num_rock):
    with open(body_in + "body_pos_rot_vel" + str(i + 1) + ".txt", "r") as file:
        dis_temp = []
        rot_temp = []
        reader = csv.reader(file)
        i = 0
        for row in reader:
            i = i + 1
            if i != 1:
                time.append(float(row[0]))
                dis_buff = []
                dis_buff.append(float(row[1]))
                dis_buff.append(float(row[2]))
                dis_buff.append(float(row[3]))
                rot_buff = []
                rot_buff.append(float(row[4]))
                rot_buff.append(float(row[5]))
                rot_buff.append(float(row[6]))
                rot_buff.append(float(row[7]))
                dis_temp.append(dis_buff)
                rot_temp.append(rot_buff)

        dis.append(dis_temp)
        rot.append(rot_temp)

dis_link = []
rot_link = []
for i in range(num_link * num_rover + num_rock):
    with open(body_in + "link_pos_rot_vel" + str(i + 1) + ".txt", "r") as file:
        dis_temp = []
        rot_temp = []
        reader = csv.reader(file)
        i = 0
        for row in reader:
            i = i + 1
            if i != 1:
                time.append(float(row[0]))
                dis_buff = []
                dis_buff.append(float(row[1]))
                dis_buff.append(float(row[2]))
                dis_buff.append(float(row[3]))
                rot_buff = []
                rot_buff.append(float(row[4]))
                rot_buff.append(float(row[5]))
                rot_buff.append(float(row[6]))
                rot_buff.append(float(row[7]))
                dis_temp.append(dis_buff)
                rot_temp.append(rot_buff)

        dis_link.append(dis_temp)
        rot_link.append(rot_temp)

# base_loc = []
# base_loc.append(dis[0][0][0])
# base_loc.append(dis[0][0][1])
# base_loc.append(dis[0][0][2])


jobid = int(sys.argv[4])
start_frame = jobid * 1 + 0
end_frame = jobid * 1 + 1
for i in range(start_frame, end_frame, 1):
    # ========= check if the png file exits or not
    image_rot_path = image_in_rot + str(i) + ".png"
    file_rot_exists = os.path.exists(image_rot_path)
    image_fix_path = image_in_fix + str(i) + ".png"
    file_fix_exists = os.path.exists(image_fix_path)
    if file_rot_exists and file_fix_exists:
        sys.exit()

    # ===========================================
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.objects.keys()

    # =========================================== load rover obj file
    # index number in each rover, 3 rovers in total:
    # 0: LRV chassis body
    # 1: LRV Rack-Pinion Steering_link
    # 2: LRV DoubleWishboneRSDA Front_Upright_L
    # 3: LRV DoubleWishboneRSDA Front_Upright_R
    # 4: LRV DoubleWishboneRSDA Front_spindle_L
    # 5: LRV DoubleWishboneRSDA Front_spindle_R
    # 6: LRV DoubleWishboneRSDA Rear_Upright_L
    # 7: LRV DoubleWishboneRSDA Rear_Upright_R
    # 8: LRV DoubleWishboneRSDA Rear_spindle_L
    # 9: LRV DoubleWishboneRSDA Rear_spindle_R
    # 10: fender_0
    # 11: fender_1
    # 12: fender_2
    # 13: fender_3
    for n_r in range(num_rover):
        obj_name = ""
        obj_name_spe = ""
        for n in range(num_body):
            if n == 0:
                obj_name = "LRV_chassis_final_without_fender_suspension_render"
                obj_name_spe = "LRV_chassis_final_without_fender_suspension_render"
            # elif n==4:
            #     obj_name = "wheel_left_render"
            #     obj_name_spe = "wheel_left_render"
            # elif n==5:
            #     obj_name = "wheel_right_render"
            #     obj_name_spe = "wheel_right_render"
            # elif n==8:
            #     obj_name = "wheel_left_render"
            #     obj_name_spe = "wheel_left_render." + str(1).zfill(3)
            # elif n==9:
            #     obj_name = "wheel_right_render"
            #     obj_name_spe = "wheel_right_render."+ str(1).zfill(3)
            elif n == 4:
                obj_name = "wheel_left_render_scaled"
                obj_name_spe = "wheel_left_render_scaled"
            elif n == 5:
                obj_name = "wheel_right_render_scaled"
                obj_name_spe = "wheel_right_render_scaled"
            elif n == 8:
                obj_name = "wheel_left_render_scaled"
                obj_name_spe = "wheel_left_render_scaled." + str(1).zfill(3)
            elif n == 9:
                obj_name = "wheel_right_render_scaled"
                obj_name_spe = "wheel_right_render_scaled." + str(1).zfill(3)
            # elif n==10:
            #     obj_name = "fender_simple_left_render"
            #     obj_name_spe = "fender_simple"
            # elif n==11:
            #     obj_name = "fender_simple_right_render"
            #     obj_name_spe = "fender_simple."+str(1).zfill(3)
            # elif n==12:
            #     obj_name = "fender_simple_left_render"
            #     obj_name_spe = "fender_simple."+str(2).zfill(3)
            # elif n==13:
            #     obj_name = "fender_simple_right_render"
            #     obj_name_spe = "fender_simple."+str(3).zfill(3)
            elif n == 10:
                obj_name = "fender_front_left_render"
                obj_name_spe = "fender_front_left_render"
            elif n == 11:
                obj_name = "fender_front_right_render"
                obj_name_spe = "fender_front_right_render"
            elif n == 12:
                obj_name = "fender_simple_left_render"
                obj_name_spe = "fender_simple"
            elif n == 13:
                obj_name = "fender_simple_right_render"
                obj_name_spe = "fender_simple." + str(1).zfill(3)
            else:
                continue

            file_loc = file_in + obj_name + ".obj"
            imported_object = bpy.ops.import_scene.obj(filepath=file_loc)
            body_id = n + n_r * num_body
            obj_object = bpy.context.object
            bpy.data.objects[obj_name_spe].location.x += dis[body_id][i][0]
            bpy.data.objects[obj_name_spe].location.y += dis[body_id][i][1]
            bpy.data.objects[obj_name_spe].location.z += dis[body_id][i][2]
            bpy.data.objects[obj_name_spe].rotation_mode = "QUATERNION"
            q = (
                rot[body_id][i][0],
                rot[body_id][i][1],
                rot[body_id][i][2],
                rot[body_id][i][3],
            )
            bpy.data.objects[obj_name_spe].rotation_quaternion = q

    # =========================================== load link obj file
    # index number in each link, 3 rovers in total:
    # 0: LRV Rack-Pinion Steering_actuator
    # 1: LRV DoubleWishboneRSDA Front_distUCA_B_R
    # 2: LRV DoubleWishboneRSDA Front_distUCA_F_L
    # 3: LRV DoubleWishboneRSDA Front_distUCA_B_L
    # 4: LRV DoubleWishboneRSDA Front_distUCA_F_R
    # 5: LRV DoubleWishboneRSDA Front_distLCA_B_R
    # 6: LRV DoubleWishboneRSDA Front_Tierod_L
    # 7: LRV DoubleWishboneRSDA Front_distLCA_F_L
    # 8: LRV DoubleWishboneRSDA Front_distLCA_B_L
    # 9: LRV DoubleWishboneRSDA Front_Tierod_R
    # 10: LRV DoubleWishboneRSDA Front_distLCA_F_R
    # 11: LRV DoubleWishboneRSDA Front_spindle_rev_L
    # 12: LRV DoubleWishboneRSDA Front_spindle_rev_R
    # 13: LRV DoubleWishboneRSDA Front_Shock_L
    # 14: LRV DoubleWishboneRSDA Front_Shock_R
    # 15: LRV DoubleWishboneRSDA Front_Spring_L
    # 16: LRV DoubleWishboneRSDA Front_Spring_R
    # 17: Null
    # 18: Null
    # 19: LRV DoubleWishboneRSDA Rear_distUCA_B_R
    # 20: LRV DoubleWishboneRSDA Rear_distUCA_F_L
    # 21: LRV DoubleWishboneRSDA Rear_distUCA_B_L
    # 22: LRV DoubleWishboneRSDA Rear_distUCA_F_R
    # 23: LRV DoubleWishboneRSDA Rear_distLCA_B_R
    # 24: LRV DoubleWishboneRSDA Rear_Tierod_L
    # 25: LRV DoubleWishboneRSDA Rear_distLCA_F_L
    # 26: LRV DoubleWishboneRSDA Rear_distLCA_B_L
    # 27: LRV DoubleWishboneRSDA Rear_Tierod_R
    # 28: LRV DoubleWishboneRSDA Rear_distLCA_F_R
    # 29: LRV DoubleWishboneRSDA Rear_spindle_rev_L
    # 30: LRV DoubleWishboneRSDA Rear_spindle_rev_R
    # 31: LRV DoubleWishboneRSDA Rear_Shock_L
    # 32: LRV DoubleWishboneRSDA Rear_Shock_R
    # 33: LRV DoubleWishboneRSDA Rear_Spring_L
    # 34: LRV DoubleWishboneRSDA Rear_Spring_R
    for n_r in range(num_rover):
        obj_name = ""
        obj_name_spe = ""
        for n in range(num_link):
            if n == 1:
                obj_name = "control_arm_short"
                obj_name_spe = "control_arm_short"
            elif n == 2:
                obj_name = "control_arm_long"
                obj_name_spe = "control_arm_long"
            elif n == 3:
                obj_name = "control_arm_short"
                obj_name_spe = "control_arm_short." + str(1).zfill(3)
            elif n == 4:
                obj_name = "control_arm_long"
                obj_name_spe = "control_arm_long." + str(1).zfill(3)
            elif n == 5:
                obj_name = "control_arm_short"
                obj_name_spe = "control_arm_short." + str(2).zfill(3)
            elif n == 7:
                obj_name = "control_arm_long"
                obj_name_spe = "control_arm_long." + str(2).zfill(3)
            elif n == 8:
                obj_name = "control_arm_short"
                obj_name_spe = "control_arm_short." + str(3).zfill(3)
            elif n == 10:
                obj_name = "control_arm_long"
                obj_name_spe = "control_arm_long." + str(3).zfill(3)
            elif n == 19:
                obj_name = "control_arm_long"
                obj_name_spe = "control_arm_long." + str(4).zfill(3)
            elif n == 20:
                obj_name = "control_arm_short"
                obj_name_spe = "control_arm_short." + str(4).zfill(3)
            elif n == 21:
                obj_name = "control_arm_long"
                obj_name_spe = "control_arm_long." + str(5).zfill(3)
            elif n == 22:
                obj_name = "control_arm_short"
                obj_name_spe = "control_arm_short." + str(5).zfill(3)
            elif n == 23:
                obj_name = "control_arm_long"
                obj_name_spe = "control_arm_long." + str(6).zfill(3)
            elif n == 25:
                obj_name = "control_arm_short"
                obj_name_spe = "control_arm_short." + str(6).zfill(3)
            elif n == 26:
                obj_name = "control_arm_long"
                obj_name_spe = "control_arm_long." + str(7).zfill(3)
            elif n == 28:
                obj_name = "control_arm_short"
                obj_name_spe = "control_arm_short." + str(7).zfill(3)
            else:
                continue

            file_loc = file_in + obj_name + ".obj"
            imported_object = bpy.ops.import_scene.obj(filepath=file_loc)
            body_id = n + n_r * num_link
            bpy.data.objects[obj_name_spe].location.x += dis_link[body_id][i][0]
            bpy.data.objects[obj_name_spe].location.y += dis_link[body_id][i][1]
            bpy.data.objects[obj_name_spe].location.z += (
                dis_link[body_id][i][2] - 0.025606
            )
            bpy.data.objects[obj_name_spe].rotation_mode = "QUATERNION"
            q = (
                rot_link[body_id][i][0],
                rot_link[body_id][i][1],
                rot_link[body_id][i][2],
                rot_link[body_id][i][3],
            )
            bpy.data.objects[obj_name_spe].rotation_quaternion = q

    # #=========================================== load rock obj file
    # for n_r in range(num_rock):
    #     obj_name = "rock3"
    #     obj_name_spe = "rock3"
    #     if n_r > 0:
    #         obj_name_spe = "rock3." + str(n_r).zfill(3)

    #     file_loc = file_in + obj_name + ".obj"
    #     imported_object = bpy.ops.import_scene.obj(filepath = file_loc)
    #     bpy.ops.transform.resize(value=(rock_scale, rock_scale, rock_scale))
    #     obj_object = bpy.context.object
    #     body_id = n_r + num_rover * num_body
    #     bpy.data.objects[obj_name_spe].location.x += dis[body_id][i][0]
    #     bpy.data.objects[obj_name_spe].location.y += dis[body_id][i][1]
    #     bpy.data.objects[obj_name_spe].location.z += dis[body_id][i][2]
    #     bpy.data.objects[obj_name_spe].rotation_mode = 'QUATERNION'
    #     q = (rot[body_id][i][0],rot[body_id][i][1],rot[body_id][i][2],rot[body_id][i][3])
    #     bpy.data.objects[obj_name_spe].rotation_quaternion = q

    bpy.context.view_layer.update()

    # ===========================================
    # ==================== Load SPH particle file
    # ===========================================
    radius_particle = 0.01

    positions = []
    dir = data_in + "fluid" + str(i) + ".csv"
    count = 0
    for line in open(dir):
        if count == 0:
            count = count + 1
            continue
        else:
            # you have to parse "x", "y", "z" and "r" from the variable "line"
            line_seg = line.split(",")
            x, y, z = line_seg[0], line_seg[1], line_seg[2]
            position_buff = (float(x), float(y), float(z))
            positions.append(position_buff)
            count = count + 1
    # print("total number of particles")
    # print(count)

    """ -------------- PARTICLE SYSTEM START-------------- """
    context = bpy.context
    # instance object
    bpy.ops.mesh.primitive_ico_sphere_add(radius=1, location=(50, 50, 50))
    pippo = radius_particle
    ico = context.object

    # 自定义的Z坐标范围
    z_min = -0.03
    z_max = 1.0

    # 创建材料
    mat = bpy.data.materials.new(name="Particle_Material")
    mat.use_nodes = True
    ico.data.materials.append(mat)

    # 设置材料节点
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    nodes.clear()

    # 创建节点
    particle_info = nodes.new(type="ShaderNodeParticleInfo")
    separate_xyz = nodes.new(type="ShaderNodeSeparateXYZ")
    map_range = nodes.new(type="ShaderNodeMapRange")
    color_ramp = nodes.new(type="ShaderNodeValToRGB")
    bsdf = nodes.new(type="ShaderNodeBsdfPrincipled")
    material_output = nodes.new(type="ShaderNodeOutputMaterial")

    # 配置Map Range节点以标准化Z坐标
    map_range.inputs["From Min"].default_value = z_min
    map_range.inputs["From Max"].default_value = z_max
    map_range.inputs["To Min"].default_value = 0.0
    map_range.inputs["To Max"].default_value = 1.0

    # 配置ColorRamp节点
    # color_ramp.color_ramp.elements[0].color = [0.230, 0.299, 0.754, 1]  # cool color
    # color_ramp.color_ramp.elements[1].color = [0.706, 0.016, 0.150, 1]  # warm color
    color_ramp.color_ramp.elements[0].color = (0.65, 0.75, 0.9, 1)  # 强蓝色
    color_ramp.color_ramp.elements[1].color = (0.706, 0.016, 0.150, 1)  # 强红色
    color_ramp.color_ramp.color_mode = 'HSV'

    # 连接节点
    links.new(particle_info.outputs["Location"], separate_xyz.inputs["Vector"])
    links.new(separate_xyz.outputs["Z"], map_range.inputs["Value"])
    links.new(map_range.outputs["Result"], color_ramp.inputs["Fac"])
    links.new(color_ramp.outputs["Color"], bsdf.inputs["Base Color"])
    links.new(bsdf.outputs["BSDF"], material_output.inputs["Surface"])

    # 将材料应用到粒子实例对象
    ico.data.materials[0] = mat

    # cube with ps
    bpy.ops.mesh.primitive_cube_add(size=0.0001)
    cube = context.object

    # ps
    ps = cube.modifiers.new("SomeName", "PARTICLE_SYSTEM").particle_system
    psname = ps.name
    ps.settings.count = count - 1
    ps.settings.lifetime = 1000
    ps.settings.frame_start = ps.settings.frame_end = 1
    ps.settings.render_type = "OBJECT"
    ps.settings.instance_object = ico

    def particle_handler(scene, depsgraph):
        ob = depsgraph.objects.get(cube.name)
        if ob:
            ps = ob.particle_systems[psname]
            f = scene.frame_current
            for m, particle in enumerate(ps.particles):
                setattr(particle, "location", positions[m])
                setattr(particle, "size", radius_particle)

    # Clear the post frame handler
    bpy.app.handlers.frame_change_post.clear()

    # Register our particleSetter with the post frame handler
    bpy.app.handlers.frame_change_post.append(particle_handler)

    # Trigger frame update
    bpy.context.scene.frame_current = 2
    """ -------------- PARTICLE SYSTEM END -------------- """

    bpy.context.view_layer.update()
    # ===========================================
    # ===========================================
    # ===========================================

    # ===========================================
    # ========================= Rendering setting
    # ===========================================
    bpy.ops.transform.rotate(value=(-math.pi * 0.5), orient_axis="X")  # value = Angle
    bpy.ops.mesh.primitive_plane_add(
        size=10000.0,
        calc_uvs=True,
        enter_editmode=False,
        align="WORLD",
        location=(0.0, 0.0, -0.40),
    )

    # ======== create light datablock, set attributes
    light_data = bpy.data.lights.new(name="light_2.80", type="POINT")  # type='SUN'
    light_data.energy = 5000
    # create new object with our light datablock
    light_object = bpy.data.objects.new(name="light_2.80", object_data=light_data)
    # link light object
    bpy.context.collection.objects.link(light_object)
    # make it active
    bpy.context.view_layer.objects.active = light_object
    # change location
    angle_pi = (45 + i * 0.5) / 180 * math.pi
    light_object.location = (
        14 * math.cos(angle_pi) + dis[0][i][0],
        14 * math.sin(angle_pi) + dis[0][i][1],
        15,
    )

    # ======== create another light datablock, set attributes
    light_data1 = bpy.data.lights.new(name="light_top", type="POINT")
    light_data1.energy = 5000
    # create new object with our light datablock
    light_object1 = bpy.data.objects.new(name="light_top", object_data=light_data1)
    # link light object
    bpy.context.collection.objects.link(light_object1)
    # make it active
    bpy.context.view_layer.objects.active = light_object1
    # change location
    light_object1.location = (dis[0][i][0], dis[0][i][1], 15)

    # ======== use GPU if available
    scene.cycles.device = "GPU"
    prefs = bpy.context.preferences
    cprefs = prefs.addons["cycles"].preferences
    # Attempt to set GPU device types if available
    for compute_device_type in ("OPTIX", "CUDA", "OPENCL", "NONE"):
        try:
            cprefs.compute_device_type = compute_device_type
            break
        except TypeError:
            pass
    # Enable all CPU and GPU devices
    cprefs.get_devices()
    for device in cprefs.devices:
        device.use = True

    # =======================
    # ======== ouput settings
    # =======================
    bpy.context.scene.render.engine = "CYCLES"
    bpy.context.scene.cycles.device = "GPU"
    bpy.context.scene.render.threads_mode = "FIXED"
    bpy.context.scene.render.threads = 12
    # bpy.context.scene.cycles.kcycles_gpu_boost = 'HIGH'
    # bpy.context.scene.cycles.kcycles_performance_mode = 'FAST'
    # bpy.context.scene.cycles.kcycles_gi_preset = 'STANDARD'
    # bpy.context.scene.cycles.kcycles_sampling_preset = 'MEDIUM'

    bpy.context.scene.render.resolution_percentage = 100
    bpy.context.scene.cycles.samples = 256
    bpy.context.scene.render.resolution_x = 2560
    bpy.context.scene.render.resolution_y = 1440
    # bpy.context.scene.render.resolution_x = 3840
    # bpy.context.scene.render.resolution_y = 2160
    # bpy.context.scene.render.image_settings.compression = 50
    bpy.context.scene.render.image_settings.color_mode = "RGBA"
    bpy.context.scene.render.image_settings.file_format = "PNG"

    # ======== create a camera and settings
    # Create and get camera
    if "Camera" not in bpy.data.objects:
        bpy.ops.object.camera_add(
            enter_editmode=False, align="WORLD", scale=(5.0, 5.0, 5.0)
        )
        cam = bpy.context.object
        cam.name = "Camera"
    else:
        cam = bpy.data.objects["Camera"]

    scene.camera = bpy.context.object
    # Set up rotational camera

    # render the rotate camera
    # add rotational angle by 1
    if not file_rot_exists:
        ini_rad = 135
        frame_stand = 0.07 * num_frame
        frame_end = 0.83 * num_frame
        rate = (135 - 45) / (frame_end - frame_stand)

        if i <= frame_stand:
            cur_rad = 135
        elif i >= frame_end:
            cur_rad = 45
        else:
            cur_rad = 135 - rate * (i - frame_stand)
        # cam.location = rot_pos((0, 0, 10), 30, cur_rad)
        # point_at(cam, (0, 0, 2), roll=math.radians(0))
        cam.location = rot_pos(
            (dis[0][i][0], dis[0][i][1], dis[0][20][2] + 6), 15, cur_rad
        )
        point_at(
            cam, (dis[0][i][0] - 1.0, dis[0][i][1], dis[0][20][2]), roll=math.radians(0)
        )
        bpy.context.scene.render.filepath = image_in_rot + str(i) + ".png"
        bpy.ops.render.render(write_still=True)

    # render the fix camera
    if not file_fix_exists:
        cur_rad = 225
        # cam.location = rot_pos((0, 0, 10), 30, cur_rad)
        # point_at(cam, (0, 0, 2), roll=math.radians(0))
        cam.location = rot_pos(
            (dis[0][i][0], dis[0][i][1], dis[0][20][2] + 6), 15, cur_rad
        )
        point_at(
            cam, (dis[0][i][0] - 2.0, dis[0][i][1], dis[0][20][2]), roll=math.radians(0)
        )
        bpy.context.scene.render.filepath = image_in_fix + str(i) + ".png"
        bpy.ops.render.render(write_still=True)
