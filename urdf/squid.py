#!/usr/bin/env python3

import math
from pathlib import Path

# ============================================================
# CONFIGURATION
# ============================================================

ROBOT_NAME = "three_arm_octobot"

# ------------------------------------------------------------
# Head geometry
# ------------------------------------------------------------
HEAD_SIDE = 500.0          # mm
HEAD_HEIGHT = 200.0        # mm from center plane to each tip
FILLET_RADIUS = 5.0        # mm

# ------------------------------------------------------------
# Arm configuration
# ------------------------------------------------------------
NUM_ARMS = 3
NUM_SECTIONS = 20
ARM_MOUNT_OFFSET = 25.0    # mm 
BASE_TOP_H = 30.0          # mm
BASE_BOT_H = 30.0          # mm
JOINT_LIMIT = 0.5          # radians
MESH_SCALE = "0.001 0.001 0.001"

SCRIPT_DIR = Path(__file__).resolve().parent
OUTPUT_FILE = SCRIPT_DIR / "squid.urdf"
PACKAGE_NAME = "octopus_description"
HEAD_MESH = "head.stl"

# ============================================================
# DYNAMIC SCALING FUNCTIONS
# ============================================================
def get_scale(idx):
    return 1.0 - (idx * (0.65 / (NUM_SECTIONS - 1)))

def get_segment_length(idx):
    scale = get_scale(idx)
    return (BASE_TOP_H + BASE_BOT_H) * scale

# ============================================================
# BASIC GEOMETRY FUNCTIONS
# ============================================================
def normalize(v):
    length = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
    return (v[0]/length, v[1]/length, v[2]/length)

def face_data():
    R = HEAD_SIDE / math.sqrt(3.0)
    v0 = (R, 0.0, 0.0)
    v1 = (R * math.cos(2.0 * math.pi / 3.0), R * math.sin(2.0 * math.pi / 3.0), 0.0)
    v2 = (R * math.cos(4.0 * math.pi / 3.0), R * math.sin(4.0 * math.pi / 3.0), 0.0)
    top = (0.0, 0.0, HEAD_HEIGHT)

    faces = [(top, v0, v1), (top, v1, v2), (top, v2, v0)]
    result = []

    for a, b, c in faces:
        center = (
            (a[0] + b[0] + c[0]) / 3.0,
            (a[1] + b[1] + c[1]) / 3.0,
            (a[2] + b[2] + c[2]) / 3.0,
        )
        ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        ac = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
        
        normal = (
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0]
        )
        normal = normalize(normal)

        if normal[2] < 0:
            normal = (-normal[0], -normal[1], -normal[2])
            
        result.append({"center": center, "normal": normal})

    return result

def normal_to_rpy(normal):
    nx, ny, nz = normal
    yaw = math.atan2(ny, nx)
    horizontal = math.sqrt(nx * nx + ny * ny)
    pitch = math.atan2(horizontal, nz)
    roll = 0.0
    return (roll, pitch, yaw)

def fmt(value):
    if abs(value) < 1e-10:
        value = 0.0
    return f"{value:.8f}"

# ============================================================
# URDF GENERATOR
# ============================================================
class URDFGenerator:
    def __init__(self):
        self.lines = []
        self.indent_level = 0

    def write(self, text=""):
        self.lines.append("  " * self.indent_level + text)

    def comment(self, text):
        self.write("<!-- ==================================================")
        self.write(f"     {text}")
        self.write("     ================================================== -->")

    def add_link(
        self, name, visual_mesh=None, collision_mesh=None,
        visual_xyz="0 0 0", visual_rpy="0 0 0",
        collision_xyz="0 0 0", collision_rpy="0 0 0",
        mass=0.1, ixx=0.001, iyy=0.001, izz=0.001,
        color_rgba=None, material_name=None, gazebo_material=None,
        is_box=False, box_size="0.02 0.02 0.01"
    ):
        self.write(f'<link name="{name}">')
        self.indent_level += 1

        self.write("<inertial>")
        self.indent_level += 1
        self.write('<origin xyz="0 0 0" rpy="0 0 0"/>')
        self.write(f'<mass value="{mass}"/>')
        self.write(f'<inertia ixx="{ixx}" ixy="0" ixz="0" iyy="{iyy}" iyz="0" izz="{izz}"/>')
        self.indent_level -= 1
        self.write("</inertial>")

        if visual_mesh is not None:
            self.write("<visual>")
            self.indent_level += 1
            self.write(f'<origin xyz="{visual_xyz}" rpy="{visual_rpy}"/>')
            self.write("<geometry>")
            self.indent_level += 1
            self.write(f'<mesh filename="model://{PACKAGE_NAME}/meshes/{visual_mesh}" scale="{MESH_SCALE}"/>')
            self.indent_level -= 1
            self.write("</geometry>")
            
            if color_rgba and material_name:
                self.write(f'<material name="{material_name}">')
                self.indent_level += 1
                self.write(f'<color rgba="{color_rgba}"/>')
                self.indent_level -= 1
                self.write("</material>")
                
            self.indent_level -= 1
            self.write("</visual>")
        elif is_box:
            self.write("<visual>")
            self.indent_level += 1
            self.write("<geometry>")
            self.indent_level += 1
            self.write(f'<box size="{box_size}"/>')
            self.indent_level -= 1
            self.write("</geometry>")
            if color_rgba and material_name:
                self.write(f'<material name="{material_name}">')
                self.indent_level += 1
                self.write(f'<color rgba="{color_rgba}"/>')
                self.indent_level -= 1
                self.write("</material>")
            self.indent_level -= 1
            self.write("</visual>")

        if collision_mesh is not None:
            self.write("<collision>")
            self.indent_level += 1
            self.write(f'<origin xyz="{collision_xyz}" rpy="{collision_rpy}"/>')
            self.write("<geometry>")
            self.indent_level += 1
            self.write(f'<mesh filename="model://{PACKAGE_NAME}/meshes/{collision_mesh}" scale="{MESH_SCALE}"/>')
            self.indent_level -= 1
            self.write("</geometry>")
            self.indent_level -= 1
            self.write("</collision>")
        elif is_box:
            self.write("<collision>")
            self.indent_level += 1
            self.write("<geometry>")
            self.indent_level += 1
            self.write(f'<box size="{box_size}"/>')
            self.indent_level -= 1
            self.write("</geometry>")
            self.indent_level -= 1
            self.write("</collision>")

        self.indent_level -= 1
        self.write("</link>")
        
        if gazebo_material:
            self.write("")
            self.write(f'<gazebo reference="{name}">')
            self.indent_level += 1
            self.write(f'<material>{gazebo_material}</material>')
            self.indent_level -= 1
            self.write("</gazebo>")

    def add_fixed_joint(self, name, parent, child, xyz="0 0 0", rpy="0 0 0"):
        self.write(f'<joint name="{name}" type="fixed">')
        self.indent_level += 1
        self.write(f'<parent link="{parent}"/>')
        self.write(f'<child link="{child}"/>')
        self.write(f'<origin xyz="{xyz}" rpy="{rpy}"/>')
        self.indent_level -= 1
        self.write("</joint>")

    def add_revolute_joint(self, name, parent, child, axis, xyz="0 0 0", rpy="0 0 0"):
        self.write(f'<joint name="{name}" type="revolute">')
        self.indent_level += 1
        self.write(f'<parent link="{parent}"/>')
        self.write(f'<child link="{child}"/>')
        self.write(f'<origin xyz="{xyz}" rpy="{rpy}"/>')
        self.write(f'<axis xyz="{axis}"/>')
        self.write(f'<limit lower="-{JOINT_LIMIT}" upper="{JOINT_LIMIT}" effort="100" velocity="2.0"/>')
        self.indent_level -= 1
        self.write("</joint>")

    def add_arm(self, arm_number, face):
        arm = f"arm{arm_number}"
        mount_link = f"{arm}_mount"
        
        center = face["center"]
        normal = face["normal"]
        roll, pitch, yaw = normal_to_rpy(normal)

        offset_center = (
            center[0] + normal[0] * ARM_MOUNT_OFFSET,
            center[1] + normal[1] * ARM_MOUNT_OFFSET,
            center[2] + normal[2] * ARM_MOUNT_OFFSET
        )

        mount_xyz = " ".join(fmt(v / 1000.0) for v in offset_center)
        mount_rpy = " ".join(fmt(v) for v in (roll, pitch, yaw))
        
        arm_colors = [
            {"rgba": "1.0 0.0 0.0 1.0", "mat": "red", "gazebo": "Gazebo/Red"},
            {"rgba": "0.0 1.0 0.0 1.0", "mat": "green", "gazebo": "Gazebo/Green"},
            {"rgba": "1.0 1.0 0.0 1.0", "mat": "yellow", "gazebo": "Gazebo/Yellow"}
        ]
        color = arm_colors[(arm_number - 1) % 3]

        self.write("")
        self.comment(f"{arm.upper()} MOUNT - OFFSET BY {ARM_MOUNT_OFFSET}mm")
        self.add_link(name=mount_link, mass=0.05)
        
        self.add_fixed_joint(
            name=f"{arm}_mount_joint",
            parent="head",
            child=mount_link,
            xyz=mount_xyz,
            rpy=mount_rpy
        )

        previous_link = mount_link

        for i in range(NUM_SECTIONS):
            segment_link = f"{arm}_segment_{i}"
            yaw_link = f"{arm}_yaw_link_{i}"
            pitch_joint = f"{arm}_pitch_joint_{i}"
            yaw_joint = f"{arm}_yaw_joint_{i}"

            current_length_mm = get_segment_length(i)
            visual_z = current_length_mm / 2.0 / 1000.0
            joint_z = current_length_mm / 1000.0

            self.add_link(
                name=segment_link,
                visual_mesh=f"segment_{i}.stl",
                collision_mesh=f"segment_{i}.stl",
                visual_xyz=(f"0 0 {fmt(visual_z)}"),
                collision_xyz=(f"0 0 {fmt(visual_z)}"),
                mass=0.10, ixx=0.001, iyy=0.001, izz=0.001,
                color_rgba=color["rgba"],
                material_name=f"{arm}_{color['mat']}",
                gazebo_material=color["gazebo"]
            )

            self.add_revolute_joint(
                name=pitch_joint, parent=previous_link, child=segment_link,
                axis="1 0 0", xyz="0 0 0", rpy="0 0 0"
            )

            self.add_link(name=yaw_link, mass=0.05)

            self.add_revolute_joint(
                name=yaw_joint, parent=segment_link, child=yaw_link,
                axis="0 1 0", xyz=f"0 0 {fmt(joint_z)}", rpy="0 0 0"
            )

            previous_link = yaw_link

    def add_ros2_control(self):
        self.write("")
        self.comment("ROS 2 CONTROL - GAZEBO")
        self.write('<ros2_control name="ThreeArmGazeboSystem" type="system">')
        self.indent_level += 1
        self.write("<hardware>")
        self.indent_level += 1
        self.write("<plugin>gz_ros2_control/GazeboSimSystem</plugin>")
        self.indent_level -= 1
        self.write("</hardware>")

        for arm_number in range(1, NUM_ARMS + 1):
            arm = f"arm{arm_number}"
            for i in range(NUM_SECTIONS):
                pitch_joint = f"{arm}_pitch_joint_{i}"
                yaw_joint = f"{arm}_yaw_joint_{i}"

                self.write(f'<joint name="{pitch_joint}">')
                self.indent_level += 1
                self.write('<command_interface name="position"/>')
                self.write('<state_interface name="position"/>')
                self.write('<state_interface name="velocity"/>')
                self.indent_level -= 1
                self.write("</joint>")

                self.write(f'<joint name="{yaw_joint}">')
                self.indent_level += 1
                self.write('<command_interface name="position"/>')
                self.write('<state_interface name="position"/>')
                self.write('<state_interface name="velocity"/>')
                self.indent_level -= 1
                self.write("</joint>")

        self.indent_level -= 1
        self.write("</ros2_control>")

    def add_gazebo_plugin(self):
        self.write("")
        self.comment("GAZEBO ROS 2 CONTROL PLUGIN")
        self.write("<gazebo>")
        self.indent_level += 1
        self.write('<plugin filename="libgz_ros2_control-system.so" name="gz_ros2_control::GazeboSimROS2ControlPlugin">')
        self.indent_level += 1
        self.write("<parameters>/home/vaibhav/doc_oct/src/octopus_description/config/bot.yaml</parameters>")
        
        # INJECTING CLOCK CONFIGURATION TO FIX 'NO CLOCK RECEIVED' WARNING
        self.write('<ros>')
        self.indent_level += 1
        self.write('<parameter name="use_sim_time" value="true"/>')
        self.indent_level -= 1
        self.write('</ros>')
        
        self.indent_level -= 1
        self.write("</plugin>")
        self.indent_level -= 1
        self.write("</gazebo>")

        # IMU Gazebo Sensor Plugin
        self.write("")
        self.comment("GAZEBO IMU SENSOR PLUGIN WITH NOISE")
        self.write('<gazebo reference="imu_link">')
        self.indent_level += 1
        self.write('<sensor name="imu_sensor" type="imu">')
        self.indent_level += 1
        self.write('<always_on>1</always_on>')
        self.write('<update_rate>100</update_rate>')
        self.write('<visualize>true</visualize>')
        self.write('<topic>imu</topic>')
        
        self.write('<plugin filename="libgz-sim-imu-system.so" name="gz::sim::systems::Imu">')
        self.indent_level += 1
        self.write('<topic>imu</topic>')
        self.indent_level -= 1
        self.write('</plugin>')
        
        # ADDING GAUSSIAN NOISE BLOCK
        self.write('<imu>')
        self.indent_level += 1
        
        self.write('<linear_acceleration>')
        self.indent_level += 1
        self.write('<x><noise type="gaussian"><mean>0.0</mean><stddev>0.005</stddev></noise></x>')
        self.write('<y><noise type="gaussian"><mean>0.0</mean><stddev>0.005</stddev></noise></y>')
        self.write('<z><noise type="gaussian"><mean>0.0</mean><stddev>0.005</stddev></noise></z>')
        self.indent_level -= 1
        self.write('</linear_acceleration>')
        
        self.write('<angular_velocity>')
        self.indent_level += 1
        self.write('<x><noise type="gaussian"><mean>0.0</mean><stddev>0.001</stddev></noise></x>')
        self.write('<y><noise type="gaussian"><mean>0.0</mean><stddev>0.001</stddev></noise></y>')
        self.write('<z><noise type="gaussian"><mean>0.0</mean><stddev>0.001</stddev></noise></z>')
        self.indent_level -= 1
        self.write('</angular_velocity>')
        
        self.indent_level -= 1
        self.write('</imu>')
        
        self.indent_level -= 1
        self.write('</sensor>')
        self.indent_level -= 1
        self.write('</gazebo>')

    def generate(self):
        self.write('<?xml version="1.0"?>')
        self.write("")
        self.write(f'<robot name="{ROBOT_NAME}">')
        self.indent_level += 1

        self.comment("ROOT BASE LINK (UNFIXED)")
        self.add_link(name="base_link", mass=1.0, ixx=0.01, iyy=0.01, izz=0.01)

        self.comment("500 mm TRIANGULAR BIPYRAMID HEAD")
        self.add_link(
            name="head", visual_mesh=HEAD_MESH, collision_mesh=HEAD_MESH,
            mass=3.0, ixx=0.03, iyy=0.03, izz=0.03,
            color_rgba="0.2 0.2 0.8 1.0", material_name="head_blue", gazebo_material="Gazebo/Blue"
        )

        self.add_fixed_joint(
            name="base_to_head", parent="base_link", child="head", xyz="0 0 0.5", rpy="3.14159265 0 0"
        )

        # ====================================================
        # IMU LINK & JOINT
        # ====================================================
        self.comment("IMU SENSOR LINK")
        self.add_link(
            name="imu_link",
            mass=0.01,
            ixx=0.0001, iyy=0.0001, izz=0.0001,
            is_box=True,
            box_size="0.03 0.03 0.01",
            color_rgba="0.8 0.1 0.1 1.0",
            material_name="imu_red",
            gazebo_material="Gazebo/Red"
        )

        self.add_fixed_joint(
            name="head_to_imu",
            parent="head",
            child="imu_link",
            xyz="0 0 0.05",
            rpy="0 0 0"
        )

        faces = face_data()
        for arm_number in range(1, NUM_ARMS + 1):
            self.add_arm(arm_number=arm_number, face=faces[arm_number - 1])

        self.add_ros2_control()
        self.add_gazebo_plugin()

        self.indent_level -= 1
        self.write("</robot>")

        OUTPUT_FILE.write_text("\n".join(self.lines) + "\n")
        return OUTPUT_FILE

def main():
    print("\n==============================================")
    print("   THREE ARM OCTOPUS URDF GENERATOR")
    print("==============================================\n")
    generator = URDFGenerator()
    generator.generate()
    print("URDF GENERATED SUCCESSFULLY\n")

if __name__ == "__main__":
    main()
