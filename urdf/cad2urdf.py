import xml.etree.ElementTree as ET
import xml.dom.minidom
import os


# ============================================================
# CONFIGURATION
# ============================================================

NUM_SECTIONS = 20

BASE_TOP_H = 0.030
BASE_BOT_H = 0.030
RESTING_GAP = 0.002

BASE_MASS = 0.1

# Mesh directory
MESH_DIR = "/home/vaibhav/doc_oct/src/octopus_description/meshes"

# Controller configuration
CONTROLLER_CONFIG = "/home/vaibhav/doc_oct/install/octopus_description/share/octopus_description/config/arm_controllers.yaml"
# Joint limits
JOINT_LOWER = -0.5
JOINT_UPPER = 0.5
JOINT_EFFORT = 10.0
JOINT_VELOCITY = 2.0

# Mesh scale
MESH_SCALE = "0.001 0.001 0.001"


# ============================================================
# TAPERING
# ============================================================

def get_scale(i):
    """
    Section 0 = scale 1.0
    Last section = scale 0.35
    """

    if NUM_SECTIONS <= 1:
        return 1.0

    return 1.0 - (i * (0.65 / (NUM_SECTIONS - 1)))


# ============================================================
# Z POSITION
# ============================================================

def get_relative_z_offset(i):
    """
    Calculates the vertical gap between consecutive sections.
    """

    if i == 0:
        return 0.0

    prev_top = BASE_TOP_H * get_scale(i - 1)
    curr_bot = BASE_BOT_H * get_scale(i)

    return prev_top + curr_bot + RESTING_GAP


# ============================================================
# CREATE WORLD LINK
# ============================================================

def create_world_link(robot):

    world_link = ET.SubElement(
        robot,
        "link",
        name="world"
    )

    return world_link


# ============================================================
# CREATE BASE LINK
# ============================================================

def create_base_link(robot):

    base_link = ET.SubElement(
        robot,
        "link",
        name="base_link"
    )

    # --------------------------------------------------------
    # Visual
    # --------------------------------------------------------

    base_vis = ET.SubElement(
        base_link,
        "visual"
    )

    ET.SubElement(
        base_vis,
        "origin",
        xyz="0 0 -0.05",
        rpy="0 0 0"
    )

    b_geom = ET.SubElement(
        base_vis,
        "geometry"
    )

    ET.SubElement(
        b_geom,
        "box",
        size="0.2 0.2 0.1"
    )

    b_mat = ET.SubElement(
        base_vis,
        "material",
        name="base_mat"
    )

    ET.SubElement(
        b_mat,
        "color",
        rgba="0.2 0.2 0.2 1.0"
    )

    # --------------------------------------------------------
    # Collision
    # --------------------------------------------------------

    base_col = ET.SubElement(
        base_link,
        "collision"
    )

    ET.SubElement(
        base_col,
        "origin",
        xyz="0 0 -0.05",
        rpy="0 0 0"
    )

    b_cgeom = ET.SubElement(
        base_col,
        "geometry"
    )

    ET.SubElement(
        b_cgeom,
        "box",
        size="0.2 0.2 0.1"
    )

    # --------------------------------------------------------
    # Inertial
    # --------------------------------------------------------

    base_inertial = ET.SubElement(
        base_link,
        "inertial"
    )

    ET.SubElement(
        base_inertial,
        "mass",
        value="10.0"
    )

    ET.SubElement(
        base_inertial,
        "inertia",
        ixx="1.0",
        ixy="0",
        ixz="0",
        iyy="1.0",
        iyz="0",
        izz="1.0"
    )

    return base_link


# ============================================================
# CREATE WORLD -> BASE JOINT
# ============================================================

def create_world_joint(robot):

    world_joint = ET.SubElement(
        robot,
        "joint",
        name="world_fixed",
        type="fixed"
    )

    ET.SubElement(
        world_joint,
        "parent",
        link="world"
    )

    ET.SubElement(
        world_joint,
        "child",
        link="base_link"
    )

    ET.SubElement(
        world_joint,
        "origin",
        xyz="0 0 0.1",
        rpy="0 0 0"
    )


# ============================================================
# CREATE VIRTUAL LINK
# ============================================================

def create_virtual_link(robot, i):

    virtual_link = f"virtual_link_{i}"

    v_link = ET.SubElement(
        robot,
        "link",
        name=virtual_link
    )

    # Very small mass to avoid massless-link problems
    inertial = ET.SubElement(
        v_link,
        "inertial"
    )

    ET.SubElement(
        inertial,
        "mass",
        value="0.001"
    )

    ET.SubElement(
        inertial,
        "inertia",
        ixx="0.0001",
        ixy="0",
        ixz="0",
        iyy="0.0001",
        iyz="0",
        izz="0.0001"
    )

    return virtual_link


# ============================================================
# CREATE REAL SEGMENT LINK
# ============================================================

def create_segment_link(robot, i, scale):

    real_link = f"segment_{i}"

    r_link = ET.SubElement(
        robot,
        "link",
        name=real_link
    )

    # --------------------------------------------------------
    # Visual
    # --------------------------------------------------------

    vis = ET.SubElement(
        r_link,
        "visual"
    )

    ET.SubElement(
        vis,
        "origin",
        xyz="0 0 0",
        rpy="0 0 0"
    )

    v_geom = ET.SubElement(
        vis,
        "geometry"
    )

    mesh_file = os.path.join(
        MESH_DIR,
        f"segment_{i}.stl"
    )

    ET.SubElement(
        v_geom,
        "mesh",
        filename=f"file://{mesh_file}",
        scale=MESH_SCALE
    )

    v_mat = ET.SubElement(
        vis,
        "material",
        name="arm_teal"
    )

    ET.SubElement(
        v_mat,
        "color",
        rgba="0.0 0.6 0.7 1.0"
    )

    # --------------------------------------------------------
    # Collision
    # --------------------------------------------------------

    col = ET.SubElement(
        r_link,
        "collision"
    )

    ET.SubElement(
        col,
        "origin",
        xyz="0 0 0",
        rpy="0 0 0"
    )

    c_geom = ET.SubElement(
        col,
        "geometry"
    )

    ET.SubElement(
        c_geom,
        "mesh",
        filename=f"file://{mesh_file}",
        scale=MESH_SCALE
    )

    # --------------------------------------------------------
    # Inertial
    # --------------------------------------------------------

    seg_mass = BASE_MASS * (scale ** 3)

    inertial = ET.SubElement(
        r_link,
        "inertial"
    )

    ET.SubElement(
        inertial,
        "mass",
        value=str(round(seg_mass, 4))
    )

    # Approximate spherical inertia
    inertia_val = str(
        round(
            (2.0 / 5.0)
            * seg_mass
            * ((0.05 * scale) ** 2),
            6
        )
    )

    ET.SubElement(
        inertial,
        "inertia",
        ixx=inertia_val,
        ixy="0",
        ixz="0",
        iyy=inertia_val,
        iyz="0",
        izz=inertia_val
    )

    return real_link


# ============================================================
# CREATE PITCH JOINT
# ============================================================

def create_pitch_joint(
    robot,
    i,
    parent_link,
    child_link,
    z_pos
):

    pitch_joint = ET.SubElement(
        robot,
        "joint",
        name=f"pitch_joint_{i}",
        type="revolute"
    )

    ET.SubElement(
        pitch_joint,
        "parent",
        link=parent_link
    )

    ET.SubElement(
        pitch_joint,
        "child",
        link=child_link
    )

    ET.SubElement(
        pitch_joint,
        "origin",
        xyz=f"0 0 {z_pos}",
        rpy="0 0 0"
    )

    ET.SubElement(
        pitch_joint,
        "axis",
        xyz="1 0 0"
    )

    ET.SubElement(
        pitch_joint,
        "limit",
        lower=str(JOINT_LOWER),
        upper=str(JOINT_UPPER),
        effort=str(JOINT_EFFORT),
        velocity=str(JOINT_VELOCITY)
    )

    ET.SubElement(
        pitch_joint,
        "dynamics",
        damping="0.5",
        friction="0.1"
    )


# ============================================================
# CREATE YAW JOINT
# ============================================================

def create_yaw_joint(
    robot,
    i,
    parent_link,
    child_link
):

    yaw_joint = ET.SubElement(
        robot,
        "joint",
        name=f"yaw_joint_{i}",
        type="revolute"
    )

    ET.SubElement(
        yaw_joint,
        "parent",
        link=parent_link
    )

    ET.SubElement(
        yaw_joint,
        "child",
        link=child_link
    )

    ET.SubElement(
        yaw_joint,
        "origin",
        xyz="0 0 0",
        rpy="0 0 0"
    )

    ET.SubElement(
        yaw_joint,
        "axis",
        xyz="0 1 0"
    )

    ET.SubElement(
        yaw_joint,
        "limit",
        lower=str(JOINT_LOWER),
        upper=str(JOINT_UPPER),
        effort=str(JOINT_EFFORT),
        velocity=str(JOINT_VELOCITY)
    )

    ET.SubElement(
        yaw_joint,
        "dynamics",
        damping="0.5",
        friction="0.1"
    )


# ============================================================
# ROS 2 CONTROL
# ============================================================

def create_ros2_control(robot):

    ros2_control = ET.SubElement(
        robot,
        "ros2_control",
        name="GazeboSimSystem",
        type="system"
    )

    # Hardware
    hw = ET.SubElement(
        ros2_control,
        "hardware"
    )

    ET.SubElement(
        hw,
        "plugin"
    ).text = "gz_ros2_control/GazeboSimSystem"

    # --------------------------------------------------------
    # All joints
    # --------------------------------------------------------

    for i in range(NUM_SECTIONS):

        for axis in ["pitch", "yaw"]:

            joint_name = f"{axis}_joint_{i}"

            j_elem = ET.SubElement(
                ros2_control,
                "joint",
                name=joint_name
            )

            # Position command
            cmd = ET.SubElement(
                j_elem,
                "command_interface",
                name="position"
            )

            ET.SubElement(
                cmd,
                "param",
                name="min"
            ).text = str(JOINT_LOWER)

            ET.SubElement(
                cmd,
                "param",
                name="max"
            ).text = str(JOINT_UPPER)

            # Position state
            ET.SubElement(
                j_elem,
                "state_interface",
                name="position"
            )

            # Velocity state
            ET.SubElement(
                j_elem,
                "state_interface",
                name="velocity"
            )


# ============================================================
# GAZEBO ROS 2 CONTROL PLUGIN
# ============================================================

def create_gazebo_plugin(robot):

    gz = ET.SubElement(
        robot,
        "gazebo"
    )

    plugin = ET.SubElement(
    gz,
    "plugin",
    filename="libgz_ros2_control-system.so",
    name="gz_ros2_control::GazeboSimROS2ControlPlugin"
    )    

    ET.SubElement(
        plugin,
        "parameters"
    ).text = CONTROLLER_CONFIG


# ============================================================
# GENERATE URDF
# ============================================================

def create_urdf():

    print("=" * 60)
    print("Generating tapered octopus arm URDF")
    print("=" * 60)

    robot = ET.Element(
        "robot",
        name="tapered_octopus_arm"
    )

    # ========================================================
    # WORLD
    # ========================================================

    create_world_link(robot)

    # ========================================================
    # BASE
    # ========================================================

    create_base_link(robot)

    create_world_joint(robot)

    # ========================================================
    # ARM SECTIONS
    # ========================================================

    prev_link = "base_link"

    for i in range(NUM_SECTIONS):

        scale = get_scale(i)

        print(
            f"Section {i:02d} | "
            f"Scale = {scale:.4f}"
        )

        # ----------------------------------------------------
        # Virtual link
        # ----------------------------------------------------

        virtual_link = create_virtual_link(
            robot,
            i
        )

        # ----------------------------------------------------
        # Real mesh link
        # ----------------------------------------------------

        real_link = create_segment_link(
            robot,
            i,
            scale
        )

        # ----------------------------------------------------
        # Z position
        # ----------------------------------------------------

        z_pos = get_relative_z_offset(i)

        # ----------------------------------------------------
        # Pitch
        # ----------------------------------------------------

        create_pitch_joint(
            robot,
            i,
            prev_link,
            virtual_link,
            z_pos
        )

        # ----------------------------------------------------
        # Yaw
        # ----------------------------------------------------

        create_yaw_joint(
            robot,
            i,
            virtual_link,
            real_link
        )

        # Next parent
        prev_link = real_link

    # ========================================================
    # ROS 2 CONTROL
    # ========================================================

    create_ros2_control(robot)

    # ========================================================
    # GAZEBO PLUGIN
    # ========================================================

    create_gazebo_plugin(robot)

    # ========================================================
    # PRETTY XML
    # ========================================================

    xml_string = ET.tostring(
        robot,
        encoding="unicode"
    )

    pretty_xml = xml.dom.minidom.parseString(
        xml_string
    ).toprettyxml(
        indent="  "
    )

    # Remove excessive blank lines
    pretty_xml = "\n".join(
        line
        for line in pretty_xml.splitlines()
        if line.strip()
    )

    # ========================================================
    # OUTPUT
    # ========================================================

    output_file = "tapered_octopus_arm.urdf"

    with open(
        output_file,
        "w"
    ) as f:

        f.write(pretty_xml)

    print()
    print("=" * 60)
    print("URDF generated successfully!")
    print("=" * 60)
    print(f"Output: {os.path.abspath(output_file)}")
    print(f"Sections: {NUM_SECTIONS}")
    print(f"Joints: {NUM_SECTIONS * 2}")
    print()
    print("Links:")
    print("  world")
    print("  base_link")
    print(f"  virtual_link_0 ... virtual_link_{NUM_SECTIONS - 1}")
    print(f"  segment_0 ... segment_{NUM_SECTIONS - 1}")
    print()
    print("Joint range:")
    print(f"  Pitch: {JOINT_LOWER} to {JOINT_UPPER} rad")
    print(f"  Yaw:   {JOINT_LOWER} to {JOINT_UPPER} rad")
    print("=" * 60)


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    create_urdf()

