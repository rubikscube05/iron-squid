import xml.etree.ElementTree as ET
import xml.dom.minidom

NUM_SECTIONS = 20
BASE_TOP_H = 0.030
BASE_BOT_H = 0.030
RESTING_GAP = 0.002
BASE_MASS = 0.1

# Matches the updated 35% minimum taper constraint
def get_scale(i):
    return 1.0 - (i * (0.65 / (NUM_SECTIONS - 1)))

# Solves the exponential spacing issue using relative offset
def get_relative_z_offset(i):
    if i == 0:
        return 0.0
    prev_top = BASE_TOP_H * get_scale(i - 1)
    curr_bot = BASE_BOT_H * get_scale(i)
    return prev_top + curr_bot + RESTING_GAP

def create_urdf(): 
    # Initialize the root element first
    robot = ET.Element("robot", name="tapered_octopus_arm")
    
    # Static Base Link (Pedestal)
    base_link = ET.SubElement(robot, "link", name="base_link")
    base_vis = ET.SubElement(base_link, "visual")
    ET.SubElement(base_vis, "origin", xyz="0 0 -0.05", rpy="0 0 0")
    b_geom = ET.SubElement(base_vis, "geometry")
    ET.SubElement(b_geom, "box", size="0.2 0.2 0.1")
    
    base_col = ET.SubElement(base_link, "collision")
    ET.SubElement(base_col, "origin", xyz="0 0 -0.05", rpy="0 0 0")
    b_cgeom = ET.SubElement(base_col, "geometry")
    ET.SubElement(b_cgeom, "box", size="0.2 0.2 0.1")
    
    base_inertial = ET.SubElement(base_link, "inertial")
    ET.SubElement(base_inertial, "mass", value="10.0")
    ET.SubElement(base_inertial, "inertia", ixx="1.0", ixy="0", ixz="0", iyy="1.0", iyz="0", izz="1.0")

    # Fixed joint connecting base to world
    world_joint = ET.SubElement(robot, "joint", name="world_fixed", type="fixed")
    ET.SubElement(world_joint, "parent", link="world")
    ET.SubElement(world_joint, "child", link="base_link")
    ET.SubElement(world_joint, "origin", xyz="0 0 0", rpy="0 0 0")

    prev_link = "base_link"
     
    robot = ET.Element("robot", name="tapered_octopus_arm")
    ET.SubElement(robot, "link", name="world")
    prev_link = "world"
    
    for i in range(NUM_SECTIONS):
        scale = get_scale(i)
        seg_mass = BASE_MASS * (scale ** 3)
        
        virtual_link = f"virtual_link_{i}"
        real_link = f"segment_{i}"
        
        v_link = ET.SubElement(robot, "link", name=virtual_link)
        ET.SubElement(v_link, "inertial").extend([
            ET.Element("mass", value="0.001"),
            ET.Element("inertia", ixx="0.0001", ixy="0", ixz="0", iyy="0.0001", iyz="0", izz="0.0001")
        ])
        
        r_link = ET.SubElement(robot, "link", name=real_link)
        for tag in ["visual", "collision"]:
            vis_col = ET.SubElement(r_link, tag)
            ET.SubElement(vis_col, "origin", xyz="0 0 0", rpy="0 0 0")
            geom = ET.SubElement(vis_col, "geometry")
            # Fixes the NAN collision errors by scaling the mesh
            ET.SubElement(geom, "mesh", filename=f"file:///home/vaibhav/doc_oct/src/octopus_description/meshes/segment_{i}.stl", scale="0.001 0.001 0.001")
            
        inertial = ET.SubElement(r_link, "inertial")
        ET.SubElement(inertial, "mass", value=str(round(seg_mass, 4)))
        inertia_val = str(round((2.0/5.0) * seg_mass * ((0.05 * scale)**2), 6))
        ET.SubElement(inertial, "inertia", ixx=inertia_val, ixy="0", ixz="0", iyy=inertia_val, iyz="0", izz=inertia_val)

        z_pos = get_relative_z_offset(i)
        
        pitch_joint = ET.SubElement(robot, "joint", name=f"pitch_joint_{i}", type="revolute")
        ET.SubElement(pitch_joint, "parent", link=prev_link)
        ET.SubElement(pitch_joint, "child", link=virtual_link)
        ET.SubElement(pitch_joint, "origin", xyz=f"0 0 {z_pos}", rpy="0 0 0")
        ET.SubElement(pitch_joint, "axis", xyz="1 0 0")
        ET.SubElement(pitch_joint, "limit", lower="-0.5", upper="0.5", effort="10.0", velocity="2.0")
        ET.SubElement(pitch_joint, "dynamics", damping="0.5", friction="0.1") 

        yaw_joint = ET.SubElement(robot, "joint", name=f"yaw_joint_{i}", type="revolute")
        ET.SubElement(yaw_joint, "parent", link=virtual_link)
        ET.SubElement(yaw_joint, "child", link=real_link)
        ET.SubElement(yaw_joint, "origin", xyz="0 0 0", rpy="0 0 0")
        ET.SubElement(yaw_joint, "axis", xyz="0 1 0")
        ET.SubElement(yaw_joint, "limit", lower="-0.5", upper="0.5", effort="10.0", velocity="2.0")
        ET.SubElement(yaw_joint, "dynamics", damping="0.5", friction="0.1")
        
        prev_link = real_link

    # ROS 2 Control Hardware Interface
    ros2_control = ET.SubElement(robot, "ros2_control", name="GazeboSimSystem", type="system")
    hw = ET.SubElement(ros2_control, "hardware")
    ET.SubElement(hw, "plugin").text = "gz_ros2_control/GazeboSimSystem"
    
    for i in range(NUM_SECTIONS):
        for axis in ["pitch", "yaw"]:
            joint_name = f"{axis}_joint_{i}"
            j_elem = ET.SubElement(ros2_control, "joint", name=joint_name)
            cmd = ET.SubElement(j_elem, "command_interface", name="position")
            ET.SubElement(cmd, "param", name="min").text = "-0.5"
            ET.SubElement(cmd, "param", name="max").text = "-0.5"
            ET.SubElement(j_elem, "state_interface", name="position")
            ET.SubElement(j_elem, "state_interface", name="velocity")
            
    gz = ET.SubElement(robot, "gazebo")
    plugin = ET.SubElement(gz, "plugin", filename="gz_ros2_control-system", name="gz_ros2_control::GazeboSimROS2ControlPlugin")
    ET.SubElement(plugin, "parameters").text = "$(find octopus_description)/config/arm_controllers.yaml"

    xml_str = xml.dom.minidom.parseString(ET.tostring(robot)).toprettyxml(indent="  ")
    with open("tapered_octopus_arm.urdf", "w") as f:
        f.write(xml_str)
    print("URDF generated successfully!")

if __name__ == "__main__":
    create_urdf()
