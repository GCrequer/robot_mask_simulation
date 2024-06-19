import xml.etree.ElementTree as ET

def create_calibration_robot_file(name):
    tree = ET.parse(name)
    root = tree.getroot()

    camera_block_dynamic = """
    <body name="camera_block" pos="0 0 0" >
        <inertial pos="0 0 0" mass="0.1" diaginertia="0.001 0.001 0.001" />
        <joint name="slide_x" type="slide" axis="1 0 0"/>
        <joint name="slide_y" type="slide" axis="0 1 0"/>
        <joint name="slide_z" type="slide" axis="0 0 1"/>
        <joint name="roll" type="hinge" axis="1 0 0"/>
        <joint name="pitch" type="hinge" axis="0 1 0"/>
        <joint name="yaw" type="hinge" axis="0 0 1"/>
        <geom type="box" size="0.05 0.05 0.05" material="red" class="visual"/>

        <camera name="front" pos="0 0 0" quat="1 0 1 0" fovy="90"/>
        <camera name="back" pos="0 0 0" quat="1 0 -1 0" fovy="90"/>
        <camera name="top" pos="0 0 0" quat="1 1 0 0" fovy="90"/>
        <camera name="bottom" pos="0 0 0" quat="1 -1 0 0" fovy="90"/>
        <camera name="left" pos="0 0 0" quat="0 1 0 0" fovy="90"/>
        <camera name="right" pos="0 0 0" quat="1 0 0 0" fovy="90"/>
    </body>
    """

    site = root.find(".//body[@name='wrist_3_link']")
    camera_block = ET.fromstring(camera_block_dynamic)
    site.append(camera_block)

    actuator = ET.Element("general", {"class": "size1", "name": "slide_x", "joint": "slide_x"})
    root.find(".//actuator").append(actuator)
    actuator = ET.Element("general", {"class": "size1", "name": "slide_y", "joint": "slide_y"})
    root.find(".//actuator").append(actuator)
    actuator = ET.Element("general", {"class": "size1", "name": "slide_z", "joint": "slide_z"})
    root.find(".//actuator").append(actuator)
    actuator = ET.Element("general", {"class": "size1", "name": "roll", "joint": "roll"}) 
    root.find(".//actuator").append(actuator)
    actuator = ET.Element("general", {"class": "size1", "name": "pitch", "joint": "pitch"})
    root.find(".//actuator").append(actuator)
    actuator = ET.Element("general", {"class": "size1", "name": "yaw", "joint": "yaw"})
    root.find(".//actuator").append(actuator)

    for key in root.findall(".//keyframe/key"):
        key.set("ctrl", key.get("ctrl") + " 0 0 0 0 0 0")
        key.set("qpos", key.get("qpos") + " 0 0 0 0 0 0")

    robot_name = name.split(".")[0] + "_calibration.xml"
    tree.write(robot_name)
    return robot_name

def create_calibrated_robot_file(name, pos, euler):
    tree = ET.parse(name)
    root = tree.getroot()

    camera_block_static = f"""
    <body name="camera_block" pos="{pos[0]} {pos[1]} {pos[2]}" euler="{euler[0]} {euler[1]} {euler[2]}" >
        <inertial pos="0 0 0" mass="0.1" diaginertia="0.001 0.001 0.001" />
        <geom type="box" size="0.1 0.1 0.1" material="invisible" class="visual"/>
        <camera name="front" pos="0 0 0" quat="1 0 1 0" fovy="90"/>
        <camera name="back" pos="0 0 0" quat="1 0 -1 0" fovy="90"/>
        <camera name="top" pos="0 0 0" quat="1 1 0 0" fovy="90"/>
        <camera name="bottom" pos="0 0 0" quat="1 -1 0 0" fovy="90"/>
        <camera name="left" pos="0 0 0" quat="0 1 0 0" fovy="90"/>
        <camera name="right" pos="0 0 0" quat="1 0 0 0" fovy="90"/>
    </body>
    """
    site = root.find(".//body[@name='wrist_3_link']")
    camera_block = ET.fromstring(camera_block_static)
    site.append(camera_block)

    robot_name = name.split(".")[0] + "_calibrated.xml"
    tree.write(robot_name)
    return robot_name

def create_scene_file(scene_name, robot_name):
    tree = ET.parse(scene_name)
    root = tree.getroot()

    #Trouver le include de l'ancien robot
    include = root.find(".//include")
    include.set("file", robot_name)

    new_scene_name = scene_name.split(".")[0] + "_" + robot_name.split("/")[-1].split(".")[0] + ".xml"
    tree.write(new_scene_name)
    return new_scene_name