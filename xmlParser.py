import xml.etree.ElementTree as ET
import numpy as np
import pinocchio.rpy

partPath = "/home/hvanoverlo/devel/DensoAirfoil/objects/wing.kinbody.xml"
scaleCoeff = 3.

tree = ET.parse(partPath)
root = tree.getroot()

### RETRIEVE THE DATA

# THE PART
part = root[0]
part = dict()
part["file"] = "/home/hvanoverlo/devel/DensoAirfoil/"+root[0][0][0].text.split()[0]
part["scale"] = float(root[0][0][0].text.split()[1])*scaleCoeff
part["color"] = root[0][0][2].text.split()
part["position"] = root[0][1].text.split()

# THE TASKS
tasks = list()
for t in root[1:]:
    task = dict()
    task["name"] = t.attrib["name"]
    task["pos"] = list(map(float, t[1].text.split()))
    for i in range(3):
        task["pos"][i]*=scaleCoeff
    task["rot"] = list(map(float, t[2].text.split()))

    r = np.array([np.array([np.cos(np.pi*task["rot"][3]/180), -np.sin(np.pi*task["rot"][3]/180), 0]),
                  np.array([np.sin(np.pi*task["rot"][3]/180), np.cos(np.pi*task["rot"][3]/180), 0]),
                  np.array([0., 0., 1.])])
    task["rpy"] = pinocchio.rpy.matrixToRpy(r)
    tasks.append(task)



### PUT IT INTO A URDF

u = open("/home/hvanoverlo/devel/hpp/src/agimus-demos/tiago/deburring/urdf/airfoil.urdf", "w")
u.write("""<robot name="Airfoil">\n\n""")

# BASE LINK
baseLinkFmt = """  <link name="base_link">\n    <visual>\n      <origin xyz="0 0 -0.8" rpy="0 0 0" />\n      <geometry>\n        <mesh filename="{0}" scale="{1} {1} {1}"/>\n      </geometry>\n       <material name="Green">\n         <color rgba="0.6 1. 0.65 1.0"/>\n       </material>\n    </visual>\n    <collision>\n      <origin xyz="0 0 0" rpy="0 0 0" />\n      <geometry>\n        <mesh filename="{0}"  scale="{1} {1} {1}"/>\n      </geometry>\n    </collision>\n"""

intermdiatePart = """<collision>\n      <origin xyz="0.05 -0.0422962331772 0.0814425373077" />\n      <geometry>\n        <box size="0.6 0.15 0.15"/>\n      </geometry>\n    </collision>\n"""

endFmt = """    <inertial>\n      <mass value="2.2"/>\n      <origin xyz="0 0 -0.8" rpy="0 0 0"/>\n      <inertia ixx="0.1"  ixy="0"  ixz="0" iyy="0.1" iyz="0" izz="0.1" />\n    </inertial>\n  </link>\n\n"""

u.write(baseLinkFmt.format(part['file'], part['scale']))
# u.write(intermdiatePart) # necessary collision ?
u.write(endFmt)

# THE TASKS
taskLinkFmt = """  <link name="{}_link">\n    <visual>\n      <geometry>\n        \<sphere radius="0.005" />\n      </geometry>\n    </visual>\n  </link>\n"""
taskJointFmt = """  <joint name="to_{0}" type="fixed">\n    <parent link="base_link"/>\n    <child link="{0}_link"/>\n    <origin xyz="{1} {2} {3}"  rpy="{4} {5} {6}"/>\n  </joint>\n"""

for t in tasks:
    u.write(taskLinkFmt.format(t['name']))
    u.write(taskJointFmt.format(t['name'], t["pos"][0], t["pos"][1], t["pos"][2],
                                t["rpy"][0], t["rpy"][1], t["rpy"][2]))

# END OF FILE
u.write("</robot>")
u.close()


### WRITE THE SRDF

s = open("/home/hvanoverlo/devel/hpp/src/agimus-demos/tiago/deburring/srdf/airfoil.srdf", "w")
s.write("""<robot name="Airfoil">\n\n""")

handleFmt = """<handle name="handle_{}" clearance="0.05">\n    <position rpy="1.5707963267948966 0 0"/>\n    <link name="{}_link" />\n    <mask>1 1 1 0 1 1</mask>\n</handle>\n\n"""

for t in tasks:
    s.write(handleFmt.format(t['name'][4:], t['name']))

s.write("</robot>")
s.close()
