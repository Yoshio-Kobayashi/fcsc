import sys

f = open('spawn_ar_marker.launch', 'w')
f.write('<launch>\r\n')

args = sys.argv
for id in range( int(args[1]), int(args[2])+1 ):
    num = -1.0 * (float(args[2]) - float(args[1]) + 1.0) / 2.0
    str = """<include file="$(find fcsc_description)/launch/spawn_objects_urdf.launch">
        <arg name="use_ar_marker" value="true"/>
        <arg name="use_xacro" value="true"/>
        <arg name="model" default="$(find fcsc_description)/model/ar_marker.urdf.xacro"/>
        <arg name="x" value="0"/>
        <arg name="y" value="%f"/>
        <arg name="z" value="0"/>
        <arg name="marker_id" value="%d"/>
        <arg name="name" value="ar_marker_%d"/>
      </include>\r\n""" % (float(id + num), id, id)
    f.write(str)

f.write('</launch>')
f.close()
