#! /usr/bin/env python
import subprocess
import sys

# args[1]:shelf_name
# args[2]:marker_id
# args[3]:marker_size

args = sys.argv

p = subprocess.Popen('rospack find fcsc_perception', shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
path = p.stdout.readline()
path = path.replace('\r','')
path = path.replace('\n','')

filename = 'shelf_%s_markers.xml' % (args[1])
marker_id = int(args[2])
marker_size_cm = float(args[3])

print 'filename:%s' % (filename)
print 'marker_size:%d' % (int(marker_size_cm))
print 'marker_id:%d' % (marker_id)

# fcsc shelf
shelf_width_cm = 90.0
shelf_depth_cm = 43.5
# shelf_depth_cm = 40.0
shelf_lower_height_cm = 58.0
# shelf_stopper_height_cm = 3.5
shelf_stopper_height_cm = 0
ar_marker_thickness_cm = 0.3
offset_cm = 30
# x = -23
# z = 61

# kdel
# shelf_width_cm = 85.4
# shelf_depth_cm = 44.7
# # shelf_depth_cm = 37.7
# shelf_lower_height_cm = 59.0
# shelf_stopper_height_cm = 0.0
# ar_marker_thickness_cm = 0.3
# offset_cm = 30

# daihen
# shelf_width_cm = 85.4
# shelf_depth_cm = 45
# # shelf_depth_cm = 37.7
# shelf_lower_height_cm = 70
# shelf_stopper_height_cm = 0.0
# ar_marker_thickness_cm = 0.3
# offset_cm = 23

f = open(path+'/config/'+filename, 'w')
f.write('<?xml version="1.0" encoding="UTF-8" standalone="no" ?>\r\n')

x = shelf_depth_cm / 2.0 - ar_marker_thickness_cm
z = shelf_lower_height_cm + marker_size_cm / 2.0 + offset_cm
f.write('<multimarker markers="1">\r\n')
for value in range(0, 1):
    if value == 0:
        # y = 25.0
        y = 0
    elif value == 1:
        # y = -15.0
        y = 0.0
    elif value == 2:
        # y = -45.0
        y = -45.0 + marker_size_cm / 2.0
    str = '\t<marker index="%d" status="%d">\r\n' % (int(marker_id+value), int(value+1))
    str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x, y+marker_size_cm/2, z-marker_size_cm/2)
    str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x, y-marker_size_cm/2, z-marker_size_cm/2)
    str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x, y-marker_size_cm/2, z+marker_size_cm/2)
    str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x, y+marker_size_cm/2, z+marker_size_cm/2)
    str += '\t</marker>\r\n'
    f.write(str)


# lower stopper

# x = -shelf_depth_cm / 2.0 - ar_marker_thickness_cm
# z = shelf_lower_height_cm + shelf_stopper_height_cm - marker_size_cm / 2.0
# f.write('<multimarker markers="3">\r\n')
# for value in range(0, 3):
#     if value == 0:
#         # y = 25.0
#         y = 45.0 - marker_size_cm / 2.0
#     elif value == 1:
#         # y = -15.0
#         y = 0.0
#     elif value == 2:
#         # y = -45.0
#         y = -45.0 + marker_size_cm / 2.0
#     str = '\t<marker index="%d" status="%d">\r\n' % (int(marker_id+value), int(value+1))
#     str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x, y+marker_size_cm/2, z-marker_size_cm/2)
#     str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x, y-marker_size_cm/2, z-marker_size_cm/2)
#     str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x, y-marker_size_cm/2, z+marker_size_cm/2)
#     str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x, y+marker_size_cm/2, z+marker_size_cm/2)
#     str += '\t</marker>\r\n'
#     f.write(str)

f.write('</multimarker>\r\n')
f.close()
