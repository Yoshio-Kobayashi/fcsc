#! /usr/bin/env python
import subprocess
import sys
import math

# args[1]:sandwich_name
# args[2]:marker_id
# args[3]:marker_size

args = sys.argv

p = subprocess.Popen('rospack find dulcinea_ur5_perception', shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
path = p.stdout.readline()
path = path.replace('\r','')
path = path.replace('\n','')

filename = 'tf_from_ar_marker_to_sandwich.yaml'
marker_size = float(args[1]) / 100.0
sandwich_height = 0.1
# sandwich_height = 0.14
sandwich_depth = 0.09
sandwich_width = 0.07
sandwich_edge = math.sqrt(sandwich_height * sandwich_height + sandwich_depth * sandwich_depth)

print 'filename:%s' % (filename)
print 'marker_size:%f' % (marker_size)

f = open(path+'/config/'+filename, 'w')
f.write('tf_from_ar_marker_to_sandwich:\r\n')
f.write('  markers:\r\n')

for value in range(0, 3):
    if value == 0:
        # front
        str = '    front:\r\n'
        # str = '    # front:\r\n'
        x = -(marker_size / 2) * sandwich_depth / sandwich_edge
        # x = -(marker_size / 2) * math.cos(math.pi / 3.0)
        y = 0.0
        z = sandwich_height + (marker_size / 2.0) * sandwich_height / sandwich_edge
        # z = sandwich_height + (marker_size / 2.0) * math.sin(math.pi / 3.0)
        roll = math.atan(sandwich_height / sandwich_depth)
        print 'sandwich angle:%f' % (roll * 180.0 / math.pi)
        # roll = math.pi / 3.0
        pitch = 0.0
        yaw = -math.pi / 2
    elif value == 1:
        str = '    back:\r\n'
        # str = '    # back:\r\n'
        x = 0.0
        y = 0.0
        z = sandwich_height - (marker_size / 2.0)
        roll = math.pi / 2.0
        pitch = 0.0
        yaw = math.pi / 2.0
    elif value == 2:
        str = '    bottom:\r\n'
        # str = '    # bottom:\r\n'
        x = - sandwich_depth + marker_size / 2.0
        y = 0.0
        z = 0.0
        roll = 0.0
        pitch = math.pi
        yaw = math.pi / 2.0
    # str += '    -\r\n'
    str += '      translation:\r\n'
    str += '        x: %f\r\n' % (x)
    str += '        y: %f\r\n' % (y)
    str += '        z: %f\r\n' % (z)
    str += '      rotation:\r\n'
    str += '        roll: %f\r\n' % (roll)
    str += '        pitch: %f\r\n' % (pitch)
    str += '        yaw: %f\r\n' % (yaw)
    f.write(str)
f.close()
