#! /usr/bin/env python
import subprocess
import sys

# args[1]:sandwich_name
# args[2]:marker_id
# args[3]:marker_size

args = sys.argv

p = subprocess.Popen('rospack find dulcinea_ur5_perception', shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
path = p.stdout.readline()
path = path.replace('\r','')
path = path.replace('\n','')

filename = 'sandwich_%s_markers.xml' % (args[1])
marker_id = int(args[2])
marker_size = float(args[3])

print 'filename:%s' % (filename)
print 'marker_id:%d' % (marker_id)
print 'marker_size:%d' % (int(marker_size))

f = open(path+'/config/'+filename, 'w')
f.write('<?xml version="1.0" encoding="UTF-8" standalone="no" ?>\r\n')
f.write('<multimarker markers="4">\r\n')

sandwich_width = 7.0 #[cm]

for value in range(0, 4):
    # x:depth y:width z:height
    str = '\t<marker index="%d" status="%d">\r\n' % (int(marker_id+value), int(value+1))

    if value == 0:
        # back
        str += '<!-- back -->\r\n'
        x = 0
        y = 0
        z = 3 + marker_size / 2
        # lower left
        str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x, y - marker_size / 2, z - marker_size / 2)
        # lower right
        str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x, y + marker_size / 2, z - marker_size / 2)
        #  upper right
        str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x, y + marker_size / 2, z + marker_size / 2)
        #  upper left
        str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x, y - marker_size / 2, z + marker_size / 2)
    elif value == 1:
        # right from front
        str += '<!-- right from front -->\r\n'
        x = - marker_size / 2
        y = - sandwich_width / 2.0
        z = 3 + marker_size / 2
        # lower left
        str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x - marker_size / 2, y, z - marker_size / 2)
        # lower right
        str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x + marker_size / 2, y, z - marker_size / 2)
        #  upper right
        str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x + marker_size / 2, y, z + marker_size / 2)
        #  upper left
        str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x - marker_size / 2, y, z + marker_size / 2)
    elif value == 2:
        # left from front
        str += '<!-- left from front -->\r\n'
        x = - marker_size / 2
        y = sandwich_width / 2.0
        z = 3 + marker_size / 2
        # lower left
        str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x + marker_size / 2, y, z - marker_size / 2)
        # lower right
        str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x - marker_size / 2, y, z - marker_size / 2)
        #  upper right
        str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x - marker_size / 2, y, z + marker_size / 2)
        #  upper left
        str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x + marker_size / 2, y, z + marker_size / 2)
    elif value == 3:
        # bottom
        str += '<!-- bottom -->\r\n'
        x = -3 - marker_size / 2
        y = 0
        z = 0
        str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x + marker_size / 2, y + marker_size / 2, z)
        # lower right
        str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x + marker_size / 2, y - marker_size / 2, z)
        #  upper right
        str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x - marker_size / 2, y - marker_size / 2, z)
        #  upper left
        str += '\t\t<corner x="%f" y="%f" z="%f"/>\r\n' % (x - marker_size / 2, y + marker_size / 2, z)
    str += '\t</marker>\r\n'
    f.write(str)

f.write('</multimarker>\r\n')
f.close()
