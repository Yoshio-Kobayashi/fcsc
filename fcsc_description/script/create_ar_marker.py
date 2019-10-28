#! /usr/bin/env python
import subprocess
import sys

args = sys.argv
path = subprocess.Popen('rospack find fcsc_description', shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
path = path.stdout.readline()
path = path.replace('\r','')
path = path.replace('\n','')
for id in range( int(args[1]), int(args[2])+1 ):
    str = """material ar_marker_%d
{
  technique
  {
    pass
    {
      texture_unit
      {
        texture MarkerData_%d.png
        filtering trilinear
      }
    }
  }
}""" % (id, id)
    cmd = 'rosrun ar_track_alvar createMarker %d' % (id)
    subprocess.call( cmd, shell=True  )
    filename = 'ar_marker_%d.material' % (id)
    f = open(path+'/media/materials/scripts/'+filename, 'w')
    f.write(str)
    f.close()
cmd = 'mv MarkerData* ' + path + '/media/materials/textures/'
subprocess.call( cmd, shell=True  )
