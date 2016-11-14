from stl import mesh
import math
import numpy
import os
# Optionally render the rotated cube faces
from matplotlib import pyplot
from mpl_toolkits import mplot3d

os.chdir('/home/robo/tmp/model/')
import xml.etree.ElementTree
e = xml.etree.ElementTree.parse('qual1.world').getroot()
world =e.find('world')
leds=[]
for model in world.findall('model'):
    if model.get('name') == 'console1':
        
        for link in model.findall('link'):
            if link.get('name') == 'visuals':
                for light in link.findall('visual'):
                    #print light.get('name')[:5]
                    #print ''
                    if light.get('name')[:5] == 'light':
                        
                        pose = light.find('pose').text
                        size = light.find('geometry').find('box').find('size').text
                        leds.append((pose,size))
                        #print pose,size
#print 'leds\n',leds
# Create a new plot
figure = pyplot.figure()
axes = mplot3d.Axes3D(figure)

i=0

for led in leds:

    i+=1
    #print led[0].split(' '),led[1].split(' ')

    px,py,pz,roll,pitch,yaw = led[0].split(' ')
    px,py,pz,roll,pitch,yaw = float(px),float(py),float(pz),float(roll),float(pitch),float(yaw)
    x,y,z = led[1].split(' ')

    x,y,z = float(x),float(y),float(z)

    #print x,y,z
    '''
    x=0.7391
    y=0.494651008
    z=0.1

    px=-0.49349
    py=-0.24203
    pz=1.00454
    roll=0.42
    pitch=0
    yaw=0
    '''
    #x = x+px
    #y=y+py
    #z=pz

    #print 'z',z

    # Create 3 faces of a cube
    data = numpy.zeros(6, dtype=mesh.Mesh.dtype)

    # Top of the cube
    data['vectors'][0] = numpy.array([[0, y, z],
                                      [x, 0, z],
                                      [0, 0, z]])
    data['vectors'][1] = numpy.array([[x, 0, z],
                                      [0, y, z],
                                      [x, y, z]])
    '''
    # Right face
    data['vectors'][2] = numpy.array([[1, 0, 0],
                                      [1, 0, 1],
                                      [1, 1, 0]])
    data['vectors'][3] = numpy.array([[1, 1, 1],
                                      [1, 0, 1],
                                      [1, 1, 0]])
    # Left face
    data['vectors'][4] = numpy.array([[0, 0, 0],
                                      [1, 0, 0],
                                      [1, 0, 1]])
    data['vectors'][5] = numpy.array([[0, 0, 0],
                                      [0, 0, 1],
                                      [1, 0, 1]])
    '''
    # Since the cube faces are from 0 to 1 we can move it to the middle by
    # substracting .5
    #data['vectors'] -= .5

    # Generate 4 different meshes so we can rotate them later
    meshes = [mesh.Mesh(data.copy()) for _ in range(1)]
    #meshes = mesh.Mesh(data.copy())
    # Rotate 90 degrees over the Y axis
    #meshes[0].rotate([pitch, roll, yaw], math.radians(0))
    if roll!=0:
        meshes[0].rotate([0.5, 0, 0], -roll)
    if pitch!=0:
        meshes[0].rotate([0.0, 0.5, 0], -pitch)
    if yaw!=0:
       meshes[0].rotate([0.0, 0, 0.5], -yaw)
    meshes[0].translate([px,py,pz])
    #meshes[0].rotate([0.0, 0.5, 0.0], math.radians(45))

    # Render the cube faces
    for m in meshes:
        axes.add_collection3d(mplot3d.art3d.Poly3DCollection(m.vectors))
        break

    # Auto scale to the mesh size
    scale = numpy.concatenate([m.points for m in meshes]).flatten(-1)
    axes.auto_scale_xyz(scale, scale, scale)

    # Show the plot to the screen
pyplot.show()
