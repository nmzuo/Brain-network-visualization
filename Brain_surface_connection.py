# -*- coding: utf-8 -*-
"""
Created on Tue Mar 21 21:17:29 2017
@author: nmzuo
The following main features
1. Rendering brain surface and connetions
2. Show and save as off-shelf figure or animation movie
3. Fast rendering since Python/Mayavi call OpenGL

Surface format:
3									# node number
-2.886495 24.886947 15.909558		# node coord
-13.916695 -58.985245 19.655311
-40.415528 8.968353 8.515955
-47.707717 -48.107485 17.630739
-26.480126 44.708967 32.840302
4									# triangle number
1 2 3								# triangle edge
1 3 5
2 3 5
3 4 5

Network format:
vertex  3							# node number
-2.886495 24.886947 15.909558		# node coord
-13.916695 -58.985245 19.655311
-40.415528 8.968353 8.515955
edge 4								# conneciton number
1 2 								# connection
1 3 

"""

#import vtk
#reader=vtk.vtkNIFTIImageReader();
#reader.SetFileName('D:/tmp/MNI152_T1_2mm_brain.nii.gz');
#reader.Update();
#print(reader.GetNIFTIHeader());                  


from mayavi import mlab;
import numpy as np;
from scipy.special import sph_harm;

outfig='D:/flex_8state/data/';

#import imageio
#imageio.plugins.ffmpeg.download()

## what to show
is_LH=0; # Left Hemi
is_RH=1; # Right Hemi
         # both 0 means Both Hemi, otherwise


## 
## Import the surface
with open('D:/flex_8state/code/surf_smoothed.txt', 'r') as infile:  # surf_smoothed.txt
    mysurf=[];
    for i in infile:
        mysurf.append(i);

numVert2=int(mysurf[0]);
vertMat2=np.zeros((numVert2, 3), float);
for i in range(numVert2):
    vertMat2[i,:]=np.array([float(x) for x in mysurf[i+1].replace('\n',' ').split()]);
    
numEdge2=int(mysurf[numVert2+1]);
edgeMat2=np.zeros((numEdge2, 3), int);
for i in range(numEdge2):
    edgeMat2[i,:]=np.array([int(x) for x in mysurf[i+2+numVert2].replace('\n',' ').split()]);
    
import scipy;    
delCount=[];
for i in range(numEdge2):
    xi, yi = edgeMat2[i,0:2]-1;
    xS=vertMat2[xi,0]; xE=vertMat2[yi,0];
    if is_LH:
        if xS>0 or xE>0:
            delCount.append(i);
    elif is_RH:        
        if xS<0 or xE<0:
            delCount.append(i);
    else:
        continue;
        
edgeMat2 = scipy.delete(edgeMat2, delCount, 0);
numEdge2 = edgeMat2.shape[0];
    
print 'Import surf... Finished!';

fig=mlab.figure(1, bgcolor=(1, 1, 1), fgcolor=(0, 0, 0), size=(700, 700));
mlab.clf();
surf=mlab.triangular_mesh(vertMat2[:,0], vertMat2[:,1], vertMat2[:,2],edgeMat2-1, opacity=1.0,  \
                          color=(0.8,0.8,0.8)); #Greys
surf.actor.property.lighting = True;
camera_light0 = fig.scene.light_manager.lights[0]
camera_light0.elevation = 40;
camera_light0.azimuth = 5;
camera_light0.intensity = 1.0;                    
         
#    # Scene lights (4 lights are used)
#    engine = mlab.get_engine();
#    scene = engine.current_scene;
#    cam_light_azimuth = [0, 0, 0, 0];
#    cam_light_elevation = [90, 90, 40, -60];
#    cam_light_intensity = [0.7, 0.7, 0.60, 0.20];
#    for i in range(0):
#        camlight = scene.scene.light_manager.lights[i];
#        camlight.activate = True;
#        camlight.azimuth = cam_light_azimuth[i];
#        camlight.elevation = cam_light_elevation[i];
#        camlight.intensity = cam_light_intensity[i];

#netFile='membership_4vis_ThreshGammaCommon_sameSize_WM.txt';
netFile='revise_avg_totQS_all_bin_consensus_modularity_WM_T5G7S453';

with open('D:/flex_8state/data/'+netFile+'.net', 'r') as infile:
    netlist=[];
    for i in infile:
        netlist.append(i);
        
vertLine=0;
edgeLine=0;  
numVert=0;
numEdge=0;      
for i in range(len(netlist)):
    thisstr=netlist[i].replace('\n',' ').split();
    if thisstr[0] == 'vertex':
        numVert=int(thisstr[1]);
        vertLine=i;
    elif thisstr[0] == 'edge':
        numEdge=int(thisstr[1]);
        edgeLine=i;
    else:
        continue;
#        
#if vertLine==0 and edgeLine==0:
#    print vertLine, edgeLine
#    print 'Invalid network TXT file!';
#    raise SystemExit, 0;
#    


if numVert >0:
    vertMat=np.zeros((numVert, 7), float);
    for i in range(1, numVert+1):
        vertMat[i-1,:]=np.array([float(x) for x in netlist[i+vertLine].replace('\n',' ').split()]);
        
    for i in range(0,numVert):
        if is_LH:
            if vertMat[i,0]<0:
                mlab.points3d(vertMat[i,0],vertMat[i,1],vertMat[i,2],vertMat[i,3]*10, scale_factor=1.0, resolution=32,  \
                      color=(vertMat[i,4],vertMat[i,5],vertMat[i,6]));
        elif is_RH:
            if vertMat[i,0]>0:
                mlab.points3d(vertMat[i,0],vertMat[i,1],vertMat[i,2],vertMat[i,3]*10, scale_factor=1.0, resolution=32,  \
                      color=(vertMat[i,4],vertMat[i,5],vertMat[i,6]));  
        else:
            mlab.points3d(vertMat[i,0],vertMat[i,1],vertMat[i,2],vertMat[i,3]*10, scale_factor=1.0, resolution=32,  \
                      color=(vertMat[i,4],vertMat[i,5],vertMat[i,6]));                        
if numEdge >0:
    edgeMat=np.zeros((numEdge, 6), float);    
    for i in range(1, numEdge+1):
        edgeMat[i-1,:]=np.array([float(x) for x in netlist[i+edgeLine].replace('\n',' ').split()]);       
    for i in range(0,numEdge):
        p1=vertMat[edgeMat[i,0]-1,0:3];
        p2=vertMat[edgeMat[i,1]-1,0:3];
        if is_LH:
            if p1[0]<0 and p2[0]<0:
                mlab.plot3d([p1[0],p2[0]],[p1[1],p2[1]],[p1[2],p2[2]], tube_radius=edgeMat[i,2]/4.0, tube_sides=16,  \
                    color=(edgeMat[i,3],edgeMat[i,4],edgeMat[i,5]));
        elif is_RH:
            if p1[0]>0 and p2[0]>0:
                mlab.plot3d([p1[0],p2[0]],[p1[1],p2[1]],[p1[2],p2[2]], tube_radius=edgeMat[i,2]/4.0, tube_sides=16,  \
                    color=(edgeMat[i,3],edgeMat[i,4],edgeMat[i,5]));
        else:
            mlab.plot3d([p1[0],p2[0]],[p1[1],p2[1]],[p1[2],p2[2]], tube_radius=edgeMat[i,2]/4.0, tube_sides=16,  \
                    color=(edgeMat[i,3],edgeMat[i,4],edgeMat[i,5]));                    

print 'Import network file... Finished!'


#nodes=mlab.points3d(vertMat[:,0],vertMat[:,1],vertMat[:,2],vertMat[:,3]); #,scale_factor=4);
#nodes.glyph.color_mode = 'color_by_vector';
#nodes.mlab_source.dataset.point_data.vectors = vertMat[:,4:7];
#nodes.mlab_source.update();


mlab.view(0,90);
mlab.savefig(outfig+netFile+'_RH_lat.tiff', magnification=2);
mlab.show();
#
raise SystemExit, 0;

### tile the images
#import matplotlib.pyplot as plt
#import matplotlib.image as mpimg
#img1=mpimg.imread('save_left.tiff');
#img2=mpimg.imread('save_right.tiff');
#fig=plt.figure();
#ax1=fig.add_subplot(1,2,1);
#ax1.imshow(img1);
#ax1.set_axis_off();
#ax2=fig.add_subplot(1,2,2);
#ax2.imshow(img2);
#ax2.set_axis_off();
#fig.savefig('save_comb.tif', dpi=300);
#plt.show();


#raise SystemExit, 0;

##########
# The following is to record a movie animation, April 14, 2017
#########

import  moviepy.editor as mpy

duration = 8 # duration of the animation in seconds (it will loop)

# ANIMATE THE FIGURE WITH MOVIEPY, WRITE AN ANIMATED GIF

def make_frame(t):
    """ Generates and returns the frame for time t. """
    mlab.view(360*t/duration, 90) # camera angle
    return mlab.screenshot(antialiased=True) # return a RGB image

animation = mpy.VideoClip(make_frame, duration=duration).resize(0.8)
#animation.write_videofile("wireframe.mp4", fps=20) ## Doesn't work
animation.write_gif(outfig+netFile+".gif", fps=40)


raise SystemExit, 0;




vertColor=np.linalg.norm(vertMat, ord=2, axis=1)
#vertColor=np.abs(vertMat)/np.tile(vertColor.reshape(vertColor.size,1), (1,3));

mesh=mlab.triangular_mesh(vertMat[:,0], vertMat[:,1], vertMat[:,2],edgeMat-1);
mesh.mlab_source.dataset.cell_data.scalars = vertColor;
mesh.mlab_source.dataset.cell_data.scalars.name = 'Cell data';
mesh.mlab_source.update();

mesh2 = mlab.pipeline.set_active_attribute(mesh, cell_scalars='Cell data');
mlab.pipeline.surface(mesh2);


mlab.points3d(50,0,0,5,scale_factor=1, opacity=0.1, color=(0.5,0,0));

conn=np.array([[40,-20],[-30,40],[30,-20]]);
mlab.points3d(conn[0,:], conn[1,:], conn[2,:],[8,6],scale_factor=1, color=(0.5,0,0));
mlab.plot3d(conn[0,:], conn[1,:], conn[2,:], color=(0,1,1), tube_radius=1.5);

mlab.view(0,90);
mlab.show()



print "Finished!"


# Create a sphere
r = 0.3
pi = np.pi
cos = np.cos
sin = np.sin
phi, theta = np.mgrid[0:pi:101j, 0:2 * pi:101j]

x = r * sin(phi) * cos(theta)
y = r * sin(phi) * sin(theta)
z = r * cos(phi)

mlab.figure(1, bgcolor=(1, 1, 1), fgcolor=(0, 0, 0), size=(400, 300))
mlab.clf()
# Represent spherical harmonics on the surface of the sphere
for n in range(1, 6):
    for m in range(n):
        s = sph_harm(m, n, theta, phi).real

        mlab.mesh(x - m, y - n, z, scalars=s, colormap='jet')

        s[s < 0] *= 0.97

        s /= s.max()
        mlab.mesh(s * x - m, s * y - n, s * z +1.3,
                  scalars=s, colormap='Spectral')

mlab.view(90, 70, 6.2, (-1.3, -2.9, 0.25))
mlab.show()