## 3D Optical Simulation.
![screenshot](https://github.com/wezu/raychaser/blob/master/screen.png)

You will need Python(3.x) and Panda3D to run this program. You can install Panda3D using pip:
```
pip install panda3d
```
A 'compiled', self contained, portable version of the program can also be created for Windows and Linux (and in theory Mac - I do not own any Apple products and can't say if it will actually work) - if someone would request one.

To run the program simply run the main.py script.

This readme file is also used as in-app help, so some text formating may be displayed incorrectly when viewed on Github.  

# General info
Features:  
Raychaser is an application build to visualize optical phenomena such as refraction and reflection in a 3D environment. It allows to import meshes (in various popular file formats via Assimp) to act as mirrors(reflect), lenses(refract), beam splitters (reflect and refract) or screens (absorb). One can create light rays (or arrays of multiple rays) and projectors. Each object can have a custom Index Of Refraction (IOR) or a real world material, rays can have specific wavelengths. Every object can be freely manipulated (moved, rotated, scaled) and viewed from any angle. The application also allows saving and loading of scenes.

User Interface:  
The buttons in the upper left corner are associated with changing the scene (save, load, create or select objects). The buttons in the lower left corner are associated with manipulating objects (move, rotate, scale). The buttons on the lower right are associated with the view (grid, camera options). Object properties are always shown on the right side of the screen (when on object is selected).

Widgets with white text are input fields, you can click on them to type in values. Input fields that take numerical values are interpreted, that means you can type in simple mathematical expressions (eg. 1+2*3) and skip decimals (the field may display for example 3.00, but you may simply enter 3). Double clicking on an input field will erase the old value. Values are applied when you click on a different input field or press Enter.

Some buttons are toggleable. Grey text indicates that the button (feature) is in Off state. For example if the Grid button is grey then the grid is hidden, if the button has green text then the grid is visible.

# Camera control
Camera Menu  
The camera menu is accessible by clicking the Camera button near on the bottom left of the window. The option there are:

* Move to object - clicking this button will move the camera to the currently selected object (if an object is selected).
* Relative pan - When this option is off, the camera will only move on the XY-plane when using the mouse to move the camera, that is moving the mouse up or down will move the camera forward and back. When this option is on, the motion will be relative to the camera itself. By default this option is off.
* Focus Widget - Clicking this option will either show or hide the Focus Widget - a small, greyish circle showing the point in space around which the camera orbits.
* Orographic - Clicking this option will switch between orographic and perspective camera view mode.
* Reset - This option will reset the position and orientation of the camera.

Keyboard controls:  
* W - move camera forward
* A - move camera left
* S - move camera back
* D - move camera right
* Q - move camera up
* Z - move camera down
* UP ARROW - view from top
* DOWN ARROW - view from bottom
* LEFT ARROW - view from left
* RIGHT ARROW - view from right
* HOME  - view from front
* END - view from back

Mouse control:  
* RIGHT MOUSE BUTTON - hold and move mouse to rotate camera
* SHIFT + LEFT MOUSE BUTTON - hold and move mouse to rotate camera
* MOUSE WHEEL - camera zoom in/out

# Creating and Selecting objects
Creating Objects:  
Objects can be created using the Create menu located at the top left corner of the window. Once clicked it will show a drop-down menu with the fallowing options:

Ray  
Creates a Ray object with some default values. See 'Object properties' and 'Object types' help topics for more information.

Projector  
Creates a Projector object with some default values. See 'Object properties' and 'Object types' help topics for more information.

Import...  
Selecting this option will show a new panel with a list of available models, and some extra options. 
The program will look in models/absorb/, models/beamsplit/, models/reflect/ and models/refract/ for any 3d models in any of the supported model file formats. The initial properties (type) of a model are determined depending on its location (eg. models loaded from models/beamsplit/ will both reflect and refract light rays), but these properties may later be changed see 'Object properties'. The list of available models can be sorted by name or type by clicking on the Name or Type buttons, clicking these again will sort the models in reverse order.
The buttons at the top of the panel allow fixing common problems when importing models in non-native formats (native formats are: egg, bam). When the Flip normal mode is active (has green text) all imported models will be flipped inside-out. When the Write bam mode is active, the program will write a bam file that can be loaded in the future. Bam files will load faster then any other format. 
The input field labeled 'Smooth' controls the look of the model, with a value of 40 the model will look smooth, with a value of 0 it will have sharp edges (per polygon normals).      
  
Selecting Objects:  
Objects can be selected in two ways, either by clicking on them or by selecting them from a list. 
Selected object have a green outline, usually selecting an object will show its properties.

Clicking on objects is the most intuitive way to select objects. Just click on an object. Objects that are frozen can not be selected this way. If objects overlap, then the outermost object gets selected.

In situations where clicking on an object to select it is difficult or impossible, you can select an object using the object list. Clicking on the Select button, located near the top left of the window, will display a list of all objects currently in the scene. The list can be sorted by name or type by clicking on the Name or Type buttons, clicking these again will sort the models in reverse order. Clicking on the name of an object will select that object.     
   
# Object types
There are 3 object types, each having some unique properties (like wavelength) and some shared properties (like position and rotation). These objects are: 

Ray:  
This object type represent a ray of light or a group (array) of light rays. Properties:
* Wavelength(nm)  
This is the wavelength of the light ray(s) given in nanometers
* Color(rgb)  
The displayed color of the rays written as red, green and blue values in 0.0-1.0 range. 
* Rows, Columns  
The number of rays projected by the object, for example 4 rows and 5 columns will project a total of 20 rays in 4 rows and 5 columns
* Offset (for rows and columns)  
The distance between rows or columns 
* Angle (Horizontal, Vertical)  
The angle at which the rays are projected. 0.0 means the rays are parallel, negative values will make the rays converge in front of the object, positive numbers result in divergent rays. The angle is measured from the horizontal or vertical plane of the object, so a 22.5 angle is equivalent to a 45 degree field of view.    

Projector:  
This is a special ray array that projects an image onto objects in the scene. A projector only ever displays 4 rays (one for each corner) but it may have many more hidden rays. The projected image is displayed on a (deformed) plane and may be displayed incorrectly when the mesh it is projected onto is too complex or the ray path is incoherent or split for some rays. Properties:
* Width, Height  
The size of the projector.
* Angle (Horizontal, Vertical)  
The angle at which the rays are projected. 0.0 means the rays are parallel, negative values will make the rays converge in front of the object, positive numbers result in divergent rays. The angle is measured from the horizontal or vertical plane of the object, so a 22.5 angle is equivalent to a 45 degree field of view.    
* Number of samples  
The number of rays projected by the object 
* Image  
The image (texture) projected by the object
* Color(rgb)  
The displayed color of the rays written as red, green and blue values in 0.0-1.0 range. 
* Depth test  
With this option you can toggle depth testing for the projected image. If depth testing is off the projected image will always be visible, even if it is obscured by other objects (or the objext it is projected onto)

Mesh:  
A mesh object is any 3D object that is neither a ray or a projector, these objects can absorb, refract and/or refract light rays (their type is then displayed as either 'absorb' or 'reflect' or 'refract', 'beamsplit' if they both reflect and refract light). Properties:
* Material  
The material the mesh is made of, this determines the index of refraction for the object
* Reflect/Refract (green=on, gray=off)  
If the Reflect mode is on and Refract is off, then the object will reflect light rays.  
If the Reflect mode is off and Refract is on, then the object will refract light rays.
If the Reflect mode is on and Refract is on, then the object will split light rays.
If the Reflect mode is off and Refract is off, then the object will absorb light rays.
* Shading  
Determines how the mesh is displayed (color, transparency, glossiness, solid or wireframe)      
 
# Object properties
Each object has a series of properties that determine where it is situated, how big it is and how it interacts with other objects in the scene. When an object is selected all of its properties are displayed in a panel on the right side of the screen. Different objects can have different sets of properties, but all have some in common.
* Name  
The name of the object is displayed in a input field in the top of the properties panel. The name can be edited at any time simply by typing in a new name and pressing enter. The name is used in the select object list.  
* Position (X,Y,Z)  
This represents the position of the object in 3D space in global coordinates. X is the left-right offset, Y is the forward-back offset and Z is the up-down offset.
* Rotation (H,P,R)  
Rotation represents the rotation of the object in 3D space in global coordinates as a set of 3 Euler angles. H is 'heading', rotation around the Z axis (like when you are looking left or right), P is 'pitch', rotation around the X axis (like when you are looking up or down), R is 'roll', rotation around the Y axis (the kind of motion you can't do when your head is attached to the rest of your body... or a airplane barrel roll).
* Scale (sX, sY, sZ)  
This represents the size of the object in all 3 axis. 
* Delete  
Delete is not actually a property, clicking this button will permanently and without asking remove the object from the scene. There is no undo!
* Frozen (green=on, gray=off)  
When an object is frozen it can not be selected, rotated, moved or scaled using the mouse. It is usefull to freeze an object you don't want to change when there are other objects around it that you want to edit.
To un-freeze an object you may need to select it using the select object list (see 'Creating and Selecting objects')
* Stashed (green=on, gray=off)   
A stashed object is hidden from view and does not interact with other objects, but it is not deleted, at any point a stashed object can be rendered visible and interactive. To un-stash an object you may need to select it using the select object list (see 'Creating and Selecting objects')
* Clone  
Clicking this option will create an identical copy of the currently selected object.
* Edit pivot  
Clicking this option will display a panel that allows to move and rotate the pivot (origin point) of an object. Moving or rotating the pivot may change the position and/or rotation of an object. 

# Manipulating objects
Manipulating object refers to changing the position, rotation and scale of that object using mouse gestures.
All manipulation types work on the same basis. First you need to select an object, then you need to select the type of manipulation (position, rotation or scale), finally click and move the mouse cursor while holding down the left mouse button.

Global (and local)  
By default all movements and rotations are done in global coordinates, but by clicking on the Global button you can toggle between global and local modes. When the Global mode is off, the manipulations are relative to the object (its pivot) but the relation between mouse movement and the applied transform may be unintuitive (moving the cursor right may move the object left etc)

Snap  
When Snap is on, all changes are made using the snap value increments (eg. moving only by 0.5 units or rotating by 5.0 degrees)     

Move:   
Movement can be done in either two directions at once (movement on a plane) or just in one direction. Use the X, Y, Z buttons that become visible after clicking on the Move button to determine the axis or plane of movement. 

Rotate:  
Rotation can be done one axis at a time. Use the X, Y, Z buttons that become visible after clicking on the Rotate button to determine the axis of rotation. When you click and hold on an object in this mode a circle helper object will be displayed - the selected object will be turned as if you where turning this circle (like a steering wheel or valve).

Scale:  
Scale allows uniform scaling (in all directions). Click on a selected object and drag the mouse cursor while holding the mouse button down to scale an object. It is best to release and click again when changing the direction of scaling.
  
# Saving and Loading
To save or load a scene click on the Save/Load button in the top left corner of the window. The panel that will appear is divided into two parts. The top part allows to save the current scene by typing in a name in the provided input field and clicking the Save button. The bottom part of the panel shows a list of all previously saved scenes, to load a scene click on a button with the name of the scene you want to load. 

Warning:   
The application (author) assumes the user knows what he or she is doing.
Loading a scene will discard the current scene without asking!
Saving a scene under a name that already exists will override the file without asking!
Closing the program window will discard the current scene without asking!

Scenes are saved in JSON format, they do not include the models and textures used in the scene, be sure to include your custom models and textures when loading scenes on different installations of the program. 

