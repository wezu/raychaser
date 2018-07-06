This is a tool for visualizing light ray refraction in 3D.
![screenshot](https://github.com/wezu/raychaser/blob/master/screen.png)

You will need Python(3.x), Panda3D, opticalmaterialspy, numpy and scipy to run this program. You can install all of these using pip:
```
pip install opticalmaterialspy
pip install panda3d
```

This is a Work in Progress, more docs will follow..

You can move(pan) the camera with WASD keys or hold down shift+left mouse button and move the mouse.

You can rotate the camera by holding down the right mouse button and moving the mouse around.

You can select object by clicking on them and un-select by clicking anywhere else, or hitting the Esc key.

When an object is selected a properties panel will pop up on the right, you can edit any properties by typing in values in input fields, moving sliders or clicking buttons.

The properties under 'SHADING' have no impact on the rays, it's just the way a object is displayed.

When you click 'Set material...' a list will appear you can choose what you need.

If you have a ray selected the properties will show a input field for wavelength (in nm), and the ray color will adjust automatically when you change the wavelength.

The buttons on the bottom allow you to move, rotate and scale objects using the mouse, select the mode, click and hold on an object and move the mouse to move/rotate/scale the object. You can move in one or two axis, rotate in one axis and scale in all three, the axis can be toggled with some extra buttons that will appear once you set the mode, and also a axis widget will be shown on top of the selected object. You can also switch between local and global coordinates.

From the buttons on the top - Save and Load are not working currently.

The 'Grid' button will show or hide the grid.

The 'Add/Select' button will show a list of all the objects in the scene, you can select the objects by clicking on their names.

Also this list panel has buttons to add new objects to the scene, but at the moment you can only add new rays - just click on '+ Add new ray...'

