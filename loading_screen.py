from contextlib import contextmanager
from panda3d.core import *
from direct.gui.OnscreenImage import OnscreenImage

@contextmanager
def loading(*args):
    '''A context manager that shows a loading screen'''
    x=base.win.get_x_size()//2
    y=base.win.get_y_size()//2
    img=loader.load_texture('gui/loading.png')
    scale=(256, 0, 256)#img size//2
    if ConfigVariableBool('framebuffer-srgb').getValue():
        tex_format=img.get_format()
        if tex_format == Texture.F_rgb:
            tex_format = Texture.F_srgb
        elif tex_format == Texture.F_rgba:
            tex_format = Texture.F_srgb_alpha
        img.set_format(tex_format)
    load_screen = OnscreenImage(image = img, scale=scale, pos = (x, 0, -y), parent=pixel2d)
    load_screen.set_transparency(TransparencyAttrib.M_alpha)
    #render 3 frames because we may be in a threaded rendering pipeline
    #we render frames to make sure the load screen is shown
    base.graphicsEngine.renderFrame()
    base.graphicsEngine.renderFrame()
    base.graphicsEngine.renderFrame()
    try:
        yield
    finally:
        base.graphicsEngine.renderFrame()
        base.graphicsEngine.renderFrame()
        base.graphicsEngine.renderFrame()
        load_screen.remove_node()
