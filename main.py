#panda3d is needed to run this script
#get it from pip:
#python -m pip install panda3d
#or download from panda3d.org

#by default a simpler version of opticalmaterialspy is used (see mat_spy.py)
#but the original can be used if needed, but that version requires both numpy and scipy
#try:
#    import opticalmaterialspy as mat_spy
#except:
#    import mat_spy
import mat_spy
from panda3d.core import *
from panda3d.egg import *
load_prc_file('options.prc')
load_prc_file_data('','textures-power-2 None')
load_prc_file_data('','load-file-type p3assimp')
#load_prc_file_data("", "threading-model Cull/Draw") #at your own risk!
#parse some options
msaa=ConfigVariableInt('antialias_msaa', 0).get_value()
if msaa > 0:
    load_prc_file_data('', 'framebuffer-multisample 1')
    load_prc_file_data('', 'multisamples '+str(msaa))

from direct.showbase import ShowBase
from direct.showbase.DirectObject import DirectObject
from direct.gui.DirectGui import *
from direct.interval.FunctionInterval import Func, Wait
from direct.interval.MetaInterval import Sequence, Parallel
from direct.interval.LerpInterval import LerpPosInterval

import io
import os
import sys
import json
import time
import shutil
import random
from math import radians
from math import sqrt
from collections import deque
from collections import namedtuple
import threading
#py 2 and 3 compatibility
if sys.version_info >= (3, 0):
    import builtins
else:
    import __builtin__ as builtins
from camera import CameraControler
from loading_screen import loading
from gui import UI
from gui import math_eval
from color_palette import ColorPalette
from custom_encoder import CustomEncoder


#set the window decoration before we start
wp = WindowProperties.getDefault()
wp.set_title("Raychaser  wezu.dev@gmail.com")
wp.set_icon_filename('gui/icon.ico')
WindowProperties.setDefault(wp)

#this is used for ray tests return values
Hit = namedtuple('Hit', 'has_hit pos normal node next_node next_pos')
Traced = namedtuple('Traced', 'is_internal vector')

class App(DirectObject):
    def __init__(self):
        builtins.app=self
        #basic stuff
        self.base = ShowBase.ShowBase()
        self.base.disableMouse()
        self.base.set_background_color(0.1, 0.1, 0.1)
        self.base.camLens.set_near_far(0.01, 500.0)
        render.set_antialias(AntialiasAttrib.MMultisample)
        try:
            NodePath(base.frameRateMeter).hide()
        except:
            pass
        with loading():
            #vars
            self.last_txt=None
            self.last_txt_ttl=0
            self.select_mask=BitMask32.bit(11)
            self.camera_mask=BitMask32.bit(22)
            self.obj_mask=BitMask32.bit(1)
            self.obj_mask.set_bit(2)
            self.ray_mask=BitMask32.bit(2)
            self.pre_click_mouse_pos=Vec2(0,0)
            self.last_mouse_pos=Vec2(0,0)
            self.mouse_is_down=False
            self.mouse_is_down_frame_skip=True
            self.selected_id=None
            self.flip_model_normal=False
            self.write_model_bam=False
            self.last_vec=None
            self.scene_buttons=[]
            self.img_buttons=[]
            self.mode='select'
            self.global_coords=True
            self.snap=False
            self.snap_pos=0.25
            self.snap_angle=5.0
            self.active_axis=['x','y','z']
            self.objects=[]
            self.scene=render.attach_new_node('scene')
            self.max_points=ConfigVariableInt('max-points', 1000).get_value()
            sys.setrecursionlimit(self.max_points*100)

            #cam mask
            base.cam.node().set_camera_mask(self.camera_mask)

            #ui
            self.gui=UI()
            self.gui.load_from_file('gui/gui.json')
            self.gui.show_hide(['button_save', 'button_create', 'button_list','button_help',
                                'button_grid','button_camera','input_grid',
                                'button_move','button_rotate','button_scale'])

            self.gui.fade_out(['button_rotate', 'button_move', 'button_scale'])
            self.set_snap(False)
            self.gui.resize_callback.append(self.on_resize)
            self.load_help_from_txt('readme.md')

            #use a better camera controller
            cam_speed=ConfigVariableDouble('camera-speed', 1.0).get_value()
            cam_zoom_speed=ConfigVariableDouble('camera-zoom-speed', 1.0).get_value()
            self.cam_driver=CameraControler(pos=(0,0,0), offset=(0, 10, 0), speed=cam_speed,  zoom_speed=cam_zoom_speed)
            #setup key binds for the camera
            key_binds={}
            key_binds['rotate']=ConfigVariableString('key-camera-rotate', 'mouse3').get_value()
            key_binds['pan']=ConfigVariableString('key-camera-pan', 'shift-mouse1').get_value()
            key_binds['zoom_in']=ConfigVariableString('key-camera-zoom-in', 'wheel_up').get_value()
            key_binds['zoom_out']=ConfigVariableString('key-camera-zoom_out', 'wheel_down').get_value()
            key_binds['left']=ConfigVariableString('key-camera-left', 'a').get_value()
            key_binds['right']=ConfigVariableString('key-camera-right', 'd').get_value()
            key_binds['forward']=ConfigVariableString('key-camera-forward', 'w').get_value()
            key_binds['back']=ConfigVariableString('key-camera-back', 's').get_value()
            self.cam_driver.bind_keys(**key_binds)
            self.cam_driver.node.set_h(45)
            self.cam_driver.gimbal.set_p(45)
            self._setup_select_buff()
            self.ortho_cam(True)
            self.cam_relative_pan(False)

            self.cam_circle=self.make_circle(segments=16, thickness=1.5, radius=0.25)
            self.cam_circle.reparent_to(self.cam_driver.node)
            self.cam_circle.set_color((0.4, 0.5, 0.4, 1.0), 1)

            #collision, mouse picking setup
            self.cam_ray_trav = CollisionTraverser()
            self.cam_ray_handler = CollisionHandlerQueue()
            self.ray_trav = CollisionTraverser()
            #self.ray_trav.show_collisions(render)
            self.ray_handler = CollisionHandlerQueue()
            self.collison_ray_np = render.attach_new_node(CollisionNode('ray'))
            self.collison_ray=CollisionRay()
            self.collison_ray_np.node().add_solid(self.collison_ray)
            self.collison_ray_np.node().set_from_collide_mask(self.ray_mask)
            self.collison_ray_np.node().set_into_collide_mask(0)

            picker_np = camera.attach_new_node(CollisionNode('mouseRay'))
            #picker_np.node().set_from_collide_mask(GeomNode.getDefaultCollideMask())
            self.mouse_ray = CollisionRay()
            picker_np.node().add_solid(self.mouse_ray)
            picker_np.node().set_from_collide_mask(BitMask32.bit(1))
            picker_np.node().set_into_collide_mask(0)
            self.cam_ray_trav.add_collider(picker_np, self.cam_ray_handler)

            self.shell = render.attach_new_node(CollisionNode('shell'))
            self.shell.node().add_solid(CollisionInvSphere(0,0,0, 1000))
            self.shell.node().set_into_collide_mask(self.ray_mask)
            self.shell.set_python_tag('shell', True)

            #load axis model
            self.axis=loader.load_model('data/axis.bam')
            self.axis.set_light_off(1)
            self.axis.reparent_to(render)
            self.axis.set_depth_test(False)
            self.axis.set_bin('fixed', 100)
            self.axis.find('x').hide()
            self.axis.find('y').hide()
            self.axis.find('z').hide()
            #visual aid
            self.circle=self.make_circle(line=False)
            self.circle.hide()
            self.circle.set_light_off()
            self.circle.set_depth_test(False)
            self.circle.set_bin('fixed', 100)
            #mouse plane
            self.plane = Plane(Vec3(0, 0, 1), Point3(0, 0, 1))
            self.last_axis_pos=None
            #grid setup
            self.make_grid()
            #shader setup
            self.lit_shader=Shader.load(Shader.SLGLSL, 'shaders/litsphere_v.glsl', 'shaders/litsphere_f.glsl')
            self.point_shader=Shader.load(Shader.SLGLSL, 'shaders/point_v.glsl', 'shaders/point_f.glsl')
            self.line_shader=Shader.load(Shader.SLGLSL, 'shaders/line_v.glsl', 'shaders/line_f.glsl')
            render.set_shader_input('litex', loader.load_texture('data/lit_sphere3.png'))
            #find data to load
            self.find_models('refract')
            self.find_models('reflect')
            self.find_models('beamsplit')
            self.find_models('absorb')
            self.find_scenes('scenes')
            self.find_images('images')
            #mouse keys
            self.accept('mouse1', self._on_mouse_down)
            self.accept('mouse1-up', self._on_mouse_up)
            #keys
            self.accept('escape', self.set_select_mode)
            #window close event - we need to clean up before exit
            base.win.set_close_request_event('exit-event')
            self.accept('exit-event', self._on_exit)
            #palette for random ray colors
            self.colors=ColorPalette()
            #thread with the raycasting
            self.in_que=deque()
            self.out_que=deque()
            self.deamon_stop_event= threading.Event()
            self.deamon = threading.Thread(name='daemon', target=self.ray_daemon, args=(self.deamon_stop_event,))
            self.deamon.setDaemon(True)
            self.deamon.start()
            # Task
            #taskMgr.add(self._on_mouse_down_task, 'mouse_lock_tsk')
            taskMgr.add(self._mouse_manipulate_task, 'mouse_tsk')
            taskMgr.add(self._update, 'update_tsk')

        self.splash=self.gui.img('gui/splash.png', 'left', (0,-384))
        self.splash.set_bin("fixed", 11)
        base.buttonThrowers[0].node().setButtonDownEvent('buttonDown')
        self.accept('buttonDown', self.remove_splash)
        self.print_txt('[Press any key to continue]', ttl=30.0)

    def remove_splash(self, key_event=None):
        '''Removes the splash screen'''
        self.splash.hide()
        self.gui.fade_screen(0.5, base.get_background_color())
        self.ignore('buttonDown')
        self.print_txt('')
        try:
            fr=NodePath(base.frameRateMeter)
            fr_pos=fr.get_pos(pixel2d)
            fr.set_x(pixel2d, fr_pos.x-256)
            fr.show()
        except:
            pass

    def load_help_from_txt(self, txt_file):
        '''Loads the text for the in-game help
        '''
        self.help_dict={}
        topic=None
        txt=''
        with open(txt_file,'r') as f:
            for line in f.readlines():
                if line.startswith('#'):
                    if topic:
                        self.help_dict[topic]=txt
                        txt=''
                    topic=line.strip('#').strip()
                else:
                    txt+=line
            else:
                if topic:
                    self.help_dict[topic]=txt
                    topic=line.strip('#').strip()

    def show_help(self, topic):
        '''Shows the help text'''
        self.gui.show_hide('help_txt_frame','help_frame')
        if topic in self.help_dict:
            self.gui['help_txt_frame_verticalScroll']['value']=1
            self.gui['help_txt_header'].set_text(topic)
            self.gui['help_txt'].set_text(self.help_dict[topic])

    def reset_cam(self):
        '''Reset the camera pos and hpr'''
        self.cam_driver.reset()
        self.cam_driver.node.set_h(45)
        self.cam_driver.gimbal.set_p(45)

    def on_resize(self):
        '''Function run when the main window is resized '''
        render.set_shader_input("screen_size", Vec2(*base.get_size()))

    def save_scene(self):
        '''Save the scene to a .json file, the name for the file is taken from gui['save_path_input']  '''
        target_file=self.gui['save_path_input'].get()
        scene=[]
        for node in self.objects:
            if node:
                data={}
                tags={}
                for key in node.get_python_tag_keys():
                    if key not in ('material', 'line'):
                        tags[key]=node.get_python_tag(key)
                data['tags']=tags
                data['pos']=node.get_pos(render)
                data['hpr']=node.get_hpr(render)
                data['scale']=node.get_scale(render)
                if node.has_color():
                    data['color']=node.get_color()
                scene.append(data)
        #dump to json
        with open(target_file, 'w') as outfile:
            json.dump(scene, outfile, cls=CustomEncoder, indent=4)

        self.gui.show_hide('', 'save_frame')
        self.print_txt('Scene saved to '+target_file)
        self.find_scenes('scenes')

    def load_scene(self, file_name):
        '''Load a scene from a  'file_name' json file'''
        self.clear_scene()
        if os.path.exists(file_name):
            with open(file_name) as f:
                try:
                    data = json.load(f)
                except:
                    self.print_txt('Could not load data!')
                    return
                for item in data:
                    if item['tags']['type']=='ray':
                        self.make_ray(
                                      color=item['tags']['line_color'],
                                      wavelength=item['tags']['wave'],
                                      model_name=item['tags']['name'],
                                      scale=item['scale'],
                                      columns=item['tags']['columns'],
                                      rows=item['tags']['rows'],
                                      column_offset=item['tags']['column_offset'],
                                      row_offset=item['tags']['row_offset'],
                                      angle_h=item['tags']['angle_h'],
                                      angle_v=item['tags']['angle_v']
                                      )
                    elif item['tags']['type']=='projector':
                        self.make_projector(
                                      color=item['tags']['line_color'],
                                      scale=item['scale'],
                                      angle_h=item['tags']['angle_h'],
                                      angle_v=item['tags']['angle_v'],
                                      texture=item['tags']['texture'],
                                      sample_v=item['tags']['sample_v'],
                                      sample_h=item['tags']['sample_h'],
                                      proj_w=item['tags']['proj_w'],
                                      proj_h=item['tags']['proj_h']
                                      )
                    else:
                        self.load_object(
                                         model=item['tags']['model_file'],
                                         select=False,
                                         object_type=item['tags']['type'],
                                         threshold=item['tags']['threshold'],
                                         flip_normal=item['tags']['flip_normal'])
                    node=self.objects[-1]
                    if 'material_name' in item['tags']:
                        self.set_material(material=item['tags']['material_name'], node=node)
                    if 'pivot_pos' in item['tags']:
                        self.move_pivot(node, Point3(*item['tags']['pivot_pos']))
                    if 'pos' in item:
                        node.set_pos(*item['pos'])
                    if 'hpr' in item:
                        node.set_hpr(*item['hpr'])
                    if 'scale' in item:
                        node.set_scale(*item['scale'])
                    if 'color' in item:
                        node.set_color(*item['color'], 1)
                    if 'gloss' in item['tags']:
                        node.set_shader_input('gloss', item['tags']['gloss'])
                    if 'stashed' in item['tags']:
                        if item['tags']['stashed']:
                            self.stash_object(node)
                    if 'name' in item['tags']:
                        node.set_python_tag('name', item['tags']['name'])    

        self.gui.show_hide('', 'load_frame')

    def set_flip_normal(self):
        '''GUI callback - sets the  flip_model_normal flag'''
        if self.flip_model_normal:
            self.flip_model_normal=False
            self.gui.fade_out('button_normal')
        else:
            self.flip_model_normal=True
            self.gui.fade_in('button_normal')


    def set_write_bam(self):
        '''GUI callback - sets the  write_model_bam flag'''
        if self.write_model_bam:
            self.write_model_bam=False
            self.gui.fade_out('button_bam')
        else:
            self.write_model_bam=True
            self.gui.fade_in('button_bam')

    def load_image(self, path):
        '''Load an image from path, apply it to the currently selected projector object'''
        if self.selected_id is not None:
            node=self.objects[self.selected_id]
            node.set_python_tag('texture', path)
            self.gui.show_hide('', 'file_list_frame')
            img_name='{0:.16}'.format(Filename(path).getBasenameWoExtension())
            self.gui.set_button_txt('button_proj_img', img_name)
            node_id=node.get_python_tag('id')
            self.trace_ray_in_thread(self.selected_id)

    def find_images(self, dir):
        '''Creates buttons for all images found in dir '''
        dir=str(Filename(dir).to_os_specific())
        self.gui['img_path_txt'].set_text('{:.50}'.format(dir))
        for button_name in self.img_buttons:
            self.gui.remove_button(button_name)
        self.gui.scroll('file_list_frame', 0)
        self.img_buttons=[]
        self.img_buttons.append('button_img_0')
        up_dir=str(Filename().from_os_specific(os.path.dirname(os.path.abspath(dir))))
        self.gui.button(txt='...',
                           mono_font=True,
                           align='left',
                           cmd="app.find_images('"+up_dir+"')",
                           width=384,
                           pos=(16,0),
                           name='button_img_0',
                           parent='file_list_frame_canvas')
        i=0
        for fn in os.listdir(dir):
            f=Filename(fn)
            full_path=os.path.join(dir,fn)
            if os.path.isdir(full_path):
                i+=1
                self.img_buttons.append('button_img_'+str(i))
                self.gui.button(txt='<DIR>{name:.45}'.format(name=f.getBasename()),
                                   mono_font=True,
                                   align='left',
                                   cmd="app.find_images('"+str(Filename().from_os_specific(full_path))+"')",
                                   width=384,
                                   pos=(16,i*32),
                                   name='button_img_'+str(i),
                                   parent='file_list_frame_canvas')
            elif f.get_extension() in ('png', 'jpg', 'jpeg', 'dds', 'bmp', 'tga'):
                i+=1
                self.img_buttons.append('button_img_'+str(i))
                self.gui.button(txt='{name:.45}'.format(name=f.getBasename()),
                                   mono_font=True,
                                   align='left',
                                   cmd="app.load_image('"+str(Filename().from_os_specific(dir))+'/'+fn+"')",
                                   width=384,
                                   pos=(16,i*32),
                                   name='button_img_'+str(i),
                                   parent='file_list_frame_canvas')

    def find_scenes(self, dir):
        ''' Find all json files in dir and make a button for each file'''
        for button_name in self.scene_buttons:
            self.gui.remove_button(button_name)
        self.scene_buttons=[]
        for fn in os.listdir(dir):
            f=Filename(fn)
            self.scene_buttons.append('button_'+fn)
            app.gui.button(txt='{name:<40}'.format(name=f.getBasename()),
                                   sort_dict={'name':fn},
                                   mono_font=True,
                                   align='left',
                                   cmd='app.load_scene("'+dir+'/'+fn+'")',
                                   width=384,
                                   pos=(16,0),
                                   name='button_'+fn,
                                   parent='load_frame_canvas')
        self.gui.sort_buttons('load_frame_canvas', 'name', False)

    def find_models(self, dir):
        '''Looks for models in a given directory, adds gui buttons for each model found'''
        for fn in os.listdir('models/'+dir):
                f=Filename(fn)
                if f.get_extension() in ('bam', 'egg', '3d', '3ds', '3mf',
                                         'ac', 'ac3d', 'acc', 'amj', 'ase',
                                         'ask', 'b3d', 'blend', 'bvh', 'cms',
                                         'cob', 'dae', 'dxf', 'enff', 'fbx',
                                         'hmb', 'ifc', 'irr', 'lwo', 'lws',
                                         'lxo', 'md2', 'md3', 'md5', 'mdc',
                                         'mdl', 'xml', 'mot', 'ms3d', 'ndo',
                                         'nff', 'obj', 'off', 'ogex', 'ply',
                                         'pmx', 'prj', 'q3o', 'q3s', 'raw',
                                         'scn', 'sib', 'smd', 'stp', 'stl',
                                         'ter', 'uc', 'vta', 'x', 'x3d', 'xgl', 'zgl'):
                    app.gui.button(txt='{name:<40} {type}'.format(name=f.getBasename(), type=dir),
                                       sort_dict={'name':fn, 'type':dir},
                                       mono_font=True,
                                       align='left',
                                       cmd="app.load_object('models/"+dir+'/'+fn+"', select=True, object_type='"+dir+"')",
                                       width=384,
                                       pos=(16,0),
                                       parent='model_frame_canvas')
        self.gui.sort_buttons('model_frame_canvas', 'type', False)

    def cam_relative_pan(self, relative=None):
        '''GUI callback -Change camera movement scheme '''
        if relative is None:
            self.cam_driver.relative_pan = not self.cam_driver.relative_pan
        else:
            self.cam_driver.relative_pan =relative
        if self.cam_driver.relative_pan:
            self.gui.fade_in('button_cam_pan')
        else:
            self.gui.fade_out('button_cam_pan')

    def show_cam_gimbal(self, show=None):
        '''GUI callback -Show/hide camera focus point thingy '''
        is_hidden=self.cam_circle.is_hidden()
        if show is None:
            if is_hidden:
                self.cam_circle.show()
            else:
                self.cam_circle.hide()
        else:
            if show:
                self.cam_circle.show()
            else:
                self.cam_circle.hide()
        if self.cam_circle.is_hidden():
            self.gui.fade_out('button_cam_gimbal')
        else:
            self.gui.fade_in('button_cam_gimbal')

    def ortho_cam(self, set_orto=None):
        '''GUI callback -sets the main camera into orthographic or perspective mode'''
        if set_orto is None:
            set_orto=base.camLens.is_perspective()
        if set_orto:
           base.camLens=OrthographicLens()
           #base.camLens.set_aspect_ratio(base.get_aspect_ratio())
           film_size=Vec2(base.get_size())
           film_size/=film_size[0]/4
           base.camLens.set_film_size(film_size)
           base.camLens.set_near_far(0.01, 500.0)
           base.cam.node().set_lens(base.camLens)
           self.select_cam.node().set_lens(base.camLens)
           self.gui.fade_in('button_cam_ortho')
        else:
            base.camLens=PerspectiveLens()
            base.camLens.set_fov(60)
            base.camLens.set_aspect_ratio(base.getAspectRatio())
            base.camLens.set_near_far(0.1, 500.0)
            base.cam.node().set_lens(base.camLens)
            self.select_cam.node().set_lens(base.camLens)
            self.gui.fade_out('button_cam_ortho')
            
    def clone(self, node=None):
        '''Create a clone from a node, if node is None, clone selected object'''
        if node is None:
            if self.selected_id is not None:            
                node=self.objects[self.selected_id]
        if node:
            tags={}
            for key in node.get_python_tag_keys():
                if key not in ('material', 'line'):
                    tags[key]=node.get_python_tag(key)            
            if tags['type']=='ray':
                self.make_ray(
                              color=tags['line_color'],
                              wavelength=tags['wave'],
                              model_name=tags['name'],
                              columns=tags['columns'],
                              rows=tags['rows'],
                              column_offset=tags['column_offset'],
                              row_offset=tags['row_offset'],
                              angle_h=tags['angle_h'],
                              angle_v=tags['angle_v']
                              )
            elif tags['type']=='projector':
                self.make_projector(
                              color=tags['line_color'],                              
                              angle_h=tags['angle_h'],
                              angle_v=tags['angle_v'],
                              texture=tags['texture'],
                              sample_v=tags['sample_v'],
                              sample_h=tags['sample_h'],
                              proj_w=tags['proj_w'],
                              proj_h=tags['proj_h']
                              )
            else:
                self.load_object(
                                 model=tags['model_file'],
                                 select=False,
                                 object_type=tags['type'],
                                 threshold=tags['threshold'],
                                 flip_normal=tags['flip_normal'])
            clone=self.objects[-1]
            if 'material_name' in tags:
                self.set_material(material=tags['material_name'], node=clone)
            if 'pivot_pos' in tags:
                self.move_pivot(clone, Point3(*tags['pivot_pos']))
            clone.set_pos(render, node.get_pos(render))
            clone.set_hpr(render, node.get_hpr(render))
            clone.set_scale(render, node.get_scale(render))
            clone.set_color(node.get_color(), 1)
            if 'gloss' in tags:
                clone.set_shader_input('gloss', tags['gloss'])
            if 'stashed' in tags:
                if tags['stashed']:
                    self.stash_object(clone)
            if 'name' in tags:
                clone.set_python_tag('name', tags['name']+'_clone')    
    
    def set_reflect(self):
        '''Toggle object properties'''
        if self.selected_id is None:
            return
        node=self.objects[self.selected_id]
        if node:
            object_type=node.get_python_tag('type')
            if object_type=='refract':
                node.set_python_tag('type','beamsplit')
                self.gui.fade_in('button_reflect')
            elif object_type=='reflect':
                node.set_python_tag('type','absorb')
                self.gui.fade_out('button_reflect')
            elif object_type=='beamsplit':
                node.set_python_tag('type','refract')
                self.gui.fade_out('button_reflect')
            elif object_type=='absorb':
                node.set_python_tag('type','reflect')
                self.gui.fade_in('button_reflect')
            name=node.get_python_tag('name')
            id=node.get_python_tag('id')
            object_type=node.get_python_tag('type')
            text_node=self.gui['button_object_'+str(id)].get_python_tag('text')
            text_node.node().set_text('{name:<34} {type:<10} {id}'.format(name=name, type=object_type, id=id))

            self.do_raytrace()

    def set_refract(self):
        '''Toggle object properties'''
        if self.selected_id is None:
            return
        node=self.objects[self.selected_id]
        if node:
            object_type=node.get_python_tag('type')
            if object_type=='refract':
                node.set_python_tag('type','absorb')
                self.gui.fade_out('button_refract')
            elif object_type=='reflect':
                node.set_python_tag('type','beamsplit')
                self.gui.fade_in('button_refract')
            elif object_type=='beamsplit':
                node.set_python_tag('type','reflect')
                self.gui.fade_out('button_refract')
            elif object_type=='absorb':
                node.set_python_tag('type','refract')
                self.gui.fade_in('button_refract')
            name=node.get_python_tag('name')
            id=node.get_python_tag('id')
            object_type=node.get_python_tag('type')
            text_node=self.gui['button_object_'+str(id)].get_python_tag('text')
            text_node.node().set_text('{name:<34} {type:<10} {id}'.format(name=name, type=object_type, id=id))

            self.do_raytrace()

    def set_depth_test(self, node=None, depth_test=None):
        '''Changes the way the projected image geom is displayed'''
        if node is None:
            if self.selected_id is None:
                return
            node=self.objects[self.selected_id]
        if node is None:
            return
        if node.has_python_tag('quad'):
            quad=node.get_python_tag('quad')
            if depth_test is None:
                depth_test=not quad.get_depth_test()
            if depth_test:            
                quad.clear_bin()
                quad.set_depth_offset(1)                
                quad.set_depth_test(True)                
                self.gui.fade_in('button_depth_test')
            else:
                quad.set_depth_test(False)
                quad.set_bin('fixed', 100)
                self.gui.fade_out('button_depth_test')
        
    
    def set_material(self, material='SiO2', node=None):
        ''' Sets the material of the currently selected object'''
        if node is None:
            if self.selected_id is None:
                return
            node=self.objects[self.selected_id]
        if node is None:
            return
        if material=='Ktp x':
            m=mat_spy.Ktp('x')
        elif material=='Ktp y':
            m=mat_spy.Ktp('y')
        elif material=='Ktp z':
            m=mat_spy.Ktp('z')
        elif material=='Ln e':
            m=mat_spy.Ln('e')
        elif material=='Ln o':
            m=mat_spy.Ln('o')
        elif material=='Tfln e':
            m=mat_spy.Tfln('e')
        elif material=='Tfln o':
            m=mat_spy.Tfln('o')
        elif material=='LnMg e':
            m=mat_spy.LnMg('e')
        elif material=='LnMg o':
            m=mat_spy.LnMg('e')
        elif material=='Bbo e':
            m=mat_spy.Bbo('e')
        elif material=='Bbo o':
            m=mat_spy.Bbo('o')
        elif material=='Bibo x':
            m=mat_spy.Bibo('x')
        elif material=='Bibo y':
            m=mat_spy.Bibo('y')
        elif material=='Bibo z':
            m=mat_spy.Bibo('z')
        elif material=='Chalcogenide As2S3':
            m=mat_spy.Chalcogenide('As2S3')
        elif material=='Chalcogenide As2Se3':
            m=mat_spy.Chalcogenide('As2Se3')
        elif material=='Chalcogenide GeSe4':
            m=mat_spy.Chalcogenide('GeSe4')
        elif material=='Chalcogen. Ge10As10Se80':
            m=mat_spy.Chalcogenide('Ge10As10Se80')
        elif material=='SiO2':
            m=mat_spy.SiO2()
        elif material=='Su8':
            m=mat_spy.Su8()
        elif material=='Al2O3 e':
            m=mat_spy.Al2O3('e')
        elif material=='Al2O3 o':
            m=mat_spy.Al2O3('o')
        elif material=='TiO2 e':
            m=mat_spy.TiO2('e')
        elif material=='TiO2 o':
            m=mat_spy.TiO2('o')
        elif material.startswith('Custom IOR:'):
            material, ior= material.split(':')
            if not ior:
                ior=self.gui['mat_ior_input'].get()
            m=mat_spy.CustomIOR(ior)
            material= material+':'+str(ior)
        else:
            m=mat_spy.SiO2()
            material='SiO2'
        node.set_python_tag('material', m)
        node.set_python_tag('material_name', material)
        self.gui['material_txt'].set_text('MATERIAL:\n'+material)
        self.gui.show_hide('', 'mat_list_frame')
        self.do_raytrace()

    def hide_grid(self):
        '''GUI callback - shows or hides the grid'''
        if self.grid.is_hidden():
            self.grid.show()
            self.gui.fade_in('button_grid')
        else:
            self.grid.hide()
            self.gui.fade_out('button_grid')

    def make_circle(self, segments=36, thickness=2.0, radius=1.0, line=False):
        '''Creates a circle '''
        l=LineSegs()
        l.set_thickness(thickness)
        l.move_to(Point3(0,0,0))
        if line:
            l.draw_to(Point3(0,radius,0))
        else:
            l.move_to(Point3(0,radius,0))
        temp=NodePath('temp')
        for i in range(segments+1):
            temp.set_h(i*360.0/segments)
            p=render.get_relative_point(temp, (0, radius, 0))
            l.draw_to(p)
        temp.remove_node()
        return render.attach_new_node(l.create())

    def make_grid(self, scale=10.0):
        '''Creates a 10x10 grid of given size'''
        l=LineSegs()
        l.set_color(Vec4(0.1, 0.38, 0.23, 0.7))
        l.set_thickness(4.0)
        for i in range(0,11):
            l.move_to(Vec3(0,i,0))
            l.draw_to(Vec3(10, i, 0))
            l.move_to(Vec3(i,0,0))
            l.draw_to(Vec3(i, 10, 0))
        self.grid=render.attach_new_node(l.create())
        self.grid.set_transparency(TransparencyAttrib.M_multisample)
        self.grid.set_antialias(AntialiasAttrib.MLine)
        #self.grid.set_depth_test(False)
        #self.grid.set_depth_write(False)
        #self.grid.set_bin('fixed', 9)
        self.grid.set_pos(-5,-5,0)
        self.grid.flatten_strong()
        self.grid.set_scale(0.1)
        self.grid.flatten_strong()
        self.grid.set_scale(scale)

    def clear_scene(self):
        '''Removes all objects from the scene'''
        for n, node in enumerate(self.objects):
            self.delete_object(n)
        self.objects=[]

    def delete_object(self, object_id=None):
        '''Removes the object from the scene,
        if object_id is None the currently selected object is removed.'''
        if object_id is None:
            object_id =self.selected_id
        if object_id <= len(self.objects):
            node=self.objects[object_id]
            if node is None:
                return
            for key in node.get_python_tag_keys():
                node.clear_python_tag(key)
            node.remove_node()
            self.gui.remove_button('button_object_'+str(object_id))
            self.gui.show_hide('', 'prop_frame')
            self.objects[object_id]=None
            self.selected_id=None
            self.axis.find('x').hide()
            self.axis.find('y').hide()
            self.axis.find('z').hide()
            self.do_raytrace()

    def center_camera(self, object_id=None):
        '''Move the camera near the object'''
        if object_id is None and self.selected_id is not None:
            object_id =self.selected_id
        if object_id is not None:
            if object_id <= len(self.objects):
                node=self.objects[object_id]
                if node:
                    pos=node.get_pos(render)
                    LerpPosInterval(self.cam_driver.node, 0.25, pos).start()

    def do_nill(self, *args, **kwargs):
        '''dummy function, does nothing  '''
        pass

    def _do_cmd_from_txt(self, txt, value):
        ''' execute commad based  on string,
        this function is to help with text entry widgets'''
        if txt == 'grid':
            self.grid.set_scale(value)
            self.gui['input_grid'].set('{0:.2f}'.format(*self.grid.get_scale(render)))
        if self.selected_id is None:
            return
        node=self.objects[self.selected_id]
        if txt == 'x':
            node.set_x(render, value)
        elif txt == 'y':
            node.set_y(render, value)
        elif txt == 'z':
            node.set_z(render, value)
        elif txt == 'h':
            node.set_h(render, value)
        elif txt == 'p':
            node.set_p(render, value)
        elif txt == 'r':
            node.set_r(render, value)
        elif txt == 'sx':
            node.set_sx(render, value)
        elif txt == 'sy':
            node.set_sy(render, value)
        elif txt == 'sz':
            node.set_sz(render, value)
        elif txt == 'R':
            c=node.get_color()
            c[0]=value
            node.set_color(c, 1)
        elif txt == 'G':
            c=node.get_color()
            c[1]=value
            node.set_color(c, 1)
        elif txt == 'B':
            c=node.get_color()
            c[2]=value
            node.set_color(c, 1)
        elif txt == 'wave':
            value=max(300, min(5000, value))
            #color=self.wavelength_to_rgb(value)
            node.set_python_tag('wave', value)
            #line=node.get_python_tag('line')
            #line.set_color(color, 1)
        elif txt == 'name':
            value =self.gui['name_input'].get() # get the full text!
            node.set_python_tag('name', value)
            id=node.get_python_tag('id')
            object_type=node.get_python_tag('type')
            text_node=self.gui['button_object_'+str(id)].get_python_tag('text')
            text_node.node().set_text('{name:<34} {type:<10} {id}'.format(name=value, type=object_type, id=id))
        elif txt=='column':
            node.set_python_tag('columns', int(value))
        elif txt=='row':
            node.set_python_tag('rows', int(value))
        elif txt=='column_offset':
            node.set_python_tag('column_offset', value)
        elif txt=='row_offset':
            node.set_python_tag('row_offset', value)
        elif txt=='angle_h':
            node.set_python_tag('angle_h', min(89.9, max(-89.9, value)))
        elif txt=='angle_v':
            node.set_python_tag('angle_v', min(89.9, max(-89.9, value)))
        elif txt=='sample_h':
            node.set_python_tag('sample_h',  max(2, int(value)))
        elif txt=='sample_v':
            node.set_python_tag('sample_v',  max(2, int(value)))
        elif txt=='proj_w':
            node.set_python_tag('proj_w',  value)
        elif txt=='proj_h':
            node.set_python_tag('proj_h',  value)
        elif txt=='pivot_x':
            pivot_pos=node.get_python_tag('pivot_pos')
            pivot_pos.x=value
            node.set_python_tag('pivot_pos', pivot_pos)
            self.move_pivot(node, pivot_pos)
        elif txt=='pivot_y':
            pivot_pos=node.get_python_tag('pivot_pos')
            pivot_pos.y=value
            node.set_python_tag('pivot_pos', pivot_pos)
            self.move_pivot(node, pivot_pos)
        elif txt=='pivot_z':
            pivot_pos=node.get_python_tag('pivot_pos')
            pivot_pos.z=value
            node.set_python_tag('pivot_pos', pivot_pos)
            self.move_pivot(node, pivot_pos)
        elif txt=='pivot_h':
            pivot_hpr=node.get_python_tag('pivot_hpr')
            pivot_hpr.x=value
            node.set_python_tag('pivot_hpr', pivot_hpr)
            self.rotate_pivot(node, pivot_hpr)
        elif txt=='pivot_p':
            pivot_hpr=node.get_python_tag('pivot_hpr')
            pivot_hpr.y=value
            node.set_python_tag('pivot_hpr', pivot_hpr)
            self.rotate_pivot(node, pivot_hpr)
        elif txt=='pivot_r':
            pivot_hpr=node.get_python_tag('pivot_hpr')
            pivot_hpr.z=value
            node.set_python_tag('pivot_hpr', pivot_hpr)
            self.rotate_pivot(node, pivot_hpr)
        elif txt=='snap':
            if self.mode=='move':
                self.snap_pos=value
            else:
                self.snap_angle=value
        self.update_ui_for_node(node)
        if node.get_python_tag('type') in ('ray', 'projector'):
            node_id=node.get_python_tag('id')
            self.trace_ray_in_thread(node_id)
        else:
            self.do_raytrace()

    def apply_input(self, txt, target=['x','y','z']):
        '''Convert text input into a command'''
        if not txt:
            if self.selected_id is None:
                return
            node=self.objects[self.selected_id]
            self.update_ui_for_node(node)
        values=[]
        #normalize input
        for v in txt.replace(',', ' ').split():
            try:
                values.append(math_eval(v))
            except:
                values.append(v)
        #process the inputs
        for value, txt  in zip(values, target):
            try:
                self._do_cmd_from_txt(txt, value)
            except:
                pass
                #self.print_txt("Wrong data format for input: '{0}', value: '{1}'".format(*(txt, value)))

    def update_ui_for_node(self, node):
        '''Updates the sliders and input fields of the object properties panel '''
        self.scale_emmiter_geom(node)
        self.gui.show_hide('prop_frame')
        self.gui['name_input'].set(node.get_python_tag('name'))
        self.gui['x_pos_input'].set('{0:.2f}'.format(*node.get_pos(render)))
        self.gui['y_pos_input'].set('{1:.2f}'.format(*node.get_pos(render)))
        self.gui['z_pos_input'].set('{2:.2f}'.format(*node.get_pos(render)))
        self.gui['h_input'].set('{0:.2f}'.format(*node.get_hpr(render)))
        self.gui['p_input'].set('{1:.2f}'.format(*node.get_hpr(render)))
        self.gui['r_input'].set('{2:.2f}'.format(*node.get_hpr(render)))
        self.gui['sx_input'].set('{0:.2f}'.format(*node.get_scale(render)))
        self.gui['sy_input'].set('{1:.2f}'.format(*node.get_scale(render)))
        self.gui['sz_input'].set('{2:.2f}'.format(*node.get_scale(render)))
        self.gui['pivot_x_pos_input'].set('{0:.2f}'.format(*node.get_python_tag('pivot_pos')))
        self.gui['pivot_y_pos_input'].set('{1:.2f}'.format(*node.get_python_tag('pivot_pos')))
        self.gui['pivot_z_pos_input'].set('{2:.2f}'.format(*node.get_python_tag('pivot_pos')))
        self.gui['pivot_h_input'].set('{0:.2f}'.format(*node.get_python_tag('pivot_hpr')))
        self.gui['pivot_p_input'].set('{1:.2f}'.format(*node.get_python_tag('pivot_hpr')))
        self.gui['pivot_r_input'].set('{2:.2f}'.format(*node.get_python_tag('pivot_hpr')))

        if node.has_python_tag('stashed'):
            self.gui.fade_in('stash_button')
        else:
            self.gui.fade_out('stash_button')
        if node.has_python_tag('frozen'):
            self.gui.fade_in('freeze_button')
        else:
            self.gui.fade_out('freeze_button')

        if node.has_python_tag('type'):
            t=node.get_python_tag('type')
            if t in ('ray', 'stashed ray'):
                self.gui.show_hide('ray_prop_frame', ['mesh_prop_frame','projector_prop_frame'])
                self.gui['wave_input'].set(str(node.get_python_tag('wave')))
                self.gui['ray_color_input'].set('{0:.2f}, {1:.2f}, {2:.2f}'.format(*node.get_python_tag('line_color')))
                self.gui['row_input'].set(str(node.get_python_tag('rows')))
                self.gui['row_off_input'].set('{0:.2f}'.format(node.get_python_tag('row_offset')))
                self.gui['column_input'].set(str(node.get_python_tag('columns')))
                self.gui['column_off_input'].set('{0:.2f}'.format(node.get_python_tag('column_offset')))
                self.gui['angle_h_input'].set('{0:.2f}'.format(node.get_python_tag('angle_h')))
                self.gui['angle_v_input'].set('{0:.2f}'.format(node.get_python_tag('angle_v')))
            elif t in ('projector', 'stashed projector'):
                self.gui.show_hide('projector_prop_frame',['mesh_prop_frame', 'ray_prop_frame'])
                self.gui['proj_angle_h_input'].set('{0:.2f}'.format(node.get_python_tag('angle_h')))
                self.gui['proj_angle_v_input'].set('{0:.2f}'.format(node.get_python_tag('angle_v')))
                self.gui['proj_sample_h_input'].set('{0}'.format(node.get_python_tag('sample_h')))
                self.gui['proj_sample_v_input'].set('{0}'.format(node.get_python_tag('sample_v')))
                self.gui['proj_width_input'].set('{0:.2f}'.format(node.get_python_tag('proj_w')))
                self.gui['proj_height_input'].set('{0:.2f}'.format(node.get_python_tag('proj_h')))
                if node.has_python_tag('quad'):
                    quad=node.get_python_tag('quad')
                    if quad.get_depth_test():
                        self.gui.fade_in('button_depth_test')
                    else:
                        self.gui.fade_out('button_depth_test')
            else:
                self.gui.show_hide('mesh_prop_frame', ['ray_prop_frame','projector_prop_frame'])
                mat_name=node.get_python_tag('material_name')
                self.gui['material_txt'].set_text('MATERIAL:\n'+mat_name)
                c=node.get_color()
                self.gui['color_input'].set('{0:.2f}, {1:.2f}, {2:.2f}'.format(*c))
                self.gui['alpha_slider']['value']=1.0-c[3]
                gloss=node.get_shader_input('gloss').get_vector()[0]
                self.gui['gloss_slider']['value']=gloss
                self.gui.fade_out(['button_refract', 'button_reflect'])
                if t in ('reflect', 'refract'):
                    self.gui.fade_in('button_'+t)
                elif t=='beamsplit':
                    self.gui.fade_in(['button_refract', 'button_reflect'])

    def freeze_object(self):
        '''Disables mouse ray collisions for the selected object '''
        if self.selected_id is None:
            return
        node=self.objects[self.selected_id]
        if node:
            cnode=node.find('+CollisionNode')
            mask=cnode.node().get_into_collide_mask()
            if mask.get_bit(1):
                mask.clear_bit(1)
                self.gui.fade_in('freeze_button')
                node.set_python_tag('frozen', True)
                self.set_select_mode()
                self.print_txt("Object frozen")
            else:
                mask.set_bit(1)
                self.gui.fade_out('freeze_button')
                node.clear_python_tag('frozen')
                self.print_txt('Object un-frozen')
            cnode.node().set_into_collide_mask(mask)

    def stash_object(self, node=None):
        '''Disable collisions and hide node '''
        if node is None:
            if self.selected_id is None:
                return
            node=self.objects[self.selected_id]
        if node:
            is_ray=False
            if node.get_python_tag('type')=='projector':
                node.set_python_tag('type', 'stashed projector')
                node.get_python_tag('line').hide()
                node.get_python_tag('quad').hide()
                is_ray=True
            elif node.get_python_tag('type')=='ray':
                node.set_python_tag('type', 'stashed ray')
                node.get_python_tag('line').hide()
                is_ray=True
            elif node.get_python_tag('type')=='stashed ray':
                node.set_python_tag('type', 'ray')
                is_ray=True
            elif node.get_python_tag('type')=='stashed projector':
                node.set_python_tag('type', 'projector')
                is_ray=True
            cnode=node.find('+CollisionNode')
            mask=cnode.node().get_into_collide_mask()
            if mask.is_zero():
                node.clear_python_tag('stashed')
                self.print_txt('Object un-stashed')
                mask=node.get_python_tag('old_mask')
                self.gui.fade_out('stash_button')
                node.show()
                if not is_ray:
                    node.show_through(self.camera_mask)
            else:
                node.set_python_tag('old_mask', BitMask32(mask))
                node.set_python_tag('stashed', True)
                self.print_txt('Object stashed')
                mask.clear()
                self.gui.fade_in('stash_button')
                node.hide()
                if not is_ray:
                    node.hide(self.camera_mask)
            cnode.node().set_into_collide_mask(mask)
            self.do_raytrace()

    def select_by_id(self, id):
        '''Select  an  object  based on  ID'''
        for node in self.objects:
            if node:
                node.hide(self.select_mask)
        self.set_select_mode()
        node=self.objects[id]
        node.show_through(self.select_mask)
        self.selected_id=id
        self.update_ui_for_node(node)

    def set_snap(self, on=None):
        ''' Toggle snap '''
        if on is None:
            self.snap=not self.snap
        else:
            self.snap=on
        if self.snap:
            self.gui.fade_in('button_snap')
        else:
            self.gui.fade_out('button_snap')

    def set_select_mode(self):
        self.gui.fade_out(['button_rotate', 'button_move', 'button_scale'])
        self.gui.show_hide('',['button_create_ray',
                               'button_create_solid',
                               'button_create_proj',
                               'load_frame',
                               'list_frame',
                               'prop_frame',
                               'button_x',
                               'button_y',
                               'button_z',
                               'button_local',
                               'button_snap',
                               'input_snap',
                               'button_cam_reset',
                               'button_cam_ortho',
                               'button_cam_gimbal',
                               'button_cam_pan',
                               'button_cam_select'])
        self.axis.find('x').hide()
        self.axis.find('y').hide()
        self.axis.find('z').hide()
        self.mode='select'
        self.selected_id=None
        for node in self.objects:
            if node:
                node.hide(self.select_mask)

    def set_move_mode(self):
        self.mode='move'
        self.gui.fade_in('button_move')
        self.gui.fade_out(['button_rotate','button_scale'])
        self.gui.show_hide(['button_x','button_y','button_z','button_local','button_snap','input_snap'],
                            ['button_cam_reset','button_cam_ortho','button_cam_gimbal','button_cam_pan','button_cam_select'])
        self.gui['input_snap'].set('{:.2f}'.format(self.snap_pos))
        for axis in self.active_axis:
            self.gui.fade_in('button_'+axis)
            if self.selected_id is not None:
                self.axis.find(axis).show()
                node=self.objects[self.selected_id]
                self.axis.set_pos(node.get_pos(render))
                if self.global_coords:
                    self.axis.set_hpr(0,0,0)
                else:
                    self.axis.set_hpr(node.get_hpr(render))
        if len(self.active_axis)>2:
            axis=self.active_axis[0]
            self.toggle_axis(axis)

    def set_rotate_mode(self):
        self.mode='rotate'
        self.gui.fade_in('button_rotate')
        self.gui.fade_out(['button_move','button_scale'])
        self.gui.show_hide(['button_x','button_y','button_z','button_local','button_snap','input_snap'],
                            ['button_cam_reset','button_cam_ortho','button_cam_gimbal','button_cam_pan','button_cam_select'])
        self.gui['input_snap'].set('{:.2f}'.format(self.snap_angle))
        for axis in self.active_axis:
            self.gui.fade_in('button_'+axis)
            if self.selected_id is not None:
                self.axis.find(axis).show()
                node=self.objects[self.selected_id]
                self.axis.set_pos(node.get_pos(render))
                if self.global_coords:
                    self.axis.set_hpr(0,0,0)
                else:
                    self.axis.set_hpr(node.get_hpr(render))
        while len(self.active_axis)>1:
            axis=self.active_axis[0]
            self.toggle_axis(axis)

    def set_scale_mode(self):
        self.mode='scale'
        self.gui.fade_in('button_scale')
        self.gui.fade_out(['button_rotate','button_move'])
        self.gui.show_hide(['button_x','button_y','button_z','button_local'],
                            ['button_snap','input_snap', 'button_cam_reset',
                            'button_cam_ortho','button_cam_gimbal','button_cam_pan','button_cam_select'])
        self.active_axis=['x','y','z']
        for axis in self.active_axis:
            self.gui.fade_in('button_'+axis)
            if self.selected_id is not None:
                self.axis.find(axis).show()
                node=self.objects[self.selected_id]
                self.axis.set_pos(node.get_pos(render))
                if self.global_coords:
                    self.axis.set_hpr(0,0,0)
                else:
                    self.axis.set_hpr(node.get_hpr(render))

    def center_pivot(self):
        '''Moves the pivot of the selected node to the centerof its bounding volume.'''
        if self.selected_id is not None:
            node=self.objects[self.selected_id]
            if node is not None:
                for child in node.get_children():
                    if isinstance(child.node(), ModelRoot):
                        bounds=child.get_bounds()
                        pos=bounds.get_center()
                        self.move_pivot(node, pos)
                        break

    def move_pivot(self, node, pos):
        '''Move the pivot of the node to the given pos'''
        for child in node.get_children():
            child.set_pos(-pos)
        node.set_pos(node.get_pos(render)+pos)
        node.set_python_tag('pivot_pos', pos)
        self.axis.set_pos(node.get_pos(render))
        self.update_ui_for_node(node)
        self.do_raytrace()

    def rotate_pivot(self, node, hpr):
        '''Rotate the pivot of node to the given hpr '''
        for child in node.get_children():
            child.set_hpr(-hpr)
        node.set_hpr(node.get_hpr(render)+hpr)
        node.set_python_tag('pivot_hpr', hpr)
        self.axis.set_pos(node.get_pos(render))
        self.update_ui_for_node(node)
        self.do_raytrace()

    def set_coords_local(self, state=None):
        '''If state is True objects will be moved/rotated/scaled
           in their own coordinates (model space),
           else global coordinates are used (world space) '''
        if state is None:
            state=self.global_coords
        if state:
            self.global_coords=False
            self.gui.fade_out('button_local')
            if self.selected_id is not None:
                node=self.objects[self.selected_id]
                pivot_hpr=node.get_python_tag('pivot_hpr')
                self.axis.set_hpr(node.get_hpr()+pivot_hpr)
        else:
            self.global_coords=True
            self.gui.fade_in('button_local')
            self.axis.set_hpr(0,0,0)

    def toggle_axis(self, axis):
        ''' Enable/disable an axis (x,y or z) for movement or rotation,
        only 2 axis can be active in move mode and 1 in rotate mode,
        excess axis will be disabled automatically'''
        if self.mode not in ('select','scale'):
            if axis in self.active_axis:
                self.active_axis.pop(self.active_axis.index(axis))
                self.axis.find(axis).hide()
                self.gui.fade_out('button_'+axis)
            else:
                self.active_axis.append(axis)
                self.axis.find(axis).show()
                self.gui.fade_in('button_'+axis)
            if self.mode == 'move':
                if len(self.active_axis)>2:
                    axis=self.active_axis[0]
                    self.toggle_axis(axis)
            elif self.mode =='rotate':
                while len(self.active_axis)>1:
                    axis=self.active_axis[0]
                    self.toggle_axis(axis)
            self.update_axis()

    def update_axis(self):
        '''Updates the pos/hpr of the axis model, and the plane node used for manipulations'''
        point=self.axis.get_pos(render)
        if 'x' in self.active_axis and 'y' in self.active_axis and 'z' in self.active_axis:
            temp=NodePath('temp')
            temp.set_pos(point)
            temp.look_at(base.cam)
            vec=temp.get_quat().get_forward()
            temp.remove_node()
        elif 'x' in self.active_axis and 'y' in self.active_axis:
            vec=self.axis.get_quat().get_up()
        elif 'x' in self.active_axis and 'z' in self.active_axis:
            vec=self.axis.get_quat().get_forward()
        elif 'y' in self.active_axis and 'z' in self.active_axis:
            vec=self.axis.get_quat().get_right()
        elif 'x' in self.active_axis:
            if self.mode == 'rotate':
                vec=self.axis.get_quat().get_right()
                self.circle.set_color(Vec4(1,0,0, 1), 1)
            else:
                vec=self.axis.get_quat().get_up()
        elif 'y' in self.active_axis:
            if self.mode == 'rotate':
                vec=self.axis.get_quat().get_forward()
                self.circle.set_color(Vec4(0,1,0, 1), 1)
            else:
                vec=self.axis.get_quat().get_up()
        elif 'z' in self.active_axis:
            if self.mode == 'rotate':
                vec=self.axis.get_quat().get_up()
                self.circle.set_color(Vec4(0,0,1,1), 1)
            else:
                vec=self.axis.get_quat().get_right()
        else:
            return
        self.plane=Plane(vec, point)

    def _on_mouse_up(self):
        '''Called when the user lets go of the left mouse button '''
        if self.mouse_is_down:
            self.mouse_is_down=False
            self.last_axis_pos=None
            self.last_hpr=None
            self.circle.hide()
            if self.selected_id is not None:
                node=self.objects[self.selected_id]
                self.update_ui_for_node(node)
                if node.get_python_tag('type')=='ray':
                    node_id=node.get_python_tag('id')
                    #points=self.trace_ray(node_id)
                    #self.draw_line(node_id, points)
                    self.trace_ray_in_thread(node_id)
                else:
                    self.do_raytrace()
                self.axis.set_pos(node.get_pos(render))
                if not self.global_coords:
                    self.axis.set_hpr(node.get_hpr(render))
       
    def _mouse_manipulate_task(self, task):
        '''This task handles movement/rotation/scaling of objects with the mouse''' 
        if self.mouse_is_down:
            if self.selected_id is None:
                return task.again
            else:
                node=self.objects[self.selected_id]
                if node is None:
                    return task.again
            #self.update_axis()
            if base.mouseWatcherNode.hasMouse():
                mpos = base.mouseWatcherNode.get_mouse()
                pos3d = Point3()
                near_point = Point3()
                far_point = Point3()
                base.camLens.extrude(mpos, near_point, far_point)
            if self.plane.intersects_line(pos3d,
                                          render.get_relative_point(base.camera, near_point),
                                          render.get_relative_point(base.camera, far_point)):
                if self.mode == 'move':
                    axis_pos=render.get_relative_point(self.axis, pos3d)
                    if self.snap:
                        axis_pos=Point3(*(self.snap_pos*round(i/self.snap_pos) for i in axis_pos))
                    if self.last_axis_pos is None:
                        self.last_axis_pos=axis_pos
                        return task.again
                    else:
                        delta=axis_pos-self.last_axis_pos
                    if 'x' not in self.active_axis:
                        delta.x=0
                    if 'y' not in self.active_axis:
                        delta.y=0
                    if 'z' not in self.active_axis:
                        delta.z=0
                    if not self.global_coords:
                        delta.x*=-1
                    node.set_pos(self.axis, node.get_pos(self.axis)+delta)
                    #store last pos
                    self.last_axis_pos=axis_pos
                elif self.mode == 'rotate':
                    #make a direction vector
                    vec=self.axis.get_pos()-pos3d
                    #visual aid
                    self.circle.set_scale(vec.length())
                    self.circle.set_pos(self.axis.get_pos())
                    self.circle.heads_up(pos3d, self.plane.get_normal())
                    self.circle.show()
                    #we just need the direction
                    vec.normalize()
                    #nothing more to do at this point if we have no stored vector
                    if self.last_vec is None:
                        self.last_vec=vec
                        return task.again
                    #get an angle between the vec and the vec from last frame;
                    #the angle is positive if you can rotate last_vec to vec clockwise (I assume along
                    #the smallest possible arc), when looking in the direction of the plane normal
                    angle=self.last_vec.signed_angle_deg(vec, self.plane.get_normal())
                    if self.snap:
                        if abs(angle) < min(0.2, self.snap_angle/0.25):
                            if angle<0:
                                angle=-self.snap_angle
                            if angle>0:
                                angle=self.snap_angle
                        else:
                            angle=self.snap_angle*round(angle/self.snap_angle)
                    q=Quat()
                    q.set_from_axis_angle(angle, self.plane.get_normal())
                    q_model = node.get_quat()
                    # multiply the model quaternion by the axis-angle-quaternion
                    q_model *= q
                    node.set_quat(q_model)
                    #print some debug)
                    #print(self.model.get_hpr(render))
                    #store the vec for next frame
                    self.last_vec=vec
                elif self.mode == 'scale':
                    distance=(self.axis.get_pos(render)-pos3d).length()
                    if self.last_scale is None:
                        self.last_scale=distance
                        return task.again
                    delta_distance= self.last_scale-distance
                    self.last_scale=distance
                    node.set_scale(node.get_scale(render)-Vec3(delta_distance))
        return task.again

    def _on_mouse_down(self):
        '''Called when the mouse button is pressed '''
        if base.mouseWatcherNode.has_mouse():
            m_pos = base.mouseWatcherNode.get_mouse()
            self.mouse_ray.set_from_lens(base.camNode, m_pos.get_x(), m_pos.get_y())

            self.cam_ray_trav.traverse(render)
            if self.cam_ray_handler.get_num_entries() > 0:
                self.cam_ray_handler.sort_entries()
                hit = self.cam_ray_handler.get_entry(0).get_into_node_path()
                #print(hit)
                if hit.get_parent().has_python_tag('id'):
                    self.selected_id=hit.get_parent().get_python_tag('id')
                    for node in self.objects:
                        if node:
                            if node.get_python_tag('id')==self.selected_id:
                                node.show_through(self.select_mask)
                            else:
                                node.hide(self.select_mask)
                    self.axis.set_pos(hit.get_parent().get_pos(render))
                    if self.global_coords:
                        self.axis.set_hpr(0,0,0)
                    else:
                        self.axis.set_hpr(hit.get_parent().get_hpr(render))
                    if self.mode != 'select':
                        self.update_ui_for_node(hit.get_parent())
                        self.update_axis()
                        self.last_axis_pos = None
                        self.last_hpr=None
                        self.last_vec=None
                        self.last_scale=None
                        self.mouse_is_down=True
                        for axis in self.active_axis:
                            self.axis.find(axis).show()
                    else:
                        self.select_by_id(self.selected_id)
                    return
            #else:
            self.set_select_mode()

    def _setup_select_buff(self):
        '''Create  an off-screen buffer to render the selected object,
        only the depth is rendered and a edge detect shader is used to render an outline.
        '''
        render.hide(self.select_mask)
        #render depth
        win_size=base.get_size()
        winprops = WindowProperties()
        winprops.set_size(*win_size)
        props = FrameBufferProperties()
        props.set_rgb_color(False)
        depth_bits=base.win.get_fb_properties().get_depth_bits()
        props.set_depth_bits(depth_bits)
        self.depth_buff =base.graphicsEngine.make_output(base.pipe, "depth_buff", -1,
                                            props, winprops,
                                            GraphicsPipe.BF_resizeable,
                                            base.win.get_gsg(), base.win)
        self.depth_tex=Texture()
        self.depth_buff.add_render_texture(tex=self.depth_tex, mode=GraphicsOutput.RTMBindOrCopy, bitplane=GraphicsOutput.RTPDepth)
        self.select_cam = base.make_camera(win=self.depth_buff,
                                          lens=base.cam.node().get_lens(),
                                          scene=render,
                                          mask=self.select_mask)
        self.select_cam.reparent_to(base.camera)

        cm = CardMaker("card")
        cm.setFrameFullscreenQuad ()
        self.overlay= NodePath(cm.generate())
        self.overlay.reparent_to(render2d)
        self.overlay.set_shader(Shader.load(Shader.SLGLSL, "shaders/select_v.glsl","shaders/select_f.glsl"),1)
        self.overlay.set_shader_input('depth_tex', self.depth_tex)
        render.set_shader_input('depth_tex', self.depth_tex)
        self.overlay.set_shader_input('color', Vec3(0.2, 0.75, 0.45))
        self.overlay.set_shader_input('thickness', 2.0)
        self.overlay.set_transparency(TransparencyAttrib.M_alpha)
        self.overlay.set_bin("fixed", -20)

    def _on_exit(self):
        '''Function called when  the window is closed  by  the user'''
        self.deamon_stop_event.set()
        render.hide()
        self.overlay.hide()
        self.gui.hide_root()
        base.destroy()
        os._exit(1) #this way we don't get error from 'hanging events'

    def render_frames(self):
        '''Wrapper  for  rendering a few frames '''
        base.graphicsEngine.render_frame()
        base.graphicsEngine.render_frame()
        if self.threading_model != '':
            base.graphicsEngine.render_frame()
            base.graphicsEngine.render_frame()

    def print_txt(self, *args, ttl=3.0):
        '''Prints to the screen'''
        text=''.join((str(i) for i in args))
        if text:
            text='\1shadow\1'+text+'\2'
            self.gui['on_screen_txt'].node().set_text(text)
            self.gui.show_hide('on_screen_txt')
            self.last_txt=text
            self.last_txt_ttl=ttl
        else:
            self.gui.show_hide('', 'on_screen_txt')

    def wavelength_to_rgb(self, wavelength, gamma=0.5):
        '''This converts a given wavelength of light to an
        approximate RGB color value. The wavelength must be given
        in nanometers in the range from 380 nm through 750 nm
        (789 THz through 400 THz).
        Based on code by Dan Bruton
        http://www.physics.sfasu.edu/astro/color/spectra.html
        '''
        wavelength = max(380, min(750, wavelength))
        if wavelength >= 380 and wavelength <= 440:
            attenuation = 0.3 + 0.7 * (wavelength - 380) / (440 - 380)
            R = ((-(wavelength - 440) / (440 - 380)) * attenuation) ** gamma
            G = 0.0
            B = (1.0 * attenuation) ** gamma
        elif wavelength >= 440 and wavelength <= 490:
            R = 0.0
            G = ((wavelength - 440) / (490 - 440)) ** gamma
            B = 1.0
        elif wavelength >= 490 and wavelength <= 510:
            R = 0.0
            G = 1.0
            B = (-(wavelength - 510) / (510 - 490)) ** gamma
        elif wavelength >= 510 and wavelength <= 580:
            R = ((wavelength - 510) / (580 - 510)) ** gamma
            G = 1.0
            B = 0.0
        elif wavelength >= 580 and wavelength <= 645:
            R = 1.0
            G = (-(wavelength - 645) / (645 - 580)) ** gamma
            B = 0.0
        elif wavelength >= 645 and wavelength <= 750:
            attenuation = 0.3 + 0.7 * (750 - wavelength) / (750 - 645)
            R = (1.0 * attenuation) ** gamma
            G = 0.0
            B = 0.0
        else:
            R = 1.0
            G = 1.0
            B = 1.0
        return Vec4(R,G,B, 1.0)

    def do_raytrace(self):
        '''Update all the rays in the scene '''
        self.in_que.clear()
        self.out_que.clear()
        for node in self.objects:
            if node:
                if node.get_python_tag('type') in ('ray', 'projector'):
                    node_id=node.get_python_tag('id')
                    self.trace_ray_in_thread(node_id)

    def ray_random_color(self):
        '''Changes the current ray color to a different, random color '''
        if self.selected_id is not None:
            node=self.objects[self.selected_id]
            if node:
                old_color=node.get_python_tag('line_color')
                color=self.colors.new_color(old_color)
                node.set_python_tag('line_color', color)
                line=node.get_python_tag('line')
                line.set_color(color, 1)
                self.gui['ray_color_input'].set('{0:.2f}, {1:.2f}, {2:.2f}'.format(*color))

    def wavelength_to_color(self):
        '''Changes the current ray color to based on its wavelength '''
        if self.selected_id is not None:
            node=self.objects[self.selected_id]
            if node:
                wavelength=node.get_python_tag('wave')
                color=self.wavelength_to_rgb(wavelength)
                node.set_python_tag('line_color', color)
                line=node.get_python_tag('line')
                line.set_color(color, 1)
                self.gui['ray_color_input'].set('{0:.2f}, {1:.2f}, {2:.2f}'.format(*color))

    def make_deformed_quad(self, points):
        '''Creates a mesh from a dict of points '''
        format = GeomVertexFormat.getV3t2()
        vdata = GeomVertexData('quad', format, Geom.UHDynamic)
        vertex = GeomVertexWriter(vdata, 'vertex')
        texcoord = GeomVertexWriter(vdata, 'texcoord')

        rows=max(points, key=lambda item:item[0])[0]+1
        columns=max(points, key=lambda item:item[1])[1]+1

        num_rows=0
        for (x, y), point in points.items():
            vertex.add_data3(point)
            texcoord.add_data2(x/(rows-1), y/(columns-1))
            num_rows+=1

        geom = Geom(vdata)
        tris = GeomTriangles(Geom.UHDynamic)
        for i in range(num_rows-1-columns):
            if (i+1)% columns !=0:
                tris.add_vertices(i, i+columns, i+1+columns)
                tris.add_vertices(i+1, i, i+1+columns)
        geom.add_primitive(tris)
        snode =GeomNode('quad')
        snode.add_geom(geom)
        return render.attach_new_node(snode)

    def draw_projection(self, node_id, points):
        '''Draw 4 rays and a geom for the projector with node_id'''
        node=self.objects[node_id]
        if node is None:
            return

        if node.has_python_tag('line_color'):
            color=node.get_python_tag('line_color')
        else:
            color=self.colors.new_color()
            node.set_python_tag('line_color', color)

        if node.has_python_tag('line'):
            node.get_python_tag('line').remove_node()
        if node.has_python_tag('quad'):
            node.get_python_tag('quad').remove_node()

        #find corners
        start_pos=node.get_pos()
        for child in node.get_children():
            if isinstance(child.node(), ModelRoot):
                start_pos=child.get_pos(render)
                break
        rows=node.get_python_tag('sample_h')
        columns=node.get_python_tag('sample_v')
        projector_size=node.get_python_tag('proj_w')*node.get_python_tag('proj_h')
        corner_rays=((0,0),
                     (0,columns-1),
                     (rows-1, 0),
                     (rows-1,columns-1)
                    )
        try:
            line=node.get_python_tag('line')
            line.remove_node()
        except:
            pass
        l=LineSegs()
        l.set_thickness(2.0)
        #l.move_to(node.get_pos(render))
        #start point may differ because of pivot shift
        for child in node.get_children():
            if isinstance(child.node(), ModelRoot):
                l.move_to(child.get_pos(render))
                break
        last_ray_id=points[0][1]
        for (point, ray_id) in points:
            if point:
                if ray_id in corner_rays:
                    if ray_id!=last_ray_id:
                        last_ray_id=ray_id
                        l.move_to(point)
                    else:
                        l.draw_to(point)

        end_points={}
        for (point, ray_id) in points:
            if point:
                end_points[ray_id]=point

        line=render.attach_new_node(l.create())
        #line.set_shader(self.line_shader)
        line.set_color(color, 1)
        line.set_transparency(TransparencyAttrib.M_multisample)
        line.set_antialias(AntialiasAttrib.MLine)
        node.set_python_tag('line', line)
        line.wrt_reparent_to(node)
        line.show_through(self.camera_mask)
        line.hide(self.select_mask)

        try:
            quad=node.get_python_tag('quad')
            quad.remove_node()
        except:
            pass
        quad=self.make_deformed_quad(end_points)
        quad.wrt_reparent_to(node)
        bounds=quad.get_bounds()
        radius=bounds.get_radius()
        if radius > projector_size*100.0:
            quad.hide()
            self.print_txt('Projection out of bounds')
        node.set_python_tag('quad', quad)
        tex=loader.load_texture(node.get_python_tag('texture'))
        tex.set_anisotropic_degree(2)
        quad.set_texture(tex, 1)
        quad.set_depth_offset(1)        
        #quad.set_depth_test(False)
        #quad.set_bin('fixed', 100)
        quad.set_two_sided(True)
        quad.set_transparency(TransparencyAttrib.M_none)
        #quad.set_render_mode_wireframe()


    def draw_line(self, node_id, points, color=None):
        '''Draw lines for node_id '''
        node=self.objects[node_id]
        if node is None:
            return
        if color is None:
            if node.has_python_tag('line_color'):
                color=node.get_python_tag('line_color')
            else:
                color=self.colors.new_color()
                node.set_python_tag('line_color', color)

        line=node.get_python_tag('line')
        #color=line.get_color()
        line.remove_node()
        l=LineSegs()
        l.set_thickness(2.0)
        #l.move_to(node.get_pos(render))
        #start point may differ because of pivot shift
        for child in node.get_children():
            if isinstance(child.node(), ModelRoot):
                l.move_to(child.get_pos(render))
                break

        last_ray_id=points[0][1]
        for (point, ray_id) in points:
            if point:
                if ray_id!=last_ray_id:
                    last_ray_id=ray_id
                    l.move_to(point)
                else:
                    l.draw_to(point)
        line=render.attach_new_node(l.create())
        #line.set_shader(self.line_shader)

        #make points
        point_node=line.copy_to(render)
        point_node.set_render_mode(RenderModeAttrib.MPoint, 1.0)
        shader_attrib = ShaderAttrib.make(self.point_shader)
        shader_attrib = shader_attrib.setFlag(ShaderAttrib.F_shader_point_size, True)
        point_node.set_attrib(shader_attrib)
        point_node.wrt_reparent_to(line)
        point_node.set_depth_test(False)
        point_node.set_depth_write(True)
        point_node.set_bin('fixed', 10)

        line.set_color(color, 1)
        line.set_transparency(TransparencyAttrib.M_multisample)
        line.set_antialias(AntialiasAttrib.MLine)
        #line.set_depth_test(False)
        #line.set_depth_write(True)
        #line.set_bin('fixed', 10)
        node.set_python_tag('line', line)
        line.wrt_reparent_to(node)
        line.show_through(self.camera_mask)
        line.hide(self.select_mask)

    def _update(self, task):
        '''Update task, called every frame'''
        dt = globalClock.getDt()
        render.set_shader_input("camera_pos", base.cam.get_pos(render))
        txt=self.gui['on_screen_txt'].node().get_text()
        if txt == self.last_txt:
            self.last_txt_ttl-=dt
        if self.last_txt_ttl<0:
            self.gui.show_hide(None, 'on_screen_txt')

        if self.out_que:
            node_id, points = self.out_que.popleft()
            node=self.objects[node_id]
            if node and points:
                if node.get_python_tag('type')=='projector':
                    self.draw_projection(node_id, points)
                else:
                    self.draw_line(node_id, points)
        return task.again

    def _get_ray_vector(self, h, p):
        q=Quat()
        q.set_hpr((h, 0.0, 0.0))  # quaternion describing heading only
        v=q.get_forward()
        q_p=Quat()  # quaternion describing pitch only
        q_p.set_hpr((0.0, p, 0.0))
        v_p=q_p.get_forward()  # "pitch vector", needed for edge-case checking
        q *= q_p  # multiply the quaternions in the correct order
        v2=q.get_forward()
        # check edge cases
        if abs(v2.y) < .001:
            v.x = 0.0 if abs(v_p.y) < .001 else v2.x
            v.y = 0.0
        elif abs(v.y) > .001:
            v2 *= v.y/v2.y  # make the Y-components of both vectors equal
        v.z = v2.z
        v.normalize()
        return v

    def ray_daemon(self, stop_event):
        '''Daemon function, executes the ray tracing in a separate thread '''
        while not stop_event.is_set():
            sleep_time=0.16666 #~1.0/60.0
            if self.in_que:
                poped= self.in_que.popleft()
                node=self.objects[poped]
                node_id=poped
                if node is None:
                    continue
                column_angle=node.get_python_tag('angle_v')
                row_angle=node.get_python_tag('angle_h')
                scale=node.get_scale()
                if node.get_python_tag('type')=='ray':
                    wavelength=node.get_python_tag('wave')
                    rows=node.get_python_tag('rows')
                    columns=node.get_python_tag('columns')
                    row_offset=node.get_python_tag('row_offset')
                    column_offset=node.get_python_tag('column_offset')
                elif node.get_python_tag('type')=='projector':
                    wavelength=550
                    rows=node.get_python_tag('sample_h')
                    columns=node.get_python_tag('sample_v')
                    w=node.get_python_tag('proj_w')
                    h=node.get_python_tag('proj_h')
                    row_offset=w/(rows-1.0)
                    column_offset=h/(columns-1.0)
                points=[]
                q=Quat()
                for child in node.get_children():
                    if isinstance(child.node(), ModelRoot):
                        node=child
                        break
                for row in range(rows):
                    #scale the horizontal angles
                    if rows>1:
                        h = row_angle-row_angle*row/(rows-1)*2.0
                    else:
                        h=row_angle
                    for column in range(columns):
                        local_origin=Point3(row*row_offset/scale.x, 0, column*column_offset/scale.z)
                        origin=render.get_relative_point(node, local_origin)
                        #scale the vertical angles
                        if columns>1:
                            p = -1.0*(column_angle-column_angle*column/(columns-1)*2.0)
                        else:
                            p= column_angle
                        v=self._get_ray_vector(h, p)
                        v*=1000.0 #target a point 1000.0 units away
                        target=render.get_relative_vector(node, v)+origin
                        self.trace_ray((row, column),origin, wavelength, origin, target, 1.0, points, [])
                self.out_que.append((node_id, points))
                sleep_time=0
            time.sleep(sleep_time)

    def trace_ray_in_thread(self, node_id):
        '''Adds a node to the raytracing que'''
        self.in_que.append(node_id)

    def trace_ray(self, ray_id, start_pos, wavelength, origin, target, last_ior, points=[], split=[]):
        '''Trace a ray from start_pos to target
        ray_id - a (row, column) tuple that is used to identity the ray
        start_pos - the point this ray starts from
        wavelength - wavelength in nm
        origin -the point from where all the rays in the chain started from
        target - the vector describing where the ray is targeted
        last_ior - the ior of rhe last objext hit (if any) or 1.0 for air
        points - list of points hit so far
        split - list of points where the ray splits'''
        if points:
            if points[-1][0] is None:
                points.append((start_pos, ray_id))
                #print('insert start')
        if len(points)>self.max_points:
            self.print_txt('To many points!')
            return
        max_ray_length=1000.0
        hit=self._ray(origin, target)
        if hit.has_hit:
            points.append((hit.pos, ray_id))
            if last_ior == 1.0:
                ior=hit.node.get_python_tag('material').n(wavelength)
            elif hit.next_node is None:
                ior=1.0
            elif not hit.next_node.has_python_tag('id'):
                ior=1.0
            elif hit.node == hit.next_node:
                ior=1.0
            elif (hit.pos-hit.next_pos).length()>0.5:
                ior=1.0
            elif (hit.pos-hit.next_pos).length()<=0.5:
                ior=hit.next_node.get_python_tag('material').n(wavelength)
            else:
                ior=hit.node.get_python_tag('material').n(wavelength)
            object_type=hit.node.get_python_tag('type')
            if object_type=='reflect':
                reflected=self.reflect(target.normalized(), hit.normal)
                target=reflected*max_ray_length
                self.trace_ray(ray_id, start_pos, wavelength, hit.pos, target, 1.0, points, split)
            elif object_type=='refract':
                refracted=self.refract(target.normalized(), hit.normal, last_ior, ior)
                #if not refracted.is_internal:
                target=refracted.vector*max_ray_length
                self.trace_ray(ray_id, start_pos, wavelength, hit.pos, target, ior, points, split)
                #else:
                #    points.append(None)
            elif object_type=='beamsplit':
                if last_ior==1.0 and not split:
                    split.append((Vec3(*target.normalized()), Vec3(*hit.normal), Point3(*hit.pos)))
                refracted=self.refract(target.normalized(), hit.normal, last_ior, ior)
                #if not refracted.is_internal:
                target=refracted.vector*max_ray_length
                self.trace_ray(ray_id, start_pos, wavelength, hit.pos, target, ior, points, split)
                #else:
                #    points.append(None)
            else:
                points.append((None, ray_id))
        else:
            #print(hit)
            points.append((hit.pos, ray_id))
            if split:
                I, N, origin= split.pop()
                points.append((None, ray_id))
                points.append((origin, ray_id))
                reflected=self.reflect(I, N)
                target=reflected*max_ray_length
                self.trace_ray(ray_id, start_pos, wavelength, origin, target, 1.0, points, split)
            else:
                if points[-1][0] is None:
                    points.append((start_pos, ray_id))
                else:
                    points.append((None, ray_id))
                #points.append(start_pos)
                #print('insert end')

    def reflect(self, I, N):
        '''Returns the reflection direction'''
        return I - N * 2.0 * I.dot(N)

    def refract(self, I, N, ior_0, ior_1):    
        '''Returns if the refraction is internal and the refraction direction'''
        cosi = max(-1.0, min(1.0, I.dot(N) ))
        etai = ior_0
        etat = ior_1
        n = N
        if (cosi < 0):
            cosi = -cosi
        else:
            etai = ior_1
            etat = ior_0
            n= -N
        eta = etai / etat;
        k = 1.0 - eta * eta * (1.0 - cosi * cosi);
        if k < 0:
            return Traced(True, self.reflect(I, n))
        else:
            return  Traced(True, I * eta  + n * (eta * cosi - sqrt(k)))


    def _ray(self, origin, target):
        '''Performs collision detection for a ray going from origin to target '''
        origin=Point3(origin)
        self.collison_ray.set_origin(origin)
        self.collison_ray.set_direction(target)
        self.ray_trav.clear_colliders()
        self.ray_trav.add_collider(self.collison_ray_np, self.ray_handler)
        self.ray_trav.traverse(render)
        return_hit=None
        #print ('from', origin)
        #print('to', target)
        if self.ray_handler.get_num_entries() > 0:
            self.ray_handler.sort_entries()
            for entry in self.ray_handler.get_entries():
                #get the data
                hit_node=entry.get_into_node_path().get_parent()
                hit=entry.get_surface_point(render)
                normal=entry.get_surface_normal(render)
                if normal.dot(target)>0.0:
                    normal*=-1.0
                    normal.normalize()
                #if there is no id, we hit the scene barrier
                if not hit_node.has_python_tag('id'):
                    self.ray_trav.clear_colliders()
                    if return_hit is not None:
                        return_hit= Hit(return_hit.has_hit, return_hit.pos, return_hit.normal, return_hit.node, hit_node, hit)
                        self.ray_trav.clear_colliders()
                        return return_hit
                    return Hit(False, target, Vec3(0,0,0), None, None, Point3(0,0,0))
                #skip false hits
                if not hit.almost_equal(origin):
                    #print(hit_node, hit, normal)
                    #we don't have any valid hit yet
                    if return_hit is None:
                        return_hit= Hit(True, hit, normal, hit_node, None, Point3(0,0,0))
                    elif return_hit.next_node is None: #what's the next hit node?
                        if not return_hit.pos.almost_equal(hit):
                            self.ray_trav.clear_colliders()
                            return Hit(return_hit.has_hit, return_hit.pos, return_hit.normal, return_hit.node, hit_node, hit)

        self.ray_trav.clear_colliders()
        if return_hit is not None:
            return return_hit
        return Hit(False, target, Vec3(0,0,0), None, None, Point3(0,0,0))

    def _make_smooth_mesh(self, triangles, threshold=20.0, flip_normal=True):
        '''Creates a smooth mesh from a list of triangles'''
        egg = EggData()
        pool = EggVertexPool('vertex_pool')
        egg.add_child(pool)
        group = EggGroup('mesh')
        egg.add_child(group)
        for points in triangles:
            epoly = EggPolygon()
            group.add_child(epoly)
            if flip_normal:
                points.reverse()
            for point in points:
                eprim = epoly
                ev = EggVertex()
                ev.set_pos(Point3D(*point))
                ev.set_normal(Vec3D(0,0,0))
                pool.add_vertex(ev)
                eprim.add_vertex(ev)
        if threshold is not None:
            egg.recompute_vertex_normals(threshold)
        return NodePath(loadEggData( egg ))

    def _get_triangles(self, mesh, points=[]):
        '''Returns all triangles in a mesh '''
        for child in mesh.get_children():
            node=child.node()
            if node.is_geom_node():
                node.decompose()
                for geom in node.get_geoms():
                    vdata = geom.getVertexData()
                    vertex = GeomVertexReader(vdata, 'vertex')
                    for prim in geom.getPrimitives():
                        num_primitives=prim.getNumPrimitives()
                        for p in range(num_primitives):
                            s = prim.getPrimitiveStart(p)
                            e = prim.getPrimitiveEnd(p)
                            triangle=[]
                            for i in range(s, e):
                                vi = prim.getVertex(i)
                                vertex.setRow(vi)
                                triangle.append(Point3(*vertex.getData3f()))
                            points.append(triangle)
            else:
                self._get_triangles(child, points)
        return points

    def set_wireframe(self):
        '''GUI callback - set the wireframe render mode on/off'''
        if self.selected_id is not None:
            node=self.objects[self.selected_id]
            if node:
                render_mode=node.get_render_mode()
                if render_mode==RenderModeAttrib.M_wireframe:
                    node.set_render_mode_filled()
                    self.gui.fade_out('button_wireframe')
                else:
                    node.set_render_mode_wireframe()
                    self.gui.fade_in('button_wireframe')


    def set_gloss(self, value):
        '''Set glossiness [0.0-1.0] for the current object '''
        if self.selected_id is not None:
            node=self.objects[self.selected_id]
            if node:
                node.set_shader_input('gloss', float(value))
                node.set_python_tag('gloss', float(value))

    def set_alpha(self, value):
        '''Set transparency [0.0-1.0] for the current object '''
        if self.selected_id is not None:
            node=self.objects[self.selected_id]
            if node:
                c=node.get_color()
                c[3]=1.01-value
                node.set_color(c)

    def scale_emmiter_geom(self, node):
        if node:
            if node.get_python_tag('type')=='ray':
                x=node.get_python_tag('rows')-1
                z=node.get_python_tag('columns')-1
                x*=node.get_python_tag('row_offset')
                z*=node.get_python_tag('column_offset')
                node.set_scale(max(x, 0.05), 1.0, max(z, 0.05))
            elif node.get_python_tag('type')=='projector':
                x=node.get_python_tag('proj_w')
                z=node.get_python_tag('proj_h')
                node.set_scale(max(x, 0.05), 1.0, max(z, 0.05))

    def make_projector(self, color=None, wavelength=None, model_name=None,
                pos=None, hpr=None, scale=None, angle_h=22.5, angle_v=22.5,
                texture='images/test1.png',sample_v=2, sample_h=2,
                proj_w=1.0, proj_h=1.0):
        '''Creates a new projector object'''
        root=self.scene.attach_new_node('project')
        model=loader.load_model('data/box')
        model.reparent_to(root)
        self.objects.append(root)
        id=len(self.objects)-1
        if model_name is None:
            model_name='Proj. '+str(id)
        if wavelength is None:
            wavelength=650
        root.set_python_tag('id', id)
        root.set_python_tag('type', 'projector')
        root.set_python_tag('angle_h', angle_h)
        root.set_python_tag('angle_v', angle_v)
        root.set_python_tag('sample_v',  sample_v)
        root.set_python_tag('sample_h',  sample_h)
        root.set_python_tag('proj_w', proj_w)
        root.set_python_tag('proj_h', proj_h)
        root.set_python_tag('pivot_pos', Point3(0,0,0))
        root.set_python_tag('pivot_hpr', Point3(0,0,0))
        root.set_python_tag('texture', texture)
        if color is None:
            color=self.colors.new_color()
        else:
            color=Vec4(*color)
        root.set_python_tag('line_color', color)
        self.gui['proj_color_input'].set('{0:.2f}, {1:.2f}, {2:.2f}'.format(*color))
        root.set_transparency(TransparencyAttrib.M_alpha)
        #collision
        c_node=root.attach_new_node(CollisionNode('cnode'))
        for points in self._get_triangles(model, []):
            if len(points)==3:
                c_node.node().add_solid(CollisionPolygon(*points))
        c_node.node().set_into_collide_mask(BitMask32.bit(1))
        root.set_python_tag('line', NodePath('line'))
        root.set_python_tag('name', model_name)
        #root.hide(self.camera_mask)
        root.set_transparency(TransparencyAttrib.M_alpha)
        root.set_color(0.5,0.5,0.5, 0.5)
        if pos is None:
            root.set_pos(0,-2, 0.125)
            self.select_by_id(id)
        else:
            root.set_pos(*pos)
        if hpr is not None:
            root.set_hpr(*hpr)
        if scale is not None:
            root.set_scale(*scale)
        else:
            self.scale_emmiter_geom(root)
        #button
        object_type='proj.'
        self.gui.button(txt='{name:<34} {type:<10} {id}'.format(name=model_name, type=object_type, id=id),
                        sort_dict={'name':model_name, 'type':object_type, 'id':id},
                        name='button_object_'+str(id),
                        width=384,
                        pos=[16,0],
                        parent='list_frame_canvas',
                        mono_font=True,
                        align='left',
                        cmd='app.select_by_id('+str(id)+')')
        self.gui.sort_buttons('list_frame_canvas', 'id', False)
        self.trace_ray_in_thread(id)

    def make_ray(self, color=None, wavelength=None, model_name=None,
                pos=None, hpr=None, scale=None,
                columns=2, rows=2, column_offset=0.1, row_offset=0.1,
                angle_h=0, angle_v=0):
        '''Creates a new ray object'''
        root=self.scene.attach_new_node('ray')
        model=loader.load_model('data/box')
        model.reparent_to(root)
        self.objects.append(root)
        id=len(self.objects)-1
        if model_name is None:
            model_name='Ray '+str(id)
        if wavelength is None:
            wavelength=650
        root.set_python_tag('id', id)
        root.set_python_tag('type', 'ray')
        root.set_python_tag('wave', wavelength)
        root.set_python_tag('rows', rows)
        root.set_python_tag('columns', columns)
        root.set_python_tag('column_offset',column_offset )
        root.set_python_tag('row_offset', row_offset)
        root.set_python_tag('angle_h', angle_h)
        root.set_python_tag('angle_v', angle_v)
        root.set_python_tag('pivot_pos', Point3(0,0,0))
        root.set_python_tag('pivot_hpr', Point3(0,0,0))
        if color is None:
            color=self.colors.new_color()
        else:
            color=Vec4(*color)
        root.set_python_tag('line_color', color)
        self.gui['ray_color_input'].set('{0:.2f}, {1:.2f}, {2:.2f}'.format(*color))
        root.set_transparency(TransparencyAttrib.M_alpha)
        #collision
        c_node=root.attach_new_node(CollisionNode('cnode'))
        for points in self._get_triangles(model, []):
            if len(points)==3:
                c_node.node().add_solid(CollisionPolygon(*points))
        c_node.node().set_into_collide_mask(BitMask32.bit(1))
        root.set_python_tag('line', NodePath('line'))
        root.set_python_tag('name', model_name)
        #mesh.hide(self.camera_mask)
        root.set_transparency(TransparencyAttrib.M_alpha)
        root.set_color(0.5,0.5,0.5, 0.5)
        if pos is None:
            root.set_pos(0,-2, 0.125)
            self.select_by_id(id)
        else:
            root.set_pos(*pos)
        if hpr is not None:
            root.set_hpr(*hpr)
        if scale is not None:
            root.set_scale(*scale)
        else:
            self.scale_emmiter_geom(root)
        #button
        object_type='ray'
        self.gui.button(txt='{name:<34} {type:<10} {id}'.format(name=model_name, type=object_type, id=id),
                        sort_dict={'name':model_name, 'type':object_type, 'id':id},
                        name='button_object_'+str(id),
                        width=384,
                        pos=[16,0],
                        parent='list_frame_canvas',
                        mono_font=True,
                        align='left',
                        cmd='app.select_by_id('+str(id)+')')
        self.gui.sort_buttons('list_frame_canvas', 'id', False)
        self.trace_ray_in_thread(id)

    def load_object(self, model, select=False, object_type='refract',
                    threshold=None, flip_normal=None):
        '''Loads a model, creates collision solids
        also adds the loaded model to self.models and creates a button on the object list
        '''
        try:
            mesh=loader.load_model(model)
            mesh.clear_model_nodes()
            mesh.flatten_strong()
        except:
            self.print_txt('Error loading model '+model)
            return
        model_name=Filename(model).getBasenameWoExtension()
        triangles=self._get_triangles(mesh, [])
        #make a new mesh and smooth it
        #if Filename(model).get_extension() not in ('bam', 'egg'):
        if threshold is None:
            threshold=self.gui['smooth_input'].get()
            try:
                threshold=math_eval(threshold)
            except:
                threshold=None
        if flip_normal is None:
            flip_normal=self.flip_model_normal
        mesh=self._make_smooth_mesh(triangles, threshold, flip_normal)
        if self.write_model_bam:
            mesh.write_bam_file(model+'.bam')
        root=self.scene.attach_new_node(model_name)
        mesh.reparent_to(root)
        self.objects.append(root)
        id=len(self.objects)-1
        root.set_python_tag('id', id)
        root.set_python_tag('flip_normal', flip_normal)
        root.set_python_tag('threshold', threshold)
        root.set_python_tag('type', object_type)
        root.set_python_tag('material', mat_spy.SiO2())
        root.set_python_tag('material_name', 'SiO2')
        root.set_python_tag('name', model_name)
        root.set_python_tag('model_file', model)
        root.set_python_tag('gloss', 0.0)
        root.set_python_tag('pivot_pos', Point3(0,0,0))
        root.set_python_tag('pivot_hpr', Point3(0,0,0))
        root.set_color(LColor(1.0))
        root.set_transparency(TransparencyAttrib.M_alpha)
        root.set_shader(self.lit_shader, 1)
        root.set_shader_input('gloss', 0.0)
        #collision
        c_node=root.attach_new_node(CollisionNode('cnode'))
        for points in triangles:
            #print(points)
            c_node.node().add_solid(CollisionPolygon(*points))
            #c_node.node().add_solid(CollisionPolygon(*reversed(points)))
        c_node.node().set_into_collide_mask(self.obj_mask)

        #mesh.set_attrib(CullFaceAttrib.make(CullFaceAttrib.M_cull_clockwise))
        #button
        self.gui.button(txt='{name:<34} {type:<10} {id}'.format(name=model_name, type=object_type, id=id),
                        sort_dict={'name':model_name, 'type':object_type, 'id':id},
                        name='button_object_'+str(id),
                        width=384,
                        pos=[16,0],
                        parent='list_frame_canvas',
                        mono_font=True,
                        align='left',
                        cmd='app.select_by_id('+str(id)+')')
        self.gui.sort_buttons('list_frame_canvas', 'id', False)
        self.gui.show_hide('', 'model_frame')
        if select:
            self.select_by_id(id)
            self.do_raytrace()
        return root

#run the app
application=App()
application.base.run()
