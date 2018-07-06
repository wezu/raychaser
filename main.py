#opticalmaterialspy and panda3d are needed to run this script
#pip install opticalmaterialspy
#pip install panda3d

import opticalmaterialspy as mat_spy
from panda3d.core import *
loadPrcFile('options.prc')
#loadPrcFileData('', 'default-model-extension .bam')
loadPrcFileData('','textures-power-2 None')
loadPrcFileData('','load-file-type p3assimp')
#loadPrcFileData("", "threading-model Cull/Draw")
#parse some options
msaa=ConfigVariableInt('antialias_msaa', 0).get_value()
if msaa > 0:
    loadPrcFileData('', 'framebuffer-multisample 1')
    loadPrcFileData('', 'multisamples '+str(msaa))
#loadPrcFileData('', 'want-pstats 1')

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
from math import sqrt
from collections import namedtuple
#py 2 and 3 compatibility
if sys.version_info >= (3, 0):
    import builtins
else:
    import __builtin__ as builtins

from camera import CameraControler
from loading_screen import loading
from gui import UI
from gui import math_eval


#set the window decoration before we start
wp = WindowProperties.getDefault()
wp.set_title("Raychaser  wezu.dev@gmail.com")
wp.set_icon_filename('gui/wezu.ico')
WindowProperties.setDefault(wp)


#this is used for ray tests return values
Hit = namedtuple('Hit', 'has_hit pos normal node')
Traced = namedtuple('Traced', 'is_internal vector')

class App(DirectObject):
    def __init__(self):
        builtins.app=self
        #basic stuff
        self.base = ShowBase.ShowBase()
        self.base.disableMouse()
        self.base.set_background_color(0.1, 0.1, 0.1)
        self.base.camLens.set_near_far(0.1, 200.0)
        render.set_shader_auto(True)
        render.set_antialias(AntialiasAttrib.MMultisample)
        with loading():
            #vars
            self.select_mask=BitMask32.bit(11)
            self.camera_mask=BitMask32.bit(22)
            self.obj_mask=BitMask32.bit(1)
            self.obj_mask.set_bit(2)
            self.ray_mask=BitMask32.bit(2)
            self.pre_click_mouse_pos=Vec2(0,0)
            self.last_mouse_pos=Vec2(0,0)
            self.mouse_lock=False
            self.mouse_lock_frame_skip=True
            self.selected_id=None

            self.mode='select'
            self.global_coords=True
            self.active_axis=['x','y','z']
            self.objects=[]
            #cam mask
            base.cam.node().set_camera_mask(self.camera_mask)

            #ui
            self.gui=UI()
            self.gui.load_from_file('gui/gui.json')
            self.gui.show_hide(['button_save', 'button_load', 'button_list',
                                'button_grid',
                                'button_move','button_rotate','button_scale'])

            self.gui.fade_out(['button_rotate', 'button_move', 'button_scale'])
            #use a better camera controller
            cam_speed=ConfigVariableDouble('camera-speed', 1.0).get_value()
            cam_zoom_speed=ConfigVariableDouble('camera-zoom-speed', 1.0).get_value()
            self.cam_driver=CameraControler(pos=(0,0,0), offset=(0, 10, 0), speed=cam_speed,  zoom_speed=cam_zoom_speed)
            #self.cam_driver.node.set_pos(0, 0.4, 1.7)
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

            #collision, mouse picking setup
            self.cam_ray_trav = CollisionTraverser()
            self.cam_ray_handler = CollisionHandlerQueue()
            self.ray_trav = CollisionTraverser()
            #self.ray_trav.show_collisions(render)
            self.ray_handler = CollisionHandlerQueue()

            picker_np = camera.attach_new_node(CollisionNode('mouseRay'))
            #picker_np.node().set_from_collide_mask(GeomNode.getDefaultCollideMask())
            self.mouse_ray = CollisionRay()
            picker_np.node().add_solid(self.mouse_ray)
            picker_np.node().set_from_collide_mask(self.obj_mask)
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

            self.make_grid()

            self.lit_shader=Shader.load(Shader.SLGLSL, 'shaders/litsphere_v.glsl', 'shaders/litsphere_f.glsl')
            render.set_shader_input('litex', loader.load_texture('data/lit_sphere3.png'))
            #find data to load
            offset=-32
            for fn in os.listdir('models'):
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
                    if not f.getBasenameWoExtension().endswith('_coll'):
                        offset+=32
                        app.gui.button(txt='{name:<40} {type}'.format(name=f.getBasenameWoExtension(), type='object'),
                                       sort_dict={'name':fn, 'type':'object'},
                                       mono_font=True,
                                       align='left',
                                       cmd="app.load_object('models/"+fn+"', select=True)",
                                       width=384,
                                       pos=(16,offset),
                                       parent='model_frame_canvas')
            #mouse keys
            self.accept('mouse1', self._on_click)
            self.accept('mouse1-up', self._unlock_mouse)
            #keys
            self.accept('escape', self.set_select_mode)

            base.win.set_close_request_event('exit-event')
            self.accept('exit-event', self._on_exit)

            # Task
            taskMgr.add(self._lock_mouse_task, 'mouse_lock_tsk')

            self._setup_select_buff()

        self.gui.fade_screen(0.5, base.get_background_color())
        self.load_object('smiley')

    def set_material(self, material='SiO2'):
        if self.selected_id is None:
            return
        node=self.objects[self.selected_id]
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
        else:
            m=mat_spy.SiO2()
            material='SiO2'
        node.set_python_tag('material', m)
        node.set_python_tag('material_name', material)
        self.gui['material_txt'].set_text('MATERIAL:\n'+material)
        self.gui.show_hide('', 'mat_list_frame')
        self.do_raytrace()

    def hide_grid(self):
        if self.grid.is_hidden():
            self.grid.show()
            self.gui.fade_in('button_grid')
        else:
            self.grid.hide()
            self.gui.fade_out('button_grid')

    def make_grid(self):
        l=LineSegs()
        l.set_color(Vec4(0.1, 0.38, 0.23, 1.0))
        l.set_thickness(2.0)
        for i in range(0,11):
            l.move_to(Vec3(0,i,0))
            l.draw_to(Vec3(10, i, 0))
            l.move_to(Vec3(i,0,0))
            l.draw_to(Vec3(i, 10, 0))
        self.grid=render.attach_new_node(l.create())
        self.grid.set_transparency(TransparencyAttrib.M_binary)
        self.grid.set_antialias(AntialiasAttrib.MLine)
        #self.grid.set_depth_test(False)
        #self.grid.set_depth_write(False)
        #self.grid.set_bin('fixed', 9)
        self.grid.set_pos(-5,-5,0)

    def clear_scene(self):
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
            node.clear_python_tag('id')
            node.clear_python_tag('type')
            node.clear_python_tag('material')
            node.clear_python_tag('material_name')
            node.clear_python_tag('name')
            node.clear_python_tag('line')
            node.remove_node()
            self.gui.remove_button('button_object_'+str(object_id))
            self.gui.show_hide('', 'prop_frame')
            self.objects[object_id]=None
            self.selected_id=None
            self.axis.find('x').hide()
            self.axis.find('y').hide()
            self.axis.find('z').hide()

    def center_camera(self, object_id=None):
        '''move  the camera to the object  '''
        if object_id is None:
            object_id =self.selected_id
        if object_id <= len(self.objects):
            node=self.objects[object_id]
            if node is None:
                return
        pos=Vec3(node.get_x(render), node.get_y(render),self.cam_driver.node.get_z(render))
        LerpPosInterval(self.cam_driver.node, 0.25, pos).start()

    def toggle_visibility(self):
        ''' hide/show currently selected  object'''
        if self.selected_id is None:
            return
        node=self.objects[self.selected_id]
        if node.is_hidden(self.camera_mask):
            node.show_through(self.camera_mask)
        else:
            node.hide(self.camera_mask)

    def do_nill(self, *args, **kwargs):
        '''dummy function, does nothing  '''
        pass

    def _do_cmd_from_txt(self, txt, value):
        ''' execute commad based  on string,
        this function is to  help with text entry widgets'''
        #print(txt)
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
            node.set_color(c)
        elif txt == 'G':
            c=node.get_color()
            c[1]=value
            node.set_color(c)
        elif txt == 'B':
            c=node.get_color()
            c[2]=value
            node.set_color(c)
        elif txt == 'wave':
            value=max(300, min(5000, value))
            color=self.wavelength_to_rgb(value)
            node.set_python_tag('wave', value)
            line=node.get_python_tag('line')
            line.set_color(color, 1)
        else:
            return
        self.update_ui_for_node(node)
        if node.get_python_tag('type')=='ray':
            self.trace_ray(node)

    def apply_input(self, txt, target=['x','y','z']):
        '''Convert text input into a command'''
        values=[]
        #normalize input
        for v in txt.replace(',', ' ').split():
            try:
                values.append(math_eval(v))
            except:
                return
        #process the inputs
        for value, txt  in zip(values, target):
            self._do_cmd_from_txt(txt, value)

    def update_ui_for_node(self, node):
        self.gui.show_hide('prop_frame')
        self.gui['name_input'].set(node.get_python_tag('name'))
        self.gui['pos_input'].set('{0:.2f}, {1:.2f}, {2:.2f}'.format(*node.get_pos(render)))
        self.gui['hpr_input'].set('{0:.2f}, {1:.2f}, {2:.2f}'.format(*node.get_hpr(render)))
        self.gui['scale_input'].set('{0:.2f}, {1:.2f}, {2:.2f}'.format(*node.get_scale(render)))

        if node.has_python_tag('type'):
            t=node.get_python_tag('type')
            if t=='mesh':
                self.gui.show_hide('mesh_prop_frame', 'ray_prop_frame')
                mat_name=node.get_python_tag('material_name')
                self.gui['material_txt'].set_text('MATERIAL:\n'+mat_name)
                c=node.get_color()
                self.gui['color_input'].set('{0:.2f}, {1:.2f}, {2:.2f}'.format(*c))
                self.gui['alpha_slider']['value']=1.0-c[3]
                gloss=node.get_shader_input('gloss').get_vector()[0]
                self.gui['gloss_slider']['value']=gloss
            elif t=='ray':
                self.gui.show_hide('ray_prop_frame', 'mesh_prop_frame')
                self.gui['wave_input'].set(str(node.get_python_tag('wave')))

    def select_by_id(self, id):
        '''Select  an  object  based on  ID'''
        self.set_select_mode()
        node=self.objects[id]
        node.show_through(self.select_mask)
        self.selected_id=id
        self.update_ui_for_node(node)

    def set_select_mode(self):
        self.gui.fade_out(['button_rotate', 'button_move', 'button_scale'])
        self.gui.show_hide('',['save_frame',
                               'load_frame',
                               'list_frame',
                               'prop_frame',
                               'button_x',
                               'button_y',
                               'button_z',
                               'button_local'])
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
        self.gui.show_hide(['button_x','button_y','button_z','button_local'])
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
        self.gui.show_hide(['button_x','button_y','button_z','button_local'])
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
        self.gui.show_hide(['button_x','button_y','button_z','button_local'])
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
                self.axis.set_hpr(node.get_hpr())
        else:
            self.global_coords=True
            self.gui.fade_in('button_local')
            self.axis.set_hpr(0,0,0)


    def toggle_axis(self, axis):
        ''' Enable/disable an axis (x,y or z) for movement or rotation,
        only 2 axis can be active in move mode and 1 in rotate mode,
        excess axis will be disabled automatically'''
        if self.mode != 'select':
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

    def _unlock_mouse(self):
        if self.mouse_lock:
            wp = WindowProperties(base.win.getRequestedProperties())
            wp.set_cursor_hidden(False)
            base.win.request_properties(wp)
            self.mouse_lock=False
            base.win.move_pointer(0, int(self.pre_click_mouse_pos.get_x()), int(self.pre_click_mouse_pos.get_y()))
            if self.selected_id is not None:
                node=self.objects[self.selected_id]
                self.update_ui_for_node(node)
                if node.get_python_tag('type')=='ray':
                    self.trace_ray(node)
                self.axis.set_pos(node.get_pos(render))
                if not self.global_coords:
                    self.axis.set_hpr(node.get_hpr(render))

    def _mouse_move_node(self, node, delta):
        if self.active_axis == ['x']:
            node.set_pos(self.axis, -delta.x-delta.y, 0, 0)
        elif self.active_axis == ['y']:
            node.set_pos(self.axis, 0,-delta.x-delta.y, 0)
        elif self.active_axis == ['z']:
            node.set_pos(self.axis, 0, 0, delta.x+delta.y)
        else:
            cam_h=self.cam_driver.node.get_h()
            h=(int(90 * round(float(cam_h)/90))%360)//90
            if h == 0:
                if 'x' in self.active_axis and 'y' in self.active_axis:
                    node.set_pos(self.axis, -delta.x, -delta.y, 0)
                elif 'x' in self.active_axis and 'z' in self.active_axis:
                    node.set_pos(self.axis, -delta.x, 0, delta.y)
                elif 'y' in self.active_axis and 'z' in self.active_axis:
                    node.set_pos(self.axis, 0, -delta.x, delta.y)
            if h == 1:
                if 'x' in self.active_axis and 'y' in self.active_axis:
                    node.set_pos(self.axis, delta.y, -delta.x, 0)
                elif 'x' in self.active_axis and 'z' in self.active_axis:
                    node.set_pos(self.axis, -delta.x, 0, delta.y)
                elif 'y' in self.active_axis and 'z' in self.active_axis:
                    node.set_pos(self.axis, 0, -delta.x, delta.y)

            elif h == 2:
                if 'x' in self.active_axis and 'y' in self.active_axis:
                    node.set_pos(self.axis, delta.x, delta.y, 0)
                elif 'x' in self.active_axis and 'z' in self.active_axis:
                    node.set_pos(self.axis, delta.x, 0, delta.y)
                elif 'y' in self.active_axis and 'z' in self.active_axis:
                    node.set_pos(self.axis, 0, -delta.x, delta.y)

            elif h == 3:
                if 'x' in self.active_axis and 'y' in self.active_axis:
                    node.set_pos(self.axis, -delta.y, delta.x, 0)
                elif 'x' in self.active_axis and 'z' in self.active_axis:
                    node.set_pos(self.axis, -delta.x, 0, delta.y)
                elif 'y' in self.active_axis and 'z' in self.active_axis:
                    node.set_pos(self.axis, 0, delta.x, delta.y)


    def _lock_mouse(self):
        if base.mouseWatcherNode.has_mouse():
            self.last_mouse_pos=base.mouseWatcherNode.get_mouse()
            self.pre_click_mouse_pos=base.win.get_pointer(0)
        wp = WindowProperties(base.win.getRequestedProperties())
        wp.set_cursor_hidden(True)
        base.win.request_properties(wp)
        half_x=base.win.get_x_size()//2
        half_y=base.win.get_y_size()//2
        self.mouse_lock_frame_skip=True
        self.mouse_lock=True

    def _lock_mouse_task(self, task):
        dt = globalClock.getDt()
        if self.mouse_lock:
            if self.mouse_lock_frame_skip:
                self.mouse_lock_frame_skip=False
                return task.again
            if base.mouseWatcherNode.has_mouse():
                m_pos=base.mouseWatcherNode.get_mouse()
                delta = Vec2(m_pos- self.last_mouse_pos)
                self.last_mouse_pos = Vec2(m_pos)
                #don't let it go outside the window
                if abs(self.last_mouse_pos.x)>0.9 or abs(self.last_mouse_pos.y)>0.9:
                    half_x=base.win.get_x_size()//2
                    half_y=base.win.get_y_size()//2
                    base.win.move_pointer(0, half_x, half_y)
                    self.last_mouse_pos=Point2(0,0)

                #make things move/rotate/scale
                if self.selected_id is not None and abs(delta.x)+abs(delta.y) > 0.001:
                    delta*=dt*60.0
                    node=self.objects[self.selected_id]
                    if self.mode == 'move':
                        self._mouse_move_node(node, delta)
                        self.axis.set_pos(node.get_pos(render))
                    elif self.mode == 'rotate':
                        if 'x' in self.active_axis:
                            node.set_p(self.axis, node.get_p(self.axis)+delta.x*30.0+delta.y*30.0)
                        elif 'y' in self.active_axis:
                            node.set_r(self.axis, node.get_r(self.axis)+delta.x*30.0+delta.y*30.0)
                        elif 'z' in self.active_axis:
                            node.set_h(self.axis, node.get_h(self.axis)+delta.x*30.0+delta.y*30.0)
                        if not self.global_coords:
                            self.axis.set_hpr(node.get_hpr(render))
                    elif self.mode == 'scale':
                        new_scale=node.get_scale()+Vec3(delta.x+delta.y)*0.5
                        if new_scale < Vec3(0.01):
                            new_scale=Vec3(0.01)
                        node.set_scale(new_scale)
            else:
                half_x=base.win.get_x_size()//2
                half_y=base.win.get_y_size()//2
                base.win.move_pointer(0, half_x, half_y)
        return task.again

    def _on_click(self):
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
                    self.axis.set_pos(hit.get_parent().get_pos(render))
                    if self.global_coords:
                        self.axis.set_hpr(0,0,0)
                    else:
                        self.axis.set_hpr(hit.get_parent().get_hpr(render))
                    if self.mode != 'select':
                        self.update_ui_for_node(hit.get_parent())
                        self._lock_mouse()
                        for axis in self.active_axis:
                            self.axis.find(axis).show()
                    else:
                        self.select_by_id(self.selected_id)
                    return
            #else:
            self.set_select_mode()

    def _setup_select_buff(self):
        '''Create  an off-screen buffer  to render the selected object,
        only the depth is renderd and a edge detect shader is used to render an outline.
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
        cam = base.make_camera(win=self.depth_buff,
                              lens=base.cam.node().get_lens(),
                              scene=render,
                              mask=self.select_mask)
        cam.reparent_to(base.camera)

        cm = CardMaker("card")
        cm.setFrameFullscreenQuad ()
        self.overlay= NodePath(cm.generate())
        self.overlay.reparent_to(render2d)
        self.overlay.set_shader(Shader.load(Shader.SLGLSL, "shaders/select_v.glsl","shaders/select_f.glsl"),1)
        self.overlay.set_shader_input('depth_tex', self.depth_tex)
        self.overlay.set_shader_input('color', Vec3(0.2, 0.75, 0.45))
        self.overlay.set_shader_input('thickness', 2.0)
        self.overlay.set_transparency(TransparencyAttrib.M_alpha)
        self.overlay.set_bin("fixed", -20)

    def _on_exit(self):
        '''Function called when  the window is closed  by  the user'''
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

    def print_txt(self, *args):
        '''Prints to the screen'''
        text=''.join((str(i) for i in args))
        if text:
            text='\1shadow\1'+text+'\2'
            self.gui['on_screen_txt'].node().set_text(text)
            self.gui.show_hide('on_screen_txt')
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
        for node in self.objects:
            if node:
                if node.get_python_tag('type')=='ray':
                    self.trace_ray(node)

    def trace_ray(self, node):
        wavelength=node.get_python_tag('wave')
        origin=node.get_pos(render)
        target=node.get_quat().get_forward()*1000.0
        hit=self._ray(origin, target)
        points=[]
        ior_0=1.0
        if hit.has_hit:
            ior_1=hit.node.get_python_tag('material').n(wavelength)
            while hit.has_hit:
                points.append(hit.pos)
                origin=hit.pos
                refracted=self.refract(target.normalized(), hit.normal, ior_0, ior_1)
                target=refracted.vector*1000.0
                hit=self._ray(origin, target)
                if hit.has_hit:
                    if refracted.is_internal: # material->material????
                        ior_1=hit.node.get_python_tag('material').n(wavelength)
                        ior_0=ior_1
                    else:
                        if ior_0 == 1.0: #material->air
                            ior_0=hit.node.get_python_tag('material').n(wavelength)
                            ior_1=1.0
                        else:           #air->material
                            ior_0=1.0
                            ior_1=hit.node.get_python_tag('material').n(wavelength)
            else:
                points.append(hit.pos)
        else:
            points=[origin, target]
        line=node.get_python_tag('line')
        color=line.get_color()
        line.remove_node()
        l=LineSegs()
        l.set_thickness(2.0)
        l.move_to(node.get_pos(render))
        for point in points:
            l.draw_to(point)
        line=render.attach_new_node(l.create())
        line.set_color(color, 1)
        line.set_transparency(TransparencyAttrib.M_binary)
        line.set_antialias(AntialiasAttrib.MLine)
        line.set_depth_test(False)
        line.set_depth_write(False)
        line.set_bin('fixed', 10)
        node.set_python_tag('line', line)
        line.wrt_reparent_to(node)
        line.show_through(self.camera_mask)

    def reflect(self, I, N):
        return I - N * 2.0 * I.dot(N)

    def refract(self, I, N, ior_0, ior_1):
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
            return Traced(True, self.reflect(I, N))
        else:
            return  Traced(False, I * eta  + n * (eta * cosi - sqrt(k)))


    def _ray(self, origin, target):
        origin=Point3(origin)
        ray = render.attach_new_node(CollisionNode('ray'))
        ray.node().add_solid(CollisionRay(origin, Vec3(target)))
        ray.node().set_from_collide_mask(self.ray_mask)
        ray.node().set_into_collide_mask(0)
        self.ray_trav.clear_colliders()
        self.ray_trav.add_collider(ray, self.ray_handler)
        self.ray_trav.traverse(render)
        if self.ray_handler.get_num_entries() > 0:
            self.ray_handler.sort_entries()
            for entry in self.ray_handler.get_entries():
                hit_node=entry.get_into_node_path().get_parent()
                if not hit_node.has_python_tag('id'):
                    ray.remove_node()
                    return Hit(False, target, Vec3(0,0,0), None)
                hit=entry.get_surface_point(render)
                normal=entry.get_surface_normal(render)
                if not hit.almost_equal(origin):
                    ray.remove_node()
                    return Hit(True, hit, normal, hit_node)
        ray.remove_node()
        return Hit(False, target, Vec3(0,0,0), None)

    def _get_triangles(self, mesh, points=[]):
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

    def set_gloss(self, value):
        if self.selected_id is not None:
            node=self.objects[self.selected_id]
            if node:
                node.set_shader_input('gloss', float(value))

    def set_alpha(self, value):
        if self.selected_id is not None:
            node=self.objects[self.selected_id]
            if node:
                c=node.get_color()
                c[3]=1.01-value
                node.set_color(c)

    def make_ray(self):
        mesh=loader.load_model('data/cylinder.egg')
        mesh.reparent_to(render)
        self.objects.append(mesh)
        id=len(self.objects)-1
        model_name='Ray #'+str(id)
        mesh.set_python_tag('id', id)
        mesh.set_python_tag('type', 'ray')
        mesh.set_python_tag('wave', 650)
        mesh.set_color(LColor(1.0))
        mesh.set_transparency(TransparencyAttrib.M_alpha)
        #collision
        c_node=mesh.attach_new_node(CollisionNode('cnode'))
        for points in self._get_triangles(mesh, []):
            if len(points)==3:
                c_node.node().add_solid(CollisionPolygon(*points))
        c_node.node().set_into_collide_mask(BitMask32.bit(1))
        #draw line:
        l=LineSegs()
        #l.set_color(Vec4(1.0,0.0,0.0, 1.0))
        l.set_thickness(2.0)
        l.move_to(Vec3(0,0,0))
        l.draw_to(Vec3(0, 100, 0))
        line=mesh.attach_new_node(l.create())
        line.set_color(self.wavelength_to_rgb(650), 1)
        line.set_transparency(TransparencyAttrib.M_binary)
        line.set_antialias(AntialiasAttrib.MLine)
        line.set_depth_test(False)
        line.set_depth_write(False)
        line.set_bin('fixed', 10)
        mesh.set_python_tag('line', line)
        mesh.set_python_tag('name', model_name)
        mesh.hide(self.camera_mask)
        line.show_through(self.camera_mask)
        mesh.set_y(-2)
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
        self.select_by_id(id)
        self.trace_ray(mesh)


    def load_object(self, model, select=False):
        '''Loads a model, creates collision solids
        also adds the loaded model to self.models and creates a button on the object list
        '''
        mesh=loader.load_model(model)
        model_name=Filename(model).getBasenameWoExtension()
        mesh.reparent_to(render)
        self.objects.append(mesh)
        id=len(self.objects)-1
        mesh.set_python_tag('id', id)
        mesh.set_python_tag('type', 'mesh')
        mesh.set_python_tag('material', mat_spy.SiO2())
        mesh.set_python_tag('material_name', 'SiO2')
        mesh.set_python_tag('name', model_name)
        mesh.set_color(LColor(1.0))
        mesh.set_transparency(TransparencyAttrib.M_alpha)
        mesh.set_shader(self.lit_shader, 1)
        mesh.set_shader_input('gloss', 0.0)
        #collision
        c_node=mesh.attach_new_node(CollisionNode('cnode'))
        for points in self._get_triangles(mesh, []):
            #print(points)
            c_node.node().add_solid(CollisionPolygon(*points))
            c_node.node().add_solid(CollisionPolygon(*reversed(points)))
        c_node.node().set_into_collide_mask(self.obj_mask)
        #button
        object_type='mesh'
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
        if select:
            self.select_by_id(id)
        #return np


application=App()
application.base.run()
