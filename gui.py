from panda3d.core import *
from direct.gui.DirectGui import *
from direct.interval.FunctionInterval import Func, Wait
from direct.interval.MetaInterval import Sequence
from direct.interval.LerpInterval import LerpPosInterval
from direct.interval.LerpInterval import LerpScaleInterval
from direct.interval.LerpInterval import LerpColorInterval
from direct.showbase.DirectObject import DirectObject
import json
import ast
import operator
from collections import deque
import copy
import sys

__all__ = ['UI']

#py 2 and 3 compatibility
if sys.version_info >= (3, 0):
    unicode = str

#Helper functions
def _pos2d(x,y):
    return Point3(x,0,-y)

def _rec2d(width, height):
    return (width, 0, 0, -height)


binOps = {
    ast.Add: operator.add,
    ast.Sub: operator.sub,
    ast.Mult: operator.mul,
    ast.Div: operator.truediv,
    ast.FloorDiv:operator.floordiv,
    ast.Mod: operator.mod,
    ast.USub: operator.neg
}

def math_eval (s):
    node = ast.parse(s, mode='eval')

    def _eval(node):
        if isinstance(node, ast.Expression):
            return _eval(node.body)
        elif isinstance(node, ast.Str):
            return node.s
        elif isinstance(node, ast.Num):
            return node.n
        elif isinstance(node, ast.BinOp):
            return binOps[type(node.op)](_eval(node.left), _eval(node.right))
        elif isinstance(node, ast.UnaryOp):
            return binOps[type(node.op)](_eval(node.operand))
        else:
            raise Exception('Unsupported type {}'.format(node))

    return _eval(node.body)


class UI(DirectObject):
    '''The User Interface class, defines widgets and functions to work with these widgets
    '''
    def __init__(self, font_size=18, big_font_size=42, mono_font_size=12):
        #txt props
        tp_mgr = TextPropertiesManager.get_global_ptr()

        tp_red = TextProperties()
        tp_red.set_text_color(0.9, 0, 0, 1)
        tp_mgr.set_properties("red", tp_red)

        tp_gray = TextProperties()
        tp_gray.set_text_color(0.8, 0.8, 0.8, 1)
        tp_mgr.set_properties("gray", tp_gray)

        tp_shadow=TextProperties()
        tp_shadow.set_shadow(0.1,0.1)
        tp_shadow.set_shadow_color(0,0,0, 1.0)
        tp_mgr.set_properties("shadow", tp_shadow)

        #load fonts
        self.font_size=font_size
        self.font = loader.load_font('gui/anonymous.ttf')
        #self.font = loader.load_font('font/VerilySerifMono.otf')
        self.font.set_space_advance(0.5)
        self.font.set_pixels_per_unit(font_size)
        self.font.set_minfilter(SamplerState.FT_nearest )
        self.font.set_magfilter(SamplerState.FT_nearest )
        self.font.set_native_antialias(True)


        self.big_font_size=big_font_size
        self.big_font=loader.load_font('gui/teko-medium.ttf')
        self.big_font.set_minfilter(SamplerState.FT_nearest )
        self.big_font.set_magfilter(SamplerState.FT_nearest )
        self.big_font.set_native_antialias(True)
        self.big_font.set_pixels_per_unit(big_font_size)
        #self.big_font.set_outline(outline_color=(1.0, 1.0, 1.0, 0.2), outline_width=2.0, outline_feather=0.6)

        self.mono_font= loader.load_font('gui/monosb.ttf')
        self.mono_font_size=mono_font_size
        self.mono_font.set_pixels_per_unit(mono_font_size)
        self.mono_font.set_minfilter(SamplerState.FT_nearest )
        self.mono_font.set_magfilter(SamplerState.FT_nearest )
        self.mono_font.set_native_antialias(True)

        #fade to black card
        cm = CardMaker("black-quad")
        cm.set_frame((2, -2, 2, -2))
        self.fade_quad = NodePath(cm.generate())
        color=base.get_background_color()
        self.fade_quad.set_color(color[0],color[1],color[2],1)
        self.fade_quad.reparent_to(aspect2d)
        self.fade_quad.set_transparency(TransparencyAttrib.M_alpha)
        self.fade_quad.set_bin("fixed", 10)

        self.elements={}
        self.autoresize={}
        #set nodes for gui placement
        self.elements['top_left'] = pixel2d.attach_new_node('TopLeft')
        self.elements['top_right'] = pixel2d.attach_new_node('TopRight')
        self.elements['bottom_right'] = pixel2d.attach_new_node('BottomRight')
        self.elements['bottom_left'] = pixel2d.attach_new_node('BottomLeft')
        self.elements['top'] = pixel2d.attach_new_node('Top')
        self.elements['bottom'] = pixel2d.attach_new_node('Bottom')
        self.elements['left'] = pixel2d.attach_new_node('Left')
        self.elements['right'] = pixel2d.attach_new_node('Right')
        self.elements['center'] = pixel2d.attach_new_node('Center')
        #Most widgets are rendered from using thease tiles
        self.tiles={
                    'top_left': self._make_tile(uv_offset=(0,0), rotation=90, tex_scale=(0.25, 0.25)),
                    'top_right': self._make_tile(uv_offset=(0,0), rotation=180, tex_scale=(0.25, 0.25)),
                    'top': self._make_tile(uv_offset=(0.5,0.75), rotation=0, tex_scale=(0.25, 0.25)),
                    'bottom': self._make_tile(uv_offset=(0.5,0.75), rotation=180, tex_scale=(0.25, 0.25)),
                    'bottom_left': self._make_tile(uv_offset=(0,0), rotation=0, tex_scale=(0.25, 0.25)),
                    'bottom_right': self._make_tile(uv_offset=(0,0), rotation=270, tex_scale=(0.25, 0.25)),
                    'left': self._make_tile(uv_offset=(0.5,0.75), rotation=270, tex_scale=(0.25, 0.25)),
                    'right': self._make_tile(uv_offset=(0.5,0.75), rotation=90, tex_scale=(0.25, 0.25)),
                    'right_scroll_top': self._make_tile(uv_offset=(0.75,0.25), rotation=0, tex_scale=(0.25, -0.25)),
                    'right_scroll_center': self._make_tile(uv_offset=(0.75,0.375), rotation=0, tex_scale=(0.25, 0.25)),
                    'right_scroll_bottom': self._make_tile(uv_offset=(0.75,0.25), rotation=0, tex_scale=(0.25, 0.25)),
                    'right_scroll_bottom_right': self._make_tile(uv_offset=(0.75,0), rotation=0, tex_scale=(0.25, 0.25)),
                    'right_scroll_top_div_right': self._make_tile(uv_offset=(0.75,0.75), rotation=0, tex_scale=(0.25, 0.25)),
                    'top_left_deco': self._make_tile(uv_offset=(0,0.75), rotation=0, tex_scale=(0.25, 0.25)),
                    'top_right_deco': self._make_tile(uv_offset=(0.25,0.75), rotation=0, tex_scale=(-0.25, 0.25)),
                    'top_deco': self._make_tile(uv_offset=(0.25,0.75), rotation=0, tex_scale=(0.25, 0.25)),
                    'bottom_deco': self._make_tile(uv_offset=(0.25,0.75), rotation=180, tex_scale=(0.25, 0.25)),
                    'bottom_left_deco': self._make_tile(uv_offset=(0,0.75), rotation=270, tex_scale=(0.25, 0.25)),
                    'bottom_right_deco': self._make_tile(uv_offset=(0,0.75), rotation=180, tex_scale=(0.25, 0.25)),
                    'left_deco': self._make_tile(uv_offset=(0.25,0.75), rotation=270, tex_scale=(0.25, 0.25)),
                    'right_deco': self._make_tile(uv_offset=(0.25,0.75), rotation=90, tex_scale=(0.25, 0.25)),
                    'div_top_left': self._make_tile(uv_offset=(0,0.5), rotation=0, tex_scale=(0.25, 0.25)),
                    'div_top': self._make_tile(uv_offset=(0.25,0.5), rotation=0, tex_scale=(0.25, 0.25)),
                    'div_top_right': self._make_tile(uv_offset=(0.25,0.5), rotation=0, tex_scale=(-0.25, 0.25)),
                    'entry_left': self._make_tile(uv_offset=(0.25,0), rotation=0, frame_size=(-16, 16, -16, 16), tex_scale=(0.125, 0.125)),
                    'entry_left_tick': self._make_tile(uv_offset=(0.375,0), rotation=0, frame_size=(-16, 16, -16, 16), tex_scale=(0.125, 0.125)),
                    'entry_center':  self._make_tile(uv_offset=(0.5,0.0), rotation=0, frame_size=(-16, 16, -16, 16), tex_scale=(0.125, 0.125)),
                    'entry_right': self._make_tile(uv_offset=(0.375,0), rotation=0, frame_size=(-16, 16, -16, 16), tex_scale=(-0.125, 0.125)),
                    'button_left': self._make_tile(uv_offset=(0.25,0.125), rotation=0, frame_size=(-16, 16, -16, 16), tex_scale=(0.125, 0.125)),
                    'button_left_tick': self._make_tile(uv_offset=(0.375,0.125), rotation=0, frame_size=(-16, 16, -16, 16), tex_scale=(0.125, 0.125)),
                    'button_center': self._make_tile(uv_offset=(0.5,0.125), rotation=0, frame_size=(-16, 16, -16, 16), tex_scale=(0.125, 0.125)),
                    'button_right': self._make_tile(uv_offset=(0.375,0.125), rotation=0, frame_size=(-16, 16, -16, 16), tex_scale=(-0.125, 0.125)),
                    'thumb_vertical': self._make_tile(uv_offset=(0.625,0.375), rotation=0, frame_size=(-16, 16, -16, 16), tex_scale=(0.125, 0.125)),
                    'thumb_horizontal':self._make_tile(uv_offset=(0.625,0.375), rotation=90, frame_size=(-16, 16, -16, 16), tex_scale=(0.125, 0.125)),
                    'slider_left': self._make_tile(uv_offset=(0.125,0.375), rotation=0, frame_size=(-16, 16, -16, 16), tex_scale=(0.125, 0.125)),
                    'slider_center': self._make_tile(uv_offset=(0.25,0.375), rotation=0, frame_size=(-16, 16, -16, 16), tex_scale=(0.125, 0.125)),
                    'slider_right': self._make_tile(uv_offset=(0.25,0.375), rotation=0, frame_size=(-16, 16, -16, 16), tex_scale=(-0.125, 0.125)),
                    'bar': self._make_tile(uv_offset=(0.0,0.3125), rotation=0, frame_size=(192, 0, 0, -16), tex_scale=(0.75, 0.0625)),
                    'big_plus_icon': self._make_tile(uv_offset=(0.625,0.0), rotation=0, frame_size=(-16, 16, -16, 16), tex_scale=(0.125, 0.125)),
                    'big_minus_icon': self._make_tile(uv_offset=(0.625,0.125), rotation=0, frame_size=(-16, 16, -16, 16), tex_scale=(0.125, 0.125)),
                    'minus_icon':self._make_tile(uv_offset=(0.125,0.25), rotation=0, frame_size=(-8, 8, -8, 8), tex_scale=(0.0625, 0.0625)),
                    'plus_icon':self._make_tile(uv_offset=(0.1875,0.25), rotation=0, frame_size=(-8, 8, -8, 8), tex_scale=(0.0625, 0.0625)),
                    'mouse_icon': self._make_tile(uv_offset=(0.5,0.375), rotation=0, frame_size=(-16, 16, -16, 16), tex_scale=(0.125, 0.125)),
                    'close_icon': self._make_tile(uv_offset=(0.375,0.375), rotation=0, frame_size=(-16, 16, -16, 16), tex_scale=(0.125, 0.125)),
                    'icon': self._make_tile(uv_offset=(0.375,0.375), rotation=0, frame_size=(-16, 16, -16, 16), tex_scale=(0.125, 0.125)),
                    'dot_full':self._make_tile(uv_offset=(0.0,0.25), rotation=0, frame_size=(-8, 8, -8, 8), tex_scale=(0.0625, 0.0625)),
                    'dot':self._make_tile(uv_offset=(0.0625,0.25), rotation=0, frame_size=(-8, 8, -8, 8), tex_scale=(0.0625, 0.0625)),
                    'line_red':None,
                    'blank': self._make_tile(uv_offset=(0.5,0.5), rotation=0, tex_scale=(0.25, 0.25))
                    }

        self.window_x = 0
        self.window_y = 0
        self.last_sort=''
        self.last_sort_reverse=False
        self.window_minimized = False
        self.window_focused = True
        self.slider_cache={}
        self.current_focus=None
        self.resize_callback=[]
        self.last_click_time=0.0

        self.accept( 'window-event', self.on_window_event)
        self.accept('enter', self.on_enter_pressed)

    def __getitem__(self, item):
        return self.elements[item]

    def hide_root(self):
        self.elements['top_left'].hide()
        self.elements['top_right'].hide()
        self.elements['bottom_right'].hide()
        self.elements['bottom_left'].hide()
        self.elements['top'].hide()
        self.elements['bottom'].hide()
        self.elements['left'].hide()
        self.elements['right'].hide()
        self.elements['center'].hide()

    def sort_buttons(self, canvas_name, sort_key, reverse=None):
        '''Sorts all buttons on a 'canvas_name' scrolled frame using 'sort_key'
        '''
        button_list=[]
        for button in self.elements[canvas_name].get_children():
            button_list.append(button.get_python_tag('sort_dict'))

        if reverse is None:
            if self.last_sort == canvas_name+sort_key:
                reverse=not self.last_sort_reverse
            else:
                reverse=False
        self.last_sort_reverse=reverse
        self.last_sort=canvas_name+sort_key

        for i, button in enumerate(sorted(button_list, key=operator.itemgetter(sort_key), reverse=reverse)):
            button['node'].set_z(-28*i)

    def fade_screen(self, time=1.5, color=(0,0,0)):
        '''Turns the screen to 'color' then fades the color away in 'time' seconds'''
        startColor=self.fade_quad.get_color()
        if startColor[3] == 0.0:
            Sequence(
            Func(self.fade_quad.show),
            LerpColorInterval(nodePath=self.fade_quad, duration=time, color=(color[0],color[1],color[2],1), startColor=startColor),
            ).start()
        else:
            Sequence(
            Func(self.fade_quad.show),
            Wait(time*0.1),
            LerpColorInterval(nodePath=self.fade_quad, duration=time*0.9, color=(color[0],color[1],color[2],0), startColor=startColor, blendType = 'easeIn'),
            Func(self.fade_quad.hide),
            ).start()

    def set_bar_scale(self, bar_name, scale):
        if scale >1.0:
            scale=1.0
        self.elements[bar_name].find('**bar').set_scale(scale, 1, 1)

    def lerp_bar(self, bar_name, end, start=0, time=0.1):
        '''Smooth sets the value of 'bar_name' from 'start' to 'end' in 'time' seconds '''
        bar=self.elements[bar_name].find('**bar')
        LerpScaleInterval(nodePath=bar, duration=time, scale=(end, 1, 1), startScale=(start, 1, 1)).start()

    def load_from_file(self, filename):
        '''Loads UI widgets from 'filename' json file'''
        with open(filename) as f:
            data=json.load(f)

        for frame in data['frame']:
            if 'autoresize' in frame:
                self.autoresize[frame['name']]=frame
            #make frame
            self.frame(**frame)
            #make buttons
            if 'button' in frame:
                for button in frame['button']:
                    if not 'parent' in button:
                        button['parent']=frame['name']
                    if 'name' in button:
                        self.button(**button)
                    else:
                        self.elements[frame['name']+'_button_'+button['txt'].lower().replace(' ', '')]=self.button(**button)
            #make text
            if 'text' in frame:
                for text in frame['text']:
                    if not 'parent' in text:
                        text['parent']=frame['name']
                    self.text(**text)
            #make sliders
            if 'slider' in frame:
                for slider in frame['slider']:
                    if not 'parent' in slider:
                        slider['parent']=frame['name']
                    self.slider(**slider)
            #make img
            if 'img' in frame:
                for img in frame['img']:
                    if not 'parent' in img:
                        img['parent']=frame['name']
                    self.img(**img)
            #make inputs
            if 'input' in frame:
                for inputs in frame['input']:
                    if not 'parent' in inputs:
                        inputs['parent']=frame['name']
                    self.input(**inputs)
            #make icons
            if 'icon' in frame:
                for icons in frame['icon']:
                    if not 'parent' in icons:
                        icons['parent']=frame['name']
                    self.icon(**icons)
            #make bars
            if 'bar' in frame:
                for bars in frame['bar']:
                    if not 'parent' in bars:
                        bars['parent']=frame['name']
                    self.bar(**bars)
            #make widget
            if 'widget' in frame:
                for widget in frame['widget']:
                    if not 'parent' in widget:
                        widget['parent']=frame['name']
                    self.widget(**widget)
        for name, button in data['button'].items():
            self.elements[name]=self.button(**button)
            self.elements[name].hide()
        for name, inputs in data['input'].items():
            self.elements[name]=self.input(**inputs)
            self.elements[name].hide()
        for name, icon in data['icon'].items():
            self.elements[name]=self.icon(**icon)
            self.elements[name].hide()
        for name,slider in data['slider'].items():
            self.elements[name]=self.slider(**slider)
            self.elements[name].hide()
        for name,bar in data['bar'].items():
            self.elements[name]=self.bar(**bar)
            self.elements[name].hide()
        for name,img in data['img'].items():
            self.elements[name]=self.img(**img)
            self.elements[name].hide()
        for name,text in data['text'].items():
            self.elements[name]=self.text(**text)
            self.elements[name].hide()

    def toggle(self, names, hide=None):
        '''This will show the 'name' widget if it's hidded
            or hide it if it's visible, it will also hide 'hide' named widget(-s)'''
        self.show_hide(None, hide)

        if isinstance(names, (str, bytes)):
            names=[names]
        for name in names:
            if name in self.elements:
                if self.elements[name].is_hidden():
                    self.elements[name].show()
                    #if name+"_verticalScroll" in self.elements:
                    #    self._bind_wheel(name)
                else:
                    self.elements[name].hide()
                   # if name+"_verticalScroll" in self.elements:
                   #     self._un_bind_wheel()

    def show_hide(self, show=None, hide=None):
        ''' This will show all 'show' widgets and hide all 'hide' widgets',
        'show' and 'hide' can be single names or lists/tuples of names.
        '''
        if hide is not None:
            if isinstance(hide, (str, bytes)):
                hide=[hide]
            for name in hide:
                if name in self.elements:
                    self.elements[name].hide()
                    #if name+"_verticalScroll" in self.elements:
                    #    self._un_bind_wheel()
        if show is not None:
            if isinstance(show, (str, bytes)):
                show=[show]
            for name in show:
                if name in self.elements:
                    self.elements[name].show()
                    #if name+"_verticalScroll" in self.elements:
                    #    self._bind_wheel(name)

    def remove_button(self, name):
        '''Permanantly remove a button'''
        if name in self.elements:
            button=self.elements.pop(name)
            button.clear_python_tag('sort_dict')
            button.destroy()

    def set_button_txt(self, name, txt):
        if name not in self.elements:
            return
        button=self.elements[name]
        if button.has_python_tag('text'):
                t=button.get_python_tag('text')
                t.node().set_text(txt)
                
    def scroll(self, name, value):
        scroll_bar=self.elements[name+"_verticalScroll"]
        scroll_bar['value']=1.0-value
    
    def fade_out(self, names):
        if isinstance(names, (str, bytes)):
            names=[names]
        for name in names:
            if name not in self.elements:
                return
            button=self.elements[name]
            if button.has_python_tag('text'):
                t=button.get_python_tag('text')
                t.node().set_text_color((0.55, 0.55, 0.55, 1.0))
            else:
                button.set_color(1,1,1, 0.25)

    def fade_in(self, names):
        if isinstance(names, (str, bytes)):
            names=[names]
        for name in names:
            if name not in self.elements:
                return
            button=self.elements[name]
            if button.has_python_tag('text'):
                t=button.get_python_tag('text')
                t.node().set_text_color((0.2, 0.75, 0.45, 1.0))
            else:
                button.set_color(1,1,1, 1.0)

    def highlight(self, button=None, on=True, name=None):
        '''Change the font color of a button or named widget to red-ish if 'on' is True,
        else change the color to while-ish
        '''
        if name is not None:
            if name not in self.elements:
                return
            button=self.elements[name]
        if button is None:
            return
        if on:
            button['text_fg']=(1.0,1.0,1.0,1)
        else:
            button['text_fg']=(0.8,0.8,0.8,1)


    #############################
    ### Widget creation methods #
    #############################

    def img(self, texture, parent, pos, tex_scale=(1,1), uv_offset=(0,0), hidden=False):
        '''Create a static on screen textured card.
        '''
        if parent in self.elements:
            parent = self.elements[parent]
        tex=loader.load_texture(texture)
        x=tex.get_orig_file_x_size()*tex_scale[0]
        y=tex.get_orig_file_y_size()*tex_scale[1]

        cm = CardMaker("img")
        cm.set_frame(_rec2d(x,-y))
        cm.set_uv_range (Point2(0,0), Point2(-1,1))
        node=parent.attach_new_node(cm.generate())
        node.set_texture(tex)
        node.set_pos(_pos2d(0,y))
        node.flatten_light()
        node.set_tex_offset(TextureStage.get_default(), uv_offset[0], uv_offset[1])
        node.set_tex_scale(TextureStage.get_default(), tex_scale[0], tex_scale[1])
        node.set_pos(_pos2d(*pos))
        node.set_transparency(TransparencyAttrib.M_alpha)
        node.set_bin("fixed", 0)
        if hidden:
            node.hide()
        return node

    def text(self, txt, pos, parent, align='center',  color=(0.2, 0.75, 0.45, 1.0), big_font=True, mono_font=False, wordwrap=None, name=None, sort=0):
        '''Creates a text widget'''
        if parent in self.elements:
            parent = self.elements[parent]
        t = TextNode('text')
        if mono_font:
            t.set_font(self.mono_font)
            font_size=self.mono_font_size
        elif big_font:
            t.set_font(self.big_font)
            font_size=self.big_font_size
        else:
            t.set_font(self.font)
            font_size=self.font_size
        if wordwrap is not None:
            t.set_wordwrap(wordwrap)
        t.set_text(txt)
        x=0
        if align=='center':
            if parent.has_python_tag('x_size'):
                x=parent.get_python_tag('x_size')//2
            else:
                bounds=parent.get_tight_bounds()
                x=(bounds[1][0]-bounds[0][0])//4*2 #round to multiple of 64
            t.set_align(TextNode.ACenter)
        elif align=='right':
            t.set_align(TextNode.ARight)
        else:
            t.set_align(TextNode.ALeft)
        t.set_text_color(*color)
        textNodePath = parent.attach_new_node(t)
        textNodePath.set_scale(font_size)
        textNodePath.set_pos(_pos2d(x,font_size))
        textNodePath.flatten_light()
        textNodePath.set_pos(_pos2d(*pos))
        textNodePath.set_transparency(TransparencyAttrib.M_alpha)
        textNodePath.set_bin("fixed", sort)
        if name is not None:
             self.elements[name]=t
             self.elements[name+'_node']=textNodePath
        return textNodePath

    def frame(self, name, size, pos, parent, header=False, scroll=False, **kwargs):
        '''Creates a frame, resized to the next multiple of 64,
        with header and scroll bar/canvas if needed.
        '''
        if parent in self.elements:
            parent = self.elements[parent]
        size=(self._parse(size[0]), self._parse(size[1]))
        pos=(self._parse(pos[0]), self._parse(pos[1]))
        x_max = int(size[0])//64
        y_max = int(size[1])//64
        center_deco=-1
        if x_max%2 == 1:
            center_deco=x_max//2
        if header:
            header=header//64
            if header < 1:
                header=1
        else:
            header=-1
        root=parent.attach_new_node('frame_'+name)

        for x in range(x_max):
            for y in range(y_max):
                if y == header and x == 0:
                    tile=self.tiles['div_top_left'].copy_to(root)
                elif y == header and x == x_max-1:
                    if scroll:
                        tile=self.tiles['right_scroll_top_div_right'].copy_to(root)
                    else:
                        tile=self.tiles['div_top_right'].copy_to(root)
                elif y == header:
                    tile=self.tiles['div_top'].copy_to(root)
                elif x == 0 and y == 0: #top left corner
                    tile=self.tiles['top_left'].copy_to(root)
                elif x == x_max-1 and y == 0: #top right
                    if  header==-1 and scroll:
                        tile=self.tiles['right_scroll_top'].copy_to(root)
                    else:
                        tile=self.tiles['top_right_deco'].copy_to(root)
                elif x == center_deco and y ==0: #top decoration
                    tile=self.tiles['top_deco'].copy_to(root)
                elif x == 0 and y == y_max-1: #bottom left
                    tile=self.tiles['bottom_left_deco'].copy_to(root)
                elif x ==x_max-1 and y== y_max-1: #bottom right
                    if scroll:
                        tile=self.tiles['right_scroll_bottom_right'].copy_to(root)
                    else:
                        tile=self.tiles['bottom_right'].copy_to(root)
                elif y ==0: #top
                    tile=self.tiles['top'].copy_to(root)
                elif y ==y_max-1: #bottom
                    tile=self.tiles['bottom'].copy_to(root)
                elif x == 0: #left
                    tile=self.tiles['left'].copy_to(root)
                elif x == x_max-1: #right
                    if scroll and y>header:
                        tile=self.tiles['right_scroll_center'].copy_to(root)
                    else:
                        tile=self.tiles['right'].copy_to(root)
                else: #anything else
                    tile=self.tiles['blank'].copy_to(root)
                tile.set_pos(_pos2d(32+x*64, 32+y*64))
        root.flatten_strong()
        root.set_pos(_pos2d(*pos))

        if scroll:
            thumb = self.tiles['thumb_vertical'].copy_to(NodePath('thumb'))
            if header == -1:
                header=0
                offset=10
            else:
                offset=0
            #canvas
            canvas=root.attach_new_node('scrolled_frame')
            canvas.set_python_tag('x_size',x_max*64)
            canvas.set_python_tag('y_size',scroll-(49+header*64))
            canvas.set_python_tag('header',49+header*64)
            #clip plane
            clipPlane = root.attachNewNode(PlaneNode('clip'))
            clipPlane.node().setPlane(Plane(Vec3(0, 0, -1), Point3(0, 0, 0)))
            clipPlane.setPos(0,0,-(49+header*64))
            canvas.setClipPlane(clipPlane)
            clipPlane2 = root.attachNewNode(PlaneNode('clip'))
            clipPlane2.node().setPlane(Plane(Vec3(0, 0, 1), Point3(0, 0, 0)))
            clipPlane2.setPos(0,0,-(y_max*64)+16)
            canvas.setClipPlane(clipPlane2)
            #create scrollbar
            s_bar=DirectScrollBar(range=(0, 1),
                                value=1,
                                thumb_geom=thumb,
                                resizeThumb=False,
                                thumb_pad=(-6, -2),
                                manageButtons=False,
                                frameSize=_rec2d(8,(y_max-max(0,header))*64-50-64+offset),
                                frameColor=(0,1,0,0.0),
                                orientation= DGG.VERTICAL,
                                thumb_relief=None,
                                command=self._scroll_canvas,
                                parent=root)
            s_bar.incButton.hide()
            s_bar.decButton.hide()
            s_bar.set_pos(_pos2d(x_max*64-27,-offset+64+header*64))
            s_bar['extraArgs']=[s_bar, canvas]
            self.elements[name+'_verticalScroll']=s_bar
            self.elements[name+'_canvas']=canvas
        self.elements[name]=root
        root.hide()
        return root

    def input(self, txt, width, pos, parent, txt_fg=(1.0,1.0,1.0,1.0), cmd=None, style='', hidden=False, name=None):
        ''' Creares a one line input field (entry)'''
        if parent in self.elements:
            parent = self.elements[parent]
        max_width = int(width)//32
        geom=NodePath('input')
        for x in range(max_width):
            if x == 0:
                tile=self.tiles['entry_left'+style].copy_to(geom)
            elif x == max_width -1:
                tile=self.tiles['entry_right'].copy_to(geom)
            else:
                tile=self.tiles['entry_center'].copy_to(geom)
            tile.set_pos(_pos2d(16+x*32, 16))
        geom.flatten_strong()

        #geom.reparent_to(parent)
        #root=geom
        root=DirectEntry(frameColor=(0,0,0, 0.0),
                         frameSize=_rec2d(width, 32),
                         initialText=txt,
                         text_font=self.mono_font,
                         text_scale=self.mono_font_size,
                         text_fg=Vec4(*txt_fg),
                         text_align=TextNode.ALeft,
                         text_pos=(9, -19),
                         width=width//12.5,
                         textMayChange=1,
                         #state=DGG.NORMAL,
                         geom=geom,
                         #geom_pos=_pos2d(-12, -2),
                         suppressMouse = True,
                         suppressKeys = True,
                         command=self.on_submit,
                         focusInCommand=self.on_focus,
                         parent=parent)
        #root.set_pos(_pos2d(12, 2))
        root['focusInExtraArgs']=[root]
        root['extraArgs']=[root, cmd]
        root.set_python_tag('cmd', cmd)
        root.bind(DGG.B1PRESS, self.on_cursor_set, [root])
        root.flatten_light()
        root.set_pos(_pos2d(*pos))
        if hidden:
            root.hide()
        if name is not None:
            self.elements[name]=root
        return root

    def button(self, txt, cmd, width, pos, parent, txt_fg=(0.2, 0.75, 0.45, 1.0), align='center', mono_font=False,  style='', hidden=False, sort_dict={}, name=None):
        '''Creates a button
        '''
        if parent in self.elements:
            parent = self.elements[parent]
        max_width = int(width)//32
        geom=NodePath('button')

        for x in range(max_width):
            if x == 0:
                tile=self.tiles['button_left'+style].copy_to(geom)
            elif x == max_width -1:
                tile=self.tiles['button_right'].copy_to(geom)
            else:
                tile=self.tiles['button_center'].copy_to(geom)
            tile.set_pos(_pos2d(16+x*32, 16))
        geom.flatten_strong()
        txt_pos=[0,1]
        if align=='left':
            txt_pos=[9,1]
        if mono_font:
            txt_pos[1]=7
        #txt_align=TextNode.ALeft
        #text_pos=(3, -17)
        #if align=='center':
        #    txt_align=TextNode.ABoxedCenter
        #    text_pos=((width//2)-14, -17)
        #elif align=='right':
        #    txt_align=TextNode.ARight
        #if mono_font:
        #    font=self.mono_font
        #    font_size=self.mono_font_size
        #else:
        #    font=self.font
        #    font_size=self.font_size

        root=DirectFrame(frameColor=(0,0,0, 0.0),
                         frameSize=None,#_rec2d(width-24, 32-4),
                         pad=(-10, -4),
                         #text=txt,
                         #text_font=font,
                         #text_scale=font_size,
                         #text_fg=Vec4(*txt_fg),
                         #text_align=txt_align,
                         #text_pos=text_pos,
                         #textMayChange=1,
                         state=DGG.NORMAL,
                         geom=geom,
                         geom_pos=_pos2d(-12, -2),
                         suppressMouse = True,
                         parent=parent)
        root.set_pos(_pos2d(12, 2))
        root.flattenLight()
        root.set_pos(_pos2d(*pos))
        root.set_transparency(TransparencyAttrib.M_alpha)
        root.set_bin("fixed", 0)
        root.bind(DGG.B1PRESS, self.on_button_press, [root, cmd])
        root.set_python_tag('x_size', width)
        #on 32 bit windows text is sometimes missing, this is a brute force fix
        t=self.text(txt=txt,
                    pos=txt_pos,
                    parent=root,
                    align=align,
                    big_font=False,
                    mono_font=mono_font,
                    wordwrap=None,
                    name=str(name)+'_button_text')
        t.node().set_text_color(Vec4(*txt_fg))
        root.set_python_tag('text', t)
        if hidden:
            root.hide()
        sort_dict['node']=root #cyclic reference warrning!
        root.set_python_tag('sort_dict', sort_dict)
        if name is not None:
            self.elements[name]=root
        return root

    def icon(self, cmd, pos, parent, img=None, name=None):
        ''' Creates a icon-button, a 32x32 button with an image'''
        if parent in self.elements:
            parent = self.elements[parent]
        geom=self.tiles['icon'].copy_to(NodePath('icon'))
        geom.flatten_strong()
        if img:
            tex=loader.load_texture(img)
            cm = CardMaker("img")
            cm.set_frame(_rec2d(32,-32))
            cm.set_uv_range (Point2(0,0), Point2(-1,1))
            node=NodePath(cm.generate())
            node.set_texture(tex)
            node.set_pos(_pos2d(-16, 16))
            node.reparent_to(geom)
            geom.flatten_strong()
        root=DirectFrame(frameColor=(1,0,0, 0.0),
                         frameSize=None,#_rec2d(32, 32),
                         state=DGG.NORMAL,
                         geom=geom,
                         geom_pos=_pos2d(16, 16),
                         pad=(-2, -2),
                         suppressMouse = True,
                         parent=parent)
        root.set_pos(_pos2d(*pos))
        root.bind(DGG.B1PRESS, self.on_button_press, [root, cmd])
        root.set_transparency(TransparencyAttrib.M_alpha)
        root.set_bin("fixed", 0)
        if name is not None:
            self.elements[name]=root
        return root

    def slider(self, value, cmd, width, pos, parent, value_range=(0,1), name=None):
        ''' Creates a horizontal slider.
        Use 'value' in the cmd text to pass the value of the slider to the callback eg.
        cmd="print('This slider is set to:', value)"
        '''
        if parent in self.elements:
            parent = self.elements[parent]
        max_width = int(width)//32
        geom=parent.attach_new_node('slider')
        for x in range(max_width):
            if x == 0:
                tile=self.tiles['slider_left'].copy_to(geom)
            elif x == max_width -1:
                tile=self.tiles['slider_right'].copy_to(geom)
            else:
                tile=self.tiles['slider_center'].copy_to(geom)
            tile.set_pos(_pos2d(16+x*32, 16))
        geom.flatten_strong()
        geom.set_pos(_pos2d(*pos))
        thumb = self.tiles['thumb_horizontal'].copy_to(NodePath('thumb'))
        slider = DirectSlider(frameSize=(max_width*32-96, 0, 0, -32),
                              frameColor=(0,1,0,0.0),
                              range=tuple(reversed(value_range)),
                              value=value,
                              thumb_geom=thumb,
                              thumb_frameColor=(0,1,0,0.0),
                              suppressMouse = False,
                              parent=geom)
        slider.set_x(slider.get_x()+48)
        slider.set_transparency(TransparencyAttrib.M_alpha)
        slider.set_bin("fixed", 0)
        slider.bind(DGG.B1RELEASE, self.on_slide, [slider, cmd])
        slider.bind(DGG.B1PRESS, self.on_slide, [slider, cmd])
        slider['command']=self.on_slide
        slider['extraArgs']=[slider, cmd]
        if name is not None:
            self.elements[name]=slider
        return slider

    def bar(self, width, color, pos, parent, name=None, hidden=False):
        '''Creates a horizontal progres/status bar (like a health bar)'''
        if parent in self.elements:
            parent = self.elements[parent]
        max_width = int(width)//32
        geom=parent.attach_new_node('slider')
        for x in range(max_width):
            if x == 0:
                tile=self.tiles['slider_left'].copy_to(geom)
            elif x == max_width -1:
                tile=self.tiles['slider_right'].copy_to(geom)
            else:
                tile=self.tiles['slider_center'].copy_to(geom)
            tile.set_pos(_pos2d(16+x*32, 16))
        geom.flatten_strong()
        geom.set_pos(_pos2d(*pos))
        bar=self.tiles['bar'].copy_to(geom)
        bar.set_scale(((max_width*32)-16)/192,1,1)
        bar.flatten_light()
        bar.set_pos(8, 0, -8)
        bar.set_color(tuple(color))
        bar.set_name('bar')
        bar.set_transparency(TransparencyAttrib.M_alpha)
        bar.set_bin("fixed", 0)
        if name is not None:
            self.elements[name]=geom
        if hidden:
            geom.hide()
        return geom

    def widget(self, geom_name,  pos, parent, cmd, color=None, name=None):
        if parent in self.elements:
            parent = self.elements[parent]
        geom=self.tiles[geom_name].copy_to(NodePath(geom_name))
        root=DirectFrame(frameColor=(1,0,0, 0.0),
                         frameSize=None,#_rec2d(32, 32),
                         state=DGG.NORMAL,
                         geom=geom,
                         geom_pos=_pos2d(16, 16),
                         pad=(-2, -2),
                         suppressMouse = True,
                         parent=parent)
        root.set_pos(_pos2d(*pos))
        root.bind(DGG.B1PRESS, self.on_button_press, [root, cmd])
        root.set_transparency(TransparencyAttrib.M_alpha)
        root.set_bin("fixed", 0)
        if color is not None:
            root.set_color(*color)
        #root.set_two_sided(True)
        if name is not None:
            self.elements[name]=root
        return root

    #############################
    ###         Events          #
    #############################
    def on_cursor_set(self, entry, event=None):
        time=globalClock.get_frame_time()
        if (time-self.last_click_time) >0.33:
            m_pos=event.get_mouse()
            entry_pos=entry.get_relative_point(render2d, Vec3(m_pos.x, 0, m_pos.y))#pos in widget space
            x=(entry_pos.x-10)//7 # -offset from the left //character width
            entry.setCursorPosition(int(x))
        else:
            entry.set('')
        self.last_click_time=time

    def on_enter_pressed(self):
        if self.current_focus:
            text=self.current_focus.get()
            widget=self.current_focus
            cmd=widget.get_python_tag('cmd')
            self.on_submit(text, widget, cmd)

    def on_submit(self, text, widget, cmd):
        if cmd:
            if '=' in cmd:
                exec(cmd.replace('txt', 'txt="'+text+'"'))
            else:
                exec(cmd.replace('txt', '"'+text+'"'))
        self.current_focus=None

    def on_focus(self, widget):
        if self.current_focus:
            if self.current_focus != widget:
                self.on_enter_pressed()
        self.current_focus=widget

    def on_slide(self, slider, cmd, event=None):
        v=round(slider['value'], 2)
        if slider in self.slider_cache:
            if self.slider_cache[slider] == v:
                return
        self.slider_cache[slider] = v
        slider['value']=v
        exec(cmd.replace('value', str(v)))

    def on_scroll(self, name, value):
        scroll_bar=self.elements[name+"_verticalScroll"]
        scroll_bar['value']+=value

    def on_button_press(self, button, cmd, event=None):
        def py_exec(command):
            exec(command)
        self.last_button_pressed=button
        button['state']=DGG.DISABLED
        pos=button.get_pos()
        Sequence(LerpPosInterval(button, 0.05, pos+_pos2d(0,2)),
                 LerpPosInterval(button, 0.05, pos),
                 Func(button.configure, **{'state':DGG.NORMAL}),
                 Func(py_exec, cmd)).start()

    def on_window_event(self, window=None):
        if window is not None: # window is none if panda3d is not started
            if self.window_x!=base.win.get_x_size() or self.window_y!=base.win.get_y_size():
                self.on_window_resize()
            elif window.get_properties().get_minimized() !=  self.window_minimized:
                self.on_window_minimize()
            elif window.get_properties().get_foreground() !=  self.window_focused:
                self.on_window_focus()

            self.window_x = base.win.get_x_size()
            self.window_y = base.win.get_y_size()

    def on_window_focus(self):
        pass

    def on_window_minimize(self):
        pass

    def on_window_resize(self):
        self._update_gui_nodes()
        for name, frame in  self.autoresize.items():
            old_frame=self.elements[name]
            if name+'_canvas' in self.elements:
                old_canvas=self.elements[name+'_canvas']
                old_scroll=self.elements[name+'_verticalScroll']

            new_frame=self.frame(**frame)
            if old_frame.is_hidden():
                new_frame.hide()
            else:
                new_frame.show()
            for child in old_frame.get_children():
                if child.get_name() not in  ('ui_tile', 'scrolled_frame'):
                    child.reparent_to(new_frame)
            if name+'_canvas' in self.elements:
                for child in old_canvas.get_children():
                    child.reparent_to(self.elements[name+'_canvas'])
                old_scroll.destroy()
                old_canvas.remove_node()

            old_frame.remove_node()
        for cmd in self.resize_callback:
            cmd()


    #############################
    ###        Private          #
    #############################

    def _scroll_canvas(self, bar, canvas):
        v=bar['value']
        size=canvas.get_python_tag('y_size')
        header=canvas.get_python_tag('header')
        z=size-int(v*size)-header
        canvas.set_z(z)

    def _un_bind_wheel(self):
        self.ignore('wheel_up-up')
        self.ignore('wheel_down-up')

    def _bind_wheel(self, scroll_bar):
        self.accept('wheel_up-up', self.on_scroll, [scroll_bar, 0.02])
        self.accept('wheel_down-up', self.on_scroll, [scroll_bar, -0.02])

    def _parse(self, val):
        r=val
        if isinstance(val, (str, bytes, unicode)):
            winX = base.win.get_x_size()
            winY = base.win.get_y_size()
            val=val.replace('win_x', str(winX))
            val=val.replace('win_y', str(winY))
            r=math_eval(val)
        return int(r)

    def _make_tile(self, uv_offset, rotation=0, frame_size=(-32, 32, -32, 32), tex='gui/ui_atlas.png', tex_scale=(0.25, 0.25) ):
        cm = CardMaker("ui_tile")
        cm.set_frame(frame_size)
        tile=NodePath(cm.generate())
        tex_atlas=loader.load_texture(tex)
        tex_atlas.set_wrap_u(Texture.WM_clamp)
        tex_atlas.set_wrap_v(Texture.WM_clamp)
        tex_atlas.set_magfilter(SamplerState.FT_nearest)
        tex_atlas.set_minfilter(SamplerState.FT_nearest)
        #tex_atlas.set_format(Texture.F_srgb_alpha)
        tile.set_texture(tex_atlas)
        tile.set_tex_offset(TextureStage.get_default(), uv_offset[0], uv_offset[1])
        tile.set_tex_scale(TextureStage.get_default(), tex_scale[0], tex_scale[1])
        tile.set_r(rotation)
        tile.set_transparency(TransparencyAttrib.M_alpha)
        tile.set_bin("fixed", 0)
        return tile

    def _update_gui_nodes(self):
        winX = base.win.get_x_size()
        winY = base.win.get_y_size()
        self.elements['top_left'].set_pos(_pos2d(0,0))
        self.elements['top_right'].set_pos(_pos2d(winX,0))
        self.elements['bottom_right'].set_pos(_pos2d(winX,winY))
        self.elements['bottom_left'].set_pos(_pos2d(0,winY))
        self.elements['top'].set_pos(_pos2d(winX//2,0))
        self.elements['bottom'].set_pos(_pos2d(winX//2,winY))
        self.elements['left'].set_pos(_pos2d(0,winY//2))
        self.elements['right'].set_pos(_pos2d(winX,winY//2))
        self.elements['center'].set_pos(_pos2d(winX//2,winY//2))
