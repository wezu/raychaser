from panda3d.core import *
from direct.showbase.DirectObject import DirectObject
from direct.interval.MetaInterval import Sequence, Parallel
from direct.interval.LerpInterval import LerpHprInterval, LerpPosInterval
from direct.interval.FunctionInterval import Func

__all__ = ['CameraControler']

class CameraControler (DirectObject):
    def __init__(self, pos=(0,0,3.0), offset=(0, 12, 12), speed=1.0,
                 zoom_speed=1.0, limits=(3.0, 500.0, -110, 110.0)):
        self.node  = render.attach_new_node("node")
        self.node.set_pos(pos)
        self.gimbal  = self.node.attach_new_node("gimbal")
        base.camera.set_pos(render, offset)
        base.camera.look_at(self.node)
        base.camera.wrt_reparent_to(self.gimbal)

        self.initial_pos=pos
        self.initial_offset=offset

        self.last_mouse_pos=Vec2(0,0)
        self.speed=speed*6000.0
        self.move_speed=speed
        self.zoom_speed=zoom_speed
        self.zoom = 0.0
        self.last_delta=Point2(0,0)
        self.limits=limits[:2]
        self.max_p=limits[2:4]

        self.key_map = {'rotate': False, 'forward':False, 'back':False,
                        'left':False, 'right':False, 'pan':False,
                        'up':False, 'down':False}
        taskMgr.add(self.update, "camcon_update")

        self.bind_keys()

    def set_view(self, view='front'):
        views={'front':(Vec3(90, 0, 0),Vec3(90, 0, 0)),
               'back': (Vec3(-90, 0, 0),Vec3(-90, 0, 0)),
               'left':(Vec3(0, 0, 0),Vec3(0, 0, 0)),
               'right':(Vec3(180, 0, 0),Vec3(180, 0, 0)),
               'top':(Vec3(90, 0, 0),Vec3(90, 90, 0)),
               'bottom':(Vec3(90, 0, 0),Vec3(90, -90, 0))}
        if view in views:
            if self.node.get_hpr(render)!=views[view][0] or self.gimbal.get_hpr(render)!=views[view][1]:
                Sequence(Parallel(LerpHprInterval(self.gimbal, 0.1, views[view][1], other=render),
                                  LerpHprInterval(self.node, 0.1, views[view][0], other=render),
                                  LerpPosInterval(self.node, 0.1, Vec3(0,0,0), other=render)),
                         Func(self.set_view, view)
                         ).start()

    def bind_keys(self, rotate='mouse3', zoom_in='wheel_up', zoom_out='wheel_down',
                    left='a', right='d', forward='w', back='s', pan='shift-mouse1',
                    up='q', down='z', view_left=['4','arrow_left'], view_right=['6', 'arrow_right'],
                    view_front=['7', 'home'], view_back=['1','end'], view_top=['8', 'arrow_up'],
                    view_bottom=['2', 'arrow_down']):
        ''' Make the camera respond to key press/mouse move'''
        self.ignoreAll()
        self.accept(rotate, self.key_map.__setitem__, ['rotate', True])
        self.accept(rotate+'-up', self.key_map.__setitem__, ['rotate', False])
        self.accept(pan, self.key_map.__setitem__, ['pan', True])
        self.accept(pan.split('-')[0]+'-up', self.key_map.__setitem__, ['pan', False])
        self.accept(pan.split('-')[1]+'-up', self.key_map.__setitem__, ['pan', False])
        self.accept(zoom_in, self.zoom_control,[1.0])
        self.accept(zoom_out,self.zoom_control,[-1.0])

        self.accept(view_left[0], self.set_view, ['left'])
        self.accept(view_left[1], self.set_view, ['left'])
        self.accept(view_right[0], self.set_view, ['right'])
        self.accept(view_right[1], self.set_view, ['right'])
        self.accept(view_top[0], self.set_view, ['top'])
        self.accept(view_top[1], self.set_view, ['top'])
        self.accept(view_bottom[0], self.set_view,[ 'bottom'])
        self.accept(view_bottom[1], self.set_view,[ 'bottom'])
        self.accept(view_front[0], self.set_view, ['front'])
        self.accept(view_front[1], self.set_view, ['front'])
        self.accept(view_back[0], self.set_view, ['back'])
        self.accept(view_back[1], self.set_view, ['back'])

        self.accept(left, self.key_map.__setitem__, ['left', True])
        self.accept(left+'-up', self.key_map.__setitem__, ['left', False])
        self.accept(right, self.key_map.__setitem__, ['right', True])
        self.accept(right+'-up', self.key_map.__setitem__, ['right', False])
        self.accept(forward, self.key_map.__setitem__, ['forward', True])
        self.accept(forward+'-up', self.key_map.__setitem__, ['forward', False])
        self.accept(back, self.key_map.__setitem__, ['back', True])
        self.accept(back+'-up', self.key_map.__setitem__, ['back', False])
        self.accept(up, self.key_map.__setitem__, ['up', True])
        self.accept(up+'-up', self.key_map.__setitem__, ['up', False])
        self.accept(down, self.key_map.__setitem__, ['down', True])
        self.accept(down+'-up', self.key_map.__setitem__, ['down', False])
        self.accept('space', self.debug)

    def debug(self):
        print('node', self.node.get_hpr(render))
        print('gimbal',self.gimbal.get_hpr(render))

    def reset(self):
        self.node.set_pos(self.initial_pos)
        self.node.set_hpr(0,0,0)
        self.gimbal.set_hpr(0,0,0)
        base.camera.set_pos(render, self.initial_offset)
        base.camera.look_at(self.node)

    def set_speed(self, speed):
        self.speed=speed*6000.0

    def set_zoom_speed(self, speed):
        self.zoom_speed=speed*5.0

    def zoom_control(self, amount):
        self.zoom+=amount*self.zoom_speed
        self.zoom=min(max(self.zoom, -6.0*self.zoom_speed), 6.0*self.zoom_speed)

    def update(self, task):
        dt = globalClock.getDt()
        if dt > 1.0/12.0:
            return task.cont
        if self.key_map['forward']:
            self.node.set_y(self.node,-self.move_speed*dt)
        elif self.key_map['back']:
            self.node.set_y(self.node,self.move_speed*dt)
        if self.key_map['left']:
            self.node.set_x(self.node,self.move_speed*dt)
        elif self.key_map['right']:
            self.node.set_x(self.node,-self.move_speed*dt)
        if self.key_map['up']:
            self.node.set_z(self.node,self.move_speed*dt)
        elif self.key_map['down']:
            self.node.set_z(self.node,-self.move_speed*dt)

        if base.mouseWatcherNode.has_mouse():
            if self.zoom != 0.0:
                distance=base.camera.get_distance(self.node)
                if (distance > self.limits[0] and self.zoom >0.0) or (distance < self.limits[1] and self.zoom < 0.0):
                    zoom_speed=self.zoom*dt
                    base.camera.set_y(base.camera, zoom_speed)
                    if base.camLens.is_orthographic():
                        film_size=Vec2(base.camLens.get_film_size())
                        film_size[0]-=film_size[0]*zoom_speed*0.1
                        film_size[1]-=film_size[1]*zoom_speed*0.1
                        base.camLens.set_film_size(film_size)
                    zoom_speed*=4.0
                    if self.zoom > 0.1:
                        self.zoom-=zoom_speed
                    elif self.zoom < -0.1:
                        self.zoom-=zoom_speed
                    else:
                        self.zoom=0.0

            m_pos=base.mouseWatcherNode.get_mouse()
            delta = m_pos- self.last_mouse_pos
            self.last_mouse_pos = Vec2(m_pos)
            #pan
            if self.key_map['pan']:
                pos=Vec3(delta.x, delta.y, 0)
                self.node.set_pos(self.node, pos*self.move_speed*dt*100.0)
            #rotate
            if not self.key_map['rotate']:
                delta=self.last_delta*0.9
                if abs(delta[0])<0.001:
                    delta[0]=0
                if abs(delta[1])<0.001:
                    delta[1]=0
            self.last_delta=delta
                #print(delta)
            p=self.gimbal.get_p()- delta[1]*self.speed*0.5*dt
            self.gimbal.set_p(p)
            self.node.set_h(self.node.get_h()- delta[0]*self.speed*0.5*dt)
        return task.again
