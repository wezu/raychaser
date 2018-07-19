from panda3d.core import *
import random

class ColorPalette:
    def __init__(self, pastel=0.5):
        self.pastel=pastel
        self.colors=[Vec4(0,0,0,1),
                     Vec4(1,1,1,1),
                     Vec4(0,0,0,0),
                     Vec4(0.1, 0.38, 0.23, 1.0)
                    ]

    def _get_random_color(self):
        return Vec4(*[round((x+self.pastel)/(1.0+self.pastel), 2) for x in [random.uniform(0.1, 1.0) for i in [1,2,3]]], 1.0)

    def new_color(self, old_color=None):
        if old_color is not None:
            if old_color in self.colors:
                del self.colors[self.colors.index(old_color)]
        max_distance = None
        best_color = None
        for i in range(0,100):
            color = self._get_random_color()
            if not self.colors:
                self.colors.append(color)
                return color
            best_distance = min([(color-c).length() for c in self.colors])
            if best_distance >0.3:
                self.colors.append(color)
                return color
            if not max_distance or best_distance > max_distance:
                max_distance = best_distance
                best_color = color
        self.colors.append(best_color)
        return best_color


