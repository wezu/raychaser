import json
from panda3d.core import NodePath

class CustomEncoder(json.JSONEncoder):
    '''A JSONEncoder that encodes all iterable objects into lists'''
    def default(self, o):
        if isinstance(o, NodePath):
            return None
        try:
            iterable = iter(o)
        except TypeError:
            pass
        else:
            return list(iterable)
        # Let the base class default method raise the TypeError
        return json.JSONEncoder.default(self, o)
