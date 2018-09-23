'''Drop in replacement for opticalmaterialspy
Only implements the ior function  - .n()
Needs a pre-made database, one should be provided in 'data/mat_spy.db'
The databse can be generated using the make_db.py script
'''
import sqlite3
from functools import lru_cache

class Material:
    '''Base class for all materials, implements all logic,
    all other classes are placeholders to mimic the opticalmaterialspy api'''
    def __init__(self, *args, **kwargs):
        self.mat_key=self.__class__.__name__
        for arg in args:
            self.mat_key+='_'+str(arg)
        for arg in kwargs.values():
            self.mat_key+='_'+str(arg)

    @lru_cache(maxsize=32)
    def n(self, wavelength=None):
        conn = sqlite3.connect('data/mat_spy.db')
        c = conn.cursor()
        c.execute('SELECT ior FROM materials WHERE material=? AND wavelength=?', [self.mat_key, wavelength])
        value=c.fetchone()
        if value:
            return value[0]
        return 1.0

class CustomIOR:
    '''A special material type that has a constant IOR for all wavelengths'''
    def __init__(self, ior):
        self.ior=float(ior)

    def n(self, wavelength=None):
        return self.ior

class Ktp(Material):
    pass

class Ln(Material):
    pass

class Tfln(Material):
    pass

class LnMg(Material):
    pass

class Bbo(Material):
    pass

class Bibo(Material):
    pass

class Chalcogenide(Material):
    pass

class SiO2(Material):
    pass

class Su8(Material):
    pass

class Al2O3(Material):
    pass

class TiO2(Material):
    pass

class Air(Material):
    pass

    def n(self, wavelength=None):
        return 1.0
