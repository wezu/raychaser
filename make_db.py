import sqlite3
import opticalmaterialspy as mat_spy

materials={'Ktp_x':mat_spy.Ktp('x'),
        'Ktp_y':mat_spy.Ktp('y'),
        'Ktp_z':mat_spy.Ktp('z'),
        'Ln_e':mat_spy.Ln('e'),
        'Ln_o':mat_spy.Ln('o'),
        'Tfln_e':mat_spy.Tfln('e'),
        'Tfln_o':mat_spy.Tfln('o'),
        'LnMg_e':mat_spy.LnMg('e'),
        'LnMg_o':mat_spy.LnMg('e'),
        'Bbo_e':mat_spy.Bbo('e'),
        'Bbo_o':mat_spy.Bbo('o'),
        'Bibo_x':mat_spy.Bibo('x'),
        'Bibo_y':mat_spy.Bibo('y'),
        'Bibo_z':mat_spy.Bibo('z'),
        'Chalcogenide_As2S3':mat_spy.Chalcogenide('As2S3'),
        'Chalcogenide_As2Se3':mat_spy.Chalcogenide('As2Se3'),
        'Chalcogenide_GeSe4':mat_spy.Chalcogenide('GeSe4'),
        'Chalcogenide_Ge10As10Se80':mat_spy.Chalcogenide('Ge10As10Se80'),
        'SiO2':mat_spy.SiO2(),
        'Su8':mat_spy.Su8(),
        'Al2O3_e':mat_spy.Al2O3('e'),
        'Al2O3_o':mat_spy.Al2O3('o'),
        'TiO2_e':mat_spy.TiO2('e'),
        'TiO2_o':mat_spy.TiO2('o')}

conn = sqlite3.connect('data/mat_spy.db')
c = conn.cursor()
# Create table
c.execute('''CREATE TABLE materials
             (material TEXT, wavelength INTEGER, ior REAL)''')

            
for name, material in materials.items():
    print("material:", material)
    for wavelength in range(300, 5000):    
        if wavelength%50.0==0:
            print('wavelength:', wavelength)
        ior=material.n(wavelength)
        try:
            c.execute("INSERT INTO materials VALUES (?,?,?)", [name, wavelength,ior])
        except Exception as err:
            print(err)
conn.commit()
conn.close()