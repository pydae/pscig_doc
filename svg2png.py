import subprocess
import os
import pickle


from_dir = os.path.join('.','svg')
to_dir = os.path.join('.','figs')
#thumb_dir = os.path.join('.','thumbnails')

files_dict = {}

if not os.path.isdir(to_dir):    
    os.mkdir(to_dir)
    
#if not os.path.isdir(thumb_dir):    
#    os.mkdir(thumb_dir)



files_data = {}

if os.path.isfile('svgfiles.pkl'):
    fobj = open('svgfiles.pkl','rb')
    files_dict = pickle.load(fobj)
    fobj.close()

for filenameExtension in os.listdir(from_dir):
    
    date = (os.path.getmtime(os.path.join(from_dir, filenameExtension)))
    files_data.update({filenameExtension:date})
    
    
    if filenameExtension in files_dict:
        if files_dict[filenameExtension] == date:
            continue
        
 
    
    filename, ext=os.path.splitext(filenameExtension)
    
    if os.name == 'posix':
        pass
        #p=subprocess.call(['/usr/bin/inkscape', from_dir + '/' + filename + '.svg', '--export-width=100', '--export-png',  thumb_dir + '/' + filename + '.png'])
    if os.name == 'nt':
        print(filenameExtension)
        inkscape_path = r"'C:\Program Files\Inkscape\bin\inkscape.exe'"
        png_path = os.path.join(to_dir , filename+'.png')
        svg_path = os.path.join(from_dir , filename+'.svg')
        p=subprocess.call([r'C:\Program Files\Inkscape\bin\inkscape.exe',f'--export-dpi=300 --export-type="png"',f'{svg_path}'])
