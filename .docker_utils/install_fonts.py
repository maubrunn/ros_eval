from matplotlib import font_manager
import matplotlib as mpl
import shutil

# nuke your font cache so that it gets rebuilt (discovering newly installed fonts)
shutil.rmtree(mpl.get_cachedir())

font_manager.findSystemFonts(fontpaths=None, fontext="ttf")
font_manager.findfont("Times New Roman")
# should print something like
# '~/.local/share/fonts/Times_New_Roman.ttf'
