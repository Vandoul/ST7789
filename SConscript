
from building import *

src = []
cwd = GetCurrentDir()
CPPPATH = [cwd]

if GetDepend(['PKG_USING_ST7789']):
    src += Glob('lcd_st7789.c')

group = DefineGroup('st7789', src, depend = [], CPPPATH = CPPPATH)

Return('group')
