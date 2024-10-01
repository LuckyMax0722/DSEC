import os
import sys
from easydict import EasyDict

CONF = EasyDict()

# path
CONF.PATH = EasyDict()
CONF.PATH.BASE = '/media/max/GAME/MA/DSEC'  # TODO: Change path to your DSEC-Toolkit dir
CONF.PATH.DATA = '/media/max/GAME/MA/datasets/demo'  # TODO: Change path to your DSEC-Data dir
CONF.PATH.LOG = os.path.join(CONF.PATH.BASE, 'log')
CONF.PATH.DATA = os.path.join(CONF.PATH.DATA, 'data')

# scene
CONF.SCENE = EasyDict()
CONF.SCENE.SEQUENCE = {
    'interlaken_00' : [51549876996000, 52174199996000],
    'interlaken_01' : [51549876996000, 52174199996000],
    'thun_00' : [51549876996000, 52174199996000],
    'thun_01' : [51549876996000, 52174199996000],
    'zurich_city_00' : [51549876996000, 52174199996000],
    'zurich_city_01' : [51549876996000, 52174199996000],
    'zurich_city_02' : [51549876996000, 52174199996000],
    'zurich_city_03' : [51549876996000, 52174199996000],
    'zurich_city_04' : [51549876996000, 52174199996000],
    'zurich_city_05' : [51549876996000, 52174199996000],
    'zurich_city_06' : [51549876996000, 52174199996000],
    'zurich_city_07' : [51549876996000, 52174199996000],
    'zurich_city_08' : [51549876996000, 52174199996000],
    'zurich_city_09' : [51549876996000, 52174199996000],
    'zurich_city_10' : [51549876996000, 52174199996000],
    'zurich_city_11' : [51549876996000, 52174199996000],
    'zurich_city_12' : [51549876996000, 52174199996000],
    'zurich_city_13' : [51549876996000, 52174199996000],
    'zurich_city_14' : [51549876996000, 52174199996000],
    'zurich_city_15' : [51549876996000, 52174199996000]
}