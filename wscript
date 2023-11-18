#!/usr/bin/env python
# encoding: utf-8

from __future__ import print_function

import os.path
import os
import sys
import subprocess
import json
import fnmatch
import shutil
sys.path.insert(0, 'ardupilot/Tools/ardupilotwaf/')
import waflib.extras.compat15
import ardupilotwaf
import boards

def copy_local_hwdef():
    # find all folders containing hwdef.dat in current directory
    hwdef_folders = []
    dirname, dirlist, filenames = next(os.walk('.'))
    for dir in dirlist:
        if os.path.isfile(os.path.join(dirname, dir, 'hwdef.dat')):
            hwdef_folders.append(dir)
    ## copy all the directories to the ardupilot directory
    for dir in hwdef_folders:
        # ensure dir is not .
        if dir == '.':
            continue
        # check if symlink exists
        realpath = os.path.realpath(os.path.join(dirname, dir))
        if os.path.islink(os.path.join('ardupilot/libraries/AP_HAL_ChibiOS/hwdef', dir)):
            # check if symlink points to correct directory
            if os.path.realpath(os.path.join('ardupilot/libraries/AP_HAL_ChibiOS/hwdef', dir)) == realpath:
                continue
            else:
                os.remove(os.path.join('ardupilot/libraries/AP_HAL_ChibiOS/hwdef', dir))
        elif os.path.isdir(os.path.join('ardupilot/libraries/AP_HAL_ChibiOS/hwdef', dir)):
            shutil.rmtree(os.path.join('ardupilot/libraries/AP_HAL_ChibiOS/hwdef', dir))
        # create symlink to directory
        os.symlink(realpath, os.path.join('ardupilot/libraries/AP_HAL_ChibiOS/hwdef', dir))
    # create symlinks to files inside bootloader
    dirname, dirlist, filenames = next(os.walk('bootloaders'))
    for file in filenames:
        realpath = os.path.realpath(os.path.join(dirname, file))
        # remove file if it exists
        if os.path.isfile(os.path.join('ardupilot/Tools/bootloaders', file)):
            os.remove(os.path.join('ardupilot/Tools/bootloaders', file))
        # copy file
        shutil.copy(realpath, os.path.join('ardupilot/Tools/bootloaders', file))

def options(opt):
    opt.parser.set_defaults(top='ardupilot')
    copy_local_hwdef()
    os.chdir('ardupilot')
    try:
        opt.recurse('ardupilot/')
    except Exception as e:
        os.chdir('..')
        raise e
    os.chdir('..')

def configure(cfg):
    copy_local_hwdef()
    os.chdir('ardupilot')
    try:
        cfg.recurse('ardupilot/')
    except Exception as e:
        os.chdir('..')
        raise e
    os.chdir('..')

def build(bld):
    copy_local_hwdef()
    bld.env = bld.env_of_name(bld.env.BOARD)
    bld.bldnode = bld.bldnode.make_node(bld.env.BOARD)
    os.chdir('ardupilot')
    bld.recurse('ardupilot/')
    os.chdir('..')
    # remove AP_Periph from build
    bld.recurse('AP_Periph/')
    bld.recurse('AP_Bootloader/')

def list_boards(ctx):
    os.chdir('ardupilot')
    print(*boards.get_boards_names())
    os.chdir('..')
