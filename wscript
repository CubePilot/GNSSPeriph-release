#!/usr/bin/env python
# encoding: utf-8

from __future__ import print_function

import os.path
import os
import sys
import shutil
sys.path.insert(0, 'ardupilot/Tools/ardupilotwaf/')
sys.path.insert(0, '.')
import waflib.extras.compat15
import ardupilotwaf
import boards
from waflib.Configure import conf
from waflib import Context, Logs, Task, Utils

def create_bl_hwdef(cfg, realpath, dirname):
    # check if directory contains hwdef-bl-main.dat and hwdef-bl-fastboot.dat
    if os.path.isfile(os.path.join(realpath, 'hwdef-bl-main.dat')):
        # Only create/update symlinks during configure and build phases, not options
        if not hasattr(cfg, 'options') or not hasattr(cfg.options, 'bootloader_fastboot'):
            return
        if cfg.options.bootloader_fastboot:
            print('Using hwdef-bl-fastboot.dat for {}'.format(dirname))
            shutil.copy(os.path.join(realpath, 'hwdef-bl-fastboot.dat'),
                os.path.join(realpath, 'hwdef-bl.dat'))
        else:
            print('Using hwdef-bl-main.dat for {}'.format(dirname))
            shutil.copy(os.path.join(realpath, 'hwdef-bl-main.dat'),
                        os.path.join(realpath, 'hwdef-bl.dat'))

def copy_local_hwdef(cfg, copy_bl_hwdef=False):
    print('Copying local hwdef directories')
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
        if copy_bl_hwdef:
            create_bl_hwdef(cfg, realpath, dir)
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
    print('Local hwdef directories copied')

def options(opt):
    opt.parser.set_defaults(top='ardupilot')
    copy_local_hwdef(opt)
    os.chdir('ardupilot')
    try:
        opt.recurse('ardupilot/')
    except Exception as e:
        os.chdir('..')
        raise e

    opt.ap_groups['configure'].add_option('--bootloader-fastboot',
                  action='store_true',
                  default=False,
                  help='use bootloader fastboot for hardware definitions')
    os.chdir('..')

def configure(cfg):
    copy_local_hwdef(cfg, True)
    os.chdir('ardupilot')
    try:
        cfg.recurse('ardupilot/')
    except Exception as e:
        os.chdir('..')
        raise e
    os.chdir('..')
    cfg.load('wscript_git')

def build(bld):
    copy_local_hwdef(bld)
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
