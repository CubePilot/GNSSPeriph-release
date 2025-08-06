#!/usr/bin/env python

"""
script to build all of our bootloaders using AP_Bootloader and put the resulting binaries in Tools/bootloaders
"""

import os
import shutil
import subprocess
import sys
import fnmatch
import re

# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='make_secure_bl')
parser.add_argument("--signing-key", type=str, default=None, help="signing key for secure bootloader")
parser.add_argument("--debug", action='store_true', default=False, help="build with debug symbols")
parser.add_argument("--periph-only", action='store_true', default=False, help="only build AP_Periph boards")
parser.add_argument("pattern", type=str, default='*', help="board wildcard pattern")
args = parser.parse_args()

if args.signing_key is not None and os.path.basename(args.signing_key).lower().find("private") != -1:
    # prevent the easy mistake of using private key
    print("You must use the public key in the bootloader")
    sys.exit(1)

os.environ['PYTHONUNBUFFERED'] = '1'

failed_boards = set()

def read_hwdef(filepath):
    '''read a hwdef file recursively'''
    fh = open(filepath)
    ret = []
    text = fh.readlines()
    for line in text:
        m = re.match(r"^\s*include\s+(.+)\s*$", line)
        if m is not None:
            ret += read_hwdef(os.path.join(os.path.dirname(filepath), m.group(1)))
        else:
            ret += [line]
    return ret

def is_ap_periph(hwdef):
    '''return True if a hwdef is for a AP_Periph board'''
    lines = read_hwdef(hwdef)
    for line in lines:
        if line.find('AP_PERIPH') != -1:
            return True
    return False

def get_board_list():
    '''add boards based on existance of hwdef-bl.dat in subdirectories for ChibiOS'''
    board_list = []
    dirname, dirlist, filenames = next(os.walk('.'))
    for d in dirlist:
        hwdef = os.path.join(dirname, d, 'hwdef-bl.dat')
        if os.path.exists(hwdef):
            if args.periph_only and not is_ap_periph(hwdef):
                continue
            board_list.append(d)
            # Check if fastboot bootloader exists
            hwdef_fastboot = os.path.join(dirname, d, 'hwdef-bl-fastboot.dat')
            if os.path.exists(hwdef_fastboot):
                board_list.append(d + '_fastboot')
    return board_list

def run_program(cmd_list):
    print("Running (%s)" % " ".join(cmd_list))
    retcode = subprocess.call(cmd_list)
    if retcode != 0:
        print("Build failed: %s" % ' '.join(cmd_list))
        return False
    return True

def build_board(board):
    # Check if this is a fastboot bootloader build
    is_fastboot = board.endswith('_fastboot')
    actual_board = board.replace('_fastboot', '') if is_fastboot else board
    
    configure_args = "--board %s -g --bootloader --no-submodule-update --Werror" % actual_board
    configure_args = configure_args.split()
    if is_fastboot:
        configure_args.append("--bootloader-fastboot")
    if args.signing_key is not None:
        print("Building secure bootloader")
        configure_args.append("--signed-fw")
    if args.debug:
        print("Building with debug symbols")
        configure_args.append("--debug")
    if not run_program(["./waf", "configure"] + configure_args):
        return False
    if not run_program(["./waf", "clean"]):
        return False
    if not run_program(["./waf", "bootloader"]):
        return False
    return True

for board in get_board_list():
    # Special handling: if pattern matches base board name, also include fastboot variant
    board_matches = fnmatch.fnmatch(board, args.pattern)
    if not board_matches:
        # Check if this is a fastboot board and the pattern matches the base board
        if board.endswith('_fastboot'):
            base_board = board.replace('_fastboot', '')
            if fnmatch.fnmatch(base_board, args.pattern):
                board_matches = True
    
    if not board_matches:
        continue
    print("Building for %s" % board)
    
    # Check if this is a fastboot bootloader build
    is_fastboot = board.endswith('_fastboot')
    actual_board = board.replace('_fastboot', '') if is_fastboot else board
    
    if not build_board(board):
        failed_boards.add(board)
        continue
    
    if is_fastboot:
        bl_file = 'bootloaders/%s_fastboot_bl.bin' % actual_board
        hex_file = 'bootloaders/%s_fastboot_bl.hex' % actual_board
        elf_file = 'bootloaders/%s_fastboot_bl.elf' % actual_board
    else:
        bl_file = 'bootloaders/%s_bl.bin' % board
        hex_file = 'bootloaders/%s_bl.hex' % board
        elf_file = 'bootloaders/%s_bl.elf' % board
    source_file = 'build/%s/bin/AP_Bootloader.bin' % actual_board
    shutil.copy(source_file, bl_file)
    if board == 'Here4':
        shutil.copy(source_file, 'bootloaders/Here4AKM_bl.bin')
    print("Created %s" % bl_file)
    shutil.copy('build/%s/bootloader/AP_Bootloader' % actual_board, elf_file)
    print("Created %s" % elf_file)
    if args.signing_key is not None:
        print("Signing bootloader with %s" % args.signing_key)
        if not run_program(["./ardupilot/Tools/scripts/signing/make_secure_bl.py", bl_file, args.signing_key]):
            print("Failed to sign bootloader for %s" % board)
            sys.exit(1)
        if not run_program(["./ardupilot/Tools/scripts/signing/make_secure_bl.py", elf_file, args.signing_key]):
            print("Failed to sign ELF bootloader for %s" % board)
            sys.exit(1)
    bl_addr = '0x08000000'
    if not run_program([sys.executable, "ardupilot/Tools/scripts/bin2hex.py", "--offset", bl_addr, bl_file, hex_file]):
        failed_boards.add(board)
        continue
    print("Created %s" % hex_file)

if len(failed_boards):
    print("Failed boards: %s" % list(failed_boards))
