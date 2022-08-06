#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''Utility functions

Copyright (C) 2010
  Yosuke Matsusaka
  Intelligent Systems Research Institute,
  National Institute of Advanced Industrial Science and Technology (AIST),
  Japan
  All rights reserved.

Copyright (C) 2017
  Isao Hara
  National Institute of Advanced Industrial Science and Technology (AIST),
  Japan
  All rights reserved.

Licensed under the Eclipse Public License -v 1.0 (EPL)
http://www.opensource.org/licenses/eclipse-1.0.txt
'''

import sys
import os
import platform
import optparse
import time

import configparser

try:
  import Tkinter as tkinter
  import tkFileDialog as filedialog
except:
  import tkinter 
  from tkinter import filedialog

try:
  import rospkg
except:
  rospkg=None

#
#
def native_path(dirname):
  return dirname.replace('/', os.path.sep)

#
#
def get_apikey(fname):
  try:
    with open(fname) as f:
      key = f.readline().strip()
      return key
  except:
    pass
  return ""

#
#
def openhri_path(filename):
  try:
    pname = os.environ['OPENHRI_ROOT']+native_path(filename)
    if os.path.exists(pname):
      return pname
  except:
    pass
  return None

#
#
class config_base(object):
  def __init__(self, hri_dir="", config_name="openhri.conf", config_file=None):
    self._platform = platform.system()

    if self._platform != "Windows":
      my_platform_list =  platform.platform().split("-")
      self.ubuntu_osname = my_platform_list[len(my_platform_list)-1]

    if hasattr(sys, "frozen"):
      self._basedir = os.path.dirname(sys.executable)
    else:
      self._basedir = utils.getHriDir()

    self._homedir = os.path.expanduser('~')
    self._configdir = os.path.join(self._homedir, '.openhri')
    if os.path.exists(self._configdir) == False:
      os.makedirs(self._configdir)


    #
    # default settings
    if hri_dir :
      hri_dir = re.sub('^%d0', self._basedir[:2], hri_dir)
      hri_dir = native_path(hri_dir)
    else:
      hri_dir = self._basedir

    if config_file is None:
      config_file = os.path.join(hri_dir, 'etc', config_name)

    # config
    if os.path.exists(config_file) :
      self._configfile = configparser.ConfigParser()
      self._configfile.read(config_file)
    else:
      self._configfile={}


  #
  #
  def getProperty(self, name, category='Default'):
    try:
      return self._configfile[category][name]
    except:
      return None


#
#  Original Parser
#
class OpenHRI_Parser(optparse.OptionParser):
  #
  #
  def _add_help_option (self):
    self.add_option("-h", "--help", action="help",
                    help="show this help message and exit")

  def _add_version_option (self):
    self.add_option("--version", action="version",
                    help="show program's version number and exit")

  def format_epilog(self, formatter):
    if self.epilog is not None:
      return self.epilog
    else:
      return ''

  def exit(self, status=0, msg=None):
    if msg is not None:
      sys.stderr.write(msg)
    sys.exit(status)

  def print_usage(self, file=None):
    if file == None :
      file = sys.stdout
    file.write(self.get_usage() + '\n')

  def print_help(self, file=None):
    if file == None :
      file = sys.stdout
    file.write(self.format_help() + '\n')

  def print_version(self, file=None):
    if file == None :
      file = sys.stdout
    file.write(self.get_version() + '\n')

#
#  Selector dialog for single file reading
#
def askopenfilename(title=''):
  root = tkinter.Tk()
  root.withdraw()
  fTyp = [("","*")]
  fname = filedialog.askopenfilename(filetypes = fTyp,
                                      initialdir = "", title=title)
  return fname
#
#  Selector dialog for multi-files reading
#
def askopenfilenames(title=''):
  root = tkinter.Tk()
  root.withdraw()
  fTyp = [("","*")]
  fname = filedialog.askopenfilenames(filetypes = fTyp,
                                      initialdir = "", title=title)
  return fname
#
#  Selector dialog for saving file
#
def asksaveasfile():
  root = tkinter.Tk()
  fTyp = [("","*")]
  filename =  filedialog.asksaveasfilename(initialdir = "",
                                           title = "Save as", filetypes = fTyp)
  return fname

#
#  option definition for rtm_manager
#
def addmanageropts(parser):
  parser.add_option('-a', '--manager-service', dest='managerservice',
                    action='store_true', default=False,
                    help='enable manager to be controlled as corba servant')

  parser.add_option('-f', '--config-file', dest='configfile',
                    action='store', default=None,
                    help='specify custom configuration file')

  parser.add_option('-o', '--option', dest='option',
                    action='append', default=None,
                    help='specify custom configuration parameter')

  parser.add_option('-p', '--port', dest='port',
                    action='store', default=None,
                    help='specify custom corba endpoint')

  parser.add_option('-d', '--master-mode', dest='mastermode',
                    action='store_true', default=False,
                     help='configure manager to be master')
#
#  option definition for rtm_manager
#
def genmanagerargs(opt):
  args = [sys.argv[0],]
  if opt.managerservice == True:
    args.append('-a')
  if opt.configfile is not None:
    args.append('-f')
    args.append(opt.configfile)
  if opt.option is not None:
    for o in opt.option:
      args.append('-o')
      args.append(o)
  if opt.port is not None:
    args.append('-p')
    args.append(port)
  if opt.mastermode == True:
    args.append('-d')
  return args

#
#
def getHriDir():
  if 'OPENHRI_ROOT' in os.environ and os.environ['OPENHRI_ROOT']:
    return os.environ['OPENHRI_ROOT']

  try:
    return rospkg.RosPack().get_path("openhri_ros")
  except:
    pass

  if 'ROS_ROOT' in os.environ and os.environ['ROS_ROOT']:
    if os.path.exists(os.path.dirname(os.environ['ROS_ROOT'])+"/openhri"):
      return os.path.dirname(os.environ['ROS_ROOT'])+"/openhri"

  #dirname=os.path.dirname(os.path.abspath(__file__))
  dirname=os.getcwd()
  seq=dirname.split(os.path.sep)
  if (os.path.splitext(seq.pop())[1] == ".pyz" ):
    return os.path.sep.join(seq)
  else:
    return dirname
