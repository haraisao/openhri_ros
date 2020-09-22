#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''configuration manager for OpenHRIVoice

Copyright (C) 2010
    Yosuke Matsusaka
    Intelligent Systems Research Institute,
    National Institute of Advanced Industrial Science and Technology (AIST),
    Japan
    All rights reserved.

Copyright (C) 2019
    Isao Hara
    National Institute of Advanced Industrial Science and Technology (AIST), Japan
    All rights reserved.

Licensed under the Eclipse Public License -v 1.0 (EPL)
http://www.opensource.org/licenses/eclipse-1.0.txt
'''

import sys
import os
import platform
import traceback
from openhri.utils import native_path, getHriDir
import re

import configparser

#
#
class config():
  def __init__(self, hri_dir="", config_name='google_speech.conf', config_file=None):
    self._platform = platform.system()

    if self._platform != "Windows":
      my_platform_list =  platform.platform().split("-")
      self.ubuntu_osname = my_platform_list[len(my_platform_list)-1]

    if hasattr(sys, "frozen"):
      self._basedir = os.path.dirname(sys.executable)
    else:
      self._basedir = getHriDir()

    self._homedir = os.path.expanduser('~')

    self._configdir = os.path.join(self._homedir, '.openhri')
    if os.path.exists(self._configdir) == False:
      os.makedirs(self._configdir)

    #
    # default settings
    if hri_dir :
      hri_dir = re.sub('^%d0', self._basedir[:2], hri_dir)
      hri_dir = hri_dir.replace('/', os.path.sep)
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
  def getProperty(self, name, section="Default"):
    try:
      return self._configfile[section][name]
    except:
      pass
    return None

