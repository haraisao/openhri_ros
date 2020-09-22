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
import re
import configparser

import openhri.utils as utils
from openhri.utils import native_path


#
#
class config():
  def __init__(self, julius_dir="", config_file=None):
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

    self._lexicondb = os.path.join(self._configdir, 'lexcon.db')

    #
    # default settings
    if julius_dir :
      julius_dir = re.sub('^%d0', self._basedir[:2], julius_dir)
      self.julius(julius_dir.replace('/', os.path.sep) )
    else:
      self.julius(os.path.join(self._basedir, "Julius") )

    # config
    #print("CONFIG:", os.path.join(self._basedir, 'etc', 'julius.cfg'))
    if config_file is None:
      config_file = os.path.join(self._basedir, 'etc', 'julius.conf')

    if os.path.exists(config_file) :
      self._configfile = configparser.ConfigParser()
      self._configfile.read(config_file)

      self.set_runkit_ja(self._configfile, self._basedir[:2])
      self.set_runkit_en(self._configfile, self._basedir[:2])
      self.set_voxforge_en(self._configfile, self._basedir[:2])
      self.set_voxforge_de(self._configfile, self._basedir[:2])
    else:
      self._configfile={}

  def getProperty(self, name):
    if name in self._configfile:
      return self._configfile[name]
    return None

  def runkit(self, *args):
   return os.path.join(self._julius_runkitdir, *args)

  def runkit_en(self, *args):
   return os.path.join(self._julius_runkitdir_en, *args)

  def voxforge(self, *args):
   return os.path.join(self._julius_voxforgedir, *args)

  def voxforge_de(self, *args):
   return os.path.join(self._julius_voxforgedir_de, *args)

  #
  #  For Julius
  #
  def julius(self, basedir):
    if self._platform == "Windows":
      self._julius_runkitdir = os.path.join(basedir, "dictation-kit")
      self._julius_bin = self.runkit("bin", "windows", "julius.exe")
        
      self._julius_voxforgedir = os.path.join(basedir, "voxforge_en")
      self._julius_hmm_en = self.voxforge("hmmdefs")
      self._julius_hlist_en = self.voxforge("tiedlist")

      self._julius_voxforgedir_de = os.path.join(basedir, "voxforge_de")
      self._julius_hmm_de = self.voxforge_de("hmmdefs")
      self._julius_hlist_de = self.voxforge_de("tiedlist")

      self._julius_runkitdir_en = os.path.join(basedir, "ENVR-v5.4.Gmm.Bin")

      self._julius_bin_en = self.runkit_en("julius-gmm.exe")
      self._julius_dict_hmm_en = self.runkit_en("ENVR-v5.3.am")
      self._julius_dict_hlist_en = self.runkit_en("ENVR-v5.3.phn")
      self._julius_dict_ngram_en = self.runkit_en("ENVR-v5.3.lm")
      self._julius_dict_dict_en = self.runkit_en("ENVR-v5.3.dct")
      self._julius_dict_htkconf_en = self.runkit_en("wav_config")
    else:
      self._julius_runkitdir = "/usr/local/julius-dictation-kit"

      if self.ubuntu_osname == "precise":
        self._julius_dict_en = "/usr/share/doc/julius-voxforge/dict.gz"
      else:
        self._julius_dict_en = "/usr/share/julius-voxforge/acoustic/dict"

      self._julius_voxforgedir = "/usr/share/julius-voxforge"
      self._julius_voxforgedir_de = "/usr/share/julius-voxforge-de"
      self._julius_bin = "/usr/local/julius-dictation-kit/bin/linux/julius"

      self._julius_hmm_en = self.voxforge("acoustic", "hmmdefs")
      self._julius_hlist_en = self.voxforge("acoustic", "tiedlist")
      self._julius_hmm_de = self.voxforge_de("acoustic", "hmmdefs")
      self._julius_hlist_de = self.voxforge_de("acoustic", "tiedlist")

    ####
    #
    self._julius_hmm_ja = self.runkit("model","phone_m","jnas-tri-3k16-gid.binhmm")
    self._julius_hlist_ja = self.runkit("model","phone_m","logicalTri-3k16-gid.bin")
    self._julius_ngram_ja = self.runkit("model", "lang_m", "bccwj.60k.bingram")
    self._julius_dict_ja  = self.runkit("model", "lang_m", "bccwj.60k.htkdic")
    #self._julius_dict_ja  = self.runkit("model", "lang_m", "web.60k.htkdic")
    #
    # for dictation
    self._julius_bingram_ja = self.runkit("model","lang_m","bccwj.60k.bingram")
    self._julius_htkdic_ja = self.runkit("model","lang_m","bccwj.60k.htkdic")
    return

  #
  #
  def set_runkit_ja(self, configfile, drv=""):
    if 'julius.runkit_ja' in configfile :
      try:
        runkit_ja = configfile['julius.runkit_ja']
        self._julius_runkitdir = native_path(runkit_ja['base_dir'])
        if drv :
          self._julius_runkitdir = re.sub('^\$d0', drv, self._julius_runkitdir)

        self._julius_bin = self.runkit(native_path(runkit_ja['executable']))
        self._julius_hmm_ja = self.runkit(native_path(runkit_ja['hmm']))
        self._julius_hlist_ja = self.runkit(native_path(runkit_ja['hlist']))
        self._julius_ngram_ja = self.runkit(native_path(runkit_ja['ngram']))
        self._julius_dict_ja  = self.runkit(native_path(runkit_ja['dict']))
        #
        # for dictation
        self._julius_bingram_ja = self.runkit(native_path(runkit_ja['bingram']))
        self._julius_htkdic_ja = self.runkit(native_path(runkit_ja['htkdic']))
      except:
        print("=== Error in set_runkit_ja")

    return

  #
  #
  def set_voxforge_en(self, configfile, drv=""):
    if 'julius.voxforge' in configfile :
      try:
        voxforge = configfile['julius.voxforge']
        self._julius_voxforgedir = native_path(voxforge['base_dir'])
        if drv :
          self._julius_voxforgedir = re.sub('^\$d0', drv, self._julius_voxforgedir)
        self._julius_hmm_en = self.voxforge(native_path(voxforge['hmm']))
        self._julius_hlist_en = self.voxforge(native_path(voxforge['hlist']))
      except:
        print("=== Error in set_voxforge_en")

    return

  def set_voxforge_de(self, configfile, drv=""):
    if 'julius.voxforge_de' in configfile :
      try:
        voxforge_de = configfile['julius.voxforge_de']
        self._julius_voxforgedir_de = native_path(voxforge_de['base_dir'])
        if drv :
          self._julius_voxforgedir_de = re.sub('^\$d0', drv, self._julius_voxforgedir_de)
        self._julius_hmm_de = self.voxforge_de(native_path(voxforge_de['hmm']))
        self._julius_hlist_de = self.voxforge_de(native_path(voxforge_de['hlist']))
      except:
        print("=== Error in set_voxforge_de")

    return

  #
  #
  def set_runkit_en(self, configfile, drv=""):
    if 'julius.runkit_en' in configfile :
      try:
        runkit_en = configfile['julius.runkit_en']
        self._julius_runkitdir_en = native_path(runkit_en['base_dir'])
        if drv :
          self._julius_runkitdir_en = re.sub('^\$d0', drv, self._julius_runkitdir_en)
        self._julius_bin_en = self.runkit_en(native_path(runkit_en['executable']))
        self._julius_dict_hmm_en = self.runkit_en(native_path(runkit_en['hmm']))
        self._julius_dict_hlist_en = self.runkit_en(native_path(runkit_en['hlist']))
        self._julius_dict_ngram_en = self.runkit_en(native_path(runkit_en['ngram']))
        self._julius_dict_dict_en = self.runkit_en(native_path(runkit_en['dict']))
        self._julius_dict_htkconf_en  = self.runkit_en(native_path(runkit_en['htkconf']))
      except:
        print("=== Error in set_runkit_en")
    return
