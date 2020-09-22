#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''OpenHRI Component base

Copyright (C) 2020
    Isao Hara
    All rights reserved.

Licensed under the Eclipse Public License -v 1.0 (EPL)
http://www.opensource.org/licenses/eclipse-1.0.txt
'''

import sys, os, platform
import time, traceback, getopt
import optparse
import re

from utils import *

import rospy


#
#   OpenHRI_Component Class
#
class OpenHRI_Component(object):
  #
  #  Constructor
  #
  def __init__(self, manager):
    self._lang = 'ja-JP'
    self._manager = manager
    self._properties = None
    self._config = manager._config

    self._copyrights = []

  #
  #  show Copyrights
  def show_copyrights(self):
    rospy.loginfo("This component depends on following softwares and data:")
    rospy.loginfo('')

    for c in self._copyrights:
      for l in c.strip('\n').split('\n'):
        rospy.loginfo('  '+l)
      rospy.loginfo('')
    return 

  #
  #  bindParameter
  def bindParameter(self, name, var, value, func=None):
    try:
      if self._config['Default'][name] :
        var = [ self._config['Default'][name] ]
        return
    except:
      pass

    var = [ value ]
    return

  #
  #  OnInitialize
  #
  def onInitialize(self):
    self.show_copyrights()
    return True

  #
  #  OnShutdown
  #
  def onShutdown(self):
    return True

  #
  #  OnFinalize
  #
  def onFinalize(self):
    return True

  #
  #  OnActivate
  #
  def onActivated(self, ec_id=0):
    return True

  #
  #  OnDeactivate
  #
  def onDeactivate(self, ec_id=0):
    return True

  #
  #  OnExecute (Do nothing)
  #
  def onExecute(self, ec_id=0):
    return True

#
#
def set_manager_info(version, usage, doc):
  global __version__, __usage__, __doc__
  __version__=version
  __usage__=usage
  __doc__=doc
  return

#
#  OpenHRI Manager Class
#
class OpenHRI_Manager(object):
  #
  #  Constructor
  #
  def __init__(self, name='openhri_manager', conf_name='openhri.conf'):
    self._parser = MyParser(version=__version__, usage=__usage__, description=__doc__)
    addmanageropts(self._parser)
    self.add_options(self._parser)

    try:
      self._opts, args = self._parser.parse_args()
    except optparse.OptionError as e:
      print('OptionError:', e)
      sys.exit(1)

    if self._opts.configfile is None:
      self._opts.configfile = openhri_path("/etc/"+conf_name)

    self._name = name

    self._args=[]
    for arg in args:
      if arg[:8] == '__name:=' :
        self._name=arg[8:]
      elif arg[:2] == '__':
        pass
      else:
        self._args.append(arg)

    self._rate = 0
    self._comp = {}

  #
  #
  def add_options(self, parser):
    return

  #
  #
  def init_node(self):
    rospy.init_node(self._name, anonymous=True)
    self.moduleInit()
    return

  #
  #  Start component
  #
  def start(self):
    for name in self._comp:
      res=self._comp[name].onActivated()
      if not res: return

    if self._rate > 0:
      self.rate = rospy.Rate(self._rate)
      while not rospy.is_shutdown():
        for name in self._comp:
          self._comp[name].onExecute()
        self.rate.sleep()

    else:
      rospy.spin()
    return

  #
  #
  def create_component(self, klass, *args):
    try:
      comp=klass(self, *args)
      comp.onInitialize()
      return comp
    except:
      print("Fail to create component %s" % str(klass))

    return None

  #
  #  shutdown manager
  #
  def shutdown(self):
    for name in self._comp:
      self._comp[name].onShutdown()
    return

  #
  #  Initialize node
  #
  def moduleInit(self):
    return

