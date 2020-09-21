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
#   OpenHRI_Componnet Class
#
class OpenHRI_Componnet(object):
  #
  #  Constructor
  #
  def __init__(self, manager):
    self._lang = 'ja'
    self._manager = manager
    self._properties = None

    self._copyrights = []

  #
  #  OnInitialize
  #
  def onInitialize(self):
    for c in self._copyrights:
      for l in c.strip('\n').split('\n'):
        rospy.loginfo('  '+l)
      rospy.loginfo('')
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
#  OpenHRI Manager Class
#
class OpenHRI_Manager(object):
  #
  #  Constructor
  #
  def __init__(self, name='openhri_manager'):
    parser = utils.MyParser(version=__version__, usage="%prog [srgsfile]",
                            description=__doc__)
    utils.addmanageropts(parser)
    parser.add_option('-g', '--gui', dest='guimode', action="store_true",
                      default=False,
                      help='show file open dialog in GUI')

    parser.add_option('-D', '--dictation', dest='dictation_mode', 
                      action="store_true",
                      default=False,
                      help='run with dictation mode')

    parser.add_option('-r', '--rebuild-lexicon', dest='rebuild_lexicon',
                      action="store_true",
                      default=False,
                      help='rebuild lexicon')
    try:
      opts, args = parser.parse_args()
    except optparse.OptionError as e:
      rospy.logerr('OptionError:', e , file=sys.stderr)
      sys.exit(1)

    if opts.configfile is None:
      try:
        cfgname = os.environ['OPENHRI_ROOT'] + "/etc/julius.conf".replace('/', os.path.sep)
        if os.path.exists(cfgname):
          opts.configfile = cfgname
      except:
        pass

    self._name=name
    self._config = config_base(config_file=opts.configfile)
    self._comp = {}

    rospy.init_node(self.name, anonymous=True)
    self.moduleInit()

  #
  #  Start component
  #
  def start(self):
    for name in self._comp:
      self._comp[name].onActivated()

    rospy.spin()
    return

  def create_component(self, name, *args):
    try:
      cls = globals()[name]
      comp=cls(self, *args)
      comp.onInitialize()
      return comp
    except:
      print("Fail to create component %s" % name)

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

