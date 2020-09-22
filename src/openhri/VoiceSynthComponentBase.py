#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''Base class for speech synthesis components

Copyright (C) 2010
  Yosuke Matsusaka
  Intelligent Systems Research Institute,
  National Institute of Advanced Industrial Science and Technology (AIST),
  Japan
  All rights reserved.

Copyright (C) 2017-2019
  Isao Hara
  National Institute of Advanced Industrial Science and Technology (AIST), Japan
  All rights reserved.

Licensed under the Eclipse Public License -v 1.0 (EPL)
http://www.opensource.org/licenses/eclipse-1.0.txt
'''

import os
import sys
import time
import signal
import tempfile
import traceback
import platform
import wave

from manager import *

import rospy
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

#
#  get current time
#
def getCurrentTime():
  if platform.system() == 'Windows':
    now = time.clock()
  else:
    now = time.time()
  return now

#
#   Voice Synthesizer Base Class
#
class VoiceSynthBase(object):
  #
  #  Constructor
  #
  def __init__(self):
    self._durationdata = ""
    self._fp = None
    self._history = []
    self._cache = {}
    self._cachesize = 10
    self._copyrights = []
  #
  #  get temporary file name
  #
  def gettempname(self):
    # get temp file name
    fn = tempfile.mkstemp()
    os.close(fn[0])
    return fn[1]

  #
  #  save Wavformatted data
  #
  def synth(self, data, samplerate, character):
    if not data:
      return
    if self._fp is not None:
      self._fp.close()
      self._fp = None
    try:
      (self._durationdata, wavfile) = self._cache[(data, samplerate, character)]
      self._fp = wave.open(wavfile, 'rb')
    except KeyError:
      (self._durationdata, wavfile) = self.synthreal(data, samplerate, character)
      self._history.append((data, samplerate, character))
      self._cache[(data, samplerate, character)] = (self._durationdata, wavfile)
      self._fp = wave.open(wavfile, 'rb')
      if len(self._history) > self._cachesize:
        d = self._history.pop(0)
        (logdata, wavfile) = self._cache[d]
        del self._cache[d]
        del logdata
        os.remove(wavfile)
    return

  #
  #  TTS conversion
  #
  def synthreal(self, data, samplerate, character):
    pass
  #
  #   read data from file.
  #
  def readdata(self, chunk):
    if self._fp is None:
      return None
    try:
      data = self._fp.readframes(chunk)
    except ValueError:
      self._fp.close()
      self._fp = None
      return None
    if data == '':
      self._fp.close()
      self._fp = None
      return None
    return data
  #
  #  terminated
  #
  def terminate(self):
    pass

#
#  Voice Synthesizer Component Base Class
#
class VoiceSynthComponentBase(OpenHRI_Component):
  #
  # Constructor
  #
  def __init__(self, manager):
    OpenHRI_Component.__init__(self, manager)
    self._wrap = None
  #
  #  OnInitialize
  #
  def onInitialize(self):
    #rospy.loginfo(self._properties.getProperty("type_name") + " version " + self._properties.getProperty("version"))
    rospy.loginfo("Copyright (C) 2010-2011 Yosuke Matsusaka")
    rospy.loginfo("Copyright (C) 2017 Isao Hara")
    rospy.loginfo("Copyright (C) 2020 Isao Hara")
    self._prevtime = time.clock()

    # configuration parameters
    self._samplerate = [16000,]
    self.bindParameter("rate", self._samplerate, 16000)
    self._character = ["male",]
    self.bindParameter("character", self._character, "male")

    self._sampling_rate = [0,]
    self.bindParameter("sampling_rate", self._sampling_rate, 0)

    # create inport
    self._tts_sub = rospy.Subscriber('/tts/data', String, self.onData)

    # create outport for wave data
    self._audio_out = rospy.Publisher('/audio_play/audio', AudioData, queue_size=1)
    self._outdata = AudioData()

    self._status = "ready"
    self._is_active = False
    return True

  #
  #
  def onShutdown(self):
    if self._wrap :
      self._wrap.terminate()
    return True

  #
  #  OnFinalize
  #
  def onFinalize(self):
    if self._wrap :
      self._wrap.terminate()
    return True

  #
  #  OnActivate
  #
  def onActivated(self, ec_id=0):
    self._is_active = True
    return True

  #
  #  OnDeactivate
  #
  def onDeactivate(self, ec_id=0):
    self._is_active = False
    return True

  #
  #  OnData (callback function)
  #
  def onData(self, data):
    try:
      if self._is_active == True:
        #udata = data.data.encode('raw-unicode-escape').decode()
        udata = data.data.decode('utf-8')


        #rospy.loginfo(udata + " " + str(self._samplerate[0]) + " " + self._character[0])
        if self._wrap is not None:
          self._wrap.synth(udata, self._samplerate[0], self._character[0])

    except:
      rospy.logerr(traceback.format_exc())

  #
  #  OnExecute (Periodic execution) 
  #
  def onExecute(self, ec_id=0):
    try:
      # send stream
      now = getCurrentTime()
      chunk = int(self._samplerate[0] * (now - self._prevtime))
      data = None
      if chunk > 0:
        self._prevtime = now
        data = self._wrap.readdata(chunk)
        if data is not None:
          if self._status != "started":
            rospy.loginfo("stream started")
            self._status = "started"
            data2 = self._wrap.readdata(int(self._samplerate[0] * 1.0))
            if data2 is not None:
              data += data2

          self._outdata.data = data
          #self._outport.write(self._outdata)
          self._audio_out.publish(self._outdata)

        else:
          if self._status != "finished":
            rospy.loginfo("stream finished")
            self._status= "finished"
          pass
    except:
      rospy.logerr(traceback.format_exc())
    return True

