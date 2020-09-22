#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''Google Speech Recognition component

Copyright (C) 2017
  Isao Hara
  National Institute of Advanced Industrial Science and Technology (AIST),
  Japan
  All rights reserved.

Licensed under the Eclipse Public License -v 1.0 (EPL)
http://www.opensource.org/licenses/eclipse-1.0.txt
'''

import sys, os, socket, subprocess, signal, threading, platform
import time, struct, traceback, getopt, wave, tempfile
import optparse

import json

from pydub import AudioSegment
from pydub.silence import *

from xml.dom.minidom import Document

import openhri

import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String


__doc__ = 'Julius Speech Recognition component.'
__version__ = openhri.__version__

#
#  
#
class JuliusCliWrap(openhri.CloudSpeechRecogBase):
  
  #
  #  Constructor
  #
  def __init__(self, rtc, host='localhost', port=10000):
    openhri.CloudSpeechRecogBase.__init__(self, 'ja')
    self._service_id={}
    self._password=""

    self._julius_host = host
    self._julius_port = port

    self._julius = openhri.JuliusCli(self._julius_host, self._julius_port)
  #
  #  Request Recaius Voice Recognition
  #
  def request_speech_recog(self, data):
     result = self._julius.request_asr(str(bytearray(data)))
     if result :
       res = json.loads(''.join(result))
     else:
       print("No result")
       res = []
     return res

#
#  JuliusCliRos Class
#
class JuliusCliRos(openhri.OpenHRI_Component):
  #
  #  Constructor
  #
  def __init__(self, manager):
    openhri.OpenHRI_Component.__init__(self, manager)
    self._recog = None
    self._lang = [ "ja-JP" ]
    self._julius_host = [ "localhost" ]
    self._julius_port = [ 10000 ]
    self._min_silence = [ 200 ]
    self._silence_thr = [ -20 ]
    self._min_buflen =  [ 8000 ]
    self._audio_topic =  '/audio_capture/audio'

  #
  #  OnInitialize
  #
  def onInitialize(self):
    rospy.loginfo("GoogleSpeechRecogRTC version " + __version__)
    rospy.loginfo("Copyright (C) 2017 Isao Hara")
    #
    #
    self.bindParameter("lang", self._lang, "jp")
    self.bindParameter("julius_host", self._julius_host, "localhost")
    self.bindParameter("julius_port", self._julius_port, "1000")
    self.bindParameter("min_silence", self._min_silence, "200")
    self.bindParameter("silence_thr", self._silence_thr, "-20")
    self.bindParameter("min_buflen", self._min_buflen, "8000")
    
    self.bindParameter("audio_topic", self._audio_topic, "/audio_capture/audio")

    #
    # create inport for audio stream
    self._audio_sub=rospy.Subscriber(self._audio_topic, AudioData, self.onData)

    #
    # create outport for result
    self._julius_result=rospy.Publisher('/julius/result', String, queue_size=10)

    self.show_copyrights()
    return True

  #
  #  OnShutdown
  #
  def onShutdown(self):
    if self._recog:
      self._recog.terminate()
      self._recog=None
    return True

  #
  #  OnFinalize
  #
  def onFinalize(self):
    if self._recog:
      self._recog.terminate()
      self._recog=None
    return True

  #
  #  OnActivate
  #
  def onActivated(self, ec_id=0):
    self._recog = JuliusCliWrap(self, self._julius_host[0], self._julius_port[0])
    self._recog.setcallback(self.onResult)

    self._recog.set_voice_detect_param(int(self._min_silence[0]),  int(self._silence_thr[0]), int(self._min_buflen[0]))

    self._recog.start()

    return True

  #
  #  OnDeactivate
  #
  def onDeactivate(self, ec_id):
    self._recog.terminate()
    self._recog=None
    return True

  #
  #  OnData (Callback from Subscriber)
  #
  def onData(self, data):
    if self._recog:
      self._recog.write(data.data)
    return


  #
  #  OnResult
  #
  def onResult(self, data):
    doc = Document()
    listentext = doc.createElement("listenText")
    doc.appendChild(listentext)

    if len(data) <= 1:
      listentext.setAttribute("state","RecognitionFailed")
    else:
      try:
        i=0
        for r in data['result']:
          i += 1
          rank = str(i)
          if 'confidence' in r :
            score=str(r['confidence'])
          else:
            score=0.0
          text=r['str']
          hypo = doc.createElement("data")
          hypo.setAttribute("rank", rank)
          hypo.setAttribute("score", score)
          hypo.setAttribute("likelihood", score)
          hypo.setAttribute("text", text)
          rospy.loginfo("#%s: %s (%s)" % (rank, text.encode('utf-8'), score))
          listentext.appendChild(hypo)

        listentext.setAttribute("state","Success")

      except:
        print (traceback.format_exc())
        listentext.setAttribute("state","ParseError")

    res_data = doc.toxml(encoding="utf-8")
    self._julius_result.publish(res_data.decode('unicode_escape'))
    #self._outdata.data = res_data.decode('unicode_escape')
    #self._outport.write()

#
#  Manager Class
#
class JuliusCliManager(openhri.OpenHRI_Manager):
  #
  #  Constructor
  #
  def __init__(self):
    openhri.set_manager_info(__version__, "%prog", __doc__)
    openhri.OpenHRI_Manager.__init__(self, name='JuliusCli',
                                     conf_name='julius.conf')
    self._config = None

  #  Initialize rtc
  #
  def moduleInit(self):
    self._comp['JuliusCli'] = self.create_component(JuliusCliRos)
    return

g_manager = None

def main():
  global g_manager
  g_manager = JuliusCliManager()
  g_manager.init_node()
  g_manager.start()
  g_manager.shutdown()

#
#  Main
#
if __name__=='__main__':
  main()

