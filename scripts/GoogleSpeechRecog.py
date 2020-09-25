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
from __future__ import print_function
import sys, os, platform
import time, struct, traceback, tempfile, optparse

import json
from pydub import AudioSegment
from pydub.silence import *

from xml.dom.minidom import Document

from openhri.google_asr.config import config
import openhri

import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String


#
#
__doc__ = 'Google Speech Recognition component.'
__version__ = '1.0'

PYTHON_MAJOR_VERSION = sys.version_info.major
if PYTHON_MAJOR_VERSION == 2:
  from io import open
  from urllib import urlencode
else:
  from urllib.parse import urlencode

import urllib3 

#####################################################
#  Google Speech-to-Text Wrapper
#
class GoogleSpeechRecogWrap(openhri.CloudSpeechRecogBase):
  #
  #  Constructor
  #
  def __init__(self, node, language='ja-JP'):
    openhri.CloudSpeechRecogBase.__init__(self, language)

    self._endpoint = 'http://www.google.com/speech-api/v2/recognize'
    self._http = urllib3.PoolManager()
    self._lang=language

    self._config = node._manager._config
    param=[""]
    self._apikey=self.bindParameter("google.speech.apikey", param, None)
    #self._lang=self.bindParameter("google.speech.lang", param, language)
    self.logdir=self.bindParameter("google.speech.logdir", param, None)
    self._logger=self.bindParameter("google.speech.logdir", param, False)

    if not self._apikey :
      try:
        self._apikey = openhri.get_apikey(os.environ['HOME']+'/.openhri/google_apikey.txt')
      except:
        pass
        
  #
  #  Set ApiKey
  #
  def set_apikey(self, key):
    self._apikey = key


  #
  #  Request Google Voice Recognition
  #
  def request_speech_recog(self, data):
    query_string = {'output': 'json', 'lang': self._lang, 'key': self._apikey}
    url = '{0}?{1}'.format(self._endpoint, urlencode(query_string)) 
    headers = {'Content-Type': 'audio/l16; rate=16000'}
    voice_data = bytearray(data)

    try:
      result = self._http.urlopen('POST', url, body=voice_data, headers=headers)
      if result.status == 200:
        response = result.data
        return response.decode('utf-8').split()
      else:
        rospy.logerr("Fail to ASR: status = %d", result.status)
        return ["Error"]

    except:
      print (url)
      print (traceback.format_exc())
      return ["Error"]

#####################################################
#  GoogleSpeechRecog Class
#
class GoogleSpeechRecog(openhri.OpenHRI_Component):
  #
  #  Constructor
  #
  def __init__(self, manager):
    openhri.OpenHRI_Component.__init__(self, manager)

    self._recog = None
    self._lang = [ "ja-JP" ]
    self._min_silence = [ 200 ]
    self._silence_thr = [ -20 ]
    self._min_buflen =  [ 8000 ]
    self._audio_topic = "/audio_capture/audio"
    self._result_topic = "/asr/result"
    self._result_raw_topic = "/asr/result_raw"
    self._result_raw_threshold = 0.5

  #
  #  OnInitialize
  #
  def onInitialize(self):
    rospy.loginfo("GoogleSpeechRecogRos version " + __version__)
    rospy.loginfo("Copyright (C) 2017 Isao Hara")

    #
    # Read parameters
    self.bindParameter("google.speech.lang", self._lang, 'ja-JP')
    self.bindParameter("google.speech.min_silence",self._min_silence,"200",int)
    self.bindParameter("google.speech.silence_thr",self._silence_thr,"-20",int)
    self.bindParameter("google.speech.min_buflen", self._min_buflen,"8000",int)
    self._audio_topic = self.bindParameter("google.speech.audio_topic",None, self._audio_topic)
    self._result_topic = self.bindParameter("google.speech.result_topic",None, self._result_topic)
    self._result_raw_topic = self.bindParameter("google.speech.result_raw_topic",None, self._result_raw_topic)
    self._result_raw_threshold = self.bindParameter("google.speech.result_raw_threshold",None, self._result_raw_threshold, float)

    #
    # create inport for audio stream
    self._audio_sub = rospy.Subscriber(self._audio_topic,AudioData,self.onData)

    #
    # create outport for result
    self._asr_result = rospy.Publisher(self._result_topic, String, queue_size=10)
    self._asr_result_raw = rospy.Publisher(self._result_raw_topic, String, queue_size=10)

    #
    #
    self.show_copyrights()
    return True


  #
  #
  def onShutdown(self):
    if self._recog:
      self._recog.terminate()
    return

  #
  #  OnActivate
  #
  def onActivated(self, ec_id=0):
    self._recog = GoogleSpeechRecogWrap(self, self._lang[0])
    self._recog.setcallback(self.onResult)

    self._recog.set_voice_detect_param(int(self._min_silence[0]),  int(self._silence_thr[0]), int(self._min_buflen[0]))

    if self._recog._apikey:
      self._recog.start()
      return True
    else:
      rospy.logerr("No API Key presened")
      return False

  #
  #  OnDeactivate
  #
  def onDeactivate(self, ec_id=0):
    if self._recog:
      self._recog.terminate()
      self._recog = None
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
        data.pop(0)
        res = ''.join(data)
        #print res;
        result=json.loads(res)
        i=0
        result_raw=None
        for r in result['result'][0]['alternative']:
          i += 1
          rank = str(i)
          if 'confidence' in r :
            score=str(r['confidence'])
          else:
            score=0.0
          text=r['transcript']
          hypo = doc.createElement("data")
          hypo.setAttribute("rank", rank)
          hypo.setAttribute("score", score)
          hypo.setAttribute("likelihood", score)
          hypo.setAttribute("text", text)
          rospy.loginfo("#%s: %s (%s)" % (rank, text.encode('utf-8'), score))
          listentext.appendChild(hypo)

          if ( result_raw is None ) and (float(score) > self._result_raw_threshold) :
            result_raw = text.encode('utf-8')
            self._asr_result_raw.publish(result_raw)

        listentext.setAttribute("state","Success")

      except:
        print (traceback.format_exc())
        listentext.setAttribute("state","ParseError")

    res_data = doc.toxml(encoding="utf-8")
    self._asr_result.publish(res_data)
    return

#
#  Manager Class
#
class GoogleSpeechRecogManager(openhri.OpenHRI_Manager):
  #
  #  Constructor
  #
  def __init__(self):
    openhri.set_manager_info(__version__, "%prog", __doc__)

    openhri.OpenHRI_Manager.__init__(self, name='google_asr',
                                     conf_name='google_speech.conf')

    self._config = config(config_file=self._opts.configfile)

  #
  #  Initialize node
  #
  def moduleInit(self):
    self._comp['GoogleAsr'] = self.create_component(GoogleSpeechRecog)
    return

#
#
g_manager = None

#
#
def main():
  global g_manager
  g_manager = GoogleSpeechRecogManager()
  g_manager.init_node()
  g_manager.start()
  g_manager.shutdown()


#
#  Main
#
if __name__=='__main__':
  main()
