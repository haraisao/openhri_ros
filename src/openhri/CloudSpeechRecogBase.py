#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''Cloud Speech Recognition Base Class

Copyright (C) 2017
  Isao Hara
  National Institute of Advanced Industrial Science and Technology (AIST),
  Japan
  All rights reserved.

Licensed under the Eclipse Public License -v 1.0 (EPL)
http://www.opensource.org/licenses/eclipse-1.0.txt
'''

import sys, os, threading, platform
import time, traceback, wave

from .core import *

from pydub import AudioSegment
from pydub.silence import *

#
#  
#
class CloudSpeechRecogBase(threading.Thread, ros_object):
  #
  #  Constructor
  #
  def __init__(self, language='ja-JP'):
    threading.Thread.__init__(self)
    self._platform = platform.system()
    self._callbacks = []

    self._buffer = b''
    self._audio = b''
    self.audio_segments = []

    self._sample_width=2
    self._frame_rate=16000
    self._channels=1

    self._min_silence=200
    self._silence_thr=-10
    self._min_buflen=8000

    self._lang=language
    self._apikey = ''
    self._logdir = 'log'
    self._logger = False

    self._running = True
    self._lock = threading.RLock()
    self._prebuf=[]

  #
  #   Write to audio data
  #
  def write(self, data):
    try:
      self._buffer=b''.join([self._buffer, data])

      if len(self._buffer) > self._min_buflen:
        audio = AudioSegment(self._buffer, sample_width=self._sample_width,
                           channels=self._channels, frame_rate=self._frame_rate)
        chunks = detect_nonsilent(audio, min_silence_len=self._min_silence,
                           silence_thresh=self._silence_thr)

        if chunks :
          if not self._audio :
            self._audio = b''.join([self._audio, self._prebuf])
          self._audio = b''.join([self._audio, self._buffer])
        else:
          self._prebuf = self._buffer
          if self._audio :
            self._audio = b''.join([self._audio, self._buffer])
            self._lock.acquire()
            self.audio_segments.append(self._audio)
            self._lock.release()

            if self._logger :
              self.save_to_wav(self.get_logfile_name(), self._audio)

            self._audio=b''
          else:
            pass
        self._buffer = b''
      else:
        pass

    except:
      print (traceback.format_exc())
      pass

    return 0

  #
  #  Set Lang
  #  (ja-JP, en-US, en-GB, en-AU, de-DE, ....)
  #
  def set_lang(self, lang):
    self._lang = lang

  #
  #
  #
  def set_voice_detect_param(self, mval, thr, buflen):
    self._min_silence = mval
    self._silence_thr=thr
    self._min_buflen=buflen

  #
  #  Set callback function
  #
  def setcallback(self, func):
    self._callbacks.append(func)

  #
  #
  #
  def get_logfile_name(self):
    return os.path.join(self._logdir, time.strftime('voice%Y%m%d_%H%M%S.wav'))

  #
  #  Save to Wav file
  #
  def save_to_wav(self, name, data):
    wave_data = wave.open(name, 'wb')
    wave_data.setnchannels(self._channels)
    wave_data.setsampwidth(self._sample_width)
    wave_data.setframerate(self._frame_rate)
    
    wave_data.setnframes(int(len(data) / (self._sample_width * self._channels)))
    wave_data.writeframesraw(bytearray(data))
    wave_data.close()

  #
  #  Request Google Voice Recognition
  #
  def request_speech_recog(self, data):
    return 0

  #
  #  Terminate (Call on Finished)
  #
  def terminate(self):
    print ('CloudSpeech: terminate')
    self._running = False
    return 0

  #
  #  Run
  #
  def run(self):
    while self._running:
      audio =[]
      self._lock.acquire()
      if len(self.audio_segments) :
        audio = self.audio_segments.pop(0)
      self._lock.release()

      if audio :
        print("request...")
        res = self.request_speech_recog(audio)
        if res :
          for c in self._callbacks:
            c(res)
        else:
          print ("----")
          pass

    print ('CloudSpeech: exit from event loop')

