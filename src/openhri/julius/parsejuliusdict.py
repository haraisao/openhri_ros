#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''Julius dict file parser

Copyright (C) 2010
    Yosuke Matsusaka
    Intelligent Systems Research Institute,
    National Institute of Advanced Industrial Science and Technology (AIST),
    Japan

Copyright (C) 2017
    Isao Hara
    National Institute of Advanced Industrial Science and Technology (AIST),
    Japan

    All rights reserved.
Licensed under the Eclipse Public License -v 1.0 (EPL)
http://www.opensource.org/licenses/eclipse-1.0.txt
'''

#from string import maketrans
import sys
import re

PYTHON_MAJOR_VERSION = sys.version_info.major
if PYTHON_MAJOR_VERSION == 2:
  from io import open

#
#  Parse Julius Pronunciation Dictionaly in dictationkit.
#
class JuliusDict:
  """ Utility class to parse Julius Pronunciation Dictionaly."""

  def __init__(self, fname, new_version=1):
    self._fname = fname
    self._dict = {}
    print(self._fname)
    self.parse(self._fname)

  #
  #  parse dict file 
  def parse(self, fname):
    f = open(fname, 'r', encoding='utf-8')
    kcode='euc-jp'
    for l in f:
      matchObj = re.search(r'\[.+\]', l)
      if matchObj :
        t = conv_encoding(matchObj.group())
        t = t[1:-1]
        #print (t)
        ph = conv_encoding(l).rsplit(']')
        if len(ph) > 1:
          st = [ ' '+ph[1].strip() ]
          try:
            self._dict[t].extend(st)
          except KeyError:
            self._dict[t] = st
    return
  #
  #  lookup 
  def lookup(self, w):
    try:
      return list(set(self._dict[w]))
    except KeyError:
      return []
#
#  Convert utf-8 or euc-jp to Unicode
#
def conv_encoding(data):
  lookup = ('utf_8', 'euc_jp')
  encode = None
  for encoding in lookup:
    try:
      data = data.decode(encoding)
      break
    except:
      pass
  #if isinstance(data, str):
  return data
  #else:
  #    raise LookupError

#
#
if __name__ == '__main__':
  dic1 = "D:\\local\\Julius\\dictation-kit-v4.4\\model\\lang_m\\web.60k.htkdic"
  dic2 = "D:\\local\\Julius\\dictation-kit-v4.4\\model\\lang_m\\bccwj.60k.htkdic"
  dic3 = "/usr/share/julius-runkit/model/lang_m/web.60k.htkdic"

  print (dic1)

  doc = JuliusDict(dic2)

  print (doc.lookup(u'叔父'))
  print (doc.lookup(u'こんにちは'))
    
