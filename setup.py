#!/usr/bin/env python

from distutils.core import setup

setup(
  version='1.0.0',
  packages=['openhri', 'openhri.google_asr', 'openhri.google_ttd', 'openhri.julius', 'openhri.openjtalk'], 
  #scripts=['bin/JuliusRos', 'bin/GoogleTTS', 'bin/GoogleASR'],
  scripts=[],
  package_dir={'': 'src'}
)

