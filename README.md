# openhri_ros
OpenHRI for ROS1

## Quick start
This package is depend on following libraries:
  * Julius dictation-kit
  * OpenJtalk ( open-jtalk, open-jtalk-mecab-naist-jdic, hts-voice-nitech-jp-atr503-m001 )
  * SoX(Sound eXchange)
  * python-pydub
  * python-urllib3
  * python-certifi
  * python-lxml
  * python-configparser
  * ros-malodic-audio-capture
  * ros-malodic-audio-play

### Setup google api_key
If you use google speech api, you need to get your 'api_key' from your Google Cloud Console.
After getting the 'api_key', save it to ~/.openhri/google_apikey.txt

### Setup Julius
1. Download dictation-kit from the official site.(https://osdn.net/di/julius/dictation-kit-4.5.zip)
2. Unzip the archive on '/usr/local/julius/'
2. Change the file permission of the 'bin/linux/\*' as '+x'


### Run 
 * roslaunch openhri julius.launch
 * roslaunch openhri google_asr.launch
 * roslaunch openhri google_tts.launch
 * roslaunch openhri openjtalk.launch
 
 ### Information
 for more Japanese infomation, please access to my homepage; http://hara-jp.com/_default/ja/Software/openhri_ros.html
