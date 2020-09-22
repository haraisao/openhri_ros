# openhri_ros
OpenHRI for ROS1

## Quick start
This package is depend on following libraries:
  * Julius dictation-kit
  * python-pydub
  * python-urllib3
  * python-certifi
  * python-lxml
  * ros-malodic-audio-capture
  * ros-malodic-audio-play

### Setup google api_key
If you use google speech api, you need to get your 'api_key' from your Google Cloud Console.
After getting the 'api_key', save it to ~/.openhri/google_apikey.txt

### Setup Julius
1. Download dictation-kit from the official site.(https://osdn.net/di/julius/dictation-kit-4.5.zip)
2. Unzip the archive on '/usr/local/julius/'


### Run 
 * roslaunch openhri julius.launch
 * roslaunch openhri google_asr.launch
 * roslaunch openhri google_tts.launch
