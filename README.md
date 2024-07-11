# Sound localization using 3 wireless microphones (ESP32 + INMP441)
## About
This project's aim is to calculate localized coordinates of a sound source. The project utilizes:
* 3x YD-ESP32-S3 (a ESP32-S3-DevKitC-1 clone)
* 3x INMP441 digital microphones (also a clone, as InvenSense no longer manufactures this module)
* Node.js for the websocket server, to which the ESP32s send audio data, which is then sent to the web client
* WebAssembly for grabbing a specific sound (currently, the loudest sound in a specific time frame), getting timestamp values of when each ESP32 "heard" the sound, then calculating the location of the sound source.

![High level diagram of the project's system](https://i.imgur.com/UaeHKsO.png)

The time difference of arrival (TDOA) is needed to do further calculations in this project's case. The solution used here is using WLS (Weighted Least Squares), but [many other more advanced techniques](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6603714/#B29-sensors-19-02554) are used.

To start getting results of a sound source, all 3 ESP32s need to be connected to the websocket server and the distances between the ESP32s need to be entered. The ESP32s will be shown visually along with the calculated coordinates of the sound:
![Screenshot of the web UI of the location results](https://i.imgur.com/zFg6ENP.png)
## TODO
This project is at the 80% done stage. It works, however a lot of polish and tweaking is needed:
* Time synchronization between the ESP32s needs to be improved (or removed altogether, by setting a predetermined offset for sending packets). Current time sync (checked manually through Audacity by clapping and looking at the waveforms):
  ![Table of time synchronization test results between the ESP32s](https://i.imgur.com/KjXVmfx.png)
  The automatic time synchronization test function provides much worse results, where the time differences can range up to 30ms between the microcontrollers, which doesn't bode well for the actual calculation of the position of the sound source.
* Check if the WebAssembly module part for the WLS method actually calculates the location properly. In my limited manual testing, the results seemed valid.
* Find a better solution for/than finding the highest amplitude in a specific time frame for the TDOA calculations (as noted before, this automatic approach provides seemingly worse results on average than just recording the audio data from the ESP32s and looking at the waveforms to determine how synchronized the ESP32s are). This can be due to the microphones producing different audio data, as such the peaks can be in the "wrong place":
  ![Screenshot from Audacity of 3 audio waves from 3 ESP32s, where the third audio wave's peak is unexpectedly in a different place](https://i.imgur.com/fJqPggy.png)
  In the screenshot above, the audio peak for the third ESP32 is seemingly 0.1ms behind of where is should be (comparing to the other 2 ESP32 audio waves). However, the error here is only 0.1ms, so it shouldn't be too big of a problem, but it is very possible that larger errors occur. This may be a result of the breadbord wires being very flimsy.
* Code cleanup for the main JavaScript file is needed
* Code cleanup for the ESP32 client code is needed
* Polish needed for the web UI
* Need to add option to set the temperature through the web UI or grab it from the ESP32s (however I suspect the built-in temperature modules wouldn't be too accurate).
* Need to add an option to change the audio sample time frame size for the sound localization from the web UI (how much time is given to listen for the "loudest" sound; this also affects how quickly new location results are shown in the web UI).
