# nCoreTemp

![An example of the program running](Images/ncoretemp_example.gif)

This is a full reworking of the [Coretemp Remote project](https://www.omnimaga.org/ti-nspire-projects/(ndless)-coretemp-remote/), which [originally used](https://github.com/compujuckel/nsocket/tree/master/ns_client/demo/coretemp) the NavNet API. It is now done using Direct Memory Access calls to the USB controller.

The program is split into the device program on the Nspire (`nCoreTemp.tns`), and the host server program (`computer_server.py`).

## Prerequisites

For the computer_server, make sure you have:

```sh
pip install pyusb psutil
```

For macOS, make sure you have

```sh
brew install macmon
```

for temperature.