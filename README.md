To setup VRPN for ArtiSynth:
- include vrpn.jar to libraries of ArtiSynth.
- add ``-Djava.library.path="path/to/artisynth-tracking/artisynth_jaw_tracking/lib`` to VM arguments of the run configurations.

In this approach, ArtiSynth will be the VRPN client which receives the data from a specified VRPN server.


The `./certus_tracker` implements a VRPN server (and a VRPN client for test purposes) to transfer the data from the
NDI Optotrak Certus to ArtiSynth. 
The `./lib` directory contains the VRPN library for C++. 
You only need to use it if your client is implemented in C (e.g. using Optotrak Certus).



