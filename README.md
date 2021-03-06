# 3284 2019 Robot Code Release
Robot Code from the 2019 season, Destination: Deep Space.
Confirmed working with WPILib 2019.4.1, REVRobotics 1.4.1, Phoenix 5.14.1
## Branches
**"master" branch** contains most development and changes. All other branches should be forked from master.
<br>
**"vision" branch** contains our Vision code for running on the Jetson TX1 for 2019.
<br>
# Vision Processing
Current working version of object tracking with the Raspberry Pi 3. Also includes new version equiped
with multi-threading and a web interface for adjusting values on the fly. Built with Python 2.7 using OpenCV 3.3 libraries and [pynetworktables](https://github.com/robotpy/pynetworktables) with [monotonic](https://pypi.org/project/monotonic/#files).
## Other Folders:
###  Values:
  This folder is meant to be placed on the desktop of the Raspberry Pi allowing the program
  to organize and store track-bar values.
###  Overclocking:
  This folder contains files that need to replace other files on the filesystem of the Raspberry Pi.
  Replace the config.txt in the `/boot/` directory and place the other files on the desktop (this is the overclocking settings).
  Make sure to add cpu_governor.sh to autostart in the `/etc/rc.local` file (remove CPU throttling).
    
Also note, if you want the vision program to autostart add the directory and command to start it to the rc.local in `/etc/`.

###  How to Setup the Web Interface
First, a webserver such as Apache2 or NGINX needs to be installed on your linux machine. We use Apache2 as it is relatively simple and fast to setup. To install Apache open a terminal and type: `sudo apt install apache2`. After the installation completes, drop the contents of `web_interface` into `/var/www/html` and reboot. 
  
To visit the website type the hostname of your computer or it's IP address. For example, if your using a Raspberry Pi, you could use `raspberrypi.local`.
  
![alt text](https://github.com/LASER3284/2019-Robot-Code/blob/vision/web_interface/Capture.PNG)
###### Vision Tuning Dashboard

### Running Vision Tracking:
To run the program, make sure to `chmod a+x` the entire directory containing the vision files. This allows the program to access the files storing the track-bar values. If you need help with the arguments to pass in just type: `sudo python VisionMain.py -h` (output is below.)
    
```console
usage: VisionMain.py [-h] [--source SOURCE] [--thread THREAD] [--gui GUI]

optional arguments:
    -h, --help            show this help message and exit
    --source SOURCE, -s SOURCE
                            Path to video file or integer representing webcam
                            index (default 0).
    --thread THREAD, -t THREAD
                            Threading mode: all (video read and video show in
                            their own threads), none (default--no multithreading)
    --gui GUI, -g GUI     Graphical User Interface: yes (show video windows for 
                            vision tuning), no (default, run supa fast)
```
