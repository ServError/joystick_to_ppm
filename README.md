Multiple (or Single) Joystick to PPM Adapter using Raspberry PI and Arduino
=========

## General
This project is a fork of https://github.com/dmpriso/joystick_to_ppm. Its goal is to add support for multiple USB joysticks as well as perform general cleanup and bugfixing. Please see that repo for the old readme.

## Hardware
All you need is a Raspberry Pi (this was tested on an old zero), a Linux compatible Joystick and an Arduino board.
Plug the Joystick into Raspberry. Connect the Raspberry to the Arduino via USB.

The contents of the example directory can be overlaid on your root to place files where they are expected. Check for appropriate read permissions, as well as execute permissions on the .sh script.

Udev rules are provided in the example folder to show how to create a symlink for your joysticks.

To obtain the model and manufacturer needed in the joystick rule file, run
```
udevadm info -q all -n /dev/input/js0
```
and repeat for each of your joysticks. Edit the symlink name to something appropriate, then modify the corresponding links in main.cpp.

Adding additional or fewer joysticks is simply a matter of copying the file descriptor code of one of the existing joysticks, and the read loop within the main loop.

An example is also included for turning off the LEDs on the Thrustmaster Warthog Throttle. This includes on rule file and the shell script it calls.


The Arduino will output an 8-Channel 22.5ms PPM signal on Pin 10.
Arduino code is based on previous work: https://github.com/ckalpha/Generate-PPM-Signal/

## Configuration
At the moment, configuration is done via a settings files.
Path of the settings files is /usr/local/etc/joystick/{joystick name}_mappings.ini

You can map axis and buttons to output channels, and define special actions for buttons.
Configuration is an INI file. Ini parsing is based on previous work: https://github.com/atomicptr/libini

### How do I find axis and button IDs?
Install jstest on your raspberry and run
```
jstest /dev/input/js{joystick number/symlink}
```

*I strongly recommend you to perform a joystick calibration and save the joystick configuration on your system. See the following article for instructions:*
http://aegidian.org/bb/viewtopic.php?t=12217#p178651

### Axis mapping
An axis mapping entry consists of the following elements:

```
[axis2]             # "2" is the axis identifier, as seen in jstest
ppm_channel_id=0    # "0" is the PPM channel ID, zero-based
weight=100          # 100% weighting. For reversing, just put a negative value here
expo=30             # 30% expo
offset=10           # 10% offset = subtrim
```

You may target the same PPM channel ID from multiple axes in order to achieve mixing.

### Button action
Button actions are defined as follows:

```
# flightmode: manual
[action_button9]    # "9" is the button identifier, from jstest
ppm_channel_id=4    # target PPM channel ID, zero-based
value=-67           # value to be set when button is pressed. Range from -100 to +100%

# Below is a release button action. One can also use it to init default PPM values.
# This one is intended to set the initial flightmode to "manual"
[action_releasebutton9]
ppm_channel_id=4
value=-67
```

### Trims and subtrims
Buttons can be set for trims and subtrims.
Trim applies to a stick input, so expo and rate are also applied to the trim.
Subtrims are applied to PPM output.

```
[trim_button15]		# Apply trim when button 15 is pressed
axis_id=1			# Target axis ID 
step=1				# Trim step, in percent

[subtrim_button20]	# This is a subtrim when button 20 is pressed
ppm_channel_id=1	# Target PPM channel ID, zero-based
step=1
``` 

Trims are automatically saved and restored. The path of the file generated is:
/usr/local/etc/joystick/user_trims.ini *TODO: make that customizable via parameter*

### Buttonization of axes
Some joysticks do report their coolie hat as axis, not as buttons. 
However the coolie hat might be useful for trimming. 
You can use an axis to trigger button actions as follows:

```
[buttonize_axis7]	# Axis ID of the coolie hat X or Y axis
low_button_id=34	# Virtual button ID to be triggered when axis is low (left)
high_button_id=35	# Virtual button ID to be triggered when axis is high (right)
```

### Sample configuration
I've added a mapping.ini to the source repo which I wrote for the Saitek X-52 Pro Joystick.

## Building and installing

### Building
Install CMake and ninja
Clone the repo
Create a build output directory (outside the repo!)
Inside build dir, run the following:

```
cmake path_to_source_dir -G Ninja
ninja
```

### Testing
Create a /usr/local/etc/joystick/mappings.ini and set it up
Simply run ./joystick_ppm_converter for testing

### Installing
In order to let the tool automatically run when the raspberry starts up, copy the executable tool to /usr/local/bin/
Then, edit /etc/rc.local and add the following line near the end, but before the exit command:
```
/usr/local/bin/joystick_ppm_converter >joystick.log &
```

Restart your raspberry to verify

## TODOs
The project is still at a very early alpha-stage.
Most important items to do are:

* Improve file parsing so invalid numbers etc. are catched and proper error messages are emitted
* Make the INI paths customizable via command parameters
* Create a GUI for editing the configuration
* Integrate the GUI and allow for live updating of configuration


