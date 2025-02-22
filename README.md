# Esp32-DCMotor

A simple library to control a DC motor with a DRV8833 driver and a quadrature encoder using an ESP32.


## Test firmware

The directory `test_fw` contains a test firmware, which can be used as an example on the usage of the library.

The firmware provides a console interface to control the motors as configured in the `main.cpp` file. The console
is accessible via USB-JTAG serial.


## Test tools

The directory `tools` contains Python scripts to test the library and to help with tuning the control parameters.

The script `run_test.py` allows to run a test of the motor control and to log the test progress. The test is
described in a text file, which is passed as an argument to the script. An example of a test file is provided in
the `tools\sample_test.txt` file. Each line of the test file describes a test step, with the following format:

```
<time-point>: <command>
```

where `<time-point>` is the time in seconds since the start of the test, and `<command>` is the command to be sent
to the motor controller. The commands are the same as the ones used in the console interface of the test firmware.
The report from the test is saved in a text file and can be plotted using the `plot_report.py` script.

If the test firmware runs out of memory while running the test, consider adding a command `everynth <n>` with a higher
`<n>` value at the start of the test to only report every nth data point and reduce the memory usage.
