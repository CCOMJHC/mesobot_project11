LCM sensor dæmon for Sonardyne AvTrak (with SMS acoustic communication)
=======================================================================

Supported downlink commands:
----------------------------

With the move to `mx0e` we no longer have a separate set of commands for the
acoustic downlink. Instead, you have access to all the directives that the
mission executive supports. See [the `mx0e` README][mx0e-readme] for details.

The directives used most often in acoustic downlinks are listed here for
convenience[^1]:

- `EJECT` or `DROP WEIGHT`

   You would typically follow this by inserting `ascent.mx`[^2] to put the vehicle into a consistent configuration.

   - `EJECT; INSERT ascent.mx`

- `DEPTH_OFFSET <delta, meters, int>`

  It might also accept a float, I need to check.

- `DRIVE <duration, seconds, int> <throttle, [-1.0, 1.0]> <heading, radians, float> <depth, meters, float>`

  Currently implemented by pushing a sequence of `GOAL1`, `JOY1`, and `WAIT` directives back into the deque.

  The first two parameters are required, third[^3] & fourth are optional, and all must be in order[^4].

  - `DRIVE 60 0.4 1.57 100`

- `GOAL1 <DOF, [0-5]> <setpoint, float>`

  - `GOAL1 2 200`
  
  - `GOAL1 5 3.14159`

Install
-------

```shell
lcm-gen --python --ppath src/avtrak/lcmtypes src/avtrak/lcmtypes/*.lcm
python3 -m pip install .
```

Debug
-----

To more easily view full messages from LCM logs prior to NA155 2023-09-30:

- set up two virtual serial ports using `socat`:

  ```
  socat -v PTY,link=/tmp/ttyA5,raw,echo=0,b9600 PTY,link=/tmp/ttyV0,raw,echo=0,b9600
  ```

- attach a `simple-serial-bridge` to the input virtual serial port:

  ```
  simple-serial-lcm-bridge -v -b9600 /tmp/ttyA5
  ```

- attach `picocom` to the output virtual serial port:

  ```
  picocom --omap crcrlf -b115200 /tmp/ttyV0
  ```

- replay the log:

  ```
  lcm-logplayer-gui ~/data/mesobot/dive/mesobot062.lcmlog
  ```
_____________

[^1]: This list and these examples may not be completely up-to-date, and
[the `mx0e` README][mx0e-readme] should be considered the primary reference.
[^2]: For some reason I keep wanting to use `ascend.mx`, so we should consider changing the name or symlinking the two.
[^3]: It's already on our list to change the heading parameter units to degrees for usability.
[^4]: We may change this if/when we have time for more sophisticated parsing.
_____________
[mx0e-readme]: https://github.com/whoidsl-mesobot/mx0e#mx-script-directives
