## Laser related plugins

### Laser PPI

Under development. Adds 3 M-codes for controlling PPI (Pulse Per Inch) mode for lasers.

* `M126 P-` turns PPI mode on or off. The P-word specifies the mode. `0` = off, `1` = on.
* `M127 P-` The P-word specifies the PPI value. Default value on startup is `600`.
* `M128 P-` The P-word specifies the pulse length in microseconds. Default value on startup is `1500`.

> [!NOTE]
> These M-codes are not standard and may change in a later release. 

A description of what PPI is and how it works can be found [here](https://www.buildlog.net/blog/2011/12/getting-more-power-and-cutting-accuracy-out-of-your-home-built-laser-system/).

I have a customized version of [ioSender](https://github.com/terjeio/ioSender) that I use for my CO2-laser, this has not yet been published but I may do so if this is of interest.

Dependencies:

Driver must support pulsing spindle on pin. Only for processors having a FPU that can be used in an interrupt context.

### Laser coolant

Under development. Adds monitoring for \(tube\) coolant controlled by `M8`, configurable by settings.

* `$378` - time in seconds after coolant is turned on before an alarm is raised if the coolant ok signal is not asserted.
* `$379` - time in minutes after program end before coolant is turned off. \(WIP\)
* `$380` - min coolant temperature allowed. \(WIP\)
* `$381` - max coolant temperature allowed. \(WIP\)
* `$382` - input value offset for temperature calculation. \(WIP\)
* `$383` - input value gain factor for temperature calculation. \(WIP\)

WIP - Work In Progress.

Dependencies:

Driver must have at least one [ioports port](../../templates/ioports.c) input available for the coolant ok signal.
An optional analog input port is required for coolant temperature monitoring.

### LightBurn clusters

The plugin unpacks the clustered S command from the input stream and deliveres standard gcode to the parser.

### CO2 laser overdrive, for dithered engravings

Adds a M-code for controlling the overdrive.

* `M129 P-` add overdrive. The P-word specifies the percentage of power to add to the programmed S-value. Use `0` to turn overdrive off.

The M-code is only available in laser mode when the _RPM controls spindle enable signal_ option is selected in settings `$9` or `$709`.  
It works by increasing the PWM output when the laser is turned off and resetting it to the programmed value when turning it on.
This delivers an initial "kick" to the laser tube when turning it on and that may result in [crisper images](https://github.com/grblHAL/core/issues/721#issuecomment-2776210888) when rendering dithered images \(Stucki, Jarvis ...\).

> [!NOTE]
> If M4 mode is used add overscan to the G-code generated so that the programmed power output is reached before the actual engraving starts.

> [!NOTE]
> Overdrive works by assuming the laser power supply has a finite response time to PWM changes, due to this the effect of a certain amount of overdrive will likely vary among different makes.

> [!NOTE]
> Ensure the PWM frequency set by `$33` outputs at least four PWM pulses during the shortest pulse sent to the laser.

---
2025-04-06
