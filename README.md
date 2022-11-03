# TorqueTuner: A self contained module for designing rotaryhaptic force feedback for digital musical instruments.

TorqueTuner  is  an  embedded  module  that  allows  Digital Musical Instrument (DMI) designers to map sensors to parameters  of  haptic  effects  and  dynamically  modify  rotary force  feedback  in  real-time. It comes with an embedded collection of haptic effects, and a is wireless bi-directional interface through [libmapper](https://github.com/libmapper/libmapper).

## Demonstration Video

[![IMAGE ALT TEXT](https://img.youtube.com/vi/KY3mpczKI3k/0.jpg)](https://www.youtube.com/watch?v=KY3mpczKI3k)

The hardware is based on the ESP32 microcontroller and the [moteus](https://mjbots.com/) platform, to implement a force feedback rotary encoder with 3600 PPR ~= 0.1 degree resolution that can display forces up to 45 Ncm (63.7oz.in).
## Documentation:
[Build Guide](./Docs/Build_guide_v1.md)
[Torquetuner Connection Guide](./Docs/connection_guide_v1.md)
[Torquetuner Firmware Update Guide](./Docs/Firmware_update_instructions.md)
[Torquetuner OSC Namespace](./Docs/TorquetunerOSC.md)

