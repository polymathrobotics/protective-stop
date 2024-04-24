# Polymath Protective Stop
Instructions and source files for our protective stop 🛡️ 🛑
By Kai Ma and Ilia Baranov

This initiative is focused on developing a low-cost, reliable protective stop mechanism for robotics and automation systems. Our protective stop system is designed to be a preventative measure that initiates a controlled shutdown with no immediate danger.

**Safety disclaimer:** This system is not designed to be an emergency stop; as such, it should not be used when there is an immediate threat to human safety or significant risk to equipment is a more drastic action used when there is an immediate threat to human safety or significant risk to equipment. It is not designed immediate, uuncontrolled shutdowns.

The main components of the system are:
- Raspberry Pi and ESP32 microcontroller for primary functionality
- SIM Cell Modem for communication over cellular networks
- Onboard roslibpy software to communicate with robots through rosbridge
- Peripherals - power button, stop button, e-ink paper display

See instructions:
- [Hardware Assembly](/docs/Hardware.md)
- [Software Set-up](/docs/Software.md)
- [3D Printed Case](/docs/Case.md)

<br><br>
<p align="center">
  <img width="70%" src="/docs/img/pstop-ces.png"> <br><i> A prototype of the P-Stop at CES 2024... </i>
</p>

## TODO
- [ ] Handle shutdown gracefully for RPi