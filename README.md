# STM32 Projekte
- Einzelprojekte aus dem Studium (1.- 4. Semester)
- Programmierung des STM32-L42KC Micocontroller
- in C und Assembly
- HAL
- Registerprogrammierung

## [Morsator](Morsator) Kurzbeschreibung:
- Über die UART Schnittstelle empfangene Daten werden in Morsecode umwandelt und mit einer LED anzeigt.
- Benutzung der HAL.
- UART.

## [Ultraschall](Ultraschall) Kurzbeschreibung:
- Entfernungsmessung mit Hilfe des HC-SR04 Ultraschallmoduls.
- Benutzung der HAL.
- Timer, I2C.

## [accelerometer_gyroscope](accelerometer_gyroscope) Kurzbeschreibung:
- Auslesen von Accelerometer und Gyroskopdaten aus der MPU6050.
- Benutzung der HAL.
- I2C, Timer.

## [blinkyprime](blinkyprime) Kurzbeschreibung:
- Toggeln einer LED durch systick-interrupts.
- OHNE HAL,  ausschließlich Registerprogrammierung.

## [charger](charger) Kurzbeschreibung:
- Aufzeichenen der Lade- und Endladekurve eins Kondensators mit Hilfe des Microcontrollers.
- Senden der Daten (Ladungs- und Endladungskurve) über UART an den PC.
- Benutzung der HAL.
- UART, ADC, Timer.

## [jukebox](jukebox) Kurzbeschreibung:
- Abspielen von mehreren Melodien auf einem Buzzer.
- Durch Knopfdruck kann Melodie gewechselt werden.
- Durch Knopfdruck kann Modus gewechselt werden, indem LEDs zum Rythmus der Melodie leuchten.
- Benutzung der HAL.
- Timer (PWM).

## [Microphone](microphone) Kurzbescheibung:
- Erfassen von Geräuschen über ein Mikrofon (MAX9814).
- Verarbeitung der Daten zur Erkennung von DTMF Tönen (durch Fourier Transformation).
- Weitergabe der Daten an den PC.
- DTMF Erkennung befindet sich in [dtmf_detection.c](https://github.com/NMIK69/STM32-Projekte/blob/main/microphone/Core/Src/dtmf_detection.c).
- Timer, ADC, DMA, UART, I2C.

## [sos_blinky](sos_blinky) Kurzbeschreibung:
- Primitives blinken einer LED (SOS Morsecode).
- OHNE HAL,  ausschließlich Registerprogrammierung in Assembly.

## [sos_blinky_modus](sos_blinky_modus) Kurzbeschreibung:
- sos_blinky weiterentwickelt.
- systick-interrupts zum zeitgenauen Steuern der LEDs genutzt.
- Hinzufügung von Taster, mit dem das Senden des SOS-Morsecodes über die LED aktiviert und deaktiviert werden kann.
- OHNE HAL,  ausschließlich Registerprogrammierung in Assembly.

## [stopwatch](stopwatch) Kurzbeschreibung:
- Zeitmessung durch das Betätigen eines Tasters.
- Zeitmessung wird dann an PC gesendet.
- OHNE HAL.
- UART.

## [trafficlights2](trafficlights2) Kurzbeschreibung:
- Simulation eines Ampelautomaten.
- Benutzung der HAL.
