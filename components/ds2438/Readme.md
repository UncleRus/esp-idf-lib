# Driver for Dallas DS2438

## DESCRIPTION
The DS2438 Smart Battery Monitor provides several functions that are desirable to carry in a battery
pack: a means of tagging a battery pack with a unique serial number, a direct-to-digital temperature
sensor which eliminates the need for thermistors in the battery pack, an A/D converter which measures
the battery voltage and current, an integrated current accumulator which keeps a running total of all
current going into and out of the battery, an elapsed time meter, and 40 bytes of nonvolatile EEPROM
memory for storage of important parameters such as battery chemistry, battery capacity, charging
methodology and assembly date. Information is sent to/from the DS2438 over a 1-Wire interface, so that
only one wire (and ground) needs to be connected from a central microprocessor to a DS2438. This
means that battery packs need only have three output connectors: battery power, ground, and the 1-Wire
interface.
Because each DS2438 contains a unique silicon serial number, multiple DS2438s can exist on the same
1-Wire bus. This allows multiple battery packs to be charged or used in the system simultaneously.
Applications for the smart battery monitor include portable computers, cellular telephones, and handheld
instrumentation battery packs in which it is critical to monitor real-time battery performance. Used in
conjunction with a microcontroller in the host system, the DS2438 provides a complete smart battery
pack solution that is fully chemistry-independent. The customization for a particular battery chemistry
and capacity is realized in the code programmed into the microcontroller and DS2438 EEPROM, and
only a software revision is necessary should a designer wish to change battery pack chemistry.

* Build on ESP-IDF v4.4.1.
* Not test with real devices.
* check **docs** for datasheet

# Driver Features

* Read Temperature and Voltages.

## Dallas DS2438 Features

* Unique 1-Wire® interface requires only one
port pin for communication
* Provides unique 64-bit serial number
* Eliminates thermistors by sensing battery
temperature on-chip
* On-board A/D converter allows monitoring
of battery voltage for end-of-charge and endof-discharge determination
* On-board integrated current accumulator
facilitates fuel gauging
* Elapsed time meter in binary format
* 40-byte nonvolatile user memory available
for storage of battery-specific data
* Reverts to low-power sleep mode on battery
pack disconnect (feature disabled on
DS2438AZ)
* Operating range -40ºC to +85ºC
* Applications include portable computers,
portable/cellular phones, consumer
electronics, and handheld instrumentation

## Examples

* example are in **examples/ds2438/main**
  
### MIT LICENSE