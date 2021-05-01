# Example for ADS1115

The datasheet can be found [here](https://www.ti.com/product/ADS1115).

## Connections

Make the connections as below:

| S. No. | ADS 1115 | ESP 32            |
|--------|----------|-------------------|
| 1.     | V_DD     | V_in or 5V source |
| 2.     | GND      | GND               |
| 3.     | SCL      | D16               |
| 4.     | SDA      | D17               |
| 5.     | A0-A3    | analog input      |

This example specifically demonstrates the simultaneous use of multiple
devices, which is why the default `DEV_COUNT` is 2. If you are using only one
IC then please change the value of `DEV_COUNT` to 1.
