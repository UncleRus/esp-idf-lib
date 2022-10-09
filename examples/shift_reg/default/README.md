# Shift register out

## What the example does

The example shift out periodicly to 2 registers [75HC595](https://www.ti.com/lit/ds/symlink/sn74hc595.pdf) a 2 bytes long number, which gets increments.

## Configuration

The following configuration was used:

| 74HC595 | GPIO |
|---------|------|
| Serial Clock | `GPIO_NUM_12` |
| Data/Signal | `GPIO_NUM_15` |
| Latch | `GPIO_NUM_13` |

| Mode | enum |
|------|------|
| output | `SHIFT_DIR_OUTPUT` |
| Most Significant Bit (MSB) | `SHIFT_BIT_MODE_MSB` |

```
static shift_reg_config_t shifter = {
		.num_reg = 2,
		.mode = {
			.dir = SHIFT_DIR_OUTPUT,
			.bit_mode = SHIFT_BIT_MODE_MSB,
		},
		.pin = {
			.clk = GPIO_NUM_12,
			.data = GPIO_NUM_13,
			.latch = GPIO_NUM_15,
		},
	};
```

## Notes

For more information, please vist the component directory [shift_reg](/components/shift_reg/).