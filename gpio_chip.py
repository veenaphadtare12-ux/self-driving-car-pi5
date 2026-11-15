"""
Open the GPIO chip and claim output pins, trying config.GPIO_CHIP_CANDIDATES in order
(gpiochip0 on current Pi 5 kernels, gpiochip4 on older ones).

    lgpio, h, chip = open_and_claim({27: 0, 22: 0}, "motors")   # {pin: initial level}
"""
import config


def open_and_claim(outputs, label):
    import lgpio
    errors = []
    for chip in config.GPIO_CHIP_CANDIDATES:
        try:
            h = lgpio.gpiochip_open(chip)
        except Exception as e:
            errors.append(f"gpiochip{chip}: open failed ({e})")
            continue
        claimed = []
        try:
            for pin, level in outputs.items():
                lgpio.gpio_claim_output(h, pin, level)
                claimed.append(pin)
            print(f"[{label}] using gpiochip{chip}")
            return lgpio, h, chip
        except Exception as e:
            errors.append(f"gpiochip{chip}: claim GPIO{pin} failed ({e})")
            for p in claimed:
                try:
                    lgpio.gpio_free(h, p)
                except Exception:
                    pass
            try:
                lgpio.gpiochip_close(h)
            except Exception:
                pass
    raise RuntimeError(f"[{label}] no usable GPIO chip: " + "; ".join(errors))
