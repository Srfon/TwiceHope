#!/usr/bin/env python3

import time
from smbus2 import SMBus

I2C_BUS = 1
I2C_ADDR = 0x6B

FIFO_STATUS1 = 0x1B
FIFO_STATUS2 = 0x1C
FIFO_DATA_TAG = 0x78


def read_fifo_level(bus):
    s = bus.read_i2c_block_data(
        I2C_ADDR,
        FIFO_STATUS1,
        2
    )

    return s[0] | ((s[1] & 0x0F) << 8)


def read_fifo_raw(bus):
    """
    FIFOから取得した7 byteをそのまま返す

    [TAG][X_L][X_H][Y_L][Y_H][Z_L][Z_H]
    """

    return bus.read_i2c_block_data(
        I2C_ADDR,
        FIFO_DATA_TAG,
        7
    )


def main():

    print(
        f"I2C bus={I2C_BUS}, "
        f"address=0x{I2C_ADDR:02X}"
    )

    with SMBus(I2C_BUS) as bus:

        while True:

            try:

                count = read_fifo_level(bus)

                if count > 0:

                    for _ in range(count):

                        data = read_fifo_raw(bus)

                        print(
                            "RAW:",
                            " ".join(
                                f"{x:02X}"
                                for x in data
                            )
                        )

                else:

                    time.sleep(0.001)

            except OSError as e:

                print(
                    f"[ERROR] I2C: {e}"
                )

                time.sleep(0.01)

            except KeyboardInterrupt:

                print("\nStopped.")

                break


if __name__ == "__main__":
    main()