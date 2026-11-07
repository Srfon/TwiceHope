import spidev
spi = spidev.SpiDev()
spi.open(0,0)
spi.xfer2([0xAA])