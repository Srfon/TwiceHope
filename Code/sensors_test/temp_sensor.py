import time
import board
import adafruit_dht

# GPIO4
dht = adafruit_dht.DHT11(board.D4, use_pulseio=False)

print("DHT11 start")

try:
    while True:
        try:
            temperature = dht.temperature
            humidity = dht.humidity

            print(
                f"Temperature : {temperature:.1f} °C | "
                f"Humidity : {humidity:.1f} %"
            )

        except RuntimeError as e:
            # DHT11は読み取りエラーが発生することがある
            print(f"Read error: {e}")

        time.sleep(2)

except KeyboardInterrupt:
    print("\nStop")

finally:
    dht.exit()