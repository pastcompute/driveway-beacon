# Build - Teensy LC

```
pio run -e tlc~driveway -t upload
```

# Original Setup

```
wget https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
echo 'PATH=$PATH:~/.platformio/penv/bin' >> ~/.profile

<login again>
```

Dependencies:

```

pio lib install "aparcar/CayenneLPP@^1.3.0" "bblanchon/ArduinoJson@^6.19.1"

cd lib
git clone https://github.com/pastcompute/arduino-MLX90393
git clone https://github.com/pastcompute/sx1276-arduino
```

