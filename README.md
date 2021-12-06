
# Build - XMC1100 xmc2go

```
pio run --target xmc1100_xmc2go
pio run -e xmc1100_xmc2go -t upload
```


# Original Setup

```
wget https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
echo 'PATH=$PATH:~/.platformio/penv/bin' >> ~/.profile

<login again>

pio project init --board xmc1100_xmc2go --board esp01
git init; git add -A; git commit -m init
```
