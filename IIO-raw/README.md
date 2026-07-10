# LSM6DSOX IIO buffer reader

This small userspace program reads and decodes the fixed 24-byte scan produced
when all six LSM6DSOX axes and the IIO timestamp are enabled.

## Build in WSL

```sh
make
file iio_buffer_reader
```

## Run on the board

Enable all scan elements before starting the reader:

```sh
IIO=/sys/bus/iio/devices/iio:device1

echo 0 | sudo tee "$IIO/buffer/enable"
for element in "$IIO"/scan_elements/*_en; do
    echo 1 | sudo tee "$element"
done
echo lsm6dsox-dev1 | sudo tee "$IIO/trigger/current_trigger"
echo 256 | sudo tee "$IIO/buffer/length"
echo 1 | sudo tee "$IIO/buffer/enable"

sudo ./iio_buffer_reader /dev/iio:device1 20

echo 0 | sudo tee "$IIO/buffer/enable"
```

The frame count defaults to 20. Pass `0` to read continuously until Ctrl+C.
