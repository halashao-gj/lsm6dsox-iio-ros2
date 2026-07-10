DTC ?= dtc
DTS := rk3576-lubancat-lsm6dsox-i2c7-m1-overlay.dts
DTBO := $(DTS:.dts=.dtbo)

.PHONY: all clean

all: $(DTBO)

$(DTBO): $(DTS)
	$(DTC) -@ -I dts -O dtb -o $@ $<

clean:
	rm -f $(DTBO)
