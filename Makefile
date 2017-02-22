TARGET:=target/thumbv7em-none-eabi/debug/tivarust
XARGO:= ~/.cargo/bin/xargo

all: $(TARGET)

target/thumbv7em-none-eabi/debug/tivarust:
	 $(XARGO) build --target=thumbv7em-none-eabi
load: $(TARGET)
	openocd -f openocd.conf -c "program  $(TARGET) verify reset" -c "shutdown"
