deps:
	sudo port install arm-none-eabi-binutils arm-none-eabi-gcc arm-none-eabi-gdb
	sudo port install openocd +stlink
	rustup target add thumbv7em-none-eabihf
	cargo install itm --vers 0.3.1

deps-clean:
	sudo port uninstall arm-none-eabi-binutils arm-none-eabi-gcc arm-none-eabi-gdb isl libmpc mpfr
	sudo port uninstall openocd libusb

clean:
	find . -name "*.DS_Store" -exec rm '{}' ';'
	find . -name "*~" -exec rm '{}' ';'

distclean:
	cargo clean

openocd:
	openocd -f interface/stlink-v2-1.cfg -f target/stm32f3x.cfg
