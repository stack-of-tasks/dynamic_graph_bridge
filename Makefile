include $(shell rospack find mk)/cmake.mk

# Forward install target to build directory.
install:
	${MAKE} -C build install

.PHONY: install
