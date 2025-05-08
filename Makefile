SHELL := /bin/bash
.PHONY: all clean init build

clean:
	rm -rf build	

build:
	cd build;cmake --build .

init: 
	git submodule update --init
	mkdir -p build
	cd build;cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake -DBUILD_TESTS=OFF ..

all: init build

