BUILD_TYPE = Debug 

ifneq ($(shell test -d .git), 0)
GIT_SHORT_HASH:= $(shell git rev-parse --short HEAD)
endif

VERSION_MAJOR = 1
VERSION_MINOR = 3
VERSION_PATCH = 1

VERSION = $(VERSION_MAJOR).$(VERSION_MINOR).$(VERSION_PATCH)

COMMON_DEFINITIONS =                                      \
	-DCMAKE_BUILD_TYPE=$(BUILD_TYPE)                      \
	-DVERSION=$(VERSION)                                  \
	-DGIT_SHORT_HASH=$(GIT_SHORT_HASH)     				

generic:
	rm -R -f build_generic
	mkdir build_generic
	cd build_generic && cmake -G"Unix Makefiles"          \
	$(COMMON_DEFINITIONS)                                 \
	-DCMAKE_TOOLCHAIN_FILE=../toolchain/generic.cmake ..

cortex-m0:
	rm -R -f build_cortex-m0
	mkdir build_cortex-m0
	cd build_cortex-m0 && cmake -G"Unix Makefiles"       \
	$(COMMON_DEFINITIONS)                                \
	-DCMAKE_TOOLCHAIN_FILE=../toolchain/cortex-m0.cmake ..

cortex-m3:
	rm -R -f build_cortex-m3
	mkdir build_cortex-m3
	cd build_cortex-m3 && cmake -G"Unix Makefiles"       \
	$(COMMON_DEFINITIONS)                                \
	-DCMAKE_TOOLCHAIN_FILE=../toolchain/cortex-m3.cmake ..

cortex-m4:
	rm -R -f build_cortex-m4
	mkdir build_cortex-m4
	cd build_cortex-m4 && cmake -G"Unix Makefiles"       \
	$(COMMON_DEFINITIONS)                                \
	-DCMAKE_TOOLCHAIN_FILE=../toolchain/cortex-m4.cmake ..

cortex-m4f:
	rm -R -f build_cortex-m4f
	mkdir build_cortex-m4f
	cd build_cortex-m4f && cmake -G"Unix Makefiles"       \
	$(COMMON_DEFINITIONS)                                \
	-DCMAKE_TOOLCHAIN_FILE=../toolchain/cortex-m4f.cmake ..

cortex-m7:
	rm -R -f build_cortex-m7
	make build_cortex-m7
	cd build_cortex-m7 && cmake -G"Unix Makefiles"       \
	$(COMMON_DEFINITIONS)                                \
	-DCMAKE_TOOLCHAIN_FILE=../toolchain/cortex-m7.cmake ..

xtensa-lx106:
	rm -R -f build_xtensa-lx106
	mkdir build_xtensa-lx106
	cd build_xtensa-lx106 && cmake -G"Unix Makefiles"      	 \
	$(COMMON_DEFINITIONS)                                \
	-DCMAKE_TOOLCHAIN_FILE=../toolchain/xtensa-lx106.cmake ..

cortex-a73:
	rm -R -f build_cortex-a73
	mkdir build_cortex-a73
	cd build_cortex-a73 && cmake -G"Unix Makefiles"      	 \
	$(COMMON_DEFINITIONS)                                \
	-DCMAKE_TOOLCHAIN_FILE=../toolchain/cortex-a73.cmake ..

xcode:
	rm -R -f xcode
	mkdir xcode 
	cd xcode && cmake -G"Xcode"       \
	$(COMMON_DEFINITIONS) ..                               

eclipse:
	rm -R -f build 
	mkdir build 
	cd build && cmake -G"Eclipse CDT4 - Unix Makefiles"       \
	$(COMMON_DEFINITIONS) ..

visual:
	rm -R -f visualstudio:wq 
	mkdir visualstudio 
	cd visualstudio && cmake -G”Visual Studio 10 Win64"       \
	$(COMMON_DEFINITIONS) ..

lib_only:
	rm -R -f build_lib_only
	mkdir build_lib_only
	cd build_lib_only && cmake $(COMMON_DEFINITIONS) -DLIB_ONLY=TRUE ..

all: generic cortex-m0 cortex-m3 cortex-m4 lib_only 

clean:
	rm -R -f bin
	rm -R -f targets
	rm -R -f include
	rm -R -f build_*
	rm -R -f ext_images
	rm -R -f xcode

unpack_images:
	rm -R -f ext_images
	7z x ext_images.7z

