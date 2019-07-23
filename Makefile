BUILD_TYPE = Debug 

ifneq ($(shell test -d .git), 0)
GIT_SHORT_HASH:= $(shell git rev-parse --short HEAD)
endif

VERSION_MAJOR = 1
VERSION_MINOR = 0
VERSION_PATCH = 0

VERSION = $(VERSION_MAJOR).$(VERSION_MINOR).$(VERSION_PATCH)-$(GIT_SHORT_HASH)

COMMON_DEFINITIONS =                                      \
	-DCMAKE_BUILD_TYPE=$(BUILD_TYPE)                      \
	-DVERSION_MAJOR=$(VERSION_MAJOR)                      \
	-DVERSION_MINOR=$(VERSION_MINOR)                      \
	-DVERSION_PATCH=$(VERSION_PATCH)                      \
	-DVERSION=$(VERSION)                                  \

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

arm64:
	rm -R -f build_arm64
	mkdir build_arm64
	cd build_arm64 && cmake -G"Unix Makefiles"      	 \
	$(COMMON_DEFINITIONS)                                \
	-DCMAKE_TOOLCHAIN_FILE=../toolchain/arm64.cmake ..

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
	rm -R -f build_*
	rm -R -f ext_images
	rm -R -f xcode

unpack_images:
	rm -R -f ext_images
	7z x ext_images.7z

