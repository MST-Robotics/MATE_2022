DEPS_CFLAGS?=$(shell env PKG_CONFIG_PATH=/usr/local/frc/lib/pkgconfig pkg-config --cflags wpilibc)
CXXFLAGS?=-std=c++17 -Wno-psabi
DEPS_LIBS?=$(shell env PKG_CONFIG_PATH=/usr/local/frc/lib/pkgconfig pkg-config --libs wpilibc)
EXE=VISION
DESTDIR?=/home/pi/
PROJECTDIR=Code
SOURCEDIR=Code/Sources
.PHONY: clean build install

build: ${EXE}

install: build
	cp ${EXE} runCamera ${DESTDIR}

clean:
	rm ${EXE} *.o

depend: ${}

OBJS=${SOURCEDIR}/FPS.o ${SOURCEDIR}/VideoGet.o ${SOURCEDIR}/VideoShow.o ${SOURCEDIR}/VideoProcess.o ${PROJECTDIR}/main.o

${EXE}: ${OBJS}
	${CXX} -pthread -g -o $@ $^ ${DEPS_LIBS} -Wl,--unresolved-symbols=ignore-in-shared-libs

.cpp.o:
	${CXX} -pthread -g -Og -c -o $@ ${CXXFLAGS} ${DEPS_CFLAGS} $<
