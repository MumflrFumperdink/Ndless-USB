GCC = nspire-gcc
AS  = nspire-as
GXX = nspire-g++
LD  = nspire-ld
GENZEHN = genzehn
LDFLAGS =

COMMONFLAGS = -Wall -W -marm -std=c++17

# ---- Release build: nStorage.tns (DEBUG_FS off, optimized) ----
RELEASE_GCCFLAGS = $(COMMONFLAGS) -Os

# ---- Debug build: nStorage-debug.tns (DEBUG_FS forced on) ----
DEBUG_GCCFLAGS = $(COMMONFLAGS) -O0 -g -DDEBUG_FS=1

SRCS_C   = $(shell find . -type f -iname \*.c)
SRCS_CPP = $(shell find . -type f -iname \*.cpp)
SRCS_S   = $(shell find . -type f -iname \*.S)

# Separate object-file suffixes per variant so both builds can coexist
# in the same source tree without overwriting each other's .o files.
RELEASE_OBJS = $(patsubst %.c,%.release.o,$(SRCS_C)) \
               $(patsubst %.cpp,%.release.o,$(SRCS_CPP)) \
               $(patsubst %.S,%.release.o,$(SRCS_S))
DEBUG_OBJS   = $(patsubst %.c,%.debug.o,$(SRCS_C)) \
               $(patsubst %.cpp,%.debug.o,$(SRCS_CPP)) \
               $(patsubst %.S,%.debug.o,$(SRCS_S))

DISTDIR = .
vpath %.tns $(DISTDIR)
vpath %.elf $(DISTDIR)

all: nStorage.tns nStorage-debug.tns

%.release.o: %.c
	$(GCC) $(RELEASE_GCCFLAGS) -c $< -o $@
%.release.o: %.cpp
	$(GXX) $(RELEASE_GCCFLAGS) -c $< -o $@
%.release.o: %.S
	$(AS) -c $< -o $@

%.debug.o: %.c
	$(GCC) $(DEBUG_GCCFLAGS) -c $< -o $@
%.debug.o: %.cpp
	$(GXX) $(DEBUG_GCCFLAGS) -c $< -o $@
%.debug.o: %.S
	$(AS) -c $< -o $@

nStorage.elf: $(RELEASE_OBJS)
	mkdir -p $(DISTDIR)
	$(LD) $^ -o $@ $(LDFLAGS)

nStorage-debug.elf: $(DEBUG_OBJS)
	mkdir -p $(DISTDIR)
	$(LD) $^ -o $@ $(LDFLAGS)

nStorage.tns: nStorage.elf
	$(GENZEHN) --input $< --output $@.zehn --name "nStorage"
	make-prg $@.zehn $@
	rm $@.zehn

nStorage-debug.tns: nStorage-debug.elf
	$(GENZEHN) --input $< --output $@.zehn --name "nStorage-debug"
	make-prg $@.zehn $@
	rm $@.zehn

clean:
	rm -f $(RELEASE_OBJS) $(DEBUG_OBJS) $(DISTDIR)/*.tns $(DISTDIR)/*.elf $(DISTDIR)/*.zehn