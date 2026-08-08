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
RELEASE_OBJS = $(patsubst %.c,$(DISTDIR)/%.release.o,$(SRCS_C)) \
       $(patsubst %.cpp,$(DISTDIR)/%.release.o,$(SRCS_CPP)) \
       $(patsubst %.S,$(DISTDIR)/%.release.o,$(SRCS_S))
DEBUG_OBJS = $(patsubst %.c,$(DISTDIR)/%.debug.o,$(SRCS_C)) \
       $(patsubst %.cpp,$(DISTDIR)/%.debug.o,$(SRCS_CPP)) \
       $(patsubst %.S,$(DISTDIR)/%.debug.o,$(SRCS_S))

DISTDIR = ./build
vpath %.tns $(DISTDIR)
vpath %.elf $(DISTDIR)

all: $(DISTDIR)/nStorage.tns $(DISTDIR)/nStorage-debug.tns

$(DISTDIR)/%.release.o: %.c
	@mkdir -p $(dir $@)
	$(GCC) $(RELEASE_GCCFLAGS) -c $< -o $@
$(DISTDIR)/%.release.o: %.cpp
	@mkdir -p $(dir $@)
	$(GXX) $(RELEASE_GCCFLAGS) -c $< -o $@
$(DISTDIR)/%.release.o: %.S
	@mkdir -p $(dir $@)
	$(AS) -c $< -o $@

$(DISTDIR)/%.debug.o: %.c
	$(GCC) $(DEBUG_GCCFLAGS) -c $< -o $@
$(DISTDIR)/%.debug.o: %.cpp
	$(GXX) $(DEBUG_GCCFLAGS) -c $< -o $@
$(DISTDIR)/%.debug.o: %.S
	$(AS) -c $< -o $@

$(DISTDIR)/nStorage.elf: $(RELEASE_OBJS)
	mkdir -p $(DISTDIR)
	$(LD) $^ -o $@ $(LDFLAGS)

$(DISTDIR)/nStorage-debug.elf: $(DEBUG_OBJS)
	mkdir -p $(DISTDIR)
	$(LD) $^ -o $@ $(LDFLAGS)

$(DISTDIR)/nStorage.tns: $(DISTDIR)/nStorage.elf
	$(GENZEHN) --input $< --output $@.zehn --name "nStorage"
	make-prg $@.zehn $@
	rm $@.zehn

$(DISTDIR)/nStorage-debug.tns: $(DISTDIR)/nStorage-debug.elf
	$(GENZEHN) --input $< --output $@.zehn --name "nStorage-debug"
	make-prg $@.zehn $@
	rm $@.zehn

clean:
	rm -f $(RELEASE_OBJS) $(DEBUG_OBJS) $(DISTDIR)/*.tns $(DISTDIR)/*.elf $(DISTDIR)/*.zehn