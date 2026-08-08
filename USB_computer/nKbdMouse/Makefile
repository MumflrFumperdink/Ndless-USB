GCC = nspire-gcc
AS = nspire-as
GXX = nspire-g++
LD = nspire-ld
GENZEHN = genzehn

LDFLAGS = 
EXE = nKbdMouse
COMMONFLAGS = -Wall -W -marm -std=c++17 -MMD -MP
GCCFLAGS = $(COMMONFLAGS) -Os

SRCS_C = $(shell find . -type f -iname \*.c)
SRCS_CPP = $(shell find . -type f -iname \*.cpp)
SRCS_S = $(shell find . -type f -iname \*.S)

DISTDIR = ./build

# Redirect all object files into the build directory
OBJS = $(patsubst %.c,$(DISTDIR)/%.release.o,$(SRCS_C)) \
       $(patsubst %.cpp,$(DISTDIR)/%.release.o,$(SRCS_CPP)) \
       $(patsubst %.S,$(DISTDIR)/%.release.o,$(SRCS_S))

# Generate matching dependency file paths
DEPS = $(OBJS:.o=.d)

all: $(DISTDIR)/$(EXE).tns

# Rule for C files
$(DISTDIR)/%.release.o: %.c
	@mkdir -p $(dir $@)
	$(GCC) $(GCCFLAGS) -c $< -o $@

# Rule for C++ files
$(DISTDIR)/%.release.o: %.cpp
	@mkdir -p $(dir $@)
	$(GXX) $(GCCFLAGS) -c $< -o $@

# Rule for Assembly files
$(DISTDIR)/%.release.o: %.S
	@mkdir -p $(dir $@)
	$(AS) -c $< -o $@

# Link the ELF binary
$(DISTDIR)/$(EXE).elf: $(OBJS)
	@mkdir -p $(DISTDIR)
	$(LD) $^ -o $@ $(LDFLAGS)

# Build the final TNS file
$(DISTDIR)/$(EXE).tns: $(DISTDIR)/$(EXE).elf
	$(GENZEHN) --input $< --output $@.zehn --name "$(EXE)"
	make-prg $@.zehn $@
	rm $@.zehn

# Completely wipe the build directory
clean:
	@if [ -d "$(DISTDIR)" ]; then \
		find "$(DISTDIR)" -type f \( -name "*.o" -o -name "*.d" -o -name "*.elf" -o -name "*.tns" \) -delete; \
	fi


# Safe include that won't error out if files don't exist yet
-include $(DEPS)
