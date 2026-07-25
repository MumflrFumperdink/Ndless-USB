DEBUG = TRUE

GCC = nspire-gcc
AS  = nspire-as
GXX = nspire-g++
LD  = nspire-ld
GENZEHN = genzehn

GCCFLAGS = -Wall -W -marm -std=c++17 
LDFLAGS = 
ZEHNFLAGS = --name "nCoreTemp"

ifeq ($(DEBUG),FALSE)
	GCCFLAGS += -Os
else
	GCCFLAGS += -O0 -g
endif

OBJS = $(patsubst %.c, %.o, $(shell find . -type f -iname \*.c))
OBJS += $(patsubst %.cpp, %.o, $(shell find . -type f -iname \*.cpp))
OBJS += $(patsubst %.S, %.o, $(shell find . -type f -iname \*.S))
EXE = nCoreTemp
DISTDIR = .
vpath %.tns $(DISTDIR)
vpath %.elf $(DISTDIR)

all: $(EXE).tns

%.o: %.c
	$(GCC) $(GCCFLAGS) -c $< -o $@

%.o: %.cpp
	$(GXX) $(GCCFLAGS) -c $< -o $@
	
%.o: %.S
	$(AS) -c $< -o $@

$(EXE).elf: main.o $(OBJS)
	mkdir -p $(DISTDIR)
	$(LD) $^ -o $@ $(LDFLAGS)

%.tns: %.elf
	$(GENZEHN) --input $^ --output $@.zehn $(ZEHNFLAGS)
	make-prg $@.zehn $@
	rm $@.zehn

clean:
	rm -f *.o $(DISTDIR)/*.tns $(DISTDIR)/*.elf $(DISTDIR)/*.zehn
