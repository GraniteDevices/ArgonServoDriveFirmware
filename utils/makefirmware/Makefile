# Makefile

CFLAGS=-c -Wall -MMD
LDFLAGS=-fno-exceptions -s

MAKEFLAGS += -s

ifeq ($(OS), Windows_NT)
	CC = mingw32-gcc
	EXE_SUFFIX = .exe
else
	CC = gcc
endif

MKFW_EXE = makefirmware$(EXE_SUFFIX)

all: $(MKFW_EXE) 

MKFW_SRC = \
	makefirmware.c 


MKFW_OBJS = $(MKFW_SRC:.c=.o)

RULES = $(MKFW_SRC:.c=.d)

%.o:%.c	Makefile
	@echo CC $<
	$(CC) $(CFLAGS) -c $< -o $@

$(MKFW_EXE): $(MKFW_OBJS)
	@echo Linking $@
	$(CC) $(LDFLAGS) $^ -o $@

clean:
	-$(RM) $(MKFW_OBJS) 
	-$(RM) $(RULES)
	-$(RM) $(MKFW_EXE)

-include $(RULES)
