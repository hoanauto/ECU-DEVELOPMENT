# Tool chain
CC = arm-none-eabi-gcc
AS = arm-none-eabi-as

# Flags
CFLAGS = -mcpu=cortex-m3 -mthumb -std=c11 -Wall -g -O0 \
	-IINC \
	-ILIB \
	-DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER

LDFLAGS = -TSTARTUP/linker.ld -nostartfiles -Wl,--gc-sections
LIBS = -lm -lc

# Source files
SRC = $(wildcard SRC/*.c)
STARTUP = STARTUP/startup_stm32f10x_md.s
STARTUP_OBJ = STARTUP/startup_stm32f10x_md.o
OBJ = $(SRC:.c=.o) $(STARTUP_OBJ)
OUT = BUILD/test.elf

# Build rule for startup assembly file
$(STARTUP_OBJ): $(STARTUP)
	$(CC) $(CFLAGS) -c $< -o $@

# Build rule
build: $(OUT)

$(OUT): $(OBJ)
	@mkdir BUILD 2>nul
	$(CC) $(CFLAGS) $(OBJ) -o $@ $(LDFLAGS) $(LIBS)

# Clean rule for Windows
clean:
	if exist SRC\*.o del /Q SRC\*.o
	if exist LIB\SPL\src\*.o del /Q LIB\SPL\src\*.o
	if exist STARTUP\*.o del /Q STARTUP\*.o
	if exist BUILD rd /S /Q BUILD

# Flash rule
Flash: $(OUT)
	openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program $(OUT) verify reset exit"
