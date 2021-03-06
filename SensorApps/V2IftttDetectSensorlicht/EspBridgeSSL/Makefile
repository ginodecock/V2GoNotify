#Position and maximum length of espfs in flash memory. This can be undefined. In this case 
#the webpages will be linked in into the executable file. If this is defined, please do a 
#'make htmlflash' to flash the espfs into the ESPs memory.
ESPFS_POS = 0x12000
ESPFS_SIZE = 0x2E000
CERTIFICATE_POS = 0x1A000

DEFCFG_POS = 0xfc000

# Output directors to store intermediate compiled files
# relative to the project directory
BUILD_BASE	= build
FW_BASE		= firmware

# Base directory for the compiler. Needs a / at the end; if not set it'll use the tools that are in
# the PATH.
XTENSA_TOOLS_ROOT = /esptools/esp-open-sdk/xtensa-lx106-elf/bin/

# base directory of the ESP8266 SDK package, absolute
SDK_BASE	= /esptools/esp-open-sdk/NONOSSDK/esp_iot_sdk_v2.0.0
#Extra Tensilica includes from the ESS VM
SDK_EXTRA_INCLUDES = /esptools/esp-open-sdk/sdk/include
#SDK_EXTRA_INCLUDES = /esptools/esp-open-sdk/xtensa-lx106-elf/xtensa-lx106-elf/sysroot/usr/include
SDK_EXTRA_LIBS = /esptools/esp-open-sdk/sdk/lib

#Esptool.py path and port
#ESPTOOL		= /mnt/Share/esp/esptool/esptool
ESPTOOL		= /mnt/Share/esp/esp_bridge/tools/esptool.py
ESPPORT		= /dev/ttyUSB0
#ESPDELAY indicates seconds to wait between flashing the two binary images
ESPDELAY	= 3
ESPBAUD		= 460800


# name for the target project
TARGET		= httpd

# which modules (subdirectories) of the project to include in compiling
#MODULES		= modules/mqtt/mqtt user modules
MODULES		= user modules/mqtt/mqtt modules
EXTRA_INCDIR	= include libesphttpd/include $(SDK_BASE)/../include

# libraries used in this project, mainly provided by the SDK
LIBS		= c gcc hal phy pp net80211 wpa main lwip ssl crypto
#Add in esphttpd lib
LIBS += esphttpd

# compiler flags using during compilation of source files
#CFLAGS		= -Os -ggdb -std=c99 -Werror -Wpointer-arith -Wundef -Wall -Wl,-EL -fno-inline-functions \
#		-nostdlib -mlongcalls -mtext-section-literals  -D__ets__ -DICACHE_FLASH -D_STDINT_H \
#		-Wno-address -Werror 
#CFLAGS		= -Os -Wpointer-arith -Wundef -Werror -Wall -Wl,-EL -fno-inline-functions -nostdlib -mlongcalls -mtext-section-literals  -D__ets__ -DICACHE_FLASH
CFLAGS		= -Os -ggdb -Wpointer-arith -Wundef -Werror -Wl,-EL -fno-inline-functions -nostdlib -mlongcalls -mtext-section-literals  -D__ets__ -DICACHE_FLASH -D_STDINT_H
# linker flags used to generate the main object file
LDFLAGS		= -nostdlib -Wl,--no-check-sections -u call_user_start -Wl,-static

# linker script used for the above linkier step
LD_SCRIPT	= eagle.app.v6.ld

# various paths from the SDK used in this project
SDK_LIBDIR	= lib
SDK_LDDIR	= ld
SDK_INCDIR	= include include/json

# select which tools to use as compiler, librarian and linker
CC		:= $(XTENSA_TOOLS_ROOT)xtensa-lx106-elf-gcc
AR		:= $(XTENSA_TOOLS_ROOT)xtensa-lx106-elf-ar
LD		:= $(XTENSA_TOOLS_ROOT)xtensa-lx106-elf-gcc


####
#### no user configurable options below here
####
SRC_DIR		:= $(MODULES)
BUILD_DIR	:= $(addprefix $(BUILD_BASE)/,$(MODULES))

SDK_LIBDIR	:= $(addprefix $(SDK_BASE)/,$(SDK_LIBDIR))
SDK_INCDIR	:= $(addprefix -I$(SDK_BASE)/,$(SDK_INCDIR))

SRC		:= $(foreach sdir,$(SRC_DIR),$(wildcard $(sdir)/*.c))
OBJ		:= $(patsubst %.c,$(BUILD_BASE)/%.o,$(SRC))
LIBS		:= $(addprefix -l,$(LIBS))
APP_AR		:= $(addprefix $(BUILD_BASE)/,$(TARGET)_app.a)
TARGET_OUT	:= $(addprefix $(BUILD_BASE)/,$(TARGET).out)

LD_SCRIPT	:= $(addprefix -T$(SDK_BASE)/$(SDK_LDDIR)/,$(LD_SCRIPT))

INCDIR	:= $(addprefix -I,$(SRC_DIR))
EXTRA_INCDIR	:= $(addprefix -I,$(EXTRA_INCDIR))
MODULE_INCDIR	:= $(addsuffix /include,$(INCDIR))

V ?= $(VERBOSE)
ifeq ("$(V)","1")
Q :=
vecho := @true
else
Q := @
vecho := @echo
endif

ifeq ("$(GZIP_COMPRESSION)","yes")
CFLAGS		+= -DGZIP_COMPRESSION
endif

ifeq ("$(USE_HEATSHRINK)","yes")
CFLAGS		+= -DESPFS_HEATSHRINK
endif

ifeq ("$(ESPFS_POS)","")
#No hardcoded espfs position: link it in with the binaries.
LIBS += -lwebpages-espfs
else
#Hardcoded espfs location: Pass espfs position to rest of code
CFLAGS += -DESPFS_POS=$(ESPFS_POS) -DESPFS_SIZE=$(ESPFS_SIZE)
endif

vpath %.c $(SRC_DIR)

define compile-objects
$1/%.o: %.c
	$(vecho) "CC $$<"
	$(Q) $(CC) $(INCDIR) $(MODULE_INCDIR) $(EXTRA_INCDIR) $(SDK_INCDIR) $(CFLAGS)  -c $$< -o $$@
endef

.PHONY: all checkdirs clean libesphttpd

all: checkdirs $(TARGET_OUT) $(FW_BASE)

libesphttpd/Makefile:
	$(Q) echo "No libesphttpd submodule found. Using git to fetch it..."
	$(Q) git submodule init
	$(Q) git submodule update

libesphttpd: libesphttpd/Makefile
	$(Q) make -C libesphttpd

$(TARGET_OUT): $(APP_AR)
	$(vecho) "LD $@"
	$(Q) $(LD) -Llibesphttpd -L$(SDK_LIBDIR) $(LD_SCRIPT) $(LDFLAGS) -Wl,--start-group $(LIBS) $(APP_AR) -Wl,--end-group -o $@

$(FW_BASE): $(TARGET_OUT)
	$(vecho) "FW $@"
	$(Q) mkdir -p $@
	$(Q) $(ESPTOOL) elf2image $(TARGET_OUT) --output $@/

$(APP_AR):  libesphttpd $(OBJ)
	$(vecho) "AR $@"
	$(Q) $(AR) cru $@ $(OBJ)

checkdirs: $(BUILD_DIR)

$(BUILD_DIR):
	$(Q) mkdir -p $@


flash: $(TARGET_OUT) $(FW_BASE)
	$(Q) $(ESPTOOL) --port $(ESPPORT) --baud $(ESPBAUD) write_flash 0x00000 $(FW_BASE)/0x00000.bin 0x20000 $(FW_BASE)/0x20000.bin $(ESPFS_POS) libesphttpd/webpages.espfs $(CERTIFICATE_POS) esp_ca_cert.bin $(DEFCFG_POS) esp_init_data_default.bin

blankflash:
	$(Q) $(ESPTOOL) --port $(ESPPORT) --baud $(ESPBAUD) write_flash 0x7E000 $(SDK_BASE)/bin/blank.bin

htmlflash: libesphttpd
	$(Q) if [ $$(stat -c '%s' libesphttpd/webpages.espfs) -gt $$(( $(ESPFS_SIZE) )) ]; then echo "webpages.espfs too big!"; false; fi
	$(Q) $(ESPTOOL) --port $(ESPPORT) --baud $(ESPBAUD) write_flash $(ESPFS_POS) libesphttpd/webpages.espfs

clean:
	$(Q) make -C libesphttpd clean
	$(Q) rm -f $(APP_AR)
	$(Q) rm -f $(TARGET_OUT)
	$(Q) find $(BUILD_BASE) -type f | xargs rm -f
	$(Q) rm -rf $(FW_BASE)

$(foreach bdir,$(BUILD_DIR),$(eval $(call compile-objects,$(bdir))))
