# project specific files
SRC += rand.c

# MCU name
MCU = SN32F268F

# Build Options
#   comment out to disable the options.
#
MAGIC_ENABLE = yes
MAGIC_KEYCODE_ENABLE = yes
BOOTMAGIC_ENABLE = yes 	# Virtual DIP switch configuration
EXTRAKEY_ENABLE = yes   	# Audio control and System control
NKRO_ENABLE = yes       	# USB Nkey Rollover
DIP_SWITCH_ENABLE = yes
LTO_ENABLE = no

# Custom Key and LED matrix handling
CUSTOM_MATRIX = no
RGB_MATRIX_ENABLE = no
RGB_MATRIX_DRIVER = SN32F26x
RGB_MATRIX_CUSTOM_KB = yes

# Reduce code size
USE_PROCESS_STACKSIZE = 0x1E0
USE_EXCEPTIONS_STACKSIZE = 0xF0
