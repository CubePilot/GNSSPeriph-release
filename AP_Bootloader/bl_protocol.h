void jump_to_app(void);
void jump_to_fallback(void);
void jump_to_address(uint32_t address, bool watchdog_enabled, bool send_comms);
void bootloader(unsigned timeout);

/*****************************************************************************
 * Chip/board functions.
 */

/* LEDs */

#ifdef BOOT_DELAY_ADDRESS
# define BOOT_DELAY_SIGNATURE1	0x92c2ecff
# define BOOT_DELAY_SIGNATURE2	0xc5057d5d
# define BOOT_DELAY_MAX		30
#endif

#define MAX_DES_LENGTH 20

#define arraySize(a) (sizeof((a))/sizeof(((a)[0])))
