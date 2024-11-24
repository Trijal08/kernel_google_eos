#include <linux/utsname.h>

#define DEBUG_KINFO_MAGIC	0xCCEEDDFF
#define DEBUG_KINFO_VENDOR_OFFSET	0x800

struct vendor_kernel_info {
	/* For linux banner */
	__u8 uts_release[__NEW_UTS_LEN];
} __packed;

struct vendor_kernel_all_info {
	__u32 magic_number;
	__u32 combined_checksum;
	struct vendor_kernel_info info;
} __packed;
