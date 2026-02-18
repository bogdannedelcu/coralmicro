#ifndef _FFCONF_H_
#define _FFCONF_H_

/*---------------------------------------------------------------------------/
/  FatFs Functional Configurations for Coral Micro SD Card
/---------------------------------------------------------------------------*/

#define FFCONF_DEF	86631	/* Revision ID */

/*---------------------------------------------------------------------------/
/ MSDK adaptation configuration
/---------------------------------------------------------------------------*/
#define SD_DISK_ENABLE

/*---------------------------------------------------------------------------/
/ Function Configurations
/---------------------------------------------------------------------------*/

#define FF_FS_READONLY	0
/* Read/Write mode enabled */

#define FF_FS_MINIMIZE	0
/* All basic functions enabled */

#define FF_USE_FIND		0
/* Disable filtered directory read */

#define FF_USE_MKFS		1
/* Enable f_mkfs() for formatting */

#define FF_USE_FASTSEEK	0
/* Disable fast seek */

#define FF_USE_EXPAND	0
/* Disable f_expand */

#define FF_USE_CHMOD	1
/* Enable attribute manipulation */

#define FF_USE_LABEL	1
/* Enable volume label functions */

#define FF_USE_FORWARD	0
/* Disable f_forward */

#define FF_USE_STRFUNC	2
/* Enable string functions with LF-CRLF conversion */
#define FF_PRINT_LLI	0
#define FF_PRINT_FLOAT	0
#define FF_STRF_ENCODE	0
/* Disable long long and float printing to avoid type issues */

/*---------------------------------------------------------------------------/
/ Locale and Namespace Configurations
/---------------------------------------------------------------------------*/

#define FF_CODE_PAGE	437
/* US */

#define FF_USE_LFN		2
/* Enable LFN with dynamic memory allocation */
#define FF_MAX_LFN		255
/* Maximum LFN length */

#define FF_LFN_UNICODE	0
/* ANSI/OEM in current CP */

#define FF_LFN_BUF		255
#define FF_SFN_BUF		12
/* Buffer sizes for LFN and SFN */

#define FF_FS_RPATH		2
/* Enable relative path with current directory */

/*---------------------------------------------------------------------------/
/ Drive/Volume Configurations
/---------------------------------------------------------------------------*/

#define FF_VOLUMES		1
/* Number of volumes (logical drives) */

#define FF_STR_VOLUME_ID	0
/* Numeric drive ID */

#define FF_MULTI_PARTITION	0
/* Single partition per volume */

#define FF_MIN_SS		512
#define FF_MAX_SS		512
/* Fixed sector size of 512 bytes */

#define FF_LBA64		0
/* Disable 64-bit LBA (not needed for most SD cards) */

#define FF_MIN_GPT		0x10000000
/* Minimum number of sectors for GPT */

#define FF_USE_TRIM		0
/* Disable TRIM command */

/*---------------------------------------------------------------------------/
/ System Configurations
/---------------------------------------------------------------------------*/

#define FF_FS_TINY		0
/* Normal buffer configuration */

#define FF_FS_EXFAT		1
/* Enable exFAT filesystem support */

#define FF_FS_NORTC		0
/* Use RTC */
#define FF_NORTC_MON	1
#define FF_NORTC_MDAY	1
#define FF_NORTC_YEAR	2025
/* Default timestamp if RTC not available */

#define FF_FS_NOFSINFO	0
/* Use FSINFO sector on FAT32 */

#define FF_FS_LOCK		0
/* Disable file lock feature */

#define FF_FS_REENTRANT	0
/* Disable reentrant for now (single-threaded filesystem access) */
#define FF_FS_TIMEOUT	1000
/* Timeout for acquiring mutex (ms) */

#endif /* _FFCONF_H_ */
