/**
 * @file
 * Exports Private lwIP MIB 
 */

#ifndef LWIP_HDR_PRIVATE_MIB_H
#define LWIP_HDR_PRIVATE_MIB_H

#ifdef __cplusplus
extern "C" {
#endif

/* export MIB */
extern const struct snmp_mib unimonMib;

void unimonMibInit(void);

#ifdef __cplusplus
}
#endif

#endif
