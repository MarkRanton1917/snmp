#pragma once

#include "lwip/apps/snmp_opts.h"
#include "lwip/err.h"
#include "lwip/apps/snmpv3.h"

err_t snmpv3_set_user_auth_algo(const char *username, snmpv3_auth_algo_t algo);
err_t snmpv3_set_user_priv_algo(const char *username, snmpv3_priv_algo_t algo);
err_t snmpv3_set_user_auth_key(const char *username, const char *password);
err_t snmpv3_set_user_priv_key(const char *username, const char *password);

void snmpv3_dummy_init(void);