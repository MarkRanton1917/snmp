#include "mib.h"

#include <string.h>
#include <stdio.h>

#include <lwip/apps/snmp.h>
#include <lwip/apps/snmp_table.h>
#include <lwip/apps/snmp_scalar.h>

#define SENSOR_COUNT 4
#define SENSOR_MAX      10
#define SENSOR_NAME_LEN 20

struct sensor_inf
{
  u8_t num;
  char file[SENSOR_NAME_LEN + 1];
  s32_t value;
};

static struct sensor_inf sensors[SENSOR_MAX];

static s16_t      sensor_count_get_value(struct snmp_node_instance* instance, void* value);
static snmp_err_t sensor_table_get_cell_instance(const u32_t* column, const u32_t* row_oid, u8_t row_oid_len, struct snmp_node_instance* cell_instance);
static snmp_err_t sensor_table_get_next_cell_instance(const u32_t* column, struct snmp_obj_id* row_oid, struct snmp_node_instance* cell_instance);
static s16_t      sensor_table_get_value(struct snmp_node_instance* instance, void* value);
static snmp_err_t sensor_table_set_value(struct snmp_node_instance* instance, u16_t len, void *value);

/* sensorentry .1.3.6.1.4.1.26381.1.1.1 (.level0.level1)
   where level 0 is the table column (temperature/file name)
   and level 1 the table row (sensor index) */
static const struct snmp_table_col_def sensor_table_columns[] = {
  { 1, SNMP_ASN1_TYPE_INTEGER,      SNMP_NODE_INSTANCE_READ_WRITE },
  { 2, SNMP_ASN1_TYPE_OCTET_STRING, SNMP_NODE_INSTANCE_READ_ONLY  }
};

/* sensortable .1.3.6.1.4.1.26381.1.1 */
static const struct snmp_table_node sensor_table = SNMP_TABLE_CREATE(
  1, sensor_table_columns, 
  sensor_table_get_cell_instance, sensor_table_get_next_cell_instance, 
  sensor_table_get_value, snmp_set_test_ok, sensor_table_set_value);

/* sensorcount .1.3.6.1.4.1.26381.1.2 */
static const struct snmp_scalar_node sensor_count = SNMP_SCALAR_CREATE_NODE_READONLY(
  2, SNMP_ASN1_TYPE_INTEGER, sensor_count_get_value); 

/* example .1.3.6.1.4.1.26381.1 */
static const struct snmp_node* const example_nodes[] = {
  &sensor_table.node.node,
  &sensor_count.node.node
};
static const struct snmp_tree_node example_node = SNMP_CREATE_TREE_NODE(1, example_nodes);

static const uint32_t unimonOid[] = { 1,3,6,1,4,1,26381,1 };
static struct snmp_obj_id unimonOidStruct;
const struct snmp_mib unimonMib = SNMP_MIB_CREATE(unimonOid, &example_node.node);

/**
 * Initialises this private MIB before use.
 * @see main.c
 */
void unimonMibInit(void)
{
  uint8_t i;
  unimonOidStruct.len = sizeof(unimonOid);
  memcpy(unimonOidStruct.id, unimonOid, unimonOidStruct.len);
  snmp_set_device_enterprise_oid(&unimonOidStruct);
  memset(sensors, 0, sizeof(sensors));
  
  for (i = 0; i < SENSOR_COUNT; i++) {
    sensors[i].num = (u8_t)(i + 1);
    snprintf(sensors[i].file, sizeof(sensors[i].file), "%d.txt", i);
    sensors[i].value = 11 * (i+1);
  }
}

/* sensorcount .1.3.6.1.4.1.26381.1.2 */
static s16_t
sensor_count_get_value(struct snmp_node_instance* instance, void* value)
{
  size_t count = 0;
  u32_t *uint_ptr = (u32_t*)value;

  LWIP_UNUSED_ARG(instance);
  
  for(count=0; count<LWIP_ARRAYSIZE(sensors); count++) {
    if(sensors[count].num == 0) {
      *uint_ptr = (u32_t)count;
      return sizeof(*uint_ptr);
    }
  }

  return 0;  
}

/* sensortable .1.3.6.1.4.1.26381.1.1 */
/* list of allowed value ranges for incoming OID */
static const struct snmp_oid_range sensor_table_oid_ranges[] = {
  { 1, SENSOR_MAX+1 }
};

static snmp_err_t
sensor_table_get_cell_instance(const u32_t* column, const u32_t* row_oid, u8_t row_oid_len, struct snmp_node_instance* cell_instance)
{
  u32_t sensor_num;
  size_t i;

  LWIP_UNUSED_ARG(column);

  /* check if incoming OID length and if values are in plausible range */
  if(!snmp_oid_in_range(row_oid, row_oid_len, sensor_table_oid_ranges, LWIP_ARRAYSIZE(sensor_table_oid_ranges))) {
    return SNMP_ERR_NOSUCHINSTANCE;
  }

  /* get sensor index from incoming OID */
  sensor_num = row_oid[0];

  /* find sensor with index */
  for(i=0; i<LWIP_ARRAYSIZE(sensors); i++) {
    if(sensors[i].num != 0) {
      if(sensors[i].num == sensor_num) {
        /* store sensor index for subsequent operations (get/test/set) */
        cell_instance->reference.u32 = (u32_t)i;
        return SNMP_ERR_NOERROR;
      }
    }
  }

  /* not found */
  return SNMP_ERR_NOSUCHINSTANCE;
}

static snmp_err_t
sensor_table_get_next_cell_instance(const u32_t* column, struct snmp_obj_id* row_oid, struct snmp_node_instance* cell_instance)
{
  size_t i;
  struct snmp_next_oid_state state;
  u32_t result_temp[LWIP_ARRAYSIZE(sensor_table_oid_ranges)];

  LWIP_UNUSED_ARG(column);
  
  /* init struct to search next oid */
  snmp_next_oid_init(&state, row_oid->id, row_oid->len, result_temp, LWIP_ARRAYSIZE(sensor_table_oid_ranges));

  /* iterate over all possible OIDs to find the next one */
  for(i=0; i<LWIP_ARRAYSIZE(sensors); i++) {
    if(sensors[i].num != 0) {
      u32_t test_oid[LWIP_ARRAYSIZE(sensor_table_oid_ranges)];

      test_oid[0] = sensors[i].num;

      /* check generated OID: is it a candidate for the next one? */
      snmp_next_oid_check(&state, test_oid, LWIP_ARRAYSIZE(sensor_table_oid_ranges), (void*)i);
    }
  }

  /* did we find a next one? */
  if(state.status == SNMP_NEXT_OID_STATUS_SUCCESS) {
    snmp_oid_assign(row_oid, state.next_oid, state.next_oid_len);
    /* store sensor index for subsequent operations (get/test/set) */
    cell_instance->reference.u32 = LWIP_CONST_CAST(u32_t, state.reference);
    return SNMP_ERR_NOERROR;
  }

  /* not found */
  return SNMP_ERR_NOSUCHINSTANCE;
}

static s16_t
sensor_table_get_value(struct snmp_node_instance* instance, void* value)
{
  u32_t i = instance->reference.u32;
  s32_t *temperature = (s32_t *)value;

  switch (SNMP_TABLE_GET_COLUMN_FROM_OID(instance->instance_oid.id))
  {
  case 1: /* sensor value */
    *temperature = sensors[i].value;
    return sizeof(s32_t);
  case 2: /* file name */
    MEMCPY(value, sensors[i].file, strlen(sensors[i].file));
    return (s16_t)strlen(sensors[i].file);
  default:
    return 0;
  }
}

static snmp_err_t
sensor_table_set_value(struct snmp_node_instance* instance, u16_t len, void *value)
{
  u32_t i = instance->reference.u32;
  s32_t *temperature = (s32_t *)value;
  sensors[i].value = *temperature;

  LWIP_UNUSED_ARG(len);

  return SNMP_ERR_NOERROR;
}
