#include "print.h"

#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <stdarg.h>
#include <string.h>

#define LOG_LEVEL LOG_LEVEL_APP
static void startPrintTask(void *argument);
static int printCore(const char *, va_list);

static QueueHandle_t printQueueHandle;
static char buf[256];

int print(uint8_t log_level, const char *format, ...) {
  int ret = -1;
  va_list args;
  if (log_level >= LOG_LEVEL) {
    va_start(args, format);
    ret = printCore(format, args);
    va_end(args);
  }
  return ret;
}

int printDebug(const char *format, ...) {
  int ret = -1;
  va_list args;
  if (LOG_LEVEL == LOG_LEVEL_DEBUG) {
    va_start(args, format);
    ret = printCore(format, args);
    va_end(args);
  }
  return ret;
}

int printAssert(const char *message) {
  int ret;
  ret = printDebug("Assert failed: \"%s\" in file \"%s\" (%d)\n", message,
                   __FILE__, __LINE__);
  return ret;
}

void printInit() {
#if LOG_LEVEL != LOG_LEVEL_NONE
  printQueueHandle = xQueueCreate(256, sizeof(char *));
  xTaskCreate(startPrintTask, "printTask", 4096, NULL, 1, NULL);
#endif
}

static int printCore(const char *format, va_list args) {  
#if LOG_LEVEL != LOG_LEVEL_NONE
  int len;
  char *msg;
  vsnprintf(buf, 256, format, args);
  len = strlen(buf);

  if ((msg = (char*)pvPortMalloc(sizeof(char) * (len + 1))) == NULL)
    return -1;
  strcpy(msg, buf);

  if (xQueueSend(printQueueHandle, &msg, pdMS_TO_TICKS(100) == pdPASS))
    return len;
  vPortFree(msg);
#endif
  return -1;
}

#if LOG_LEVEL != LOG_LEVEL_NONE
static void startPrintTask(void *argument) {
  (void)argument;
  char *msg;

  Serial.begin(115200);

  while (1) {
    xQueueReceive(printQueueHandle, &msg, portMAX_DELAY);
    Serial.print(msg);
    vPortFree(msg);
  }
}
#endif