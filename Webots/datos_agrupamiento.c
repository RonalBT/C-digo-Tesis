#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/gps.h>
#include <stdio.h>
#include <string.h>

#define TIME_STEP 32

int main() {
  wb_robot_init();
  WbDeviceTag gps = wb_robot_get_device("gps");
  WbDeviceTag emitter = wb_robot_get_device("emitter");
  wb_gps_enable(gps, TIME_STEP);

  const char *name = wb_robot_get_name();
  printf("🔵 Baliza %s iniciada.\n", name);

  while (wb_robot_step(TIME_STEP) != -1) {
    const double *pos = wb_gps_get_values(gps);
    char msg[128];
    sprintf(msg, "BALIZA,%s,%.2f,%.2f", name, pos[0], pos[1]);
    wb_emitter_send(emitter, msg, strlen(msg) + 1);
  }

  wb_robot_cleanup();
  return 0;
}
