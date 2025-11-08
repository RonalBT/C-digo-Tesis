/*
 * Pioneer 3-DX — Controlador de navegación con destino asignado por Supervisor
 * ------------------------------------------------------------
 * - Recibe por Receiver: DESTINO,<nombre>,<x>,<y>
 * - Se dirige al punto asignado (X,Y)
 * - Evita obstáculos con sensores IR
 * - Se detiene al llegar
 * - Compatible con múltiples robots y balizas activas
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <webots/compass.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/receiver.h>
#include <webots/robot.h>

// --- Parámetros de configuración ---
#define MAX_SPEED 5.24
#define MAX_SENSOR_NUMBER 16
#define DELAY 70
#define MAX_SENSOR_VALUE 1024
#define MIN_DISTANCE 1.0
#define WHEEL_WEIGHT_THRESHOLD 100
#define KP_ANGLE 2.0
#define STOP_DISTANCE 0.5

// --- Estructura de sensores IR ---
typedef struct {
  WbDeviceTag tag;
  double weight[2];
} Sensor;

// --- Configuración de pesos del Pioneer ---
static Sensor sensors[MAX_SENSOR_NUMBER] = {
  {.weight = {150, 0}}, {.weight = {200, 0}}, {.weight = {300, 0}}, {.weight = {600, 0}},
  {.weight = {0, 600}}, {.weight = {0, 300}}, {.weight = {0, 200}}, {.weight = {0, 150}},
  {.weight = {0, 0}},   {.weight = {0, 0}},   {.weight = {0, 0}},   {.weight = {0, 0}},
  {.weight = {0, 0}},   {.weight = {0, 0}},   {.weight = {0, 0}},   {.weight = {0, 0}}
};

// --- Función auxiliar para distancia 2D ---
static double dist2d(double x1, double y1, double x2, double y2) {
  double dx = x1 - x2, dy = y1 - y2;
  return sqrt(dx * dx + dy * dy);
}

int main() {
  wb_robot_init();
  int ts = wb_robot_get_basic_time_step();

  // --- Motores ---
  WbDeviceTag lw = wb_robot_get_device("left wheel");
  WbDeviceTag rw = wb_robot_get_device("right wheel");
  wb_motor_set_position(lw, INFINITY);
  wb_motor_set_position(rw, INFINITY);
  wb_motor_set_velocity(lw, 0);
  wb_motor_set_velocity(rw, 0);

  // --- Sensores IR ---
  for (int i = 0; i < MAX_SENSOR_NUMBER; i++) {
    char n[5];
    sprintf(n, "so%d", i);
    sensors[i].tag = wb_robot_get_device(n);
    wb_distance_sensor_enable(sensors[i].tag, ts);
  }

  // --- Sensores principales ---
  WbDeviceTag gps = wb_robot_get_device("gps");
  WbDeviceTag compass = wb_robot_get_device("compass");
  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_gps_enable(gps, ts);
  wb_compass_enable(compass, ts);
  wb_receiver_enable(receiver, ts);

  // --- LEDs ---
  WbDeviceTag red_led[3];
  red_led[0] = wb_robot_get_device("red led 1");
  red_led[1] = wb_robot_get_device("red led 2");
  red_led[2] = wb_robot_get_device("red led 3");

  printf("Controlador Pioneer iniciado — Esperando destino del Supervisor...\n");

  // --- Variables de navegación ---
  double goal_x = 0, goal_y = 0;
  int target_found = 0;
  char current_target[32] = "SIN_DESTINO";

  int led_number = 0, delay = 0;
  double left_speed = 0, right_speed = 0;

  while (wb_robot_step(ts) != -1) {

    // === 1. Recepción de mensajes del Supervisor ===
// === 1. Recepción de mensajes del Supervisor ===
while (wb_receiver_get_queue_length(receiver) > 0) {
  const char *msg = wb_receiver_get_data(receiver);
  char tag[16], nombre_robot[32], destino[32];
  double gx, gy;

  // Formato: DESTINO,<robot>,<baliza>,<x>,<y>
  if (sscanf(msg, "%[^,],%[^,],%[^,],%lf,%lf", tag, nombre_robot, destino, &gx, &gy) == 5) {
    // Solo aceptar mensajes dirigidos a este robot
    const char *mi_nombre = wb_robot_get_name();
    if (strcmp(tag, "DESTINO") == 0 && strcmp(nombre_robot, mi_nombre) == 0) {
      goal_x = gx;
      goal_y = gy;
      target_found = 1;
      strcpy(current_target, destino);
      printf("Nuevo destino asignado: %s (%.2f, %.2f)\n", destino, gx, gy);
    }
  }
  wb_receiver_next_packet(receiver);
}


    // === 2. Si no hay destino, esperar ===
    if (!target_found)
      continue;

    // === 3. Lectura de posición y orientación ===
    const double *g = wb_gps_get_values(gps);
    const double *n = wb_compass_get_values(compass);
    double x = g[0];
    double y = g[1];

    double dx = goal_x - x;
    double dy = goal_y - y;
    double d_goal = dist2d(x, y, goal_x, goal_y);

    printf("Posición → X: %.2f | Y: %.2f | Distancia: %.2f m | Destino: %s\n", x, y, d_goal, current_target);

    // === 4. Si llegó, detenerse ===
    if (d_goal < STOP_DISTANCE) {
      wb_motor_set_velocity(lw, 0);
      wb_motor_set_velocity(rw, 0);
      printf("Meta alcanzada en (X: %.2f, Y: %.2f) — Destino: %s\n", x, y, current_target);
      continue;
    }

    // === 5. Control de orientación ===
    double yaw = atan2(n[0], n[2]);
    double target_yaw = atan2(dy, dx);
    double err = target_yaw - yaw;
    while (err > M_PI) err -= 2 * M_PI;
    while (err < -M_PI) err += 2 * M_PI;
    double turn = KP_ANGLE * err;

    // === 6. Evasión de obstáculos IR ===
    double ww[2] = {0, 0};
    for (int i = 0; i < MAX_SENSOR_NUMBER; i++) {
      double sv = wb_distance_sensor_get_value(sensors[i].tag);
      double mod = 0;
      if (sv > 0) {
        double dist = 5.0 * (1.0 - (sv / MAX_SENSOR_VALUE));
        if (dist < MIN_DISTANCE)
          mod = 1 - (dist / MIN_DISTANCE);
      }
      ww[0] += sensors[i].weight[0] * mod;
      ww[1] += sensors[i].weight[1] * mod;
    }

    // === 7. Movimiento combinado ===
    if (ww[0] > WHEEL_WEIGHT_THRESHOLD) {
      left_speed = 0.8 * MAX_SPEED;
      right_speed = -0.8 * MAX_SPEED;
    } else if (ww[1] > WHEEL_WEIGHT_THRESHOLD) {
      left_speed = -0.8 * MAX_SPEED;
      right_speed = 0.8 * MAX_SPEED;
    } else {
      double base = MAX_SPEED * 0.8;
      left_speed = base - turn;
      right_speed = base + turn;
    }

    // === 8. Limitación de velocidades ===
    if (left_speed >  MAX_SPEED) left_speed =  MAX_SPEED;
    if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
    if (left_speed < -MAX_SPEED) left_speed = -MAX_SPEED;
    if (right_speed < -MAX_SPEED) right_speed = -MAX_SPEED;

    // === 9. LEDs parpadeantes ===
    delay++;
    if (delay == DELAY) {
      wb_led_set(red_led[led_number], 0);
      led_number = (led_number + 1) % 3;
      wb_led_set(red_led[led_number], 1);
      delay = 0;
    }

    // === 10. Aplicar velocidades ===
    wb_motor_set_velocity(lw, left_speed);
    wb_motor_set_velocity(rw, right_speed);
  }

  wb_robot_cleanup();
  return 0;
}