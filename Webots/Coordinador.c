/*
 * ============================================================
 *  SUPERVISOR
 * ============================================================
 * 
 * FUNCIÓN PRINCIPAL:
 * Este supervisor se encarga de coordinar un conjunto de robots
 * Pioneer 3-DX según la posición de balizas activas en el entorno.
 * ============================================================
 */

#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
// PARÁMETROS GLOBALES DE CONFIGURACIÓN
#define TIME_STEP      32          // paso de simulación en ms
#define CHANNEL_EMIT    7           // canal de salida hacia robots
#define CHANNEL_RECV    8           // canal de entrada desde balizas
#define EPSILON         9.5         // radio máximo para agrupar balizas (m)
#define REASSIGN_PERIOD 10.0        // tiempo entre reasignaciones en s

// Límites de datos (Por si en algún momento se le quieren agregar más agentes)
#define MAX_BEACONS     50
#define MAX_CLUSTERS    20
#define MAX_ROBOTS      32

// Almacenamiento de balizas
typedef struct {
  double x, y;          // posición en el plano
  char name[32];        // nombre de la baliza (En este caso BEACONn)
} Beacon;

// FUNCIÓN para calcular la distancia entre dos puntos en plano 2D
// ============================================================
static double dist2(double x1, double y1, double x2, double y2) {
  double dx = x1 - x2, dy = y1 - y2;
  return sqrt(dx * dx + dy * dy);
}

// FUNCIÓN PRINCIPAL DEL SUPERVISOR
int main() {
  wb_robot_init();  // inicializa el controlador Supervisor

  // Dispositivos de comunicación 
  WbDeviceTag emitter  = wb_robot_get_device("emitter");   // canal hacia los robots
  WbDeviceTag receiver = wb_robot_get_device("receiver");  // canal desde las balizas
  wb_receiver_enable(receiver, TIME_STEP);

  // Lista de robots a disposición
  const char *robots[MAX_ROBOTS] = {
    "PIONEER1","PIONEER2","PIONEER3","PIONEER4","PIONEER5",
    "PIONEER6","PIONEER7","PIONEER8","PIONEER9","PIONEER10",
    "PIONEER11","PIONEER12","PIONEER13"
  };
  int n_robots = 13;   // cantidad activa de robots en la simulación

  printf("Supervisor iniciado — escuchando balizas dinámicamente...\n");

  double time_elapsed = 0.0;   // reloj interno en segundos

  // BUCLE PRINCIPAL DEL SUPERVISOR
  // ============================================================
  while (wb_robot_step(TIME_STEP) != -1) {

    // Contador de tiempo 
    time_elapsed += TIME_STEP / 1000.0;

    // Solo recalcula asignaciones cada REASSIGN_PERIOD segundos (en este caso 10s)
    static double last_assign = -REASSIGN_PERIOD;
    if (time_elapsed - last_assign < REASSIGN_PERIOD)
      continue;
    last_assign = time_elapsed;

    // RECOLECTAR BALIZAS ACTIVAS DEL CANAL
    Beacon beacons[MAX_BEACONS];
    int n_beacons = 0;

    while (wb_receiver_get_queue_length(receiver) > 0) {
      const char *msg = wb_receiver_get_data(receiver);

      char tag[16], name[32];
      double x, y;

      // Identifica las balizas con nombre y ubicacion xy
      if (sscanf(msg, "%15[^,],%31[^,],%lf,%lf", tag, name, &x, &y) == 4 &&
          strcmp(tag, "BALIZA") == 0) {

        // Verifica si la baliza ya estaba registrada y/o (actualiza posición)
        int found = 0;
        for (int i = 0; i < n_beacons; i++) {
          if (strcmp(beacons[i].name, name) == 0) {
            beacons[i].x = x;
            beacons[i].y = y;
            found = 1;
            break;
          }
        }

        // Si es nueva, la agrega a la lista
        if (!found && n_beacons < MAX_BEACONS) {
          snprintf(beacons[n_beacons].name,
                   sizeof(beacons[n_beacons].name),
                   "%s", name);
          beacons[n_beacons].x = x;
          beacons[n_beacons].y = y;
          n_beacons++;
        }
      }

      // Libera el mensaje 
      wb_receiver_next_packet(receiver);
    }

    // Si no hay balizas, no se hace nada 
    if (n_beacons == 0)
      continue;

    printf("\nReasignación @ t = %.1f s (balizas = %d)\n", time_elapsed, n_beacons);

    // AGRUPAR BALIZAS EN CLUSTERS (DBSCAN)
    // ============================================================
    int cluster_id[MAX_BEACONS];
    for (int i = 0; i < n_beacons; i++)
      cluster_id[i] = -1; // -1 = sin asignar

    int n_clusters = 0;
    for (int i = 0; i < n_beacons; i++) {
      if (cluster_id[i] != -1) continue;   // ya pertenece a un cluster
      cluster_id[i] = n_clusters;

      // Busca otras balizas cercanas a la actual
      for (int j = i + 1; j < n_beacons; j++) {
        if (cluster_id[j] == -1 &&
            dist2(beacons[i].x, beacons[i].y, beacons[j].x, beacons[j].y) < EPSILON)
          cluster_id[j] = n_clusters;  // misma agrupación
      }

      n_clusters++;
      if (n_clusters >= MAX_CLUSTERS) break;  // límite de seguridad
    }

    // ============================================================
    // 3️⃣ CALCULAR CENTROIDES DE CADA CLUSTER
    // ============================================================
    double cx[MAX_CLUSTERS] = {0}, cy[MAX_CLUSTERS] = {0};
    int balizas_por_cluster[MAX_CLUSTERS] = {0};

    // Suma coordenadas por cluster
    for (int i = 0; i < n_beacons; i++) {
      int c = cluster_id[i];
      cx[c] += beacons[i].x;
      cy[c] += beacons[i].y;
      balizas_por_cluster[c]++;
    }

    // Divide entre la cantidad de balizas → centroide promedio
    for (int c = 0; c < n_clusters; c++) {
      cx[c] /= (balizas_por_cluster[c] > 0 ? balizas_por_cluster[c] : 1);
      cy[c] /= (balizas_por_cluster[c] > 0 ? balizas_por_cluster[c] : 1);
      printf("Cluster %d: %d balizas → centroide (%.2f, %.2f)\n",
             c, balizas_por_cluster[c], cx[c], cy[c]);
    }

    // ============================================================
    // ASIGNACIÓN PROPORCIONAL DE ROBOTS 
    // ============================================================
    // Cada cluster recibe robots en proporción al # de balizas que contiene.
    int total_balizas = 0;
    for (int c = 0; c < n_clusters; c++)
      total_balizas += balizas_por_cluster[c];

    int robots_por_cluster[MAX_CLUSTERS] = {0};
    double residuos[MAX_CLUSTERS] = {0.0};
    int suma_entera = 0;

    if (total_balizas > 0) {
      // Cuota fraccionaria de robots por cluster
      for (int c = 0; c < n_clusters; c++) {
        double cuota = ((double)balizas_por_cluster[c] / total_balizas) * n_robots;
        int ent = (int)floor(cuota);   // parte entera
        robots_por_cluster[c] = ent;
        residuos[c] = cuota - ent;     // parte decimal
        suma_entera += ent;
      }

      // Asignar los robots restantes (por redondeo)
      int restantes = n_robots - suma_entera;
      for (int k = 0; k < restantes; k++) {
        int best = -1; double best_res = -1.0;
        for (int c = 0; c < n_clusters; c++) {
          if (residuos[c] > best_res) { best_res = residuos[c]; best = c; }
        }
        if (best >= 0) {
          robots_por_cluster[best]++;
          residuos[best] = -1.0;  // evita volver a elegirlo
        }
      }
    } else {
      // Caso excepcional: repartir equitativamente
      for (int c = 0; c < n_clusters; c++)
        robots_por_cluster[c] = n_robots / n_clusters;
    }

    // Reporte de asignación
    for (int c = 0; c < n_clusters; c++)
      printf("Cluster %d: %d robots asignados\n", c, robots_por_cluster[c]);

    // ============================================================
    // 5️⃣ ENVÍO DE DESTINOS A LOS ROBOTS
    // ============================================================
    int robot_idx = 0;  // índice de robot actual

    for (int c = 0; c < n_clusters; c++) {
      for (int r = 0; r < robots_por_cluster[c]; r++) {
        if (robot_idx >= n_robots) break;  // sin más robots

        // Construye el mensaje para el robot
        char msg[128];
        sprintf(msg, "DESTINO,%s,CLUSTER%d,%.2f,%.2f",
                robots[robot_idx], c, cx[c], cy[c]);

        // Envía el mensaje por el emisor (canal 7)
        wb_emitter_send(emitter, msg, strlen(msg) + 1);

        printf("📡 %s → CLUSTER%d (%.2f, %.2f)\n",
               robots[robot_idx], c, cx[c], cy[c]);
        robot_idx++;
      }
    }
  }

  // ============================================================
  // 🧹 LIMPIEZA Y SALIDA
  // ============================================================
  wb_robot_cleanup();
  return 0;
}
