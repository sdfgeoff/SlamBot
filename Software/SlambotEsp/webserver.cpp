#include <WebServer.h>
#include "events.h"

WebServer server(80);

void handleRoot() {
  char message[200];
  sprintf(
    message,
    "{\"t\":%d,\"rx\":%f,\"ry\":%f,\"rz\":%f,\"ax\":%f,\"ay\":%f,\"az\":%f,\"dl\":%d,\"dr\":%d}",
    latest_time,
    latest_gyro.gyro.x,
    latest_gyro.gyro.y,
    latest_gyro.gyro.z,
    latest_accelerometer.acceleration.x,
    latest_accelerometer.acceleration.y,
    latest_accelerometer.acceleration.z,
    latest_lidar_left,
    latest_lidar_right
  );
  server.send(200, "application/json", message);
}

void handleControl() {
    

  int speed = 0;
  int direction = 0;
  for (uint8_t i = 0; i < server.args(); i++) {
    String argname = server.argName(i);
    String argval = server.arg(i);
    if (argname == "speed") {
      speed = argval.toInt();
    } else if (argname == "direction") {
      direction = argval.toInt();
    }
  }

  latest_control_speed = speed;
  latest_control_direction = direction;
  latest_control_time = millis();

  char message[200];
  sprintf(message, "Speed: %d, direction: %d", speed, direction);
  server.send(200, "application/json", message);
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  message += "\nRangeHeader: ";
  message += server.header("Range");
  server.send(404, "text/plain", message);
}


void init_webserver() {
  server.on("/", handleRoot);
  server.on("/control", handleControl);
  server.onNotFound(handleNotFound);
  server.begin();
}

void poll_webserver() {
  server.handleClient();
}