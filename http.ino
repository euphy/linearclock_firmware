/*
Linear Clock driver
Copyright Sandy Noble (sandy.noble@gmail.com) 2019

This file has the parts to do with the web server.

The web server will be triggered by a button or a magnet (or something), and that's how
you'll set the time if the RTC loses it.

*/

void http_initServer() {
  httpServerInitialised = false;
  WiFi.mode(WIFI_AP);           //Only Access point
  WiFi.softAP(ssid);  //Start HOTspot removing password will disable security
 
  IPAddress myIP = WiFi.softAPIP(); //Get IP address
  Serial.print("Linear clock IP address: ");
  Serial.println(myIP);
 
  server.on("/", http_handleRoot);      //Which routine to handle at root location
 
  server.begin();                  //Start server
  Serial.println("HTTP server started");

  server.on("/", http_handleRoot);
  server.on("/test.svg", http_drawGraph);
  server.on("/inline", []() {
    server.send(200, "text/plain", "this works as well");
  });
  server.onNotFound(http_handleNotFound);
  httpServerInitialised = true;

}

void http_handleClient() {
  if (httpServerInitialised) {
    server.handleClient();
  }
}


void http_handleRoot() {
  char temp[600];
  DateTime now = rtc.now();
  
  int sec = now.second();
  int min = now.minute();
  int hour = now.hour();

  int day = now.day();
  int month = now.month();
  int year = now.year();

  snprintf(temp, 600,

           "<html>\
  <head>\
    <meta http-equiv='refresh' content='5'/>\
    <title>Realtime clock output</title>\
    <style>\
      body { font-size: 4em;}\
    </style>\
  </head>\
  <body>\
    <h1>Real time clock says:</h1>\
    <p>%02d:%02d:%02d</p>\
    <p>%02d/%02d/%04d</p>\
    <img src=\"/test.svg\" />\
    <form method=\"post\">\
      <input type=\"datetime-local\" name=\"dateTimeInput\" />\
      <input type=\"submit\" value=\"submit\" />\
    </form>\
  </body>\
</html>",

           hour, min, sec, day, month, year
          );
  server.send(200, "text/html", temp);
  
}

void http_handleNotFound() {
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

  server.send(404, "text/plain", message);

}


void http_drawGraph() {
  
  String out = "";
  char temp[100];
  out += "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\"400\" height=\"150\">\n";
  out += "<rect width=\"400\" height=\"150\" fill=\"rgb(250, 230, 210)\" stroke-width=\"1\" stroke=\"rgb(0, 0, 0)\" />\n";
  out += "<g stroke=\"black\">\n";
  int y = rand() % 130;
  for (int x = 10; x < 390; x += 10) {
    int y2 = rand() % 130;
    sprintf(temp, "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke-width=\"1\" />\n", x, 140 - y, x + 10, 140 - y2);
    out += temp;
    y = y2;
  }
  out += "</g>\n</svg>\n";

  server.send(200, "image/svg+xml", out);
}
