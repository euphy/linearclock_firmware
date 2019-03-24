/*
Linear Clock driver
Copyright Sandy Noble (sandy.noble@gmail.com) 2019

This file has the parts to do with the web server.

The web server will be triggered by a button or a magnet (or something), and that's how
you'll set the time if the RTC loses it.

*/

void http_initServer() {
  httpServerInitialised = false;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid); // add a password here to secure the AP
 
  Serial.print("Linear clock IP address: ");
  Serial.println(WiFi.softAPIP());

  
  server.begin();
  Serial.println("HTTP server started");

  // Set up routes
  server.on("/", HTTP_GET, http_handleRoot);
  server.on("/update", HTTP_GET, http_handleTimeChangePage);
  server.on("/update", HTTP_POST, http_handleTimeChangeSubmit);
  server.onNotFound(http_handleNotFound);
  httpServerInitialised = true;

}


void http_handleTimeChangeSubmit(AsyncWebServerRequest *request) {

  const char* DATE_TIME_INPUT_FIELD = "dateTimeInput";

  // format is dddd-mm-ddThh:mm
  //           0123456789012345 - 16 chars

  //List all parameters
  int params = request->params();
  for (int i=0; i<params; i++) {
    AsyncWebParameter* p = request->getParam(i);
    Serial.print("\nParam name: '");
    Serial.print(p->name());
    Serial.print("'\tValue: '");
    Serial.print(p->value());
    Serial.println("'.");

  }


  if (request->hasParam(DATE_TIME_INPUT_FIELD, true)) {
    Serial.println("Datetime has been submitted: ");
    AsyncWebParameter* p = request->getParam(0);
    String param = p->value().c_str();
    if (param.length() == 16) {
      Serial.println("Size is right.");
      long year = param.substring(0, 4).toInt();
      long month = param.substring(5, 7).toInt();
      long day = param.substring(8, 10).toInt();
      long hour = param.substring(11, 13).toInt();
      long minute = param.substring(14, 16).toInt();

      Serial.print(year);
      Serial.print("...");
      Serial.print(month);
      Serial.print("...");
      Serial.print(day);
      Serial.print("...");
      Serial.print(hour);
      Serial.print("...");
      Serial.print(minute);
      Serial.println("...");

      DateTime dt = DateTime(year, month, day, hour, minute, 0);
      serialPrintTime(dt);

      Serial.println("Before:");
      serialPrintTime(rtc.now());
      rtc.adjust(dt);

      Serial.println("After:");
      serialPrintTime(rtc.now());
      
    }
  }
  else {
    Serial.print("No param found! ");
    Serial.print("Was looking for '");
    Serial.print(DATE_TIME_INPUT_FIELD);
    Serial.println("'");
  }
  
  String message = "Changing time to:\n\n";
  message += "URL: ";
  message += request->url();
  message += "\nMethod: ";
  message += request->method();
  message += "\nArguments: ";
  message += request->params();
  message += "\n";

  //List all parameters
  for(int i=0;i<params;i++){
    AsyncWebParameter* p = request->getParam(i);
    message += "\nParam name: '";
    message += p->name().c_str();
    message += "'\tValue: '";
    message += p->value().c_str();
    message += "'";
  }

  request->redirect("/");  
}

void http_handleTimeChangePage(AsyncWebServerRequest *request) {

  String page = "<html>\
  <head>\
    <title>Realtime clock output</title>\
    <style>\
      body { font-size: 4em;}\
    </style>\
  </head>\
  <body>\
    <h1>Set the time</h1>\
    <form method=\"post\">\
      <label for=\"dateTimeInput\">Date and time</label>\
      <input type=\"datetime-local\" id=\"dateTimeInput\" name=\"dateTimeInput\" />\
      <input type=\"submit\" value=\"submit\" />\
    </form>\
  </body>\
</html>";

  request->send(200, "text/html", page);

}

void http_handleRoot(AsyncWebServerRequest *request) {
 
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
    <a href=\"update\">Change the time</a>\
  </body>\
</html>",

           hour, min, sec, day, month, year
          );
  request->send(200, "text/html", temp);
  
}

void http_handleNotFound(AsyncWebServerRequest *request) {
  String message = "File Not Found\n\n";
  message += "URL: ";
  message += request->url();
  message += "\nMethod: ";
  message += request->method();
  message += "\nArguments: ";
  message += request->params();
  message += "\n";

  //List all parameters
  int params = request->params();
  for(int i=0;i<params;i++){
    AsyncWebParameter* p = request->getParam(i);
    message += "\nParam name: ";
    message += p->name().c_str();
    message += "\tValue: ";
    message += p->value().c_str();
  }

  request->send(404, "text/plain", message);

}
