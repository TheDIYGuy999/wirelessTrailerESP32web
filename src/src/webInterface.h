
#include <Arduino.h>

//
// =======================================================================================================
// WEB INTERFACE
// =======================================================================================================
//

void webInterface()
{

  static unsigned long currentTime = millis(); // Current time
  static unsigned long previousTime = 0;       // Previous time
  const long timeoutTime = 2000;               // Define timeout time in milliseconds (example: 2000ms = 2s)

  static bool Mode = false; // TODO

  if (true)
  { // Wifi on
    // if (WIFI_ON == 1) {     //Wifi on
    WiFiClient client = server.available(); // Listen for incoming clients

    if (client)
    { // If a new client connects,
      currentTime = millis();
      previousTime = currentTime;
      Serial.println("New Client."); // print a message out in the serial port
      String currentLine = "";       // make a String to hold incoming data from the client
      while (client.connected() && currentTime - previousTime <= timeoutTime)
      { // loop while the client's connected
        currentTime = millis();
        if (client.available())
        {                         // if there's bytes to read from the client,
          char c = client.read(); // read a byte, then
          Serial.write(c);        // print it out the serial monitor
          header += c;
          if (c == '\n')
          { // if the byte is a newline character
            // if the current line is blank, you got two newline characters in a row.
            // that's the end of the client HTTP request, so send a response:
            if (currentLine.length() == 0)
            {
              // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
              // and a content-type so the client knows what's coming, then a blank line:
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/html");
              client.println("Connection: close");
              client.println();

              // Display the HTML web page
              client.println("<!DOCTYPE html><html>");
              client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
              client.println("<link rel=\"icon\" href=\"data:,\">");
              client.println("<title>TheDIYGuy999 Wireless Trailer</title>");

#if defined USE_CSS
              // CSS styles for buttons

#if defined MODERN_CSS // The modern CSS with scaling for better adaption between different devices
              client.println("<style>html { font-family: Verdana, Helvetica, sans-serif; display: inline-block; margin: 0px auto; text-align: center; color: #0009ff; background-color: #white;}");

              client.println("h1 {font-size: clamp(1.5rem, 2.5vw, 2.5rem);}");
              client.println("h2 {font-size: clamp(1.3rem, 2.0vw, 2.0rem);}");
              client.println("p {font-size: clamp(1rem, 1.5vw, 1.5rem); color: black; }");
              client.println("a {font-size: clamp(1rem, 1.5vw, 1.5rem); color: black; cursor: pointer; text-decoration: underline;}");

              client.println("input[type=\"checkbox\"] {cursor: pointer; zoom: 1.5;}");

              client.println(".slider { -webkit-appearance: none; width: 95%; height: 25px; background: #d3d3d3; outline: none; border: clamp(0.15rem, 0.15vw, 0.3rem) solid black; margin: 5px; border-radius: 10px;}");
              client.println(".slider::-webkit-slider-thumb { -webkit-appearance: none; cursor: pointer; width: 95%; width: 35px; height: 35px; background: #fff; outline: none; border: clamp(0.15rem, 0.15vw, 0.3rem) solid black; border-radius: 10px;}");

              client.println(".textbox {cursor: pointer; border: clamp(0.15rem, 0.15vw, 0.3rem) solid black; font-size: clamp(1rem, 2vw, 2rem); padding: clamp(0.2rem, 1vw, 1rem); text-align: center; border-radius: 10px;}");
              client.println(".buttonGreen {background-color: #4CAF50; color: white;}");
              client.println(".buttonRed {background-color: #ff0000; color: white;}");
              client.println(".buttonGrey {background-color: #7A7A7A; color: black;}");
              client.println(".button { cursor: pointer; border: clamp(0.15rem, 0.15vw, 0.3rem) solid black; padding: clamp(0.2rem, 1vw, 1rem); margin: clamp(0.2rem, 1vw, 1rem); font-size: clamp(1rem, 2vw, 2rem); width: 95%; border-radius: 10px;");

              client.println("</style></head>");

#else // Old CSS with green background
              client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center; background-color: rgb(60, 161, 120);}");
              client.println(".button { border: yes; color: white; padding: 10px 40px; width: 95%;");
              client.println("text-decoration: none; font-size: 16px; margin: 2px; cursor: pointer;}");
              client.println(".slider { -webkit-appearance: none; width: 95%; height: 25px; background: #d3d3d3; outline: none; opacity: 0.7; -webkit-transition: .2s; transition: opacity .2s; }");
              client.println(".buttonGreen {background-color: #4CAF50;}");
              client.println(".buttonRed {background-color: #ff0000;}");
              client.println(".buttonGrey {background-color: #7A7A7A;}");
              client.println(".textbox {font-size: 16px; text-align: center;}");
              client.println("</style></head>");
#endif
#endif

              // Website title
              client.println("</head><body><h1>TheDIYGuy999 Wireless Trailer</h1>");
              client.printf("<p>Software version %s\n", codeVersion);

#if defined BATTERY_PROTECTION
              client.println("<hr>"); // Horizontal line *******************************************************************************************************
              client.printf("<p>Battery voltage: %.2f V\n", batteryVolts());
              client.printf("<p>Number of cells: %i (%iS battery)\n", numberOfCells, numberOfCells);
              client.printf("<p>Battery cutoff voltage: %.2f V\n", batteryCutoffvoltage);
              client.println("<hr>"); // Horizontal line *******************************************************************************************************
#endif

              // Settings ------------------------------------------------------------------------------------------------------------

              // Set1 (ssid) ----------------------------------
              valueString = ssid;              // Read current value
              client.println("<p>SSID: <br>"); // Display current value

              client.println("<input type=\"text\" id=\"Setting1Input\" size=\"31\" maxlength=\"31\" class=\"textbox\" oninput=\"Setting1change(this.value)\" value=\"" + valueString + "\" /></p>"); // Set new value
              client.println("<script> function Setting1change(pos) { ");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Set1=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Set1=") >= 0)
              {
                pos1 = header.indexOf('=');
                pos2 = header.indexOf('&');
                valueString = header.substring(pos1 + 1, pos2);
                Serial.println(valueString);
                ssid = valueString;
              }

              // Set2 (password) ----------------------------------
              valueString = password;                                // Read current value
              client.println("<p>Password (min. length = 8): <br>"); // Display current value

              client.println("<input type=\"text\" id=\"Setting2Input\" size=\"31\" maxlength=\"31\" class=\"textbox\" oninput=\"Setting2change(this.value)\" value=\"" + valueString + "\" /></p>"); // Set new value
              client.println("<script> function Setting2change(pos) { ");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Set2=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Set2=") >= 0)
              {
                pos1 = header.indexOf('='); // Start pos
                pos2 = header.indexOf('&'); // End pos
                valueString = header.substring(pos1 + 1, pos2);
                password = valueString;
              }

              client.println("<hr>"); // Horizontal line *******************************************************************************************************

              // Checkbox1 (use custom mac) ----------------------------------
              if (useCustomMac == true)
              {
                client.println("<p><input type=\"checkbox\" id=\"tc\" checked onclick=\"Checkbox1Change(this.checked)\"> use custom MAC (save settings & restart required): </input></p>");
              }
              else
              {
                client.println("<p><input type=\"checkbox\" id=\"tc\" unchecked onclick=\"Checkbox1Change(this.checked)\"> use custom MAC (save settings & restart required): </input></p>");
              }
              client.println("<script> function Checkbox1Change(pos) { ");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Checkbox1=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Checkbox1=true") >= 0)
              {
                useCustomMac = true;
                Serial.println("use custom MAC (save to EEPROM & reboot required");
              }
              else if (header.indexOf("GET /?Checkbox1=false") >= 0)
              {
                useCustomMac = false;
                Serial.println("don't use custom MAC (save to EEPROM & reboot required");
              }

              // Set3 (MAC0) ----------------------------------
              valueString = String(customMACAddress[0], HEX); // Read current value
              // client.println("<p>Custom MAC: "); // Display title

              client.println("<input type=\"text\" style=\"text-transform: uppercase\" size=\"2\" maxlength=\"2\" class=\"textbox\" oninput=\"Setting3change(this.value)\" value=\"" + valueString + "\" />"); // Set new value (no </p> = no new line)
              client.println("<script> function Setting3change(pos) { ");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Set3=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Set3=") >= 0)
              {
                pos1 = header.indexOf('=');
                pos2 = header.indexOf('&');
                valueString = header.substring(pos1 + 1, pos2);
                customMACAddress[0] = strtol(valueString.c_str(), NULL, 16);
              }

              // Set4 (MAC1) ----------------------------------
              valueString = String(customMACAddress[1], HEX); // Read current value
              client.println(":");                            // Display title

              client.println("<input type=\"text\" style=\"text-transform: uppercase\" size=\"2\" maxlength=\"2\" class=\"textbox\" oninput=\"Setting4change(this.value)\" value=\"" + valueString + "\" />"); // Set new value (no </p> = no new line)
              client.println("<script> function Setting4change(pos) { ");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Set4=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Set4=") >= 0)
              {
                pos1 = header.indexOf('=');
                pos2 = header.indexOf('&');
                valueString = header.substring(pos1 + 1, pos2);
                customMACAddress[1] = strtol(valueString.c_str(), NULL, 16);
              }

              // Set5 (MAC2) ----------------------------------
              valueString = String(customMACAddress[2], HEX); // Read current value
              client.println(":");                            // Display title

              client.println("<input type=\"text\" style=\"text-transform: uppercase\" size=\"2\" maxlength=\"2\" class=\"textbox\" oninput=\"Setting5change(this.value)\" value=\"" + valueString + "\" />"); // Set new value (no </p> = no new line)
              client.println("<script> function Setting5change(pos) { ");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Set5=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Set5=") >= 0)
              {
                pos1 = header.indexOf('=');
                pos2 = header.indexOf('&');
                valueString = header.substring(pos1 + 1, pos2);
                customMACAddress[2] = strtol(valueString.c_str(), NULL, 16);
              }

              // Set6 (MAC3) ----------------------------------
              valueString = String(customMACAddress[3], HEX); // Read current value
              client.println(":");                            // Display title

              client.println("<input type=\"text\" style=\"text-transform: uppercase\" size=\"2\" maxlength=\"2\" class=\"textbox\" oninput=\"Setting6change(this.value)\" value=\"" + valueString + "\" />"); // Set new value (no </p> = no new line)
              client.println("<script> function Setting6change(pos) { ");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Set6=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Set6=") >= 0)
              {
                pos1 = header.indexOf('=');
                pos2 = header.indexOf('&');
                valueString = header.substring(pos1 + 1, pos2);
                customMACAddress[3] = strtol(valueString.c_str(), NULL, 16);
              }

              // Set7 (MAC4) ----------------------------------
              valueString = String(customMACAddress[4], HEX); // Read current value
              client.println(":");                            // Display title

              client.println("<input type=\"text\" style=\"text-transform: uppercase\" size=\"2\" maxlength=\"2\" class=\"textbox\" oninput=\"Setting7change(this.value)\" value=\"" + valueString + "\" />"); // Set new value (no </p> = no new line)
              client.println("<script> function Setting7change(pos) { ");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Set7=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Set7=") >= 0)
              {
                pos1 = header.indexOf('=');
                pos2 = header.indexOf('&');
                valueString = header.substring(pos1 + 1, pos2);
                customMACAddress[4] = strtol(valueString.c_str(), NULL, 16);
              }

              // Set8 (MAC5) ----------------------------------
              valueString = String(customMACAddress[5], HEX); // Read current value
              client.println(":");                            // Display title

              client.println("<input type=\"text\" style=\"text-transform: uppercase\" size=\"2\" maxlength=\"2\" class=\"textbox\" oninput=\"Setting8change(this.value)\" value=\"" + valueString + "\" /></p>"); // Set new value
              client.println("<script> function Setting8change(pos) { ");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Set8=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Set8=") >= 0)
              {
                pos1 = header.indexOf('=');
                pos2 = header.indexOf('&');
                valueString = header.substring(pos1 + 1, pos2);
                customMACAddress[5] = strtol(valueString.c_str(), NULL, 16);
              }

              client.printf("<p>Use HEX values (0-9, A-F) only, always starting with FE");

              // Currently uses MAC address info ----------------------------------
              client.print("<p>Currently used MAC address: ");
              client.println(WiFi.macAddress());

              client.println("<hr>"); // Horizontal line

              // Checkbox2 (5th wheel setting) ----------------------------------
              if (fifthWhweelDetectionActive == true)
              {
                client.println("<p><input type=\"checkbox\" id=\"tc\" checked onclick=\"Checkbox2Change(this.checked)\"> 5th wheel detection switch active = lights off, if not coupled </input></p>");
              }
              else
              {
                client.println("<p><input type=\"checkbox\" id=\"tc\" unchecked onclick=\"Checkbox2Change(this.checked)\"> 5th wheel detection switch active = lights off, if not coupled </input></p>");
              }
              client.println("<script> function Checkbox2Change(pos) { ");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Checkbox2=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Checkbox2=true") >= 0)
              {
                fifthWhweelDetectionActive = true;
                Serial.println("5th wheel detection switch active");
              }
              else if (header.indexOf("GET /?Checkbox2=false") >= 0)
              {
                fifthWhweelDetectionActive = false;
                Serial.println("5th wheel detection switch inactive");
              }

              // Slider1 (taillight brightness) ----------------------------------
              valueString = String(tailLightBrightness, DEC);
              client.println("<p>Taillight brightness % : <span id=\"textSlider1Value\">" + valueString + "</span><br>"); // Label
              client.println("<input type=\"range\" min=\"5\" max=\"100\" step=\"5\" class=\"slider\" id=\"Slider1Input\" onchange=\"Slider1Change(this.value)\" value=\"" + valueString + "\" /></p>");
              client.println("<script> function Slider1Change(pos) { ");
              client.println("var slider1Value = document.getElementById(\"Slider1Input\").value;");
              client.println("document.getElementById(\"textSlider1Value\").innerHTML = slider1Value;");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Slider1=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Slider1=") >= 0)
              {
                pos1 = header.indexOf('=');
                pos2 = header.indexOf('&');
                valueString = header.substring(pos1 + 1, pos2);
                tailLightBrightness = (valueString.toInt());
              }

              // Slider2 (sidelight brightness) ----------------------------------
              valueString = String(sideLightBrightness, DEC);
              client.println("<p>Sidelight brightness % : <span id=\"textSlider2Value\">" + valueString + "</span><br>");
              client.println("<input type=\"range\" min=\"5\" max=\"100\" step=\"5\" class=\"slider\" id=\"Slider2Input\" onchange=\"Slider2Change(this.value)\" value=\"" + valueString + "\" /></p>");
              client.println("<script> function Slider2Change(pos) { ");
              client.println("var slider2Value = document.getElementById(\"Slider2Input\").value;");
              client.println("document.getElementById(\"textSlider2Value\").innerHTML = slider2Value;");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Slider2=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Slider2=") >= 0)
              {
                pos1 = header.indexOf('=');
                pos2 = header.indexOf('&');
                valueString = header.substring(pos1 + 1, pos2);
                sideLightBrightness = (valueString.toInt());
              }

              // Slider3 (reversing light brightness) ----------------------------------
              valueString = String(reversingLightBrightness, DEC);
              client.println("<p>Reversing light brightness % : <span id=\"textSlider3Value\">" + valueString + "</span><br>");
              client.println("<input type=\"range\" min=\"5\" max=\"100\" step=\"5\" class=\"slider\" id=\"Slider3Input\" onchange=\"Slider3Change(this.value)\" value=\"" + valueString + "\" /></p>");
              client.println("<script> function Slider3Change(pos) { ");
              client.println("var slider3Value = document.getElementById(\"Slider3Input\").value;");
              client.println("document.getElementById(\"textSlider3Value\").innerHTML = slider3Value;");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Slider3=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Slider3=") >= 0)
              {
                pos1 = header.indexOf('=');
                pos2 = header.indexOf('&');
                valueString = header.substring(pos1 + 1, pos2);
                reversingLightBrightness = (valueString.toInt());
              }

              // Slider4 (indicator light brightness) ----------------------------------
              valueString = String(indicatorLightBrightness, DEC);
              client.println("<p>Indicator light brightness % : <span id=\"textSlider4Value\">" + valueString + "</span><br>");
              client.println("<input type=\"range\" min=\"5\" max=\"100\" step=\"5\" class=\"slider\" id=\"Slider4Input\" onchange=\"Slider4Change(this.value)\" value=\"" + valueString + "\" /></p>");
              client.println("<script> function Slider4Change(pos) { ");
              client.println("var slider4Value = document.getElementById(\"Slider4Input\").value;");
              client.println("document.getElementById(\"textSlider4Value\").innerHTML = slider4Value;");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?Slider4=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?Slider4=") >= 0)
              {
                pos1 = header.indexOf('=');
                pos2 = header.indexOf('&');
                valueString = header.substring(pos1 + 1, pos2);
                indicatorLightBrightness = (valueString.toInt());
              }

              client.println("<hr>"); // Horizontal line *******************************************************************************************************

              // button1 (Save settings to EEPROM) ----------------------------------
              client.println("<p><a href=\"/save/on\"><button class=\"button buttonRed\" onclick=\"restartPopup()\" >Save settings & restart</button></a></p>");

              if (header.indexOf("GET /save/on") >= 0)
              {
                eepromWrite();
                delay(1000);
                ESP.restart();
              }
              client.println("<script> function restartPopup() {");
              client.println("alert(\"Controller restarted, you may need to reconnect WiFi!\"); ");
              client.println("} </script>");

              client.println("<p>It is recommended to power cycle the controller as well as to close and reopen the browser window after using this button!</p>");

              client.println("<hr>"); // Horizontal line *******************************************************************************************************

              client.println("<br>More informations on my <a href=\"https://thediyguy999.github.io/TheDIYGuy999_ESP32_Web_Flasher/index.html\" target=\"_blank\">Website</a><br>");

              /*
                                                    // button2 (Legs up) ----------------------------------
                                                    client.println("<p><a href=\"/legs/up\"><button class=\"button buttonGrey\" >Legs up</button></a></p>");

                                                    if (header.indexOf("GET /legs/up") >= 0)
                                                    {
                                                      Serial.println("Legs up");
                                                    }

                                                                  // button3 (Legs down) ----------------------------------
                                                                  client.println("<p><a href=\"/legs/down\"><button class=\"button buttonGrey\" >Legs down</button></a></p>");

                                                                  if (header.indexOf("GET /legs/down") >= 0)
                                                                  {
                                                                    Serial.println("Legs down");
                                                                  }

                                                                  // button4 (test) ----------------------------------
                                                                  client.println("<p><button class=\"button buttonGrey\" onclick=\"buttonTestPressed(this)\" >Test button</button></p>");
                                                                  client.println("<p id=\"demo\"></p>");
                                                                  */
              /*
                            client.println("<script> function buttonTestPressed() { ");
                              //document.getElementById("demo").innerHTML = "Hello World";
                            client.println("var slider3Value = document.getElementById(\"demo\").value;");
                            client.println("} </script>");
              */
              /*
                            client.print(F("<a href='/datastart' target='DataBox'><button type='button'>Agg.Auto</button></a>"));
              */
              client.println("<script> function buttonTestPressed(pos) { ");
              client.println("var xhr = new XMLHttpRequest();");
              client.println("xhr.open('GET', \"/?testButton=\" + pos + \"&\", true);");
              client.println("xhr.send(); } </script>");

              if (header.indexOf("GET /?testButton=true") >= 0)
              {
                // fifthWhweelDetectionActive = true;
                Serial.println("Test pressed");
              }
              else if (header.indexOf("GET /?testButton=false") >= 0)
              {
                // fifthWhweelDetectionActive = false;
                Serial.println("Test not pressed");
              }

              //-----------------------------------------------------------------------------------------------------------------------
              client.println("</body></html>");

              // The HTTP-response ends with an empty column
              client.println();
              // Break out of the while loop
              break;
            }
            else
            { // if you got a newline, then clear currentLine
              currentLine = "";
            }
          }
          else if (c != '\r')
          {                   // if you got anything else but a carriage return character,
            currentLine += c; // add it to the end of the currentLine
          }
        }
      }
      // Clear header
      header = "";
      // Disconnect client
      client.stop();
      Serial.println("Client disconnected.");

      Serial.print("5th wheel detection active: ");
      Serial.println(fifthWhweelDetectionActive);
      Serial.println("");
    }
  }
}