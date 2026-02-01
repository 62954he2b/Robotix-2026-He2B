#include "access_point.h"

const char* ssid     = "ESP32-Access-Point";
const char* password = "123456789";

bool page_loaded = false;
String input_message = "";

AsyncWebServer server(80);

// HTML web page to handle 3 input fields (inputString, inputInt, inputFloat)
const char index_html[] PROGMEM = R"rawliteral(
	<!DOCTYPE HTML><html><head>
	  <title> Robot Motion Control </title>
	  <meta name="viewport" content="width=device-width, initial-scale=1">
	  <script>
		function submitMessage() {
		  setTimeout(function(){ document.location.reload(false); }, 100);   
		}
	  </script></head><body>
	  <form action="/get" target="hidden-form">
		Commande : <input type="text" name="inputString">
		<input type="submit" value="Submit" onclick="submitMessage()">
	  </form><br>
	  <iframe style="display:none" name="hidden-form"></iframe>
	</body></html>)rawliteral";

const char* PARAM_STRING = "inputString";


void initiate_access_point(const char* ssid, const char* password) {
	WiFi.softAP(ssid);

	IPAddress IP = WiFi.softAPIP();
	Serial.print("AP IP address: ");
	Serial.println(IP);

	// Send web page with input fields to client
	server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
	request->send_P(200, "text/html", index_html);
	});

	server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
		// GET inputString value from ESP32 WebServer
		if (request->hasParam(PARAM_STRING)) {
			input_message = request->getParam(PARAM_STRING)->value();
			page_loaded = true;
		}
		else {
		input_message = "No message sent";
		}
		Serial.println(input_message);
		request->send(200, "text/text", input_message);
	});

	server.begin();
}

