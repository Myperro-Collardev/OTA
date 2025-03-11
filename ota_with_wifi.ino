#include <WiFi.h>
#include <ArduinoOTA.h>

String ssid = "nagesh____";
String password = "rahul12345";
// Create WiFiServer object
WiFiServer server(8080);

// Function to connect to the WiFi network
void connect_to_wifi(const char* ssid, const char* password)
{
    // Begin WiFi connection with provided SSID and password
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");

    // Wait for the connection to establish
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(200);
        Serial.print(".");
    }

    // WiFi connected, print the IP address
    Serial.println();
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Start the server on port 8080
    server.begin();
}
String receive_from_client(WiFiClient &client)
{
    // Check if the client has sent any data
    if (client.available())
    {
        // Read the client's request
        String request = client.readStringUntil('\n');
        request.trim(); // Remove any leading/trailing whitespace
        client.flush(); // Clear the input buffer

        // Convert the request to uppercase
        request.toUpperCase();

        // Send back the uppercase request as confirmation
        client.print("sent: ");
        client.println(request);

        // If the request is "CLOSE", disconnect the client
        if (request == "CLOSE")
        { // Case insensitive check after conversion
            client.stop();
            Serial.println("Client disconnected");
            return "client disconnected";
        }
        

        // Return the uppercase request
        return request;
    }
    // Return an empty string if no data is available
    return "";
}

void setup() {
    Serial.begin(115200);

    // Connect to WiFi
    // WiFi.begin(ssid, password);
    // while (WiFi.status() != WL_CONNECTED) {
    //     delay(500);
    //     Serial.print(".");
    // }
    // Serial.println("\nWiFi connected!");
    connect_to_wifi(ssid.c_str(), password.c_str());
    Serial.println("ESP Started");

    // OTA Setup
    ArduinoOTA.setHostname("ESP32-OTA");  // Optional: Set the OTA device name
    ArduinoOTA.setPassword("1234");  // Optional: Set OTA password

    ArduinoOTA.onStart([]() {
        Serial.println("Start updating...");
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("\nEnd");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();
    Serial.println("OTA Ready");
}

void loop() {
    ArduinoOTA.handle();  // Always call this in the loop
    for(int i=0;i<10;i++){
      Serial.println(i);
      delay(100);
    }
    Serial.println("Cycle Completed");
    delay(1000);
}
