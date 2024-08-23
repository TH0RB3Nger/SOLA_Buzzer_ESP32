#include <Arduino.h>
#include <ArduinoLog.h>
#include <WiFi.h>
#include <HTTPClient.h>

// ##########################################################################################
// ##                                KONFIGURATION                                         ##
// ##########################################################################################
// WLAN
const char *SSID = "Sola";             /** SSID des WLAN-Netzwerks */
const char *password = "Johannes3,16"; /** Passwort des WLAN-Netzwerks */
const int MaxConnectTime = 30;         /** Zeit in Sekunden bis zum Timeout */
// HTTP
const char *serverName = "http://192.168.10.104:8000/press/bank/2/32"; /** URL an die das HTTP-Request gesendet wird */
// Pin Mode
const int buttonPin = 5;            /** Pin, an dem der Taster angeschlossen ist */
const int PinMode = INPUT_PULLDOWN; /** Verwendeter Pin Mode der Ardroinos */
// Debounce
const unsigned long debounceDelay = 50; /** Entprellzeit in Millisekunden */
// Verbindungsversuche
const int maxReconnectAttempts = 20;                     /** Maximale Anzahl von Wiederverbindungsversuchen */
const unsigned long reconnectTimeWindow = 5 * 60 * 1000; /** Zeitfenster für die max. Versuche in Millisekunden (5 Minuten) */
// Stromsparmodus
const unsigned long deepSleepDuration = 5 * 60 * 1000; /** Dauer des Tiefschlafs in Millisekunden (5 Minuten) */

// ##########################################################################################
// ##                                 Declaration                                          ##
// ##########################################################################################
// Variablen zur Überwachung des Tastenzustands initialisieren
int buttonState = 0;                                /** Aktueller Status des Buttons */
int lastButtonState = 0;                            /** Status des Buttons aus dem letzten Zyklus */
unsigned long lastDebounceTime = 0;                 /** Zeitpunkt des letzten Zustandswechsels */
unsigned long firstReconnectAttemptTime = 0;        /** Startzeitpunkt des ersten Wiederverbindungsversuchs */
int reconnectAttempts = 0;                          /** Zähler für die Wiederverbindungsversuche */
unsigned long lastRSSIOutput = 0;                   /** Zeitpunkt der letzten RSSI-Ausgabe */
const unsigned long rssiOutputInterval = 30 * 1000; // Intervall für RSSI-Ausgabe (30 Sekunden)
#define GET_VARIABLE_NAME(Variable) (#Variable)     /** Gibt den Namen der variable zurück */

//*********************************************************************
/**
 * @brief Baut eine Verbindung zu einem bestimmten WLAN auf
 *
 * @param SSID SSID des WLAN-Netzwerks
 * @param Password Passwort zur Authentifizierung
 * @param Timeout Zeit in Sekunden zum Herstellen der Verbindung
 * @return true Verbindung erfolgreich
 * @return false Verbindung nicht erfolgreich
 */
bool ConnectWlan(const char *SSID, const char *Password, int Timeout);
/**
 * @brief Returns char* of the wl_status integer
 *
 * @param status Status to convert
 * @return const char*
 */
const char *wl_status_to_string(wl_status_t status);
/**
 * @brief Get called when Wifi is disconnectet from Station
 *
 * @param event Event ID
 * @param info Event Info
 */
void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info);
/**
 * @brief get called when Wifi connection is getting a IP Adress
 *
 * @param event Event ID
 * @param info Event Info
 */
void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info);
/**
 * @brief Get called when Wifi is connected to Station
 *
 * @param event Event ID
 * @param info Event Info
 */
void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info);
/**
 * @brief converts the disconnect Reason to String
 *
 * @param reason Reason ID
 * @return const char*
 */
const char *wifi_sta_disconnected_reason_to_string(uint8_t reason);
//*********************************************************************

// ##########################################################################################
// ##                                    SETUP                                             ##
// ##########################################################################################
/**
 * @brief Setup the Programm
 * Its the first Entry and executes before the loop part below
 */
void setup()
{
  // Start Logging
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  // Serielle Kommunikation starten
  Serial.begin(115200);

  // Taster als Eingang definieren
  pinMode(buttonPin, INPUT_PULLDOWN);
  Log.verboseln("Pinmode of Pin %d Set to %s", buttonPin, GET_VARIABLE_NAME(INPUT_PULLDOWN));

  // WLAN-Event Handling
  WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  // WLAN-Verbindung herstellen
  ConnectWlan(SSID, password, MaxConnectTime);
}

// ##########################################################################################
// ##                                     LOOP                                             ##
// ##########################################################################################

/**
 * @brief Main Programm Cycle
 * Main Programm Cycle to handle the Button Press.
 * This is Necessary for the Arduino
 * its the Main entry part after the Setup
 */
void loop()
{
  // Aktuellen Zustand des Tasters lesen
  int reading = digitalRead(buttonPin);

  // Entprellen des Tasters
  if (reading != lastButtonState)
  {
    lastDebounceTime = millis(); // Zeitpunkt des letzten Wechsels speichern
  }

  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    // Wenn der aktuelle Zustand sich von dem letzten stabilen Zustand unterscheidet
    if (reading != buttonState)
    {
      buttonState = reading;

      // Nur auf den steigenden Flankenwechsel reagieren
      if (buttonState == HIGH)
      {
        Serial.println("Rising Edge erkannt! Taster gedrückt!");

        // HTTP-Request senden
        if (WiFi.status() == WL_CONNECTED)
        {
          HTTPClient http;
          http.begin(serverName);            // URL spezifizieren
          int httpResponseCode = http.GET(); // GET-Request senden

          if (httpResponseCode > 0)
          {
            String response = http.getString(); // Antwort speichern
            Serial.println(httpResponseCode);   // HTTP-Antwortcode ausgeben
            Serial.println(response);           // Antwortinhalt ausgeben
          }
          else
          {
            Serial.print("Fehler beim Senden der GET-Anfrage: ");
            Serial.println(httpResponseCode);
          }
          http.end(); // Verbindung schließen
        }
        else
        {
          Serial.println("WLAN-Verbindung verloren.");
        }
      }
    }
  }

  // RSSI-Wert regelmäßig ausgeben
  if (millis() - lastRSSIOutput >= rssiOutputInterval)
  {
    int32_t rssi = WiFi.RSSI();
    Serial.printf("RSSI: %d dBm\n", rssi);
    lastRSSIOutput = millis();
  }

  // Letzten Tastenzustand aktualisieren
  lastButtonState = reading;
}

// ##########################################################################################
// ##                                   FUNKTIONEN                                         ##
// ##########################################################################################

const char *wl_status_to_string(wl_status_t status)
{
  switch (status)
  {
  case WL_NO_SHIELD:
    return "WL_NO_SHIELD";
  case WL_IDLE_STATUS:
    return "WL_IDLE_STATUS";
  case WL_NO_SSID_AVAIL:
    return "WL_NO_SSID_AVAIL";
  case WL_SCAN_COMPLETED:
    return "WL_SCAN_COMPLETED";
  case WL_CONNECTED:
    return "WL_CONNECTED";
  case WL_CONNECT_FAILED:
    return "WL_CONNECT_FAILED";
  case WL_CONNECTION_LOST:
    return "WL_CONNECTION_LOST";
  case WL_DISCONNECTED:
    return "WL_DISCONNECTED";
  default:
    return "No Status";
  }
}

bool ConnectWlan(const char *SSID, const char *Password, int Timeout)
{
  unsigned long startAttemptTime = millis();

  WiFi.begin(SSID, Password);
  Log.verboseln("Verbindung mit %s wird hergestellt ...", SSID);

  while (millis() - startAttemptTime < Timeout * 1000)
  {
    wl_status_t status = WiFi.status();
    Log.verboseln("Verbindungsstatus: %s", wl_status_to_string(status));

    if (status == WL_CONNECTED)
    {
      reconnectAttempts = 0; // Reset the reconnect attempts counter on successful connection
      return true;           // Verbindung erfolgreich
    }

    if (status == WL_NO_SHIELD || status == WL_NO_SSID_AVAIL || status == WL_CONNECT_FAILED)
    {
      Log.errorln("Verbindung Fehlgeschlagen");
      break;
    }

    delay(500);
    Log.verboseln("Verbleibende Zeit: %u", (Timeout * 1000) - (millis() - startAttemptTime));
  }

  Log.errorln("Verbindung Fehlgeschlagen Zeitüberschreitung");
  return false; // Timeout, Verbindung fehlgeschlagen
}

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Log.info("\nWLAN verbunden.");
  reconnectAttempts = 0; // Reset attempts when successfully connected
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Log.info("\nIP-Adresse: %s", WiFi.localIP().toString().c_str());
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Log.errorln("Disconnected from WiFi access point");
  Log.noticeln("WiFi lost connection. Reason: %s ", wifi_sta_disconnected_reason_to_string(info.wifi_sta_disconnected.reason));

  // Track reconnection attempts
  if (reconnectAttempts == 0)
  {
    firstReconnectAttemptTime = millis(); // Record the time of the first attempt
  }

  reconnectAttempts++;

  if (reconnectAttempts <= maxReconnectAttempts && millis() - firstReconnectAttemptTime <= reconnectTimeWindow)
  {
    delay(5000); // Wait 5 seconds before trying to reconnect
    Log.infoln("Trying to Reconnect (%d/%d)", reconnectAttempts, maxReconnectAttempts);
    WiFi.begin(SSID, password);
  }
  else
  {
    Log.errorln("Max reconnect attempts reached or time window exceeded. Entering deep sleep mode.");
    esp_deep_sleep_start(); // Enter deep sleep mode
  }
}

const char *wifi_sta_disconnected_reason_to_string(uint8_t reason)
{
  switch (reason)
  {
  case WIFI_REASON_UNSPECIFIED:
    return "WIFI_REASON_UNSPECIFIED";
  case WIFI_REASON_AUTH_EXPIRE:
    return "WIFI_REASON_AUTH_EXPIRE";
  case WIFI_REASON_AUTH_LEAVE:
    return "WIFI_REASON_AUTH_LEAVE";
  case WIFI_REASON_ASSOC_EXPIRE:
    return "WIFI_REASON_ASSOC_EXPIRE";
  case WIFI_REASON_ASSOC_TOOMANY:
    return "WIFI_REASON_ASSOC_TOOMANY";
  case WIFI_REASON_NOT_AUTHED:
    return "WIFI_REASON_NOT_AUTHED";
  case WIFI_REASON_NOT_ASSOCED:
    return "WIFI_REASON_NOT_ASSOCED";
  case WIFI_REASON_ASSOC_LEAVE:
    return "WIFI_REASON_ASSOC_LEAVE";
  case WIFI_REASON_ASSOC_NOT_AUTHED:
    return "WIFI_REASON_ASSOC_NOT_AUTHED";
  case WIFI_REASON_DISASSOC_PWRCAP_BAD:
    return "WIFI_REASON_DISASSOC_PWRCAP_BAD";
  case WIFI_REASON_DISASSOC_SUPCHAN_BAD:
    return "WIFI_REASON_DISASSOC_SUPCHAN_BAD";
  case WIFI_REASON_IE_INVALID:
    return "WIFI_REASON_IE_INVALID";
  case WIFI_REASON_MIC_FAILURE:
    return "WIFI_REASON_MIC_FAILURE";
  case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
    return "WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT";
  case WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT:
    return "WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT";
  case WIFI_REASON_IE_IN_4WAY_DIFFERS:
    return "WIFI_REASON_IE_IN_4WAY_DIFFERS";
  case WIFI_REASON_GROUP_CIPHER_INVALID:
    return "WIFI_REASON_GROUP_CIPHER_INVALID";
  case WIFI_REASON_PAIRWISE_CIPHER_INVALID:
    return "WIFI_REASON_PAIRWISE_CIPHER_INVALID";
  case WIFI_REASON_AKMP_INVALID:
    return "WIFI_REASON_AKMP_INVALID";
  case WIFI_REASON_UNSUPP_RSN_IE_VERSION:
    return "WIFI_REASON_UNSUPP_RSN_IE_VERSION";
  case WIFI_REASON_INVALID_RSN_IE_CAP:
    return "WIFI_REASON_INVALID_RSN_IE_CAP";
  case WIFI_REASON_802_1X_AUTH_FAILED:
    return "WIFI_REASON_802_1X_AUTH_FAILED";
  case WIFI_REASON_CIPHER_SUITE_REJECTED:
    return "WIFI_REASON_CIPHER_SUITE_REJECTED";
  case WIFI_REASON_BEACON_TIMEOUT:
    return "WIFI_REASON_BEACON_TIMEOUT";
  case WIFI_REASON_NO_AP_FOUND:
    return "WIFI_REASON_NO_AP_FOUND";
  case WIFI_REASON_AUTH_FAIL:
    return "WIFI_REASON_AUTH_FAIL";
  case WIFI_REASON_ASSOC_FAIL:
    return "WIFI_REASON_ASSOC_FAIL";
  case WIFI_REASON_HANDSHAKE_TIMEOUT:
    return "WIFI_REASON_HANDSHAKE_TIMEOUT";
  default:
    return "UNKNOWN_REASON";
  }
}
