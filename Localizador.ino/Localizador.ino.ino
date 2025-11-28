#include <WiFi.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// ----------------------
// 1. Configurações
// ----------------------
static const int RXPin = 16; 
static const int TXPin = 17; 
static const uint32_t GPSBaud = 9600;

// --- SEUS DADOS WIFI ---
const char* ssid = "Coord_Uirapuru";
const char* password = "@@Etec@230";

// --- SEU SERVIDOR (Use o IP do computador, NÃO use localhost) ---
const char* host = "192.168.0.178"; 
const int httpPort = 80; 
const char* urlPath = "/teste/localizar.php"; 
const char* AUTH_TOKEN = "chave_secreta_dispositivo12"; 

// ----------------------
// 2. Objetos Globais
// ----------------------
TinyGPSPlus tgps;
HardwareSerial GPS_Serial(2); 
WiFiClient client;

unsigned long lastTransmission = 0;
const long transmissionInterval = 10000; // Envia a cada 10s

// ----------------------
// 3. Conexão WiFi
// ----------------------
void connectToWiFi() {
    Serial.print("Conectando WiFi: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 30) {
        delay(500);
        Serial.print(".");
        tries++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi OK!");
        Serial.print("IP: "); Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nFalha no WiFi.");
    }
}

// ----------------------
// 4. Transmissão (Lógica Alterada)
// ----------------------
void transmitLocation() {
    
    // Inicializa com zero
    double lat = 0.0;
    double lng = 0.0;
    double alt = 0.0;
    String statusGPS = "SEM_SINAL";

    // Verifica se o GPS tem dados válidos. 
    // Se tiver, sobrescreve os zeros. Se não tiver, mantém os zeros.
    if (tgps.location.isValid()) {
        lat = tgps.location.lat();
        lng = tgps.location.lng();
        alt = '0';
        statusGPS = "VALIDO";
    } else {
        Serial.println("GPS Indoor (Sem fix). Preparando envio de coordenadas 0.0...");
    }

    // --- BLOCO DE ENVIO (Agora roda sempre) ---
    if (client.connect(host, httpPort)) {
        Serial.print("Enviando ["); Serial.print(statusGPS);
        Serial.print("] -> Lat: "); Serial.print(lat, 6);
        Serial.print(" Lon: "); Serial.println(lng, 6);

        // Monta a URL
        String fullUrl = String(urlPath) + "?token=" + String(AUTH_TOKEN) + 
                         "&lat=" + String(lat, 6) + 
                         "&lon=" + String(lng, 6) + 
                         "&alt=" + String(alt);

        client.print(String("GET ") + fullUrl + " HTTP/1.1\r\n" +
                     "Host: " + host + "\r\n" +
                     "Connection: close\r\n" +
                     "\r\n");

        // Tratamento rápido da resposta para evitar travar o loop
        unsigned long timeout = millis();
        while (client.available() == 0) {
            if (millis() - timeout > 2000) {
                client.stop();
                return;
            }
        }
        client.stop();
        Serial.println("Dados enviados.");
    } else {
        Serial.println("Erro: Não foi possível conectar ao servidor PHP.");
    }
}

// ----------------------
// 5. SmartDelay (Essencial para não travar o GPS)
// ----------------------
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (GPS_Serial.available())
      tgps.encode(GPS_Serial.read());
  } while (millis() - start < ms);
}

// ----------------------
// 6. Setup e Loop
// ----------------------
void setup() {
    Serial.begin(115200);
    GPS_Serial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
    
    Serial.println("Iniciando...");
    connectToWiFi();
}

void loop() {
    // 1. Mantém o GPS lendo dados o tempo todo
    smartDelay(1000); 

    // 2. Verifica timer de envio
    if (millis() - lastTransmission >= transmissionInterval) {
        
        // Garante WiFi antes de enviar
        if (WiFi.status() != WL_CONNECTED) connectToWiFi();
        
        // Chama a função que envia (seja 0.0 ou dados reais)
        transmitLocation();
        
        lastTransmission = millis();
    }
}