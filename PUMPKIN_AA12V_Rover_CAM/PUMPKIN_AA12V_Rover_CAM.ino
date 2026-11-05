#include "esp_camera.h"
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ================= 팀 설정 (팀마다 이 숫자만 바꾸세요) =================
#define TEAM_ID 1                 // 1 ~ 99

#define CAM_MODE_AP  1
#define CAM_MODE_STA 2
#define CAM_MODE CAM_MODE_AP      // 메이커톤: AP / 소수 팀·인터넷 필요: STA

// 채널 계획: 3 = 1/6/11 (모든 노트북 호환, 기본)
//            4 = 1/5/9/13 (11b 를 끄므로 거의 겹치지 않음, 13번 채널을 못 보는 노트북이 있으면 3 사용)
#define CHANNEL_PLAN 3

// 송신 출력: 로버가 노트북에서 10m 이내면 WIFI_POWER_11dBm, 더 멀면 15dBm, 최대 19.5dBm
#define TX_POWER WIFI_POWER_13dBm

// 영상 기본값 (GCS 에서 바꿀 수 있음)
#define DEFAULT_FRAMESIZE FRAMESIZE_QVGA   // 320x240. 팀이 적으면 FRAMESIZE_VGA
#define DEFAULT_QUALITY   12               // 10(고화질) ~ 30(저화질·적은 데이터)
#define DEFAULT_FPS       12               // 초당 최대 장수

// STA 모드용 노트북 핫스팟 정보
const char *STA_SSID = "PUMPKIN-ROVER";
const char *STA_PASS = "12345678";
#define USE_STATIC_IP 0
IPAddress staticIP(192, 168, 137, 50), gateway(192, 168, 137, 1), subnet(255, 255, 255, 0);
// =====================================================================

// ---------------- AI-Thinker 핀 ----------------
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#define LED_RED 33     // 보드 뒷면 빨간 LED (LOW = 켜짐)
#define LED_FLASH 4    // 앞면 플래시 LED (전류를 많이 먹으므로 필요할 때만)

httpd_handle_t webHttpd = NULL, streamHttpd = NULL;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

int flashLevel = 0;
volatile int maxFps = DEFAULT_FPS;
int wifiChannel = 0;
char apSsid[24], apPass[16];

void flashSet(int v) {
  flashLevel = constrain(v, 0, 255);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(LED_FLASH, flashLevel);
#else
  ledcWrite(7, flashLevel);
#endif
}

static void addCors(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
}

// ---------- /capture ----------
static esp_err_t captureHandler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  addCors(req);
  esp_err_t r = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return r;
}

// ---------- /stream (포트 81) : 초당 장수 제한 ----------
static esp_err_t streamHandler(httpd_req_t *req) {
  char part[64];
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  addCors(req);
  unsigned long last = 0;
  while (true) {
    int fps = maxFps < 1 ? 1 : maxFps;
    unsigned long gap = 1000UL / fps;
    unsigned long now = millis();
    if (now - last < gap) { delay(gap - (now - last)); }
    last = millis();
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }
    size_t hl = snprintf(part, sizeof(part), STREAM_PART, fb->len);
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, part, hl);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;      // 클라이언트 연결 끊김
  }
  return res;
}

String myIp() { return (CAM_MODE == CAM_MODE_AP) ? WiFi.softAPIP().toString() : WiFi.localIP().toString(); }

// ---------- /status ----------
static esp_err_t statusHandler(httpd_req_t *req) {
  sensor_t *s = esp_camera_sensor_get();
  char json[320];
  int rssi = (CAM_MODE == CAM_MODE_STA) ? WiFi.RSSI() : 0;
  snprintf(json, sizeof(json),
           "{\"team\":%d,\"mode\":\"%s\",\"ssid\":\"%s\",\"channel\":%d,\"framesize\":%d,\"quality\":%d,"
           "\"fps\":%d,\"brightness\":%d,\"vflip\":%d,\"hmirror\":%d,\"led\":%d,\"rssi\":%d,\"ip\":\"%s\",\"heap\":%u}",
           TEAM_ID, CAM_MODE == CAM_MODE_AP ? "AP" : "STA", CAM_MODE == CAM_MODE_AP ? apSsid : STA_SSID, wifiChannel,
           s->status.framesize, s->status.quality, maxFps, s->status.brightness, s->status.vflip,
           s->status.hmirror, flashLevel, rssi, myIp().c_str(), (unsigned)ESP.getFreeHeap());
  httpd_resp_set_type(req, "application/json");
  addCors(req);
  return httpd_resp_send(req, json, strlen(json));
}

// ---------- /control?var=&val= ----------
static esp_err_t controlHandler(httpd_req_t *req) {
  char query[96], var[24], val[12];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "var", var, sizeof(var)) != ESP_OK ||
      httpd_query_key_value(query, "val", val, sizeof(val)) != ESP_OK) {
    httpd_resp_send_404(req); return ESP_FAIL;
  }
  int v = atoi(val);
  sensor_t *s = esp_camera_sensor_get();
  int r = 0;
  if (!strcmp(var, "vflip")) r = s->set_vflip(s, v);
  else if (!strcmp(var, "hmirror")) r = s->set_hmirror(s, v);
  else if (!strcmp(var, "framesize")) { if (s->pixformat == PIXFORMAT_JPEG) r = s->set_framesize(s, (framesize_t)v); }
  else if (!strcmp(var, "quality")) r = s->set_quality(s, constrain(v, 8, 40));
  else if (!strcmp(var, "brightness")) r = s->set_brightness(s, v);
  else if (!strcmp(var, "led")) flashSet(v);
  else if (!strcmp(var, "fps")) maxFps = constrain(v, 1, 30);
  else r = -1;
  addCors(req);
  if (r < 0) return httpd_resp_send_500(req);
  return httpd_resp_send(req, "OK", 2);
}

// ---------- / (안내 페이지) ----------
static esp_err_t indexHandler(httpd_req_t *req) {
  String ip = myIp();
  String html = "<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width'>"
                "<title>PUMPKIN CAM</title><body style='font-family:sans-serif;background:#111;color:#eee'>"
                "<h3>PUMPKIN IDEA FACTORY MAKERSPACE</h3>"
                "<p>AA-12V Rover CAM — TEAM " + String(TEAM_ID) + "</p><p>IP: " + ip + " · 채널 " + String(wifiChannel) + "</p>"
                "<p>GCS 에 이 IP 를 입력하세요. 스트림: http://" + ip + ":81/stream</p>"
                "<img src='http://" + ip + ":81/stream' style='width:100%;max-width:640px'></body>";
  httpd_resp_set_type(req, "text/html");
  addCors(req);
  return httpd_resp_send(req, html.c_str(), html.length());
}

void startServers() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_uri_handlers = 8;
  httpd_uri_t uIndex = {"/", HTTP_GET, indexHandler, NULL};
  httpd_uri_t uCap = {"/capture", HTTP_GET, captureHandler, NULL};
  httpd_uri_t uStat = {"/status", HTTP_GET, statusHandler, NULL};
  httpd_uri_t uCtl = {"/control", HTTP_GET, controlHandler, NULL};
  if (httpd_start(&webHttpd, &config) == ESP_OK) {
    httpd_register_uri_handler(webHttpd, &uIndex);
    httpd_register_uri_handler(webHttpd, &uCap);
    httpd_register_uri_handler(webHttpd, &uStat);
    httpd_register_uri_handler(webHttpd, &uCtl);
  }
  config.server_port = 81;
  config.ctrl_port += 1;
  httpd_uri_t uStream = {"/stream", HTTP_GET, streamHandler, NULL};
  if (httpd_start(&streamHttpd, &config) == ESP_OK) httpd_register_uri_handler(streamHttpd, &uStream);
}

void blinkDigit(int n) {
  if (n == 0) { digitalWrite(LED_RED, LOW); delay(900); digitalWrite(LED_RED, HIGH); delay(500); return; }
  for (int i = 0; i < n; i++) { digitalWrite(LED_RED, LOW); delay(220); digitalWrite(LED_RED, HIGH); delay(280); }
  delay(700);
}
void blinkNumber(int num) {
  char s[4]; snprintf(s, sizeof(s), "%d", num);
  for (int i = 0; s[i]; i++) blinkDigit(s[i] - '0');
}

// 전파 혼잡을 줄이는 공통 설정 (AP/STA 모두)
void tuneRadio(wifi_interface_t ifx) {
  WiFi.setSleep(false);
  WiFi.setTxPower(TX_POWER);
  esp_wifi_set_bandwidth(ifx, (wifi_bandwidth_t)1);   // 1 = 20MHz. 40MHz 사용 안 함 → 옆 채널 침범 방지
                                                      // (ESP-IDF 버전마다 이름이 WIFI_BW_HT20 / WIFI_BW20 로 달라 숫자로 씀)
  esp_wifi_config_11b_rate(ifx, true);          // 1~11Mbps 느린 11b 속도 끄기
}

int teamChannel() {
#if CHANNEL_PLAN == 4
  const int plan[4] = {1, 5, 9, 13};
  return plan[(TEAM_ID - 1) % 4];
#else
  const int plan[3] = {1, 6, 11};
  return plan[(TEAM_ID - 1) % 3];
#endif
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);   // 모터 기동 시 전압강하로 인한 리셋 방지
  Serial.begin(115200);
  pinMode(LED_RED, OUTPUT); digitalWrite(LED_RED, HIGH);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(LED_FLASH, 5000, 8);
#else
  ledcSetup(7, 5000, 8); ledcAttachPin(LED_FLASH, 7);
#endif
  flashSet(0);

  camera_config_t c;
  c.ledc_channel = LEDC_CHANNEL_0; c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM; c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM; c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM; c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.grab_mode = CAMERA_GRAB_LATEST;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  if (psramFound()) { c.frame_size = FRAMESIZE_VGA; c.jpeg_quality = DEFAULT_QUALITY; c.fb_count = 2; }
  else { c.frame_size = FRAMESIZE_QVGA; c.jpeg_quality = DEFAULT_QUALITY + 2; c.fb_count = 1; c.fb_location = CAMERA_FB_IN_DRAM; }

  if (esp_camera_init(&c) != ESP_OK) {
    Serial.println("Camera init FAILED - 카메라 리본 케이블/전원 확인");
    while (true) { digitalWrite(LED_RED, !digitalRead(LED_RED)); delay(100); }   // 빠른 깜빡임 = 카메라 오류
  }
  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, DEFAULT_FRAMESIZE);

#if CAM_MODE == CAM_MODE_AP
  snprintf(apSsid, sizeof(apSsid), "PUMPKIN-CAM-%02d", TEAM_ID);
  snprintf(apPass, sizeof(apPass), "pumpkin%02d", TEAM_ID);
  wifiChannel = teamChannel();
  WiFi.mode(WIFI_AP);
  // SSID, 비밀번호, 채널, 숨김 아님, 최대 접속 1대
  WiFi.softAP(apSsid, apPass, wifiChannel, 0, 1);
  delay(200);
  tuneRadio(WIFI_IF_AP);
  Serial.printf("AP READY  SSID %s  PASS %s  CH %d\n", apSsid, apPass, wifiChannel);
  Serial.println("CAM READY  http://192.168.4.1:81/stream");
  startServers();
  blinkNumber(TEAM_ID);      // 빨간 LED 로 팀 번호 표시
#else
  WiFi.mode(WIFI_STA);
#if USE_STATIC_IP
  WiFi.config(staticIP, gateway, subnet);
#endif
  WiFi.begin(STA_SSID, STA_PASS);
  Serial.print("WiFi 접속 중");
  while (WiFi.status() != WL_CONNECTED) { delay(400); Serial.print('.'); digitalWrite(LED_RED, !digitalRead(LED_RED)); }
  digitalWrite(LED_RED, HIGH);
  tuneRadio(WIFI_IF_STA);
  wifiChannel = WiFi.channel();
  Serial.println();
  Serial.print("CAM READY  http://"); Serial.print(WiFi.localIP()); Serial.println(":81/stream");
  startServers();
  blinkNumber(WiFi.localIP()[3]);   // IP 마지막 숫자 표시
#endif
}

void loop() {
  static unsigned long lastCheck = 0, lastBlink = 0;
#if CAM_MODE == CAM_MODE_STA
  if (millis() - lastCheck > 5000) {
    lastCheck = millis();
    if (WiFi.status() != WL_CONNECTED) { WiFi.disconnect(); WiFi.reconnect(); }
  }
  if (millis() - lastBlink > 30000 && WiFi.status() == WL_CONNECTED) { lastBlink = millis(); blinkNumber(WiFi.localIP()[3]); }
#else
  if (millis() - lastBlink > 30000) { lastBlink = millis(); blinkNumber(TEAM_ID); }
#endif
  delay(50);
}
