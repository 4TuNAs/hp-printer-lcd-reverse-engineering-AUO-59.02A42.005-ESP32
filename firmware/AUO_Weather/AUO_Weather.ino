// =====================================================================
//  AUO 59.02A42.005 (родственник A024CN02) -> ESP32 DevKit V1
//  Часы + погода в Малаге
//
//  Вывод на LCD:     I2S0 + DMA, режим матрицы UPS052 320RGB (такты 0 R G B), DCLK 10..20 МГц
//  Ядро 0 (control): перерисовка экрана по событиям, кнопка BOOT
//  Ядро 0 (net):     окно связи раз в 10 минут: Wi-Fi -> NTP -> погода
//                    (Open-Meteo) -> Wi-Fi полностью выключается
//
//  Кнопка BOOT: часы/погода <-> экран калибровки (полосы R G B).
//  Serial 115200: отладка (DEBUG_LOG 1/0).
// =====================================================================


#include <Arduino.h>
#include "esp_sntp.h"
#include <math.h>
#include <ctype.h>
#include <time.h>
#include <sys/time.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include "soc/gpio_struct.h"
// ---- отладка в Serial (115200). 0 = выключить всю печать ----
#define DEBUG_LOG 1
#if DEBUG_LOG
  #define LOG(...) do { Serial.printf("[%6.1f] ", millis() / 1000.0f); Serial.printf(__VA_ARGS__); Serial.println(); } while (0)
#else
  #define LOG(...) do {} while (0)
#endif
#include "soc/i2s_struct.h"
#include "soc/i2s_reg.h"
#include "soc/gpio_sig_map.h"
#include "esp_intr_alloc.h"
#include "esp_rom_gpio.h"
#include "driver/gpio.h"
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #include "esp_private/periph_ctrl.h"
#else
  #include "driver/periph_ctrl.h"
#endif


// ============================ НАСТРОЙКИ ==============================
#define USE_WIFI    1        // Wi-Fi/NTP/Open-Meteo
// Copy secrets.example.h to secrets.h and enter your own network credentials.
// secrets.h is ignored by git.
#if __has_include("secrets.h")
  #include "secrets.h"
#else
  #define WIFI_SSID "YOUR_WIFI_SSID"
  #define WIFI_PASS "YOUR_WIFI_PASSWORD"
#endif


#define LATITUDE    "36.7213"
#define LONGITUDE   "-4.4214"
#define TZ_INFO     "CET-1CEST,M3.5.0,M10.5.0/3"     // Europe/Madrid


#define WEATHER_PERIOD_MS  (10UL * 60UL * 1000UL)    // раз в 10 минут
#define WEATHER_RETRY_MS   (60UL * 1000UL)           // при ошибке — через минуту


static const char* WEATHER_QUERY =
  "/v1/forecast?latitude=" LATITUDE "&longitude=" LONGITUDE
  "&current=temperature_2m,relative_humidity_2m,apparent_temperature,"
  "is_day,weather_code,wind_speed_10m,wind_direction_10m"
  "&daily=temperature_2m_max,temperature_2m_min,sunrise,sunset"
  "&timezone=Europe%2FMadrid&forecast_days=1&wind_speed_unit=ms";


// ============================== ПИНЫ =================================
#define LCD_D0     13
#define LCD_D1     14
#define LCD_D2     16
#define LCD_D3     17
#define LCD_D4     18
#define LCD_D5     19
#define LCD_D6     21
#define LCD_D7     22
#define LCD_DCLK   23
#define LCD_HSYNC  25
#define LCD_VSYNC  26
#define BTN_BOOT   0


// ============================= МАТРИЦА ===============================
#define DOTS   480
#define LINES  234


#define H_SYNC    20
#define H_BACK     82
#define H_FRONT    34
#define V_SYNC     1
#define V_BACK    18
#define V_FRONT   4          // кадр 1+18+234+4 = 257 строк -> ~50 Гц при 20 МГц

// Никакого программного срезания слева: выводим все 480 точек.
#define X_SHIFT_DOTS 0


// ================= КАЛИБРОВКА (меняется клавишами) ===================
uint8_t phase[2] = {0, 1};     // строки 1,3,5... / 2,4,6...
bool    mirX = false;
bool    mirY = false;


// ============================ БУФЕР ==================================
// Кадровый буфер: RGB565, 160 пикселей x 234 строки x 2 байта = 75 КБ.
// (8 бит на канал = 112 КБ — тогда Wi-Fi не хватает памяти.)
uint16_t* fb[LINES];
uint8_t  chanOf[2][DOTS];


volatile uint8_t  lineMask   = 0;
volatile uint32_t frameCount = 0;


Preferences prefs;
int8_t SIN8[256];


// =====================================================================
//   ТИПЫ ДАННЫХ — обязательно ДО первой функции: Arduino IDE сама
//   вставляет прототипы функций перед первой функцией скетча
// =====================================================================


struct C3 { int r, g, b; };                     // цвет (int — с запасом на сложение)


struct Glyph { uint16_t cp; uint8_t r[7]; };    // символ шрифта 5x7


#define TXT_MAXCOL 220
#define TXT_MAXSTR 48


struct TextItem
{
  bool set;                    // текст задан (даже если пустой)
  int ncol;                    // 0 = пусто
  int x0, y0, sx, sy;          // позиция и масштаб (точек / строк на пиксель шрифта)
  int ox, oy;                  // смещение тени
  int bx0, by0, bx1, by1;      // область перерисовки
  C3  color;
  char str[TXT_MAXSTR];
  uint8_t cols[TXT_MAXCOL];    // столбцы глифов, бит j = строка j
};


struct Weather
{
  bool  valid;
  float temp, feels, wind, tmax, tmin;
  int   hum, code, windDir, isDay;
  int   sunrise, sunset;            // минуты от полуночи, -1 если нет
  uint32_t gotAt;
};


// =====================================================================
//                      ВЫВОД НА LCD (ядро 1)
// =====================================================================


#define SEA_Y  202                   // линия горизонта (нужна и выводу: небо/горы рисуются при выводе)

// ---- АППАРАТНЫЙ ВЫВОД: I2S0 в режиме LCD + DMA ----
// Такт DCLK, HSYNC, VSYNC и данные выдаёт железо, процессоры на сигнал не влияют.
// 16-битные отсчёты: биты 0..7 = D0..D7, бит 8 = HSYNC, бит 9 = VSYNC, WS = DCLK.
//
// Матрица работает в режиме UPS052 320RGB (даташит, раздел c-1):
//   пиксель = 4 такта: R, G, B, пусто(00); 320 пикселей = 1280 тактов;
//   задний отступ 220..283 (берём 256, проверено на линейке), строка 1472..1644,
//   DCLK 16..27 МГц, кадр >= 50 Гц. Дельта-раскладку матрица делает сама.
// Наш логический пиксель (160 по ширине) уходит двумя пикселями матрицы.
#define HS_S      60                    // HSYNC = 0, тактов
#define ACT0_S    256                   // начало картинки (задний отступ)
#define H_SAMP    1544                  // строка: 256 + 1280 + 8
#define V_TOTAL   (V_SYNC + V_BACK + LINES + V_FRONT)    // 257
#define V_ACT0    (V_SYNC + V_BACK)
#define BCK_DIV   2
#define HS_BIT    (1u << 8)
#define VS_BIT    (1u << 9)
#define NBUF      6          // кольцо буферов активных строк
#define AHEAD     3          // заполняем строку на 3 вперёд

// переключаемые из консоли (сохраняются):
//   'f' частота DCLK: 160 МГц / (N + B/A) / 4
//   частота = 160 МГц / (N + B/A) / BCK / 2
struct ClkOpt { uint8_t n, b, a, bck; const char* name; };
#define NCLK 5
static const ClkOpt CLK_OPTS[NCLK] =
{
  { 4, 0, 1, 2, "10" },
  { 3, 0, 1, 2, "13.3" },
  { 2, 1, 2, 2, "16 (дробный)" },
  { 2, 0, 1, 2, "20" },
  { 5, 0, 1, 1, "16 (целый)" },
};
volatile uint8_t clkSel  = 2;           // 2 = 16 МГц (проверено: лучше всего)
volatile bool    clkInv  = true;        // 'i': инверсия DCLK (проверено: нужна)
volatile bool    pairSwap = true;       // 'o': порядок отсчётов в паре (проверено: 1)
volatile uint8_t slotRot = 1;           // 'k': сдвиг пикселя по тактам; 1 = 0,R,G,B (проверено)

DMA_ATTR uint16_t lineBuf[NBUF][H_SAMP];
DMA_ATTR uint16_t blankBuf[H_SAMP];
DMA_ATTR uint16_t vsyncBuf[H_SAMP];

typedef struct DmaDesc
{
  volatile uint32_t size : 12, length : 12, offset : 5, sosf : 1, eof : 1, owner : 1;
  volatile uint8_t* buf;
  volatile struct DmaDesc* next;
} DmaDesc;
DMA_ATTR DmaDesc lcdDesc[V_TOTAL];

void fillBlank(uint16_t* b, bool vsLow)
{
  uint16_t vs = vsLow ? 0 : VS_BIT;
  for (int s = 0; s < H_SAMP; s++)
    b[s ^ 1] = vs | (s < HS_S ? 0 : HS_BIT);     // HSYNC/VSYNC одинаковы в паре — порядок не важен
}

// 5/6 бит -> отсчёт (8 бит данных + HSYNC=1 + VSYNC=1); заполняются в buildLuts()
DRAM_ATTR static uint16_t L5[32];
DRAM_ATTR static uint16_t L6[64];
void buildLuts()
{
  for (int i = 0; i < 32; i++) L5[i] = 0x300 | (uint16_t)((i << 3) | (i >> 2));
  for (int i = 0; i < 64; i++) L6[i] = 0x300 | (uint16_t)((i << 2) | (i >> 4));
}

// Пара отсчётов (a, затем b) одним 32-битным словом с учётом порядка в паре
static inline uint32_t IRAM_ATTR pairWord(uint16_t a, uint16_t b, bool sw)
{
  return sw ? (((uint32_t)a << 16) | b) : (((uint32_t)b << 16) | a);
}

// ---------------- ФОН «НА ЛЕТУ» (небо + горы) ----------------
// Небо и горы НЕ хранятся в кадровом буфере: при выводе строки они берутся из
// маленьких таблиц — полные 320 точек по ширине и 8 бит на цвет (без ступенек).
// В кадровом буфере там, где на небе ничего не нарисовано, лежит FB_KEY («прозрачно»).
#define FB_KEY      0x0821          // почти чёрный; настоящий цвет с таким кодом сдвигается на 1
#define MOUNT_MAXH  20
DRAM_ATTR uint32_t skyW[SEA_Y][2];                         // цвет неба на строку (2 слова = 4 такта)
DRAM_ATTR uint32_t mountW[MOUNT_MAXH + 1][MOUNT_MAXH + 1][2]; // [dy = SEA_Y - y][h]
DRAM_ATTR uint8_t  mhP[320];                               // высота гор на колонку матрицы

// Заполнить строку y: каждый логический пиксель (R,G,B из fb) -> R G B 0 R G B 0
static inline void IRAM_ATTR fillLine(uint16_t* b, int y)
{
  uint8_t mask = lineMask;
  bool hide = (mask == 1 && (y & 1)) || (mask == 2 && !(y & 1));
  bool sw = pairSwap;
  const uint16_t base = VS_BIT | HS_BIT;
  uint32_t* w = (uint32_t*)(b + ACT0_S);          // ACT0_S чётное
  if (hide)
  {
    uint32_t z = pairWord(base, base, sw);
    for (int i = 0; i < 1280 / 2; i++) w[i] = z;
    return;
  }
  const uint16_t* p = fb[y];
  // такт j пикселя матрицы берёт v[(j - rot) & 3], где v = {R, G, B, 0}
  // rot = 1 -> такты: 0, R, G, B (так ждёт матрица)
  const int r = slotRot & 3;
  const int i0 = (0 - r) & 3, i1 = (1 - r) & 3, i2 = (2 - r) & 3, i3 = (3 - r) & 3;
  uint16_t v[4];
  v[3] = base;
  int ly = mirY ? (LINES - 1 - y) : y;          // логическая строка (для неба/гор)
  bool sky = ly < SEA_Y;
  const uint32_t* skw = sky ? skyW[ly] : skyW[0];
  int dy = SEA_Y - ly;                            // 1.. над горизонтом
  bool mountRows = sky && dy <= MOUNT_MAXH;
  // каждый логический пиксель -> 2 пикселя матрицы (по 4 такта)
  for (int i = 0; i < DOTS / 3; i++, w += 4)
  {
    uint16_t px = p[i];
    if (px == FB_KEY && sky)
    {
      // прозрачно: небо или горы, отдельно для каждой из двух колонок матрицы
      const uint32_t* s0 = skw;
      const uint32_t* s1 = skw;
      if (mountRows)
      {
        int h0 = mhP[2 * i], h1 = mhP[2 * i + 1];
        if (h0 >= dy) s0 = mountW[dy][h0];
        if (h1 >= dy) s1 = mountW[dy][h1];
      }
      w[0] = s0[0]; w[1] = s0[1]; w[2] = s1[0]; w[3] = s1[1];
    }
    else
    {
      v[0] = L5[px >> 11]; v[1] = L6[(px >> 5) & 63]; v[2] = L5[px & 31];
      uint32_t a0 = pairWord(v[i0], v[i1], sw), a1 = pairWord(v[i2], v[i3], sw);
      w[0] = a0; w[1] = a1; w[2] = a0; w[3] = a1;
    }
  }
}

// перезаполнить пустые части всех буферов
void refillAllLineBufs()
{
  for (int i = 0; i < NBUF; i++) fillBlank(lineBuf[i], false);
}

volatile uint32_t isrMaxCyc = 0;     // макс. время прерывания (тактов CPU)
volatile uint32_t isrSkip   = 0;     // сколько раз пропущены строки
static int isrLastLine = -1;

void IRAM_ATTR lcdIsr(void*)
{
  uint32_t c0 = xthal_get_ccount();
  uint32_t st = I2S0.int_st.val;
  I2S0.int_clr.val = st;
  if (!(st & I2S_OUT_EOF_INT_ST)) return;
  DmaDesc* d = (DmaDesc*)I2S0.out_eof_des_addr;
  int line = d - lcdDesc;
  if (line < 0 || line >= V_TOTAL) return;
  // сколько строк прошло с прошлого раза (обычно 1; больше — значит, пропуск)
  int n = 1;
  if (isrLastLine >= 0)
  {
    n = line - isrLastLine;
    if (n <= 0) n += V_TOTAL;
    if (n > 1) isrSkip++;
    if (n > AHEAD - 1) n = AHEAD - 1;         // не трогаем строку, которую DMA выводит сейчас
  }
  isrLastLine = line;
  if (line == V_TOTAL - 1 || (n > 1 && line < n)) frameCount++;
  // заполнить все строки, которые должны были заполниться (с догоном)
  for (int k = n - 1; k >= 0; k--)
  {
    int t = line + AHEAD - k;
    if (t >= V_TOTAL) t -= V_TOTAL;
    if (t < 0) t += V_TOTAL;
    int y = t - V_ACT0;
    if (y >= 0 && y < LINES) fillLine(lineBuf[y % NBUF], y);
  }
  uint32_t dc = xthal_get_ccount() - c0;
  if (dc > isrMaxCyc) isrMaxCyc = dc;
}

void startLcdOutput()
{
  fillBlank(blankBuf, false);
  fillBlank(vsyncBuf, true);
  for (int i = 0; i < NBUF; i++) fillBlank(lineBuf[i], false);
  for (int y = 0; y < AHEAD && y < LINES; y++) fillLine(lineBuf[y % NBUF], y);

  for (int l = 0; l < V_TOTAL; l++)
  {
    uint16_t* b;
    if (l < V_SYNC) b = vsyncBuf;
    else if (l < V_ACT0 || l >= V_ACT0 + LINES) b = blankBuf;
    else b = lineBuf[(l - V_ACT0) % NBUF];
    lcdDesc[l].size = H_SAMP * 2;
    lcdDesc[l].length = H_SAMP * 2;
    lcdDesc[l].offset = 0;
    lcdDesc[l].sosf = 0;
    lcdDesc[l].eof = 1;
    lcdDesc[l].owner = 1;
    lcdDesc[l].buf = (uint8_t*)b;
    lcdDesc[l].next = &lcdDesc[(l + 1) % V_TOTAL];
  }

  // пины через матрицу (16-битный режим -> сигналы OUT8..OUT23)
  const uint8_t pins[10] = { LCD_D0, LCD_D1, LCD_D2, LCD_D3, LCD_D4, LCD_D5, LCD_D6, LCD_D7, LCD_HSYNC, LCD_VSYNC };
  for (int i = 0; i < 10; i++)
  {
    pinMode(pins[i], OUTPUT);
    gpio_set_drive_capability((gpio_num_t)pins[i], GPIO_DRIVE_CAP_3);   // максимальная сила выхода
    esp_rom_gpio_connect_out_signal(pins[i], I2S0O_DATA_OUT8_IDX + i, false, false);
  }
  pinMode(LCD_DCLK, OUTPUT);
  gpio_set_drive_capability((gpio_num_t)LCD_DCLK, GPIO_DRIVE_CAP_3);
  esp_rom_gpio_connect_out_signal(LCD_DCLK, I2S0O_WS_OUT_IDX, clkInv, false);

  periph_module_enable(PERIPH_I2S0_MODULE);

  I2S0.conf.tx_reset = 1; I2S0.conf.tx_reset = 0;
  I2S0.conf.rx_reset = 1; I2S0.conf.rx_reset = 0;
  I2S0.conf.tx_fifo_reset = 1; I2S0.conf.tx_fifo_reset = 0;
  I2S0.lc_conf.out_rst = 1; I2S0.lc_conf.out_rst = 0;
  I2S0.lc_conf.ahbm_rst = 1; I2S0.lc_conf.ahbm_rst = 0;
  I2S0.lc_conf.ahbm_fifo_rst = 1; I2S0.lc_conf.ahbm_fifo_rst = 0;

  I2S0.conf2.val = 0;
  I2S0.conf2.lcd_en = 1;

  I2S0.sample_rate_conf.val = 0;
  I2S0.sample_rate_conf.tx_bits_mod = 16;
  I2S0.sample_rate_conf.tx_bck_div_num = CLK_OPTS[clkSel].bck;

  I2S0.clkm_conf.val = 0;
  I2S0.clkm_conf.clka_en = 0;
  I2S0.clkm_conf.clkm_div_a = CLK_OPTS[clkSel].a;
  I2S0.clkm_conf.clkm_div_b = CLK_OPTS[clkSel].b;
  I2S0.clkm_conf.clkm_div_num = CLK_OPTS[clkSel].n;
  I2S0.clkm_conf.clk_en = 1;

  I2S0.fifo_conf.val = 0;
  I2S0.fifo_conf.tx_fifo_mod_force_en = 1;
  I2S0.fifo_conf.tx_fifo_mod = 1;
  I2S0.fifo_conf.tx_data_num = 32;
  I2S0.fifo_conf.dscr_en = 1;

  I2S0.conf1.val = 0;
  I2S0.conf1.tx_stop_en = 0;
  I2S0.conf1.tx_pcm_bypass = 1;

  I2S0.conf_chan.val = 0;
  I2S0.conf_chan.tx_chan_mod = 1;

  I2S0.conf.tx_right_first = 1;
  I2S0.timing.val = 0;

  I2S0.lc_conf.val = 0;
  I2S0.lc_conf.out_data_burst_en = 1;
  I2S0.lc_conf.outdscr_burst_en = 1;
  I2S0.lc_conf.check_owner = 0;

  // прерывание на ядре 1 (setup() выполняется на ядре 1)
  I2S0.int_clr.val = 0xFFFFFFFF;
  I2S0.int_ena.val = 0;
  I2S0.int_ena.out_eof = 1;
  esp_intr_alloc(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_IRAM, lcdIsr, NULL, NULL);

  I2S0.out_link.addr = (uint32_t)&lcdDesc[0];
  I2S0.out_link.start = 1;
  I2S0.conf.tx_start = 1;
}


// =====================================================================
//                       БАЗОВОЕ РИСОВАНИЕ
// =====================================================================


static inline uint8_t clamp8(int v) { return v < 0 ? 0 : (v > 255 ? 255 : v); }
static inline float   clampf(float v, float a, float b) { return v < a ? a : (v > b ? b : v); }


static inline C3 mix(const C3& a, const C3& b, int t)      // t = 0..256
{
  if (t <= 0) return a;
  if (t >= 256) return b;
  return { a.r + (b.r - a.r) * t / 256,
           a.g + (b.g - a.g) * t / 256,
           a.b + (b.b - a.b) * t / 256 };
}


static inline C3 mixF(const C3& a, const C3& b, float t)   // t = 0..1
{
  return mix(a, b, (int)(t * 256.0f));
}


void rebuildChan()
{
  for (int p = 0; p < 2; p++)
    for (int x = 0; x < DOTS; x++)
      chanOf[p][x] = x % 3;      // UPS052: R,G,B подряд, дельту делает матрица
}


// Логический пиксель p (0..159) строки y: все три канала из ОДНОГО цвета c.
// Пиксель матрицы = полубайты 3p (R), 3p+1 (G), 3p+2 (B).
// 'd': дизеринг 0 = выкл, 1 = везде, 2 = только фон (небо/горы/море), текст и иконки чистые
volatile uint8_t ditherMode = 2;
bool sceneIsBg = true;              // scene() сообщает: пиксель — чистый фон (без текста/иконок)
bool scenePureSky = false;          // scene() сообщает: чистое небо/горы (без звёзд и рисунка)
static inline void putPix(int p, int y, const C3& c, bool bg)
{
  int pp = mirX ? (DOTS / 3 - 1 - p) : p;
  int py = mirY ? (LINES - 1 - y) : y;
  static const uint8_t BAYER[4][4] = { { 0, 8, 2, 10 }, { 12, 4, 14, 6 }, { 3, 11, 1, 9 }, { 15, 7, 13, 5 } };
  bool dz = (ditherMode == 1) || (ditherMode == 2 && bg);
  int d = dz ? BAYER[py & 3][pp & 3] : 8;        // 0..15
  // 8 бит -> 5/6/5 бит с упорядоченным дизерингом
  int qr = (clamp8(c.r) + (d >> 1)) >> 3; if (qr > 31) qr = 31;
  int qg = (clamp8(c.g) + (d >> 2)) >> 2; if (qg > 63) qg = 63;
  int qb = (clamp8(c.b) + (d >> 1)) >> 3; if (qb > 31) qb = 31;
  uint16_t v16 = (uint16_t)((qr << 11) | (qg << 5) | qb);
  if (v16 == FB_KEY) v16 ^= 1;                   // не путать с «прозрачным»
  fb[py][pp] = v16;
}

static inline void putKey(int p, int y)
{
  int pp = mirX ? (DOTS / 3 - 1 - p) : p;
  int py = mirY ? (LINES - 1 - y) : y;
  fb[py][pp] = FB_KEY;
}

// Строка y, точки [x0, x1): пиксель p закрывает точки 3p, 3p+1, 3p+2 —
// считаем сцену во всех трёх и усредняем (ровные сглаженные края, без цветной каймы).
// Если все три — чистое небо/горы, пишем «прозрачно»: фон нарисует вывод сам.
static inline void renderRow(int y, int x0, int x1, C3 (*fn)(int, int))
{
  int p0 = x0 / 3, p1 = (x1 + 2) / 3;
  if (p1 > DOTS / 3) p1 = DOTS / 3;
  for (int p = p0; p < p1; p++)
  {
    int sr = 0, sg = 0, sb = 0;
    bool allSky = true, allBg = true;
    for (int k = 0; k < 3; k++)
    {
      sceneIsBg = true;
      scenePureSky = false;
      C3 c = fn(p * 3 + k, y);
      sr += c.r; sg += c.g; sb += c.b;
      allSky &= scenePureSky;
      allBg  &= sceneIsBg;
    }
    if (allSky) putKey(p, y);                 // небо/горы нарисует вывод сам, в полном качестве
    else
    {
      C3 c = { (sr + 1) / 3, (sg + 1) / 3, (sb + 1) / 3 };
      putPix(p, y, c, allBg);
    }
  }
}


C3 hue(int h)
{
  h %= 1536;
  if (h < 0) h += 1536;
  int f = h & 255;
  switch (h >> 8)
  {
    case 0:  return {255, f, 0};
    case 1:  return {255 - f, 255, 0};
    case 2:  return {0, 255, f};
    case 3:  return {0, 255 - f, 255};
    case 4:  return {f, 0, 255};
    default: return {255, 0, 255 - f};
  }
}


// Экран калибровки (как в шаге 1)
void drawCalibration()
{
  for (int y = 0; y < LINES; y++)
    for (int x = 1; x < DOTS; x += 3)          // центр каждого пикселя
    {
      C3 c = {0, 0, 0};
      if (y < 150)
      {
        if      (x < 160) c = {255, 0, 0};
        else if (x < 320) c = {0, 255, 0};
        else              c = {0, 0, 255};
        if (x >= 12 && x < 60 && y >= 10 && y < 42) c = {255, 255, 255};
      }
      else if (y >= 156 && y < 192) { int v = x * 255 / (DOTS - 1); c = {v, v, v}; }
      else if (y >= 198) c = hue(x * 1535 / (DOTS - 1));
      putPix(x / 3, y, c, false);
    }
}


// ---------------------------------------------------------------------
// Геометрия: точка = 0.10 мм, строка = 0.1525 мм.
// Для круглых фигур используем единицы ~0.05 мм:  X = x*2+1, Y = y*3+1.5
// ---------------------------------------------------------------------


static inline float discF(float dx, float dy, float r)        // 0..1, мягкий край
{
  float d2 = dx * dx + dy * dy;
  float ro = r + 1.5f;
  if (d2 >= ro * ro) return 0.0f;
  float ri = r - 1.5f;
  if (ri > 0 && d2 <= ri * ri) return 1.0f;
  return (ro - sqrtf(d2)) / 3.0f;
}


static inline float capsuleF(float px, float py, float ax, float ay,
                             float bx, float by, float r)
{
  float vx = bx - ax, vy = by - ay, wx = px - ax, wy = py - ay;
  float L2 = vx * vx + vy * vy;
  float t = L2 > 0 ? (wx * vx + wy * vy) / L2 : 0.0f;
  t = clampf(t, 0.0f, 1.0f);
  float dx = wx - t * vx, dy = wy - t * vy;
  float d2 = dx * dx + dy * dy;
  float ro = r + 1.5f;
  if (d2 >= ro * ro) return 0.0f;
  return clampf((ro - sqrtf(d2)) / 3.0f, 0.0f, 1.0f);
}


static inline uint32_t hash32(uint32_t x)
{
  x ^= x >> 16; x *= 0x7feb352dU; x ^= x >> 15; x *= 0x846ca68bU; x ^= x >> 16;
  return x;
}


// =====================================================================
//                    ШРИФТ 5x7 (кириллица + латиница)
// =====================================================================


#define G(cp,a,b,c,d,e,f,g) { cp, { 0b##a, 0b##b, 0b##c, 0b##d, 0b##e, 0b##f, 0b##g } }


static const Glyph FONT[] =
{
  G(' ',    00000,00000,00000,00000,00000,00000,00000),
  G('0',    01110,10001,10001,10001,10001,10001,01110),
  G('1',    00100,01100,00100,00100,00100,00100,01110),
  G('2',    01110,10001,00001,00010,00100,01000,11111),
  G('3',    11111,00010,00100,00010,00001,10001,01110),
  G('4',    00010,00110,01010,10010,11111,00010,00010),
  G('5',    11111,10000,11110,00001,00001,10001,01110),
  G('6',    00110,01000,10000,11110,10001,10001,01110),
  G('7',    11111,00001,00010,00100,01000,01000,01000),
  G('8',    01110,10001,10001,01110,10001,10001,01110),
  G('9',    01110,10001,10001,01111,00001,00010,01100),
  G(':',    00000,01100,01100,00000,01100,01100,00000),
  G('.',    00000,00000,00000,00000,00000,01100,01100),
  G(',',    00000,00000,00000,00000,01100,00100,01000),
  G('-',    00000,00000,00000,11111,00000,00000,00000),
  G('+',    00000,00100,00100,11111,00100,00100,00000),
  G('%',    11000,11001,00010,00100,01000,10011,00011),
  G('/',    00000,00001,00010,00100,01000,10000,00000),
  G('?',    01110,10001,00001,00010,00100,00000,00100),
  G('!',    00100,00100,00100,00100,00100,00000,00100),
  G('(',    00010,00100,01000,01000,01000,00100,00010),
  G(')',    01000,00100,00010,00010,00010,00100,01000),
  G(0x00B0, 01100,10010,10010,01100,00000,00000,00000),   // °
  G(0x2191, 00100,01110,10101,00100,00100,00100,00100),   // ↑
  G(0x2193, 00100,00100,00100,00100,10101,01110,00100),   // ↓


  // Латиница
  G('A',    01110,10001,10001,11111,10001,10001,10001),
  G('B',    11110,10001,10001,11110,10001,10001,11110),
  G('C',    01110,10001,10000,10000,10000,10001,01110),
  G('D',    11100,10010,10001,10001,10001,10010,11100),
  G('E',    11111,10000,10000,11110,10000,10000,11111),
  G('F',    11111,10000,10000,11110,10000,10000,10000),
  G('G',    01110,10001,10000,10111,10001,10001,01111),
  G('H',    10001,10001,10001,11111,10001,10001,10001),
  G('I',    01110,00100,00100,00100,00100,00100,01110),
  G('J',    00111,00010,00010,00010,00010,10010,01100),
  G('K',    10001,10010,10100,11000,10100,10010,10001),
  G('L',    10000,10000,10000,10000,10000,10000,11111),
  G('M',    10001,11011,10101,10101,10001,10001,10001),
  G('N',    10001,10001,11001,10101,10011,10001,10001),
  G('O',    01110,10001,10001,10001,10001,10001,01110),
  G('P',    11110,10001,10001,11110,10000,10000,10000),
  G('Q',    01110,10001,10001,10001,10101,10010,01101),
  G('R',    11110,10001,10001,11110,10100,10010,10001),
  G('S',    01111,10000,10000,01110,00001,00001,11110),
  G('T',    11111,00100,00100,00100,00100,00100,00100),
  G('U',    10001,10001,10001,10001,10001,10001,01110),
  G('V',    10001,10001,10001,10001,10001,01010,00100),
  G('W',    10001,10001,10001,10101,10101,10101,01010),
  G('X',    10001,10001,01010,00100,01010,10001,10001),
  G('Y',    10001,10001,01010,00100,00100,00100,00100),
  G('Z',    11111,00001,00010,00100,01000,10000,11111),


  // Кириллица
  G(0x0410, 01110,10001,10001,11111,10001,10001,10001),   // А
  G(0x0411, 11111,10000,10000,11110,10001,10001,11110),   // Б
  G(0x0412, 11110,10001,10001,11110,10001,10001,11110),   // В
  G(0x0413, 11111,10000,10000,10000,10000,10000,10000),   // Г
  G(0x0414, 01110,01010,01010,01010,01010,11111,10001),   // Д
  G(0x0415, 11111,10000,10000,11110,10000,10000,11111),   // Е
  G(0x0401, 01010,00000,11111,10000,11110,10000,11111),   // Ё
  G(0x0416, 10101,10101,10101,01110,10101,10101,10101),   // Ж
  G(0x0417, 01110,10001,00001,00110,00001,10001,01110),   // З
  G(0x0418, 10001,10001,10011,10101,11001,10001,10001),   // И
  G(0x0419, 01010,00100,10001,10011,10101,11001,10001),   // Й
  G(0x041A, 10001,10010,10100,11000,10100,10010,10001),   // К
  G(0x041B, 00111,01001,01001,01001,01001,01001,10001),   // Л
  G(0x041C, 10001,11011,10101,10101,10001,10001,10001),   // М
  G(0x041D, 10001,10001,10001,11111,10001,10001,10001),   // Н
  G(0x041E, 01110,10001,10001,10001,10001,10001,01110),   // О
  G(0x041F, 11111,10001,10001,10001,10001,10001,10001),   // П
  G(0x0420, 11110,10001,10001,11110,10000,10000,10000),   // Р
  G(0x0421, 01110,10001,10000,10000,10000,10001,01110),   // С
  G(0x0422, 11111,00100,00100,00100,00100,00100,00100),   // Т
  G(0x0423, 10001,10001,10001,01111,00001,10001,01110),   // У
  G(0x0424, 00100,01110,10101,10101,10101,01110,00100),   // Ф
  G(0x0425, 10001,10001,01010,00100,01010,10001,10001),   // Х
  G(0x0426, 10010,10010,10010,10010,10010,11111,00001),   // Ц
  G(0x0427, 10001,10001,10001,01111,00001,00001,00001),   // Ч
  G(0x0428, 10101,10101,10101,10101,10101,10101,11111),   // Ш
  G(0x0429, 10101,10101,10101,10101,10101,11111,00001),   // Щ
  G(0x042A, 11000,01000,01000,01110,01001,01001,01110),   // Ъ
  G(0x042B, 10001,10001,10001,11101,10011,10011,11101),   // Ы
  G(0x042C, 10000,10000,10000,11110,10001,10001,11110),   // Ь
  G(0x042D, 01110,10001,00001,00111,00001,10001,01110),   // Э
  G(0x042E, 10010,10101,10101,11101,10101,10101,10010),   // Ю
  G(0x042F, 01111,10001,10001,01111,00101,01001,10001),   // Я
};


static const int FONT_N = sizeof(FONT) / sizeof(FONT[0]);


const Glyph* findGlyph(uint32_t cp)
{
  if (cp >= 'a' && cp <= 'z') cp -= 32;
  if (cp >= 0x0430 && cp <= 0x044F) cp -= 0x20;     // строчные -> заглавные
  if (cp == 0x0451) cp = 0x0401;                    // ё -> Ё
  for (int i = 0; i < FONT_N; i++) if (FONT[i].cp == cp) return &FONT[i];
  return findGlyph('?');
}


uint32_t nextCP(const char*& s)
{
  uint8_t c = (uint8_t)*s++;
  if (c < 0x80) return c;
  if ((c & 0xE0) == 0xC0 && *s)
  {
    uint32_t cp = (c & 0x1F) << 6;
    cp |= ((uint8_t)*s++) & 0x3F;
    return cp;
  }
  if ((c & 0xF0) == 0xE0 && s[0] && s[1])
  {
    uint32_t cp = (c & 0x0F) << 12;
    cp |= (((uint8_t)*s++) & 0x3F) << 6;
    cp |= ((uint8_t)*s++) & 0x3F;
    return cp;
  }
  return '?';
}


// ------------------------ Текстовые элементы -------------------------


enum { T_CITY, T_DATE, T_TEMP, T_DESC, T_FEELS, T_MINMAX, T_HUM, T_WIND, NUM_TXT };
TextItem txt[NUM_TXT];


enum { AL_LEFT, AL_RIGHT };


// Возвращает true, если текст/цвет изменился
bool setText(int slot, const char* s, int x, int y, int sx, int sy, C3 color, int align)
{
  TextItem& t = txt[slot];
  if (t.set && strcmp(t.str, s) == 0 && t.color.r == color.r && t.color.g == color.g &&
      t.color.b == color.b && t.sx == sx) return false;


  strncpy(t.str, s, TXT_MAXSTR - 1);
  t.str[TXT_MAXSTR - 1] = 0;


  int n = 0;
  const char* p = s;
  bool first = true;
  while (*p && n < TXT_MAXCOL - 6)
  {
    uint32_t cp = nextCP(p);
    if (!first) t.cols[n++] = 0;           // 1 пустой столбец между символами
    first = false;


    if (cp == ' ') { t.cols[n++] = 0; t.cols[n++] = 0; continue; }


    const Glyph* gl = findGlyph(cp);
    uint8_t m = 0;
    for (int j = 0; j < 7; j++) m |= gl->r[j];
    int c0 = 0, c1 = 4;
    bool digit = (cp >= '0' && cp <= '9');
    if (!digit && m)
    {
      while (c0 < 4 && !(m & (0x10 >> c0))) c0++;
      while (c1 > 0 && !(m & (0x10 >> c1))) c1--;
    }
    for (int c = c0; c <= c1 && n < TXT_MAXCOL; c++)
    {
      uint8_t col = 0;
      for (int j = 0; j < 7; j++) if (gl->r[j] & (0x10 >> c)) col |= (1 << j);
      t.cols[n++] = col;
    }
  }


  t.set  = true;
  t.ncol = n;
  t.sx = sx; t.sy = sy;
  t.color = color;
  t.ox = sx * 2 / 3; if (t.ox < 1) t.ox = 1;
  t.oy = sy * 2 / 3; if (t.oy < 1) t.oy = 1;
  int w = n * sx;
  t.x0 = (align == AL_RIGHT) ? x - w : x;
  t.y0 = y;
  t.bx0 = t.x0 - sx;            t.by0 = t.y0 - sy;
  t.bx1 = t.x0 + w + sx + t.ox; t.by1 = t.y0 + 8 * sy + t.oy;
  if (t.bx0 < 0) t.bx0 = 0;
  if (t.by0 < 0) t.by0 = 0;
  if (t.bx1 > DOTS) t.bx1 = DOTS;
  if (t.by1 > LINES) t.by1 = LINES;
  return true;
}


static inline int colBit(const TextItem& t, int i, int j)
{
  if (i < 0 || i >= t.ncol || j < 0 || j > 6) return 0;
  return (t.cols[i] >> j) & 1;
}


// Покрытие 0..256 с плавным (билинейным) масштабированием глифов
int textCov(const TextItem& t, int x, int y)
{
  int fx = ((((x - t.x0) * 2 + 1) * 128) / t.sx) - 128;
  int fy = ((((y - t.y0) * 2 + 1) * 128) / t.sy) - 128;
  int i = fx >> 8, u = fx & 255;
  int j = fy >> 8, v = fy & 255;
  int b00 = colBit(t, i, j),     b10 = colBit(t, i + 1, j);
  int b01 = colBit(t, i, j + 1), b11 = colBit(t, i + 1, j + 1);
  if (!(b00 | b10 | b01 | b11)) return 0;
  int top = b00 * (256 - u) + b10 * u;
  int bot = b01 * (256 - u) + b11 * u;
  int val = (top * (256 - v) + bot * v) >> 8;
  int cov = (val - 70) * 256 / 90;
  return cov < 0 ? 0 : (cov > 256 ? 256 : cov);
}


// Для крупного текста: пиксели шрифта соединяются «штрихами» (капсулами),
// диагонали становятся ровными — получается гладкий векторный вид
static inline float segD2(float px, float py, float ax, float ay, float bx, float by)
{
  float vx = bx - ax, vy = by - ay, wx = px - ax, wy = py - ay;
  float L2 = vx * vx + vy * vy;
  float t = L2 > 0 ? (wx * vx + wy * vy) / L2 : 0.0f;
  t = clampf(t, 0.0f, 1.0f);
  float dx = wx - t * vx, dy = wy - t * vy;
  return dx * dx + dy * dy;
}


int strokeCov(const TextItem& t, int x, int y)
{
  float fx = (x - t.x0 + 0.5f) / t.sx;
  float fy = (y - t.y0 + 0.5f) / t.sy;
  int ic = (int)floorf(fx), jc = (int)floorf(fy);
  float best = 100.0f;


  for (int j = jc - 2; j <= jc + 1; j++)
    for (int i = ic - 2; i <= ic + 2; i++)
    {
      if (!colBit(t, i, j)) continue;
      float ax = i + 0.5f, ay = j + 0.5f;
      float d = (fx - ax) * (fx - ax) + (fy - ay) * (fy - ay);
      if (d < best) best = d;
      if (colBit(t, i + 1, j))
      { d = segD2(fx, fy, ax, ay, ax + 1, ay); if (d < best) best = d; }
      if (colBit(t, i, j + 1))
      { d = segD2(fx, fy, ax, ay, ax, ay + 1); if (d < best) best = d; }
      if (colBit(t, i + 1, j + 1) && !colBit(t, i + 1, j) && !colBit(t, i, j + 1))
      { d = segD2(fx, fy, ax, ay, ax + 1, ay + 1); if (d < best) best = d; }
      if (colBit(t, i - 1, j + 1) && !colBit(t, i - 1, j) && !colBit(t, i, j + 1))
      { d = segD2(fx, fy, ax, ay, ax - 1, ay + 1); if (d < best) best = d; }
    }


  const float R = 0.56f;
  float aa = 1.4f / t.sx;
  float cov = (R + aa * 0.5f - sqrtf(best)) / aa;
  if (cov <= 0) return 0;
  if (cov >= 1) return 256;
  return (int)(cov * 256);
}


static inline int glyphCov(const TextItem& t, int x, int y)
{
  return (t.sx >= 6) ? strokeCov(t, x, y) : textCov(t, x, y);
}


// =====================================================================
//                          ПОГОДА И СОСТОЯНИЕ
// =====================================================================


Weather wx = {};                    // используется UI
Weather wxPending = {};
volatile bool wxNew   = false;
volatile bool wxForce = false;
portMUX_TYPE  wxMux   = portMUX_INITIALIZER_UNLOCKED;


enum { NET_CONNECTING, NET_ONLINE, NET_ERROR };
volatile int netState = NET_CONNECTING;
char lastNetMsg[64] = "";


enum { IC_CLEAR, IC_FEW, IC_PARTLY, IC_OVERCAST, IC_FOG, IC_DRIZZLE, IC_RAIN, IC_SNOW, IC_STORM };
enum { MODE_DAY, MODE_GOLDEN, MODE_NIGHT };


int iconFor(int code)
{
  if (code == 0) return IC_CLEAR;
  if (code == 1) return IC_FEW;
  if (code == 2) return IC_PARTLY;
  if (code == 3) return IC_OVERCAST;
  if (code == 45 || code == 48) return IC_FOG;
  if (code >= 51 && code <= 57) return IC_DRIZZLE;
  if ((code >= 61 && code <= 67) || (code >= 80 && code <= 82)) return IC_RAIN;
  if ((code >= 71 && code <= 77) || code == 85 || code == 86) return IC_SNOW;
  if (code >= 95) return IC_STORM;
  return IC_PARTLY;
}


const char* descFor(int code)
{
  switch (code)
  {
    case 0:  return "ЯСНО";
    case 1:  return "МАЛООБЛАЧНО";
    case 2:  return "ОБЛАЧНО";
    case 3:  return "ПАСМУРНО";
    case 45: case 48: return "ТУМАН";
    case 51: case 53: case 55: return "МОРОСЬ";
    case 56: case 57: return "ЛЕДЯНАЯ МОРОСЬ";
    case 61: return "СЛАБЫЙ ДОЖДЬ";
    case 63: return "ДОЖДЬ";
    case 65: return "СИЛЬНЫЙ ДОЖДЬ";
    case 66: case 67: return "ЛЕДЯНОЙ ДОЖДЬ";
    case 71: return "СЛАБЫЙ СНЕГ";
    case 73: return "СНЕГ";
    case 75: return "СИЛЬНЫЙ СНЕГ";
    case 77: return "СНЕЖНАЯ КРУПА";
    case 80: case 81: return "ЛИВЕНЬ";
    case 82: return "СИЛЬНЫЙ ЛИВЕНЬ";
    case 85: case 86: return "СНЕГОПАД";
    case 95: return "ГРОЗА";
    case 96: case 99: return "ГРОЗА С ГРАДОМ";
    default: return "ОБЛАЧНО";
  }
}


const char* windDirName(int deg)
{
  static const char* D[8] = { "С", "СВ", "В", "ЮВ", "Ю", "ЮЗ", "З", "СЗ" };
  int i = ((deg % 360 + 360) % 360 + 22) / 45;
  return D[i % 8];
}


// Демо-режим: {код погоды, режим неба}; 0 = настоящая погода
struct Demo { int code; int mode; };
static const Demo DEMOS[] =
{
  {0, MODE_DAY}, {0, MODE_NIGHT}, {1, MODE_DAY}, {2, MODE_NIGHT}, {3, MODE_DAY},
  {45, MODE_DAY}, {53, MODE_DAY}, {63, MODE_DAY}, {73, MODE_NIGHT}, {95, MODE_DAY},
  {0, MODE_GOLDEN},
};
static const int NUM_DEMOS = sizeof(DEMOS) / sizeof(DEMOS[0]);
int demoIdx = 0;                     // 0 = выкл, 1..NUM_DEMOS


// =====================================================================
//                         ПАЛИТРА И ФОН
// =====================================================================


// SEA_Y (линия горизонта) определён выше, в разделе вывода


struct Palette
{
  C3 skyTop, skyBot, seaTop, seaBot, seaHL, mount;
  C3 text, accent, clock;
  bool stars;
  int  night;                        // 1 ночью (тусклее облака)
  int  golden;
};


Palette pal;
C3 skyRow[SEA_Y];
uint8_t mountH[DOTS];


int curIcon = IC_CLEAR;
int curMode = MODE_NIGHT;
uint32_t animT = 0;                  // счётчик кадров анимации
float secFrac = 0;                   // доля минуты (полоска секунд)
float subSec  = 0;                   // доля секунды (мигание двоеточия)
bool  colonOn = true;                // двоеточие: горит половину периода, половину погашено
#define COLON_PERIOD_S  2            // период мигания двоеточия, секунд (1 с горит, 1 с нет)


static C3 desat(const C3& c, int k)  // приглушить цвет к серому
{
  int a = (c.r + c.g + c.b) * 29 / 100;
  return mix(c, {a, a, a + 6}, k);
}


void applyPalette(int mode, int icon)
{
  if (mode == MODE_DAY)
  {
    pal.skyTop = {28, 92, 190};   pal.skyBot = {125, 185, 240};
    pal.seaTop = {35, 115, 180};  pal.seaBot = {8, 50, 112};   pal.seaHL = {210, 235, 255};
    pal.mount  = {75, 110, 150};
    pal.clock  = {255, 255, 255}; pal.accent = {255, 228, 150};
  }
  else if (mode == MODE_GOLDEN)
  {
    pal.skyTop = {45, 50, 125};   pal.skyBot = {250, 150, 95};
    pal.seaTop = {120, 80, 125};  pal.seaBot = {25, 22, 65};   pal.seaHL = {255, 195, 130};
    pal.mount  = {85, 50, 85};
    pal.clock  = {255, 235, 205}; pal.accent = {255, 215, 140};
  }
  else
  {
    pal.skyTop = {3, 5, 20};      pal.skyBot = {20, 30, 80};
    pal.seaTop = {18, 28, 66};    pal.seaBot = {2, 5, 18};     pal.seaHL = {110, 140, 205};
    pal.mount  = {10, 14, 36};
    pal.clock  = {165, 215, 255}; pal.accent = {255, 205, 125};
  }
  pal.text = {245, 246, 252};


  int k = 0;                                   // насколько «серая» погода
  if (icon == IC_PARTLY)   k = 50;
  if (icon == IC_OVERCAST) k = 150;
  if (icon == IC_FOG)      k = 170;
  if (icon == IC_DRIZZLE || icon == IC_RAIN || icon == IC_SNOW) k = 160;
  if (icon == IC_STORM)    k = 190;


  pal.skyTop = desat(pal.skyTop, k);  pal.skyBot = desat(pal.skyBot, k);
  pal.seaTop = desat(pal.seaTop, k);  pal.seaBot = desat(pal.seaBot, k);
  pal.mount  = desat(pal.mount, k);


  pal.stars  = (mode == MODE_NIGHT) && (icon <= IC_PARTLY);
  pal.night  = (mode == MODE_NIGHT);
  pal.golden = (mode == MODE_GOLDEN);


  for (int y = 0; y < SEA_Y; y++)
    skyRow[y] = mix(pal.skyTop, pal.skyBot, y * 256 / (SEA_Y - 1));
  buildBgTables();
}


void initMountains()
{
  for (int x = 0; x < DOTS; x++)
  {
    float hl = 15.0f - fabsf(x - 85.0f) * 0.11f + 2.5f * sinf(x * 0.09f);
    float hr = 11.0f - fabsf(x - 410.0f) * 0.09f + 2.0f * sinf(x * 0.13f + 2.0f);
    float h = hl > hr ? hl : hr;
    mountH[x] = h > 0 ? (uint8_t)h : 0;
  }
}


// Цвет -> 2 слова DMA (4 такта пикселя матрицы) с текущим порядком тактов
static void colorWords(const C3& c, uint32_t out[2])
{
  uint16_t v[4] = { (uint16_t)(0x300 | clamp8(c.r)), (uint16_t)(0x300 | clamp8(c.g)),
                    (uint16_t)(0x300 | clamp8(c.b)), (uint16_t)0x300 };
  int r = slotRot & 3;
  int i0 = (0 - r) & 3, i1 = (1 - r) & 3, i2 = (2 - r) & 3, i3 = (3 - r) & 3;
  out[0] = pairWord(v[i0], v[i1], pairSwap);
  out[1] = pairWord(v[i2], v[i3], pairSwap);
}

// Пересчитать таблицы фона (после смены палитры, зеркала, порядка тактов)
void buildBgTables()
{
  for (int y = 0; y < SEA_Y; y++) colorWords(skyRow[y], skyW[y]);
  for (int dy = 1; dy <= MOUNT_MAXH; dy++)
    for (int h = dy; h <= MOUNT_MAXH; h++)
      colorWords(mix(pal.mount, skyRow[SEA_Y - 1], dy * 80 / (h + 1)), mountW[dy][h]);
  for (int xp = 0; xp < 320; xp++)
  {
    int lx = mirX ? (319 - xp) : xp;               // колонка матрицы -> логическая
    int x = (lx * 3 + 1) / 2;                      // -> точка сцены 0..479
    if (x > DOTS - 1) x = DOTS - 1;
    int h = mountH[x];
    mhP[xp] = (uint8_t)(h > MOUNT_MAXH ? MOUNT_MAXH : h);
  }
}


// ------------------------------ Звёзды -------------------------------
#define STAR_CW     160              // ячейка звезды = 3 точки x 2 строки
#define STAR_CH     98
#define NUM_STARS   40
uint8_t starBits[(STAR_CW * STAR_CH + 7) / 8];
struct StarCell { uint8_t cx, cy; };
StarCell starList[NUM_STARS];


void initStars()
{
  memset(starBits, 0, sizeof(starBits));
  uint32_t s = 2024;
  int n = 0;
  while (n < NUM_STARS)
  {
    s = s * 1103515245UL + 12345UL;
    int cx = (s >> 8) % STAR_CW;
    s = s * 1103515245UL + 12345UL;
    int cy = (s >> 8) % STAR_CH;
    int x = cx * 3, y = cy * 2;
    if (x < 306 && y >= 22 && y < 114) continue;     // не под часами
    if (x < 200 && y >= 116 && y < 168) continue;    // не под большой температурой
    if (y >= SEA_Y - 18) continue;                   // не у гор
    int idx = cy * STAR_CW + cx;
    starBits[idx >> 3] |= (1 << (idx & 7));
    starList[n].cx = cx; starList[n].cy = cy;
    n++;
  }
}


// ------------------------------- Море --------------------------------
C3 seaPixel(int x, int y)
{
  int d = y - SEA_Y;                                         // 0..31
  C3 c = mix(pal.seaTop, pal.seaBot, d * 256 / (LINES - 1 - SEA_Y));
  int rowW = SIN8[(uint8_t)(1400 / (d + 5) + animT * 3)];
  uint32_t ph = (uint32_t)(animT * 2 + d * 37) * 256u + (uint32_t)x * ((30u << 8) / (d + 7));
  int s  = rowW + (SIN8[(uint8_t)(ph >> 8)] >> 1);
  int hl = s - 70;
  if (hl > 0)
  {
    hl = hl * 2;                                             // 0..240
    int a = hl * (pal.night ? 120 : 170) / 256;
    c = mix(c, pal.seaHL, a);
  }
  return c;
}


// =====================================================================
//                             ЧАСЫ
// =====================================================================


#define CLK_Y0    28
#define DIG_W     54
#define SLANT     0.08f
#define SEG_R     8.0f
#define CLK_X0    10
#define CLK_X1    304
#define CLK_RY0   24
#define CLK_RY1   106
#define COLON_X   146
#define COLON_RX0 144
#define COLON_RX1 172
#define COLON_RY0 42
#define COLON_RY1 90
#define SEC_X0    14
#define SEC_X1    300
#define SEC_Y0    109
#define SEC_Y1    112


const int digX[4] = {14, 78, 172, 236};


// Сегменты в единицах цифры (ширина 108, высота 216): ax, ay, bx, by
static const float SEGS[7][4] =
{
  { 21,  10,  87,  10 },   // a
  { 98,  21,  98,  97 },   // b
  { 98, 119,  98, 195 },   // c
  { 21, 206,  87, 206 },   // d
  { 10, 119,  10, 195 },   // e
  { 10,  21,  10,  97 },   // f
  { 21, 108,  87, 108 },   // g
};


static const uint8_t DIGIT_SEGS[11] =
{ 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x40 /* '-' */ };


uint8_t  segTarget[4] = {0x40, 0x40, 0x40, 0x40};
uint16_t segLev[4][7];
bool     clockAnimating = true;


static inline float segCov(float u, float v, const float* s)
{
  float dx, dy;
  if (s[1] == s[3]) { dx = u < s[0] ? s[0] - u : (u > s[2] ? u - s[2] : 0); dy = v - s[1]; }
  else              { dy = v < s[1] ? s[1] - v : (v > s[3] ? v - s[3] : 0); dx = u - s[0]; }
  float lim = SEG_R + 1.5f;
  if (dx > lim || dx < -lim || dy > lim || dy < -lim) return 0.0f;
  float d = sqrtf(dx * dx + dy * dy);
  return clampf((lim - d) / 3.0f, 0.0f, 1.0f);
}


C3 clockLayer(C3 c, int x, int y)
{
  float v  = (y - CLK_Y0) * 3 + 1.5f;


  // Двоеточие
  if (x >= COLON_RX0 && x < COLON_RX1)
  {
    float u  = (x - COLON_X) * 2 + 1.0f;
    float u0 = u - SLANT * (216.0f - v);
    float a1 = discF(u0 - 12, v - 70, 9);
    float a2 = discF(u0 - 12, v - 146, 9);
    float a  = a1 > a2 ? a1 : a2;
    if (!colonOn) return c;                        // погашено — только фон
    if (a > 0)
    {
      float s1 = discF(u0 - 12 - 5, v - 70 - 7, 9), s2 = discF(u0 - 12 - 5, v - 146 - 7, 9);
      float sh = s1 > s2 ? s1 : s2;
      c = mixF(c, {0, 0, 15}, sh * 0.45f);
      c = mixF(c, pal.clock, a);
    }
    else
    {
      float s1 = discF(u0 - 12 - 5, v - 70 - 7, 9), s2 = discF(u0 - 12 - 5, v - 146 - 7, 9);
      float sh = s1 > s2 ? s1 : s2;
      if (sh > 0) c = mixF(c, {0, 0, 15}, sh * 0.45f);
    }
    return c;
  }


  int k = -1;
  for (int i = 0; i < 4; i++)
    if (x >= digX[i] - 1 && x < digX[i] + DIG_W + 9) { k = i; break; }
  if (k < 0) return c;
  if (v < -12 || v > 230) return c;


  float u  = (x - digX[k]) * 2 + 1.0f;
  float u0 = u - SLANT * (216.0f - v);


  float lit = 0, shadow = 0;
  for (int s = 0; s < 7; s++)
  {
    float lev = segLev[k][s] / 256.0f;
    float cv  = segCov(u0, v, SEGS[s]);
    float a   = cv * (0.10f + 0.90f * lev);              // погашенный сегмент — «призрак» 10%
    if (a > lit) lit = a;
    if (lev > 0.01f)
    {
      float sc = segCov(u0 - 5, v - 7, SEGS[s]) * lev;   // тень вниз-вправо
      if (sc > shadow) shadow = sc;
    }
  }
  if (shadow > 0) c = mixF(c, {0, 0, 15}, shadow * 0.45f);
  if (lit > 0)    c = mixF(c, pal.clock, lit);
  return c;
}


C3 secLayer(C3 c, int x)
{
  float fill = SEC_X0 + secFrac * (SEC_X1 - SEC_X0);
  int a = (x < fill) ? 200 : 45;
  float dh = fabsf(x - fill);
  if (dh < 8) a += (int)((8 - dh) * 7);
  return mix(c, pal.clock, a > 256 ? 256 : a);
}


// =====================================================================
//                         ИКОНКА ПОГОДЫ
// =====================================================================


#define ICON_X0 318
#define ICON_X1 476
#define ICON_Y0 20
#define ICON_Y1 116


// Координаты иконки — в единицах X = x*2, Y = y*3 (центр ~ 794, 204)


static const int8_t  DROP_DX[12] = { -62, -40, -18, 4, 26, 48, -52, -30, -8, 14, 36, 58 };
static const uint8_t DROP_PH[12] = { 0, 140, 60, 200, 30, 170, 90, 10, 220, 120, 50, 180 };
#define DROP_L 110.0f


// Параметры анимации считаются один раз за кадр (а не для каждой точки)
struct IconAnim
{
  float drift, rot, pulse;
  bool  flash;
  float dropPos[12], dropFade[12];
  float flakeX[10], flakeY[10], flakeFade[10];
} ia;


void prepIconAnim()
{
  ia.drift = 10.0f * SIN8[(uint8_t)(animT / 2)] / 127.0f;
  ia.rot   = (animT % 105) * (1.2566371f / 105.0f);
  ia.pulse = 0.85f + 0.15f * SIN8[(uint8_t)(animT * 2)] / 127.0f;
  uint32_t cyc = animT % 110;
  ia.flash = (cyc < 4) || (cyc >= 7 && cyc < 10);


  float speed = (curIcon == IC_DRIZZLE) ? 1.4f : (curIcon == IC_STORM ? 2.8f : 2.6f);
  uint32_t tt = animT % 1100;
  for (int i = 0; i < 12; i++)
  {
    float pos = fmodf(tt * speed + DROP_PH[i] * (DROP_L / 256.0f), DROP_L);
    ia.dropPos[i]  = pos;
    ia.dropFade[i] = 1.0f - (pos / DROP_L) * (pos / DROP_L);
  }
  for (int i = 0; i < 10; i++)
  {
    float pos = fmodf(tt * 0.9f + DROP_PH[i] * (DROP_L / 256.0f), DROP_L);
    ia.flakeX[i]    = DROP_DX[i] + 9.0f * SIN8[(uint8_t)(animT * 3 + DROP_PH[i])] / 127.0f;
    ia.flakeY[i]    = pos;
    ia.flakeFade[i] = 1.0f - (pos / DROP_L) * (pos / DROP_L) * 0.8f;
  }
}


C3 cloudTint(const C3& c)
{
  if (pal.night)  return mix(c, {40, 46, 75}, 150);
  if (pal.golden) return mix(c, {255, 170, 140}, 60);
  return c;
}


// Облако: возвращает покрытие, в shade кладёт 0 (верх) .. 1 (низ)
float cloudCov(float X, float Y, float cx, float cy, float s, float& shade)
{
  if (X < cx - 98 * s || X > cx + 88 * s || Y < cy - 76 * s || Y > cy + 48 * s) return 0.0f;
  float a = discF(X - (cx - 58 * s), Y - (cy + 8 * s),  36 * s);
  float b = discF(X - (cx - 8 * s),  Y - (cy - 22 * s), 50 * s);
  float c = discF(X - (cx + 46 * s), Y - (cy + 2 * s),  38 * s);
  float d = capsuleF(X, Y, cx - 58 * s, cy + 20 * s, cx + 46 * s, cy + 20 * s, 24 * s);
  float m = a;
  if (b > m) m = b;
  if (c > m) m = c;
  if (d > m) m = d;
  shade = clampf((Y - (cy - 72 * s)) / (116 * s), 0.0f, 1.0f);
  return m;
}


C3 drawCloud(C3 c, float X, float Y, float cx, float cy, float s, C3 top, C3 bot)
{
  float shade;
  // мягкая тень под облаком
  float sh = cloudCov(X - 6, Y - 9, cx, cy, s, shade);
  if (sh > 0) c = mixF(c, {0, 0, 20}, sh * 0.25f);
  float cv = cloudCov(X, Y, cx, cy, s, shade);
  if (cv > 0) c = mixF(c, cloudTint(mixF(top, bot, shade)), cv);
  return c;
}


C3 drawSun(C3 c, float X, float Y, float cx, float cy, float r, bool rays)
{
  float dx = X - cx, dy = Y - cy;
  float d2 = dx * dx + dy * dy;
  float gr = r * 2.6f;
  if (d2 > gr * gr) return c;
  float d = sqrtf(d2);


  // свечение
  float g = 1.0f - d / gr;
  c = mixF(c, {255, 215, 120}, g * g * 0.55f * ia.pulse);


  // лучи
  if (rays && d > r * 1.15f && d < r * 1.85f)
  {
    const float sector = 6.2831853f / 10.0f;
    float ang = atan2f(dy, dx) - ia.rot;
    float idxf = floorf(ang / sector + 0.5f);
    float delta = fabsf(ang - idxf * sector);
    int   ri = ((int)idxf % 2 + 2) % 2;
    float r2 = ri ? r * 1.62f : r * 1.85f;
    float perp = d * delta;
    float aw = clampf((5.0f + 1.5f - perp) / 3.0f, 0.0f, 1.0f);
    float ar = clampf((d - r * 1.28f + 1.5f) / 3.0f, 0.0f, 1.0f) *
               clampf((r2 - d + 1.5f) / 3.0f, 0.0f, 1.0f);
    float a = aw * ar;
    if (a > 0) c = mixF(c, {255, 205, 80}, a);
  }


  // диск
  float disc = discF(dx, dy, r);
  if (disc > 0)
  {
    C3 core = mixF({255, 248, 185}, {255, 190, 60}, clampf(d / r, 0.0f, 1.0f));
    c = mixF(c, core, disc);
  }
  return c;
}


C3 drawMoon(C3 c, float X, float Y, float cx, float cy, float r)
{
  float dx = X - cx, dy = Y - cy;
  float d2 = dx * dx + dy * dy;
  float gr = r * 2.3f;
  if (d2 > gr * gr) return c;
  float d = sqrtf(d2);
  float g = 1.0f - d / gr;
  c = mixF(c, {170, 185, 235}, g * g * 0.35f);
  float body = discF(dx, dy, r);
  float cut  = discF(dx + r * 0.45f, dy + r * 0.32f, r * 0.88f);
  float a = body * (1.0f - cut);
  if (a > 0) c = mixF(c, mixF({250, 245, 215}, {215, 210, 185}, clampf(d / r, 0.0f, 1.0f)), a);
  return c;
}


C3 drawDrops(C3 c, float X, float Y, float cx, float y0, int n, float len, float r)
{
  for (int i = 0; i < n; i++)
  {
    float dxp = cx + DROP_DX[i];
    if (X < dxp - 14 || X > dxp + 8) continue;
    float yy = y0 + ia.dropPos[i];
    if (Y < yy - 4 || Y > yy + len + 6) continue;
    float a = capsuleF(X, Y, dxp, yy, dxp - len * 0.3f, yy + len, r);
    if (a > 0)
      c = mixF(c, pal.night ? C3{120, 160, 230} : C3{140, 195, 255}, a * ia.dropFade[i]);
  }
  return c;
}


C3 drawFlakes(C3 c, float X, float Y, float cx, float y0)
{
  for (int i = 0; i < 10; i++)
  {
    float fx = cx + ia.flakeX[i];
    float fy = y0 + ia.flakeY[i];
    if (fabsf(X - fx) > 9 || fabsf(Y - fy) > 9) continue;
    float a = discF(X - fx, Y - fy, 5.5f);
    if (a > 0) c = mixF(c, {255, 255, 255}, a * ia.flakeFade[i]);
  }
  return c;
}


C3 iconLayer(C3 c, int x, int y)
{
  float X = x * 2 + 1.0f, Y = y * 3 + 1.5f;
  bool night = pal.night;
  float drift = ia.drift;


  const C3 W1 = {252, 252, 255}, W2 = {205, 215, 232};    // светлое облако
  const C3 M1 = {212, 218, 230}, M2 = {160, 168, 184};    // серое
  const C3 D1 = {165, 170, 185}, D2 = {120, 126, 142};    // тёмное


  switch (curIcon)
  {
    case IC_CLEAR:
      c = night ? drawMoon(c, X, Y, 794, 200, 52) : drawSun(c, X, Y, 794, 200, 50, true);
      break;


    case IC_FEW:
      c = night ? drawMoon(c, X, Y, 780, 180, 46) : drawSun(c, X, Y, 780, 182, 44, true);
      c = drawCloud(c, X, Y, 868 + drift * 0.6f, 272, 0.55f, W1, W2);
      break;


    case IC_PARTLY:
      c = night ? drawMoon(c, X, Y, 735, 150, 38) : drawSun(c, X, Y, 735, 152, 38, true);
      c = drawCloud(c, X, Y, 825 + drift, 236, 0.85f, W1, W2);
      break;


    case IC_OVERCAST:
      c = drawCloud(c, X, Y, 852 - drift * 0.7f, 168, 0.80f, D1, D2);
      c = drawCloud(c, X, Y, 772 + drift, 238, 1.00f, M1, M2);
      break;


    case IC_FOG:
    {
      c = drawCloud(c, X, Y, 794 + drift * 0.5f, 152, 0.85f, M1, M2);
      for (int i = 0; i < 4; i++)
      {
        float yy  = 214 + i * 34;
        float off = 22.0f * SIN8[(uint8_t)(animT * 2 + i * 64)] / 127.0f;
        float x0 = 690 + (i & 1) * 26 + off, x1 = 900 - ((i + 1) & 1) * 22 + off;
        float a = capsuleF(X, Y, x0, yy, x1, yy, 8);
        if (a > 0) c = mixF(c, cloudTint({215, 220, 230}), a * 0.85f);
      }
      break;
    }


    case IC_DRIZZLE:
      c = drawDrops(c, X, Y, 794, 222, 6, 12, 2.6f);
      c = drawCloud(c, X, Y, 794 + drift, 168, 1.0f, M1, M2);
      break;


    case IC_RAIN:
      c = drawDrops(c, X, Y, 794, 222, 12, 20, 3.4f);
      c = drawCloud(c, X, Y, 794 + drift, 168, 1.0f, D1, D2);
      break;


    case IC_SNOW:
      c = drawFlakes(c, X, Y, 794, 222);
      c = drawCloud(c, X, Y, 794 + drift, 168, 1.0f, M1, M2);
      break;


    case IC_STORM:
    {
      bool flash = ia.flash;
      c = drawDrops(c, X, Y, 794, 222, 8, 20, 3.2f);
      if (flash)
      {
        const float P[4][2] = { {800, 222}, {778, 262}, {800, 262}, {774, 326} };
        float a = 0;
        for (int i = 0; i < 3; i++)
        {
          float s = capsuleF(X, Y, P[i][0], P[i][1], P[i + 1][0], P[i + 1][1], 5);
          if (s > a) a = s;
        }
        if (a > 0) c = mixF(c, {255, 238, 120}, a);
      }
      C3 t1 = flash ? C3{235, 235, 255} : C3{125, 130, 148};
      C3 t2 = flash ? C3{185, 185, 215} : C3{75, 80, 96};
      c = drawCloud(c, X, Y, 794 + drift * 0.5f, 165, 1.05f, t1, t2);
      break;
    }
  }
  return c;
}


// =====================================================================
//                       КОМПОЗИЦИЯ СЦЕНЫ
// =====================================================================


C3 textLayer(C3 c, const TextItem& t, int x, int y)
{
  int s = glyphCov(t, x - t.ox, y - t.oy);
  if (s) c = mix(c, {0, 0, 15}, s * 150 / 256);
  int v = glyphCov(t, x, y);
  if (v) c = mix(c, t.color, v);
  return c;
}


C3 scene(int x, int y)
{
  C3 c;
  bool star = false;
  if (y < SEA_Y)
  {
    c = skyRow[y];
    if (pal.stars && (y >> 1) < STAR_CH)
    {
      int idx = (y >> 1) * STAR_CW + x / 3;
      if (starBits[idx >> 3] & (1 << (idx & 7)))
      {
        uint32_t h = hash32(idx);
        int b = 95 + SIN8[(uint8_t)((h & 255) + animT * (2 + ((h >> 8) & 3)))];
        if (b > 0) { c = mix(c, {255, 252, 235}, b); star = true; }
      }
    }
    int h = mountH[x];
    if (h && y >= SEA_Y - h)
    {
      c = mix(pal.mount, skyRow[SEA_Y - 1], (SEA_Y - y) * 80 / (h + 1));
      star = false;
    }
  }
  else
  {
    c = seaPixel(x, y);
  }


  C3 bg = c;
  if (x >= ICON_X0 && x < ICON_X1 && y >= ICON_Y0 && y < ICON_Y1) c = iconLayer(c, x, y);
  if (x >= CLK_X0  && x < CLK_X1  && y >= CLK_RY0 && y < CLK_RY1) c = clockLayer(c, x, y);
  if (x >= SEC_X0  && x < SEC_X1  && y >= SEC_Y0  && y < SEC_Y1)  c = secLayer(c, x);


  for (int i = 0; i < NUM_TXT; i++)
  {
    const TextItem& t = txt[i];
    if (t.ncol && x >= t.bx0 && x < t.bx1 && y >= t.by0 && y < t.by1)
      c = textLayer(c, t, x, y);
  }
  // если поверх фона что-то нарисовано (текст, цифры, иконка) — без дизеринга
  sceneIsBg = (c.r == bg.r && c.g == bg.g && c.b == bg.b);
  scenePureSky = (y < SEA_Y) && !star && sceneIsBg;
  return c;
}


void redrawRect(int x0, int y0, int x1, int y1)
{
  if (x0 < 0) x0 = 0;
  if (y0 < 0) y0 = 0;
  if (x1 > DOTS) x1 = DOTS;
  if (y1 > LINES) y1 = LINES;
  for (int y = y0; y < y1; y++)
    renderRow(y, x0, x1, scene);
}


// =====================================================================
//                         ТЕКСТЫ ЭКРАНА
// =====================================================================


static const char* WD[7]  = { "ВС", "ПН", "ВТ", "СР", "ЧТ", "ПТ", "СБ" };
static const char* MON[12] = { "ЯНВ", "ФЕВ", "МАР", "АПР", "МАЯ", "ИЮН",
                               "ИЮЛ", "АВГ", "СЕН", "ОКТ", "НОЯ", "ДЕК" };


int iround(float v) { return (int)lroundf(v); }


// Возвращает true, если что-то поменялось
bool rebuildTexts(bool timeValid, const struct tm& lt, int code)
{
  bool ch = false;
  char b[TXT_MAXSTR];


  ch |= setText(T_CITY, "МАЛАГА", 12, 5, 3, 2, pal.accent, AL_LEFT);


  if (timeValid) snprintf(b, sizeof(b), "%s, %d %s", WD[lt.tm_wday], lt.tm_mday, MON[lt.tm_mon]);
  else           snprintf(b, sizeof(b), "%s", netState == NET_CONNECTING ? "WI-FI..." : "ВРЕМЯ...");
  ch |= setText(T_DATE, b, 468, 5, 3, 2, pal.text, AL_RIGHT);


  if (wx.valid) snprintf(b, sizeof(b), "%d°", iround(wx.temp));
  else          snprintf(b, sizeof(b), "--°");
  ch |= setText(T_TEMP, b, 12, 122, 9, 6, pal.text, AL_LEFT);


  C3 dc = pal.text;
  int ic = iconFor(code);
  if (ic <= IC_PARTLY && !pal.night) dc = {255, 220, 130};
  if (ic == IC_DRIZZLE || ic == IC_RAIN || ic == IC_STORM) dc = {160, 205, 255};


  const char* desc;
  if (wx.valid || demoIdx) desc = descFor(code);
  else desc = (netState == NET_CONNECTING) ? "ПОДКЛЮЧЕНИЕ..." : "НЕТ ДАННЫХ";
  ch |= setText(T_DESC, desc, 200, 122, 3, 2, dc, AL_LEFT);


  if (wx.valid)
  {
    snprintf(b, sizeof(b), "ОЩУЩАЕТСЯ %d°", iround(wx.feels));
    ch |= setText(T_FEELS, b, 200, 140, 3, 2, pal.text, AL_LEFT);
    snprintf(b, sizeof(b), "↑%d°  ↓%d°", iround(wx.tmax), iround(wx.tmin));
    ch |= setText(T_MINMAX, b, 200, 158, 3, 2, pal.text, AL_LEFT);
    snprintf(b, sizeof(b), "ВЛАЖН. %d%%", wx.hum);
    ch |= setText(T_HUM, b, 12, 178, 3, 2, pal.text, AL_LEFT);
    snprintf(b, sizeof(b), "ВЕТЕР %s %d М/С", windDirName(wx.windDir), iround(wx.wind));
    ch |= setText(T_WIND, b, 468, 178, 3, 2, pal.text, AL_RIGHT);
  }
  else
  {
    ch |= setText(T_FEELS, "", 200, 140, 3, 2, pal.text, AL_LEFT);
    ch |= setText(T_MINMAX, "", 200, 158, 3, 2, pal.text, AL_LEFT);
    ch |= setText(T_HUM, "", 12, 178, 3, 2, pal.text, AL_LEFT);
    ch |= setText(T_WIND, "", 468, 178, 3, 2, pal.text, AL_RIGHT);
  }
  return ch;
}


int computeMode(bool timeValid, const struct tm& lt)
{
  int sr = (wx.valid && wx.sunrise >= 0) ? wx.sunrise : 8 * 60;
  int ss = (wx.valid && wx.sunset  >= 0) ? wx.sunset  : 20 * 60;
  if (!timeValid) return (wx.valid && wx.isDay) ? MODE_DAY : MODE_NIGHT;
  int m = lt.tm_hour * 60 + lt.tm_min;
  if (abs(m - sr) <= 35 || abs(m - ss) <= 35) return MODE_GOLDEN;
  return (m > sr && m < ss) ? MODE_DAY : MODE_NIGHT;
}


// =====================================================================
//                     СЕТЬ: Wi-Fi, NTP, Open-Meteo
// =====================================================================


static bool findNum(const char* from, const char* key, float& out)
{
  const char* p = strstr(from, key);
  if (!p) return false;
  p += strlen(key);
  while (*p == ' ' || *p == '[' || *p == ':') p++;
  char* e;
  out = strtof(p, &e);
  return e != p;
}


static int findHHMM(const char* from, const char* key)
{
  const char* p = strstr(from, key);
  if (!p) return -1;
  p = strchr(p, 'T');
  if (!p || !isdigit((unsigned char)p[1])) return -1;
  int h = atoi(p + 1);
  int m = atoi(p + 4);
  return h * 60 + m;
}


bool parseWeather(const char* js, Weather& w)
{
  const char* cur = strstr(js, "\"current\":{");
  const char* day = strstr(js, "\"daily\":{");
  if (!cur) return false;


  float v;
  w = {};
  if (!findNum(cur, "\"temperature_2m\":", w.temp)) return false;
  if (findNum(cur, "\"relative_humidity_2m\":", v)) w.hum = iround(v);
  if (!findNum(cur, "\"apparent_temperature\":", w.feels)) w.feels = w.temp;
  if (findNum(cur, "\"is_day\":", v)) w.isDay = iround(v);
  if (findNum(cur, "\"weather_code\":", v)) w.code = iround(v);
  findNum(cur, "\"wind_speed_10m\":", w.wind);
  if (findNum(cur, "\"wind_direction_10m\":", v)) w.windDir = iround(v);


  w.tmax = w.tmin = w.temp;
  w.sunrise = w.sunset = -1;
  if (day)
  {
    findNum(day, "\"temperature_2m_max\":", w.tmax);
    findNum(day, "\"temperature_2m_min\":", w.tmin);
    w.sunrise = findHHMM(day, "\"sunrise\":");
    w.sunset  = findHHMM(day, "\"sunset\":");
  }
  w.valid = true;
  w.gotAt = millis();
  return true;
}


bool httpFetch(bool secure, String& body, int& code)
{
  String url = String(secure ? "https://" : "http://") + "api.open-meteo.com" + WEATHER_QUERY;
  HTTPClient http;
  WiFiClient plain;
  WiFiClientSecure tls;
  http.setConnectTimeout(8000);
  http.setTimeout(10000);


  bool ok;
  if (secure) { tls.setInsecure(); ok = http.begin(tls, url); }
  else        { ok = http.begin(plain, url); }
  if (!ok) { code = -100; LOG("HTTP: begin() не удался"); return false; }


  LOG("HTTP%s: GET ... (heap %u, max блок %u)", secure ? "S" : "", (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
  code = http.GET();
  if (code == 200) body = http.getString();
  LOG("HTTP%s: код %d, ответ %u байт", secure ? "S" : "", code, (unsigned)body.length());
  http.end();
  return code == 200;
}


bool fetchWeather(Weather& w)
{
  String body;
  int code = 0;
  bool ok = httpFetch(false, body, code);
  if (!ok)
  {
    ok = httpFetch(true, body, code);
  }
  if (!ok)
  {
    snprintf(lastNetMsg, sizeof(lastNetMsg), "ошибка, код %d", code);
    return false;
  }
  if (!parseWeather(body.c_str(), w))
  {
    LOG("погода: не разобрал ответ: %.120s", body.c_str());
    snprintf(lastNetMsg, sizeof(lastNetMsg), "не разобрал ответ");
    return false;
  }
  snprintf(lastNetMsg, sizeof(lastNetMsg), "OK %.1f°C код %d", w.temp, w.code);
  LOG("погода: OK %.1f C, код %d", w.temp, w.code);
  return true;
}


void netTask(void*)
{
  vTaskDelay(pdMS_TO_TICKS(4000));
  for (;;)
  {
    LOG("Wi-Fi: включаю (heap %u, max блок %u)", (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
    WiFi.persistent(false);
    bool mOk = WiFi.mode(WIFI_STA);
    LOG("Wi-Fi: mode(STA) = %d", mOk);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    LOG("Wi-Fi: begin(\"%s\")", WIFI_SSID);
    uint32_t t0 = millis();
    int lastSt = -1;
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000)
    {
      int st = WiFi.status();
      if (st != lastSt) { LOG("Wi-Fi: статус %d", st); lastSt = st; }
      vTaskDelay(pdMS_TO_TICKS(200));
    }
    bool ok = false;
    if (WiFi.status() == WL_CONNECTED)
    {
      LOG("Wi-Fi: подключено, IP %s, RSSI %d", WiFi.localIP().toString().c_str(), WiFi.RSSI());
      configTzTime(TZ_INFO, "pool.ntp.org", "time.google.com", "hora.roa.es");
      uint32_t t1 = millis();
      while (time(nullptr) < 1700000000 && millis() - t1 < 10000)
        vTaskDelay(pdMS_TO_TICKS(200));
      LOG("NTP: %s", time(nullptr) > 1700000000 ? "время получено" : "НЕ получено");
      Weather w;
      if (fetchWeather(w))
      {
        portENTER_CRITICAL(&wxMux);
        wxPending = w;
        wxNew = true;
        portEXIT_CRITICAL(&wxMux);
        netState = NET_ONLINE;
        ok = true;
      }
      vTaskDelay(pdMS_TO_TICKS(1500));
    }
    else LOG("Wi-Fi: НЕ подключилось за 20 с, статус %d", WiFi.status());
    // SNTP обязательно остановить ДО выключения Wi-Fi,
    // иначе lwip падает (assert pbuf_free) и ESP32 перезагружается = белый экран
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_sntp_stop();
#else
    sntp_stop();
#endif
    vTaskDelay(pdMS_TO_TICKS(300));
    WiFi.disconnect(true);
    vTaskDelay(pdMS_TO_TICKS(300));
    WiFi.mode(WIFI_OFF);
    uint32_t wait = ok ? WEATHER_PERIOD_MS : WEATHER_RETRY_MS;
    LOG("Wi-Fi: выключено, следующее окно через %lu с", (unsigned long)(wait / 1000));
    uint32_t tw = millis();
    while (millis() - tw < wait && !wxForce) vTaskDelay(pdMS_TO_TICKS(500));
    wxForce = false;
  }
}


// =====================================================================
//                     UI: клавиши, кадры анимации
// =====================================================================


bool calibScreen = false;


// =====================================================================
//                    СТАБИЛЬНАЯ СТАТИЧНАЯ ОТРИСОВКА
// =====================================================================
// Полный dashboard рисуется только по событию:
// смена минуты, новая погода, команда пользователя.
// Никакой анимации 25 FPS нет.
void renderDashboardStatic()
{
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  time_t now = tv.tv_sec;
  bool timeValid = now > 1700000000;
  struct tm lt = {};
  localtime_r(&now, &lt);
  int code = demoIdx ? DEMOS[demoIdx - 1].code : (wx.valid ? wx.code : 2);
  int mode = demoIdx ? DEMOS[demoIdx - 1].mode : computeMode(timeValid, lt);
  curIcon = iconFor(code);
  curMode = mode;
  applyPalette(curMode, curIcon);
  // Часы сразу в конечное состояние, без анимации.
  uint8_t tgt[4];
  if (timeValid)
  {
    tgt[0] = DIGIT_SEGS[lt.tm_hour / 10];
    tgt[1] = DIGIT_SEGS[lt.tm_hour % 10];
    tgt[2] = DIGIT_SEGS[lt.tm_min / 10];
    tgt[3] = DIGIT_SEGS[lt.tm_min % 10];
  }
  else
  {
    for (int i = 0; i < 4; i++) tgt[i] = DIGIT_SEGS[10];
  }
  for (int d = 0; d < 4; d++)
  {
    segTarget[d] = tgt[d];
    for (int s = 0; s < 7; s++)
      segLev[d][s] = ((tgt[d] >> s) & 1) ? 256 : 0;
  }
  clockAnimating = false;
  subSec  = tv.tv_usec / 1000000.0f;
  secFrac = timeValid ? (lt.tm_sec + subSec) / 60.0f : 0.0f;
  // Тексты принудительно перестраиваются.
  // Если wx.valid=true, сюда попадёт настоящая температура.
  for (int i = 0; i < NUM_TXT; i++) txt[i].set = false;
  rebuildTexts(timeValid, lt, code);
  // Иконка статична.
  animT++;
  prepIconAnim();
  // Рисуем постепенно на core 0.
  // LCD при этом продолжает непрерывно сканироваться на core 1.
  for (int y = 0; y < LINES; y++)
  {
    renderRow(y, 0, DOTS, scene);
    if ((y & 7) == 7)
      vTaskDelay(pdMS_TO_TICKS(1));
  }
}


void updateClockStaticOnly()
{
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  time_t now = tv.tv_sec;
  bool timeValid = now > 1700000000;
  struct tm lt = {};
  localtime_r(&now, &lt);
  if (!timeValid)
    return;
  uint8_t tgt[4];
  tgt[0] = DIGIT_SEGS[lt.tm_hour / 10];
  tgt[1] = DIGIT_SEGS[lt.tm_hour % 10];
  tgt[2] = DIGIT_SEGS[lt.tm_min / 10];
  tgt[3] = DIGIT_SEGS[lt.tm_min % 10];
  for (int d = 0; d < 4; d++)
  {
    segTarget[d] = tgt[d];
    for (int s = 0; s < 7; s++)
      segLev[d][s] = ((tgt[d] >> s) & 1) ? 256 : 0;
  }
  subSec  = tv.tv_usec / 1000000.0f;
  secFrac = (lt.tm_sec + subSec) / 60.0f;
  int code = demoIdx ? DEMOS[demoIdx - 1].code : (wx.valid ? wx.code : 2);
  // Обновляем текст даты, но не пересчитываем весь экран.
  rebuildTexts(true, lt, code);
  // Только часы + верхняя строка даты/города.
  // Это намного меньше работы, чем перерисовывать 480x234 каждую минуту.
  redrawRect(CLK_X0, CLK_RY0, CLK_X1, CLK_RY1);
  redrawRect(0, 0, DOTS, 24);
}


void redrawCurrentStatic()
{
  if (calibScreen)
  {
    drawCalibration();
  }
  else
  {
    renderDashboardStatic();
  }
}


// =====================================================================
//                               SETUP
// =====================================================================
// Управление и редкие перерисовки — только core 0.
// Вывод на матрицу делает I2S + DMA, процессоры на него не влияют.
// =====================================================================
//              КОНСОЛЬ (Serial 115200): калибровка и статус
// =====================================================================
volatile bool consoleRedraw = false;

void saveCalib()
{
  prefs.putUChar("ph0", phase[0]);
  prefs.putUChar("ph1", phase[1]);
  prefs.putBool("mx", mirX);
  prefs.putBool("my", mirY);
}

void printStatus()
{
  static const char* maskName[3] = { "все", "1,3,5", "2,4,6" };
  static uint32_t f0 = 0, t0 = 0;
  uint32_t f = frameCount, t = millis();
  float fps = (t0 && t > t0) ? (f - f0) * 1000.0f / (t - t0) : 0.0f;
  f0 = f; t0 = t;
  time_t now = time(nullptr);
  struct tm lt;
  localtime_r(&now, &lt);
  Serial.printf("DCLK=%s МГц  инверсия=%d  пара=%d  пиксель=%s  mirror X=%d Y=%d  маска=%s  кадров/с=%.1f  heap=%u (блок %u)\n",
                CLK_OPTS[clkSel].name, clkInv, pairSwap,
                (const char*[]){ "RGB0", "0RGB", "B0RG", "GB0R" }[slotRot & 3], mirX, mirY, maskName[lineMask], fps,
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
  Serial.printf("вывод: прерывание макс=%.1f мкс (строка 77 мкс), пропусков строк=%lu\n",
                isrMaxCyc / 240.0f, (unsigned long)isrSkip);
  isrMaxCyc = 0; isrSkip = 0;
  Serial.printf("Wi-Fi=%s  время=%s %02d:%02d:%02d  погода: %s  демо=%d  экран=%s\n",
                WiFi.status() == WL_CONNECTED ? "OK" : "нет",
                now > 1700000000 ? "OK" : "нет", lt.tm_hour, lt.tm_min, lt.tm_sec,
                lastNetMsg[0] ? lastNetMsg : "ещё нет", demoIdx, calibScreen ? "калибровка" : "погода");
}

void printHelp()
{
  Serial.println();
  Serial.println("=== AUO LCD: погода в Малаге ===");
  Serial.println("Режим матрицы UPS052 320RGB (R G B 0), дельту делает сама матрица");
  Serial.println("m - маска строк");
  Serial.println("f - частота DCLK (10 / 13.3 / 16 дробн. / 20 / 16 целый)   i - инверсия DCLK");
  Serial.println("d - дизеринг: выкл / везде / только фон (по умолчанию)");
  Serial.println("k - сдвиг пикселя на 1 такт (порядок 0RGB)   o - порядок отсчётов в паре");
  Serial.println("x/y - зеркало   c - экран калибровки   w - демо погоды");
  Serial.println("r - обновить погоду   p - статус   (BOOT = c)");
  printStatus();
}

void nextDemo()
{
  demoIdx = (demoIdx + 1) % (NUM_DEMOS + 1);
  if (demoIdx) Serial.printf("Демо %d/%d: %s\n", demoIdx, NUM_DEMOS, descFor(DEMOS[demoIdx - 1].code));
  else         Serial.println("Демо выкл — настоящая погода");
}

void handleKey(char c)
{
  switch (c)
  {
    case 'x': case 'X': mirX = !mirX; saveCalib(); buildBgTables(); consoleRedraw = true; printStatus(); break;
    case 'y': case 'Y': mirY = !mirY; saveCalib(); consoleRedraw = true; printStatus(); break;
    case 'm': case 'M': lineMask = (lineMask + 1) % 3; printStatus(); break;
    case 'o': case 'O': pairSwap = !pairSwap; prefs.putBool("sw", pairSwap); buildBgTables(); printStatus(); break;
    case 'i': case 'I':
      clkInv = !clkInv; prefs.putBool("ci", clkInv);
      esp_rom_gpio_connect_out_signal(LCD_DCLK, I2S0O_WS_OUT_IDX, clkInv, false);
      printStatus(); break;
    case 'f': case 'F':
      clkSel = (clkSel + 1) % NCLK; prefs.putUChar("cs", clkSel);
      I2S0.sample_rate_conf.tx_bck_div_num = CLK_OPTS[clkSel].bck;
      I2S0.clkm_conf.clkm_div_a = CLK_OPTS[clkSel].a;
      I2S0.clkm_conf.clkm_div_b = CLK_OPTS[clkSel].b;
      I2S0.clkm_conf.clkm_div_num = CLK_OPTS[clkSel].n;
      printStatus(); break;
    case 'd': case 'D':
      ditherMode = (ditherMode + 1) % 3; prefs.putUChar("dm", ditherMode); consoleRedraw = true;
      Serial.printf("дизеринг: %s\n", ditherMode == 0 ? "выкл" : ditherMode == 1 ? "везде" : "только фон");
      break;
    case 'k': case 'K': slotRot = (slotRot + 1) & 3; prefs.putUChar("sr", slotRot); buildBgTables(); printStatus(); break;
    case 'c': case 'C': calibScreen = !calibScreen; consoleRedraw = true; break;
    case 'w': case 'W': nextDemo(); break;
    case 'r': case 'R': wxForce = true; Serial.println("Обновляю погоду..."); break;
    case 'p': case 'P': case '?': case 'h': case 'H': printHelp(); break;
    default: break;
  }
}

void controlTask(void*)
{
  uint32_t lastPress = 0;
  int lastBtn = HIGH;
  int lastMinute = -999;
  bool lastTimeValid = false;
  int lastNetState = -999;
  int lastDemoIdx = -999;
  for (;;)
  {
    bool fullRedraw = false;
    bool clockRedraw = false;
    // ---------- консоль ----------
    while (Serial.available()) handleKey((char)Serial.read());
    if (consoleRedraw) { consoleRedraw = false; fullRedraw = true; }
    // ---------- BOOT: калибровка <-> dashboard ----------
    int bt = digitalRead(BTN_BOOT);
    if (lastBtn == HIGH && bt == LOW && millis() - lastPress > 250)
    {
      lastPress = millis();
      calibScreen = !calibScreen;
      fullRedraw = true;
    }
    lastBtn = bt;
    // ---------- новая погода ----------
    if (wxNew)
    {
      portENTER_CRITICAL(&wxMux);
      wx = wxPending;
      wxNew = false;
      portEXIT_CRITICAL(&wxMux);
      if (!calibScreen)
        fullRedraw = true;
    }
    // ---------- время ----------
    time_t now = time(nullptr);
    bool timeValid = now > 1700000000;
    struct tm lt = {};
    localtime_r(&now, &lt);
    if (timeValid != lastTimeValid)
    {
      lastTimeValid = timeValid;
      if (!calibScreen)
        fullRedraw = true;
    }
    // ВАЖНО:
    // при смене минуты больше НЕ перерисовываем весь экран.
    // Только область часов и верхнюю строку.
    if (timeValid && lt.tm_min != lastMinute)
    {
      lastMinute = lt.tm_min;
      if (!calibScreen && !fullRedraw)
        clockRedraw = true;
    }
    // ---------- сеть ----------
    if (netState != lastNetState)
    {
      lastNetState = netState;
      // Состояние сети важно в основном пока нет времени/погоды.
      if (!calibScreen && (!timeValid || !wx.valid))
        fullRedraw = true;
    }
    // ---------- демо ----------
    if (demoIdx != lastDemoIdx)
    {
      lastDemoIdx = demoIdx;
      if (!calibScreen)
        fullRedraw = true;
    }
    if (fullRedraw)
      redrawCurrentStatic();
    else if (clockRedraw)
    {
      updateClockStaticOnly();
    }
    // ---------- двоеточие мигает (период COLON_PERIOD_S), полоска секунд растёт ----------
    if (!calibScreen && timeValid)
    {
      static time_t lastSec = 0;
      struct timeval tv;
      gettimeofday(&tv, nullptr);
      if (tv.tv_sec != lastSec)
      {
        lastSec = tv.tv_sec;
        // полоска секунд — каждую секунду
        struct tm lt2;
        localtime_r(&tv.tv_sec, &lt2);
        subSec  = 0;
        secFrac = lt2.tm_sec / 60.0f;
        redrawRect(SEC_X0, SEC_Y0, SEC_X1, SEC_Y1);
        // двоеточие — первую половину периода горит, вторую нет
        bool on = (tv.tv_sec % COLON_PERIOD_S) < (COLON_PERIOD_S + 1) / 2;
        if (on != colonOn)
        {
          colonOn = on;
          redrawRect(COLON_RX0, COLON_RY0, COLON_RX1, COLON_RY1);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}


void setup()
{
  Serial.begin(115200);
  delay(100);
  LOG("старт, heap %u, max блок %u", (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
  pinMode(BTN_BOOT, INPUT_PULLUP);
  for (int i = 0; i < 256; i++)
    SIN8[i] =
      (int8_t)lroundf(
        127.0f *
        sinf(i * 2.0f * (float)M_PI / 256.0f)
      );
  // Один framebuffer, как в рабочей V4.
  for (int y = 0; y < LINES; y++)
  {
    fb[y] = (uint16_t*)malloc((DOTS / 3) * 2);
    if (!fb[y])
    {
      while (true)
        delay(1000);
    }
    memset(fb[y], 0, (DOTS / 3) * 2);
  }
  // Новый namespace, чтобы не тянуть настройки из сломанных V3/V5.
  prefs.begin("auov6", false);  // сохраняем калибровку V6
  phase[0] = prefs.getUChar("ph0", 0) % 3;
  phase[1] = prefs.getUChar("ph1", 2) % 3;
  // По твоему наблюдению предыдущая калибровка была зеркальна:
  // B-G-R и белый->чёрный. Поэтому X по умолчанию включён.
  mirX = prefs.getBool("mx", false);
  pairSwap = prefs.getBool("sw", true);
  clkInv   = prefs.getBool("ci", true);
  clkSel   = prefs.getUChar("cs", 2) % NCLK;     // по умолчанию 16 МГц (лучший по тесту)
  ditherMode = prefs.getUChar("dm", 2) % 3;
  slotRot  = prefs.getUChar("sr", 1) & 3;
  mirY = prefs.getBool("my", false);
  rebuildChan();
  initMountains();
  initStars();
  wx = {};
  wxPending = {};
  wxNew = false;
  wxForce = false;

  // Стартовый экран рисуется ДО запуска непрерывного вывода.
  calibScreen = false;
  renderDashboardStatic();
  // Запуск аппаратного вывода (дальше такт идёт от железа)
  buildLuts();
  startLcdOutput();
  LOG("вывод на матрицу запущен, heap %u, max блок %u", (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
  printHelp();
  // Управление/перерисовка на core 0.
  xTaskCreatePinnedToCore(
    controlTask,
    "control",
    8192,
    NULL,
    1,
    NULL,
    0
  );
#if USE_WIFI
  // Сеть тоже core 0.
  xTaskCreatePinnedToCore(
    netTask,
    "net",
    12288,
    NULL,
    1,
    NULL,
    0
  );
#else
  netState = NET_ERROR;
#endif
}


// =====================================================================
//              LOOP — пусто: картинку выводит I2S + DMA
// =====================================================================
void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}
