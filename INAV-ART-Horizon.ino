// ============================================================
// INAV ARTIFICIAL HORIZON
// ESP32-S3 SuperMini + GC9A01A 1.28" 240x240 Round TFT
//
// TFT wiring:
// BLK -> GPIO 1
// CS  -> GPIO 2
// RST -> GPIO 4
// MOSI/SDA -> GPIO 5
// SCLK/SCL -> GPIO 6
// DC  -> GPIO 8
//
// INAV MSP UART:
// FC TX -> ESP32 GPIO 17 (RX)
// FC RX <- ESP32 GPIO 18 (TX)
// GND   -> GND
//
// IMPORTANT:
// TEST_MODE = 1 : animated demo, FC not required
// TEST_MODE = 0 : real attitude + GPS data from INAV
// ============================================================

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <math.h>
#include <string.h>

// ------------------------- TFT PINS -------------------------

#define TFT_BL   1
#define TFT_CS   2
#define TFT_DC   8
#define TFT_RST  4
#define TFT_MOSI 5
#define TFT_SCLK 6

// ------------------------- INAV UART -------------------------

#define FC_RX 17   // ESP32 RX  <- FC TX
#define FC_TX 18   // ESP32 TX  -> FC RX

// ------------------------- TEST MODE -------------------------

// 1 = animated test/demo
// 0 = real data from INAV
#define TEST_MODE 0

// ------------------------- DISPLAY -------------------------

#define W 240
#define H 240
#define CX 120
#define CY 120

#define SCREEN_RADIUS 119
#define ROLL_RADIUS    94
#define DEG_TO_RAD     0.01745329251994329577f

// Change these only if movement is reversed.
#define ROLL_SIGN   1.0f
#define PITCH_SIGN  -1.0f

Adafruit_GC9A01A tft(TFT_CS, TFT_DC, TFT_RST);

uint16_t *fb = nullptr;

// ------------------------- COLORS -------------------------

#define C_BLACK     0x0000
#define C_SKY       0x03FF
#define C_GROUND    0x39E7
#define C_WHITE     0xFFFF
#define C_YELLOW    0xFFE0
#define C_GREEN     0x07E0
const uint16_t C_DARK_GREEN = 0x04A0;
#define C_AIRCRAFT  0xE7E0
#define C_BOX       0x2104

// ------------------------- MSP -------------------------

#define MSP_ATT      108
#define MSP_RAW_GPS  106

// ------------------------- FLIGHT DATA -------------------------

struct FlightData {
  float rollDeg;
  float pitchDeg;
  int16_t headingDeg;

  int32_t altitudeCm;
  uint16_t speedCms;
};

FlightData flight = {0, 0, 0, 0, 0};

float smoothRoll = 0.0f;
float smoothPitch = 0.0f;

// ------------------------- ROUND SCREEN MASK -------------------------

int16_t circleLeft[H];
int16_t circleRight[H];

void buildCircleMask() {

  const int rr =
    SCREEN_RADIUS *
    SCREEN_RADIUS;

  for (int y = 0; y < H; y++) {

    int dy =
      y - CY;

    int remain =
      rr -
      dy * dy;

    if (remain < 0) {

      circleLeft[y] = 1;
      circleRight[y] = 0;

    } else {

      int dx =
        (int)sqrtf(
          (float)remain
        );

      circleLeft[y] =
        max(
          0,
          CX - dx
        );

      circleRight[y] =
        min(
          W - 1,
          CX + dx
        );
    }
  }
}

static inline bool inRoundScreen(
  int x,
  int y
) {

  if (
    (unsigned)x >= W ||
    (unsigned)y >= H
  ) {
    return false;
  }

  int dx =
    x - CX;

  int dy =
    y - CY;

  return (

    dx * dx +
    dy * dy

    <=

    SCREEN_RADIUS *
    SCREEN_RADIUS
  );
}

static inline void putPixel(
  int x,
  int y,
  uint16_t color
) {

  if (
    inRoundScreen(x, y)
  ) {

    fb[
      y * W + x
    ] =
      color;
  }
}

// ------------------------- DRAWING -------------------------

void drawDisk(
  int x,
  int y,
  int radius,
  uint16_t color
) {

  int rr =
    radius *
    radius;

  for (
    int dy = -radius;
    dy <= radius;
    dy++
  ) {

    int remain =
      rr -
      dy * dy;

    int half =
      (int)sqrtf(
        (float)remain
      );

    for (
      int dx = -half;
      dx <= half;
      dx++
    ) {

      putPixel(
        x + dx,
        y + dy,
        color
      );
    }
  }
}

void drawSolidLine(
  int x0,
  int y0,
  int x1,
  int y1,
  int thickness,
  uint16_t color
) {

  int dx =
    abs(x1 - x0);

  int sx =
    x0 < x1
    ? 1
    : -1;

  int dy =
    -abs(y1 - y0);

  int sy =
    y0 < y1
    ? 1
    : -1;

  int err =
    dx + dy;

  int radius =
    max(
      0,
      (thickness - 1) / 2
    );

  while (true) {

    if (
      radius == 0
    ) {

      putPixel(
        x0,
        y0,
        color
      );

    } else {

      drawDisk(
        x0,
        y0,
        radius,
        color
      );
    }

    if (
      x0 == x1 &&
      y0 == y1
    ) {
      break;
    }

    int e2 =
      2 * err;

    if (e2 >= dy) {

      err += dy;
      x0 += sx;
    }

    if (e2 <= dx) {

      err += dx;
      y0 += sy;
    }
  }
}

void fillRect(
  int x0,
  int y0,
  int x1,
  int y1,
  uint16_t color
) {

  if (x0 < 0) x0 = 0;
  if (y0 < 0) y0 = 0;

  if (x1 >= W) x1 = W - 1;
  if (y1 >= H) y1 = H - 1;

  for (
    int y = y0;
    y <= y1;
    y++
  ) {

    for (
      int x = x0;
      x <= x1;
      x++
    ) {

      putPixel(
        x,
        y,
        color
      );
    }
  }
}

// ============================================================
// MSP
// ============================================================

enum MspState {

  WAIT_DOLLAR,
  WAIT_M,
  WAIT_DIRECTION,
  WAIT_SIZE,
  WAIT_COMMAND,
  WAIT_PAYLOAD,
  WAIT_CHECKSUM
};

MspState mspState =
  WAIT_DOLLAR;

uint8_t mspSize = 0;
uint8_t mspCommand = 0;
uint8_t mspChecksum = 0;
uint8_t mspIndex = 0;

uint8_t mspPayload[64];

static inline int16_t readS16(
  const uint8_t *p
) {

  return (
    int16_t
  )(
    (uint16_t)p[0] |
    ((uint16_t)p[1] << 8)
  );
}

static inline uint16_t readU16(
  const uint8_t *p
) {

  return

    (uint16_t)p[0] |

    ((uint16_t)p[1] << 8);
}

void requestMSP(
  uint8_t command
) {

  Serial1.write('$');
  Serial1.write('M');
  Serial1.write('<');

  Serial1.write(
    (uint8_t)0
  );

  Serial1.write(
    command
  );

  Serial1.write(
    command
  );
}

void processMSP() {

  if (

    mspCommand == MSP_ATT

    &&

    mspSize >= 6

  ) {

    flight.rollDeg =

      readS16(
        &mspPayload[0]
      ) *
      0.1f;


    flight.pitchDeg =

      readS16(
        &mspPayload[2]
      ) *
      0.1f;

    // MSP_ATT payload:
    // roll (0.1 deg), pitch (0.1 deg), heading (deg)
    flight.headingDeg =
      readS16(
        &mspPayload[4]
      );
  }


  if (

    mspCommand ==
    MSP_RAW_GPS

    &&

    mspSize >= 14

  ) {

    // INAV MSP_RAW_GPS altitude is cm.
    flight.altitudeCm =

      readS16(
        &mspPayload[10]
      );


    // Speed is cm/s.
    flight.speedCms =

      readU16(
        &mspPayload[12]
      );
  }
}

void readMSP() {

  while (
    Serial1.available()
  ) {

    uint8_t c =
      Serial1.read();

    switch (
      mspState
    ) {

      case WAIT_DOLLAR:

        mspState =
          c == '$'
          ? WAIT_M
          : WAIT_DOLLAR;

        break;


      case WAIT_M:

        mspState =
          c == 'M'
          ? WAIT_DIRECTION
          : WAIT_DOLLAR;

        break;


      case WAIT_DIRECTION:

        mspState =
          c == '>'
          ? WAIT_SIZE
          : WAIT_DOLLAR;

        break;


      case WAIT_SIZE:

        mspSize =
          c;

        mspChecksum =
          c;

        mspIndex =
          0;

        mspState =
          WAIT_COMMAND;

        break;


      case WAIT_COMMAND:

        mspCommand =
          c;

        mspChecksum ^=
          c;

        mspState =
          mspSize
          ? WAIT_PAYLOAD
          : WAIT_CHECKSUM;

        break;


      case WAIT_PAYLOAD:

        if (

          mspIndex <
          sizeof(mspPayload)

        ) {

          mspPayload[
            mspIndex
          ] =
            c;
        }

        mspIndex++;

        mspChecksum ^=
          c;

        if (

          mspIndex >=
          mspSize

        ) {

          mspState =
            WAIT_CHECKSUM;
        }

        break;


      case WAIT_CHECKSUM:

        if (

          c ==
          mspChecksum

          &&

          mspSize <=
          sizeof(mspPayload)

        ) {

          processMSP();
        }

        mspState =
          WAIT_DOLLAR;

        break;
    }
  }
}

// ============================================================
// BACKGROUND + HORIZON
// ============================================================

void drawBackground(
  float rollRad,
  float pitchDeg
) {

  float s =
    sinf(
      rollRad
    );

  float c =
    cosf(
      rollRad
    );


  float offset =

    sinf(

      pitchDeg *
      DEG_TO_RAD *
      PITCH_SIGN

    ) *
    112.0f;


  for (
    int y = 0;
    y < H;
    y++
  ) {

    int left =
      circleLeft[y];

    int right =
      circleRight[y];

    if (
      left > right
    ) {
      continue;
    }

    float dy =
      y - CY;

    for (
      int x = left;
      x <= right;
      x++
    ) {

      float dx =
        x - CX;

      float side =

        dx * s +

        dy * c -

        offset;


      fb[
        y * W + x
      ] =

        side < 0

        ? C_SKY

        : C_GROUND;
    }
  }


  // Horizon line

  float nx = s;
  float ny = c;

  float tx = c;
  float ty = -s;

  float half =
    180.0f;


  float baseX =
    CX +
    nx * offset;

  float baseY =
    CY +
    ny * offset;


  int x1 =
    (int)lroundf(
      baseX -
      tx * half
    );

  int y1 =
    (int)lroundf(
      baseY -
      ty * half
    );

  int x2 =
    (int)lroundf(
      baseX +
      tx * half
    );

  int y2 =
    (int)lroundf(
      baseY +
      ty * half
    );


  drawSolidLine(

    x1,
    y1,

    x2,
    y2,

    3,

    C_WHITE
  );
}

// ============================================================
// PITCH LADDER
// ============================================================

void drawPitchLadder(
  float rollRad,
  float pitchDeg
) {
  const float c = cosf(rollRad);
  const float sn = sinf(rollRad);

  // Linear pitch scale: constant spacing at all pitch angles.
  const float PX_PER_DEG = 2.05f;

  // Extended pitch ladder: more marks above and below the horizon.
  // Still uses 5° spacing, so the spacing remains perfectly constant.
  for (int mark = -70; mark <= 70; mark += 5) {
    if (mark == 0) continue;

    const float localY =
      (pitchDeg * PITCH_SIGN - mark) * PX_PER_DEG;

    // Keep generating lines farther away so additional pitch marks remain
    // available as the horizon moves up or down.
    if (fabsf(localY) > 180.0f) continue;

    const bool major = (abs(mark) % 10) == 0;

    // Clean ladder: short 5° marks, long 10° marks.
    const float half = major ? 30.0f : 16.0f;
    const int thickness = major ? 3 : 2;

    const int x1 = (int)lroundf(
      CX - half * c + localY * sn
    );
    const int y1 = (int)lroundf(
      CY + half * sn + localY * c
    );

    const int x2 = (int)lroundf(
      CX + half * c + localY * sn
    );
    const int y2 = (int)lroundf(
      CY - half * sn + localY * c
    );

    drawSolidLine(
      x1, y1,
      x2, y2,
      thickness,
      C_WHITE
    );
  }
}

// ============================================================
// ROLL SCALE
// ============================================================

// ============================================================
// ARTIFICIAL HORIZON OUTER BORDER
// ============================================================

void drawHorizonOuterBorder() {
  // 3 px border starting exactly at the artificial-horizon edge
  // and expanding outward.
  const int borderStartR = ROLL_RADIUS;
  const int borderWidth = 3;

  for (int r = borderStartR; r < borderStartR + borderWidth; r++) {
    for (int deg = 0; deg < 360; deg++) {
      float a = deg * DEG_TO_RAD;
      int x = (int)lroundf(CX + cosf(a) * r);
      int y = (int)lroundf(CY + sinf(a) * r);

      if (inRoundScreen(x, y)) {
        putPixel(x, y, C_BLACK);
      }
    }
  }
}

void drawRollScale(
  float rollDeg
) {

  // Opaque annulus: pitch ladder never enters the roll scale area.
  const int innerR = ROLL_RADIUS - 5;
  const int outerR = ROLL_RADIUS + 31;

  for (int y = CY - outerR; y <= CY + outerR; y++) {
    if ((unsigned)y >= H) continue;
    for (int x = CX - outerR; x <= CX + outerR; x++) {
      if ((unsigned)x >= W) continue;
      int dx = x - CX;
      int dy = y - CY;
      int d2 = dx * dx + dy * dy;
      if (d2 >= innerR * innerR &&
          d2 <= outerR * outerR &&
          inRoundScreen(x, y)) {
        fb[y * W + x] = C_BLACK;
      }
    }
  }

  for (int worldDeg = -180; worldDeg <= 180; worldDeg += 5) {

    float screenDeg = worldDeg - rollDeg * ROLL_SIGN;

    while (screenDeg > 180.0f) screenDeg -= 360.0f;
    while (screenDeg < -180.0f) screenDeg += 360.0f;

    if (screenDeg < -65.0f || screenDeg > 65.0f) continue;

    float a = screenDeg * DEG_TO_RAD;
    float sn = sinf(a);
    float cs = cosf(a);

    int length;
    int thickness;
    uint16_t color;

    int absRoll = abs(worldDeg);
    if (absRoll > 180) absRoll = 360 - absRoll;

    // The actual 0° world mark is special and easy to spot.
    if (worldDeg == 0) {
      length = 28;
      thickness = 7;
      color = C_WHITE;
    } else {
      length = (worldDeg % 10 == 0) ? 18 : 11;
      thickness = (worldDeg % 10 == 0) ? 5 : 3;

      // Green normal -> yellow -> orange -> red dangerous.
      // Extra 5° tick moved from yellow to green,
      // and one extra 5° tick moved from yellow to orange.
      if (absRoll <= 25) {
        color = C_GREEN;
      } else if (absRoll <= 50) {
        color = C_YELLOW;
      } else if (absRoll <= 80) {
        color = 0xFD20; // orange
      } else {
        color = 0xF800; // red
      }
    }

    int x1 = (int)lroundf(CX + ROLL_RADIUS * sn);
    int y1 = (int)lroundf(CY - ROLL_RADIUS * cs);
    int x2 = (int)lroundf(CX + (ROLL_RADIUS + length) * sn);
    int y2 = (int)lroundf(CY - (ROLL_RADIUS + length) * cs);

    drawSolidLine(x1, y1, x2, y2, thickness, color);
  }

  // Fixed roll reference triangle.
  // Draw a small white outline first, then the green triangle inside it.
  int pointerY = CY - ROLL_RADIUS - 3;

  // White outline: slightly larger triangle.
  for (int row = 0; row < 12; row++) {
    for (int x = -(row + 1); x <= (row + 1); x++) {
      putPixel(CX + x, pointerY + row, C_BLACK);
    }
  }

  // Green triangle on top, leaving the white border visible.
  for (int row = 1; row < 10; row++) {
    for (int x = -(row - 1); x <= (row - 1); x++) {
      putPixel(CX + x, pointerY + row, C_GREEN);
    }
  }
}

// ============================================================
// AIRCRAFT SYMBOL
// ============================================================

void drawAircraft() {

  // Connected "-V-" flight reference symbol.
  // Draw a thicker black version first, then the green symbol on top.
  // This creates a clean ~2 px black outline.
  const int outlineThick = 10;
  const int symbolThick = 6;
  const int stemOutlineThick = 9;
  const int stemThick = 5;

  // ---- Black outline ----
  drawSolidLine(CX - 42, CY,      CX - 10, CY,      outlineThick, C_BLACK);
  drawSolidLine(CX - 10, CY,      CX, CY + 10,      outlineThick, C_BLACK);
  drawSolidLine(CX, CY + 10,      CX + 10, CY,      outlineThick, C_BLACK);
  drawSolidLine(CX + 10, CY,      CX + 42, CY,      outlineThick, C_BLACK);
  drawSolidLine(CX, CY + 10,      CX, CY + 18,      stemOutlineThick, C_BLACK);

  // ---- Green symbol ----
  drawSolidLine(CX - 42, CY,      CX - 10, CY,      symbolThick, C_DARK_GREEN);
  drawSolidLine(CX - 10, CY,      CX, CY + 10,      symbolThick, C_DARK_GREEN);
  drawSolidLine(CX, CY + 10,      CX + 10, CY,      symbolThick, C_DARK_GREEN);
  drawSolidLine(CX + 10, CY,      CX + 42, CY,      symbolThick, C_DARK_GREEN);
  drawSolidLine(CX, CY + 10,      CX, CY + 18,      stemThick, C_DARK_GREEN);
}

// ============================================================
// SIMPLE DIGITS FOR SIDE BOXES
// ============================================================

const uint8_t digitSegments[10][7] = {

  {1,1,1,1,1,1,0},
  {0,1,1,0,0,0,0},
  {1,1,0,1,1,0,1},
  {1,1,1,1,0,0,1},
  {0,1,1,0,0,1,1},
  {1,0,1,1,0,1,1},
  {1,0,1,1,1,1,1},
  {1,1,1,0,0,0,0},
  {1,1,1,1,1,1,1},
  {1,1,1,1,0,1,1}
};

void segLine(
  int x0,
  int y0,
  int x1,
  int y1,
  int t,
  uint16_t color
) {

  drawSolidLine(
    x0,
    y0,
    x1,
    y1,
    t,
    color
  );
}

void drawDigit(
  int digit,
  int x,
  int y,
  int scale,
  int thickness,
  uint16_t color
) {

  if (
    digit < 0 ||
    digit > 9
  ) {
    return;
  }

  int w =
    6 * scale;

  int h =
    10 * scale;

  int hh =
    h / 2;


  const uint8_t *s =
    digitSegments[digit];


  if (s[0])
    segLine(
      x,
      y,
      x + w,
      y,
      thickness,
      color
    );

  if (s[1])
    segLine(
      x + w,
      y,
      x + w,
      y + hh,
      thickness,
      color
    );

  if (s[2])
    segLine(
      x + w,
      y + hh,
      x + w,
      y + h,
      thickness,
      color
    );

  if (s[3])
    segLine(
      x,
      y + h,
      x + w,
      y + h,
      thickness,
      color
    );

  if (s[4])
    segLine(
      x,
      y + hh,
      x,
      y + h,
      thickness,
      color
    );

  if (s[5])
    segLine(
      x,
      y,
      x,
      y + hh,
      thickness,
      color
    );

  if (s[6])
    segLine(
      x,
      y + hh,
      x + w,
      y + hh,
      thickness,
      color
    );
}

void drawNumber(
  int value,
  int x,
  int y,
  int scale,
  int thickness,
  uint16_t color
) {

  char buffer[12];

  snprintf(
    buffer,
    sizeof(buffer),
    "%d",
    value
  );

  int cursor =
    x;

  for (
    int i = 0;
    buffer[i];
    i++
  ) {

    if (
      buffer[i] >= '0' &&
      buffer[i] <= '9'
    ) {

      drawDigit(

        buffer[i] - '0',

        cursor,
        y,

        scale,
        thickness,

        color
      );

      cursor +=
        9 *
        scale;

    } else if (
      buffer[i] == '-'
    ) {

      segLine(

        cursor,

        y +
        5 * scale,

        cursor +
        6 * scale,

        y +
        5 * scale,

        thickness,

        color
      );

      cursor +=
        9 *
        scale;
    }
  }
}

int numberWidth(
  int value,
  int scale
) {

  char buffer[12];

  snprintf(
    buffer,
    sizeof(buffer),
    "%d",
    value
  );

  return

    strlen(buffer) *

    9 *

    scale;
}


// ============================================================
// ACTUAL RENDERED NUMBER BOUNDS
// ============================================================
// Uses only primitive types in function parameters so the Arduino
// preprocessor can generate prototypes safely.

void expandInkBounds(
  int *minX, int *maxX,
  int *minY, int *maxY,
  int x0, int y0,
  int x1, int y1,
  int thickness
) {
  int radius = max(0, (thickness - 1) / 2);
  int lx = min(x0, x1) - radius;
  int rx = max(x0, x1) + radius;
  int ty = min(y0, y1) - radius;
  int by = max(y0, y1) + radius;

  *minX = min(*minX, lx);
  *maxX = max(*maxX, rx);
  *minY = min(*minY, ty);
  *maxY = max(*maxY, by);
}

void numberInkBounds(
  int value,
  int scale,
  int thickness,
  int *minX,
  int *maxX,
  int *minY,
  int *maxY
) {
  *minX = 32767;
  *maxX = -32768;
  *minY = 32767;
  *maxY = -32768;

  char buffer[12];
  snprintf(buffer, sizeof(buffer), "%d", value);

  int cursor = 0;
  int w = 6 * scale;
  int h = 10 * scale;
  int hh = h / 2;

  for (int i = 0; buffer[i]; i++) {
    if (buffer[i] >= '0' && buffer[i] <= '9') {
      int digit = buffer[i] - '0';
      const uint8_t *seg = digitSegments[digit];

      if (seg[0]) expandInkBounds(minX, maxX, minY, maxY, cursor,     0,  cursor + w, 0,  thickness);
      if (seg[1]) expandInkBounds(minX, maxX, minY, maxY, cursor + w, 0,  cursor + w, hh, thickness);
      if (seg[2]) expandInkBounds(minX, maxX, minY, maxY, cursor + w, hh, cursor + w, h,  thickness);
      if (seg[3]) expandInkBounds(minX, maxX, minY, maxY, cursor,     h,  cursor + w, h,  thickness);
      if (seg[4]) expandInkBounds(minX, maxX, minY, maxY, cursor,     hh, cursor,     h,  thickness);
      if (seg[5]) expandInkBounds(minX, maxX, minY, maxY, cursor,     0,  cursor,     hh, thickness);
      if (seg[6]) expandInkBounds(minX, maxX, minY, maxY, cursor,     hh, cursor + w, hh, thickness);

      cursor += 9 * scale;
    } else if (buffer[i] == '-') {
      expandInkBounds(minX, maxX, minY, maxY,
                      cursor, 5 * scale,
                      cursor + 6 * scale, 5 * scale,
                      thickness);
      cursor += 9 * scale;
    }
  }

  if (*maxX < *minX) {
    *minX = *maxX = 0;
    *minY = *maxY = 0;
  }
}

void drawNumberCenteredInCell(
  int value,
  int cellLeft,
  int cellTop,
  int cellRight,
  int cellBottom,
  int scale,
  int thickness,
  uint16_t color
) {
  int minX, maxX, minY, maxY;
  numberInkBounds(
    value, scale, thickness,
    &minX, &maxX, &minY, &maxY
  );

  int inkW = maxX - minX + 1;
  int inkH = maxY - minY + 1;

  int targetLeft = cellLeft + (cellRight - cellLeft + 1 - inkW) / 2;
  int targetTop  = cellTop  + (cellBottom - cellTop + 1 - inkH) / 2;

  int drawX = targetLeft - minX;
  int drawY = targetTop - minY;

  drawNumber(value, drawX, drawY, scale, thickness, color);
}

// ============================================================
// ALTITUDE / SPEED CASCADE BOXES
// ============================================================

void drawCascadeBox(
  bool leftSide,
  int value
) {
  const int boxW = 58;
  const int boxH = 108;
  const int borderT = 3;

  int x0 = leftSide ? 2 : W - boxW - 2;

  const int compassTopY = 196;
  const int topBorderY = compassTopY - boxH;
  const int bottomBorderY = compassTopY;

  // The three normal number cells (+1, -1, -2) are equal height.
  // The highlighted green center cell is deliberately larger.
  const int innerTop = topBorderY + borderT;
  const int innerBottom = bottomBorderY - borderT;
  const int innerHeight = innerBottom - innerTop + 1;

  // Two 3 px internal borders surround the larger green cell, plus one
  // 3 px separator between the two lower normal cells.
  const int internalBorderTotal = borderT * 3;
  const int usableCellHeight = innerHeight - internalBorderTotal;

  // Center cell gets 1.5x the height of each normal cell.
  // 3 normal cells + 1.5 normal cells = 4.5 units.
  const int normalCellH = usableCellHeight / 4.5f;
  const int centerCellH = usableCellHeight - normalCellH * 3;

  // Cell boundaries, calculated from the top down.
  const int upperCellTop = innerTop;
  const int upperCellBottom = upperCellTop + normalCellH - 1;

  const int centerTopBorderY = upperCellBottom + 1;
  const int centerCellTop = centerTopBorderY + borderT;
  const int centerCellBottom = centerCellTop + centerCellH - 1;
  const int centerBottomBorderY = centerCellBottom + 1;

  const int lower1CellTop = centerBottomBorderY + borderT;
  const int lower1CellBottom = lower1CellTop + normalCellH - 1;

  const int separatorY = lower1CellBottom + 1;
  const int lower2CellTop = separatorY + borderT;
  const int lower2CellBottom = innerBottom;

  // Opaque rectangle background. It reaches fully to the exterior side.
  fillRect(x0, topBorderY, x0 + boxW, bottomBorderY, C_BLACK);

  if (leftSide) {
    fillRect(
      x0,
      innerTop,
      x0 + boxW - borderT,
      innerBottom,
      C_BOX
    );
  } else {
    fillRect(
      x0 + borderT,
      innerTop,
      x0 + boxW,
      innerBottom,
      C_BOX
    );
  }

  // Outer borders.
  drawSolidLine(x0, topBorderY, x0 + boxW, topBorderY, borderT, C_WHITE);
  drawSolidLine(x0, bottomBorderY, x0 + boxW, bottomBorderY, borderT, C_WHITE);

  // Inner vertical border facing the center.
  int innerX = leftSide ? x0 + boxW : x0;
  drawSolidLine(innerX, topBorderY, innerX, bottomBorderY, borderT, C_WHITE);

  // Green highlighted center cell fills its entire cell width.
  if (leftSide) {
    fillRect(
      x0,
      centerCellTop,
      innerX - borderT,
      centerCellBottom,
      C_GREEN
    );
  } else {
    fillRect(
      innerX + borderT,
      centerCellTop,
      x0 + boxW,
      centerCellBottom,
      C_GREEN
    );
  }

  // Borders around center cell and separator.
  drawSolidLine(x0, centerTopBorderY, x0 + boxW, centerTopBorderY, borderT, C_WHITE);
  drawSolidLine(x0, centerBottomBorderY, x0 + boxW, centerBottomBorderY, borderT, C_WHITE);
  drawSolidLine(x0, separatorY, x0 + boxW, separatorY, borderT, C_WHITE);

  // ----------------------------------------------------------
  // Numbers centered in their actual cells.
  // ----------------------------------------------------------
  const int smallDigitHeight = 14;
  const int currentDigitHeight = 20;

  int above1 = value + 1;
  int above1Width = numberWidth(above1, 1);
  int above1Y =
    upperCellTop +
    (normalCellH - smallDigitHeight) / 2;

  int currentScale = 2;
  int currentWidth = numberWidth(value, currentScale);
  int currentY =
    centerCellTop +
    (centerCellH - currentDigitHeight) / 2;

  int below1 = value - 1;
  int below1Width = numberWidth(below1, 1);
  int below1Y =
    lower1CellTop +
    (normalCellH - smallDigitHeight) / 2;

  int below2 = value - 2;
  int below2Width = numberWidth(below2, 1);
  int lower2H = lower2CellBottom - lower2CellTop + 1;
  int below2Y =
    lower2CellTop +
    (lower2H - smallDigitHeight) / 2;

  drawNumber(above1, x0 + (boxW - above1Width) / 2, above1Y, 1, 2, C_WHITE);
  drawNumber(value, x0 + (boxW - currentWidth) / 2, currentY, currentScale, 3, C_BLACK);
  drawNumber(below1, x0 + (boxW - below1Width) / 2, below1Y, 1, 2, C_WHITE);
  drawNumber(below2, x0 + (boxW - below2Width) / 2, below2Y, 1, 2, C_WHITE);
}

// ============================================================
// BOTTOM COMPASS TAPE
// ============================================================

const uint8_t compassLetters[4][7] = {
  {0b10001,0b11001,0b10101,0b10011,0b10001,0b10001,0b10001}, // N
  {0b11111,0b10000,0b10000,0b11110,0b10000,0b10000,0b11111}, // E
  {0b01111,0b10000,0b10000,0b01110,0b00001,0b00001,0b11110}, // S
  {0b10001,0b10001,0b10001,0b10101,0b10101,0b11011,0b10001}  // W
};

void drawCompassLetter(
  char letter,
  int x,
  int y,
  int scale,
  uint16_t color
) {
  int index =
    letter == 'N' ? 0 :
    letter == 'E' ? 1 :
    letter == 'S' ? 2 :
    letter == 'W' ? 3 : -1;

  if (index < 0) return;

  for (int row = 0; row < 7; row++) {
    for (int col = 0; col < 5; col++) {
      if (compassLetters[index][row] & (1 << (4 - col))) {
        for (int yy = 0; yy < scale; yy++) {
          for (int xx = 0; xx < scale; xx++) {
            putPixel(
              x + col * scale + xx,
              y + row * scale + yy,
              color
            );
          }
        }
      }
    }
  }
}

void drawCompassPair(
  char first,
  char second,
  int centerX,
  int y,
  int scale,
  uint16_t color
) {
  const int charW = 5 * scale;
  const int gap = scale + 1;
  const int totalW = charW * 2 + gap;

  int x = centerX - totalW / 2;

  drawCompassLetter(first,  x,                  y, scale, color);
  drawCompassLetter(second, x + charW + gap,    y, scale, color);
}

void drawCompassBorder() {
  const int topY = 196;
  const int bottomY = 239;
  const int thickness = 3;

  // Top border of compass area.
  for (int t = 0; t < thickness; t++) {
    drawSolidLine(0, topY + t, W - 1, topY + t, 1, C_WHITE);
  }


}

void drawCompass() {

  const int bandTop = 196;
  const int bandBottom = 239;
  const int tickBaseY = 217;
  const float pxPerDeg = 0.92f;

  // Completely repaint the compass area each frame.
  for (int y = bandTop; y <= bandBottom; y++) {
    for (int x = 0; x < W; x++) {
      if (inRoundScreen(x, y)) {
        fb[y * W + x] = C_BLACK;
      }
    }
  }

  // INAV/MSP heading normalized to 0...359.
  int heading = flight.headingDeg % 360;
  if (heading < 0) heading += 360;

  // ----------------------------------------------------------
  // Direction labels every 45 degrees:
  // N, NE, E, SE, S, SW, W, NW
  // Cardinal directions are larger; diagonal directions smaller.
  // ----------------------------------------------------------
  int firstDirection = (heading / 45) * 45 - 180;

  for (
    int worldUnwrapped = firstDirection;
    worldUnwrapped <= heading + 225;
    worldUnwrapped += 45
  ) {
    int world = worldUnwrapped % 360;
    if (world < 0) world += 360;

    int x = (int)lroundf(
      CX + (worldUnwrapped - heading) * pxPerDeg
    );

    if (x < -20 || x > W + 20) continue;

    bool cardinal = (world % 90) == 0;

    // Direction tick.
    drawSolidLine(
      x,
      tickBaseY - (cardinal ? 15 : 11),
      x,
      tickBaseY,
      cardinal ? 3 : 2,
      cardinal ? C_YELLOW : C_WHITE
    );

    if (cardinal) {
      char letter =
        world == 0   ? 'N' :
        world == 90  ? 'E' :
        world == 180 ? 'S' : 'W';

      drawCompassLetter(
        letter,
        x - 5,
        221,
        2,
        C_YELLOW
      );

    } else {
      // Smaller diagonal labels.
      char first =
        world == 45  ? 'N' :
        world == 135 ? 'S' :
        world == 225 ? 'S' : 'N';

      char second =
        world == 45  ? 'E' :
        world == 135 ? 'E' :
        world == 225 ? 'W' : 'W';

      drawCompassPair(
        first,
        second,
        x,
        224,
        1,
        C_WHITE
      );
    }
  }

  // Minor ticks every 10 degrees, but no numbers.
  int firstTick = (heading / 10) * 10 - 100;

  for (
    int worldUnwrapped = firstTick;
    worldUnwrapped <= heading + 100;
    worldUnwrapped += 10
  ) {
    int world = worldUnwrapped % 360;
    if (world < 0) world += 360;

    // Skip direction positions already drawn.
    if ((world % 45) == 0) continue;

    int x = (int)lroundf(
      CX + (worldUnwrapped - heading) * pxPerDeg
    );

    if (x < 1 || x >= W - 1) continue;

    bool major = (world % 30) == 0;

    drawSolidLine(
      x,
      tickBaseY - (major ? 9 : 5),
      x,
      tickBaseY,
      major ? 2 : 1,
      C_WHITE
    );
  }

  // Fixed center pointer.
  // Top is cut by 3 px so it starts below the compass top border.
  drawSolidLine(CX,     199, CX, 217, 3, C_GREEN);
  drawSolidLine(CX - 7, 199, CX, 203, 3, C_GREEN);
  drawSolidLine(CX + 7, 199, CX, 203, 3, C_GREEN);
}

// ============================================================
// RENDER
// ============================================================

void renderFrame() {

  float rollRad =

    smoothRoll *
    DEG_TO_RAD *
    ROLL_SIGN;


  drawBackground(

    rollRad,
    smoothPitch
  );


  drawPitchLadder(

    rollRad,
    smoothPitch
  );


  // Roll scale on top of pitch lines

  drawHorizonOuterBorder();

  drawRollScale(
    smoothRoll
  );


  drawAircraft();


  // Altitude in meters

  int altitudeMeters =

    flight.altitudeCm /
    100;


  // Speed in km/h

  int speedKmh =

    (
      flight.speedCms *
      36
    ) /
    1000;


  drawCascadeBox(
    true,
    altitudeMeters
  );


  drawCascadeBox(
    false,
    speedKmh
  );

  drawCompass();
  drawCompassBorder();

  // Redraw the rectangle/compass contact borders last.
  // drawCompass() clears its own background, so this guarantees a
  // continuous white border exactly where the two UI elements meet.
  drawSolidLine(
    2, 196,
    2 + 58, 196,
    2, C_WHITE
  );

  drawSolidLine(
    W - 58 - 2, 196,
    W - 2, 196,
    2, C_WHITE
  );


  tft.drawRGBBitmap(

    0,
    0,

    fb,

    W,
    H
  );
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  pinMode(
    TFT_BL,
    OUTPUT
  );

  digitalWrite(
    TFT_BL,
    HIGH
  );


  Serial.begin(
    115200
  );


  SPI.begin(

    TFT_SCLK,

    -1,

    TFT_MOSI,

    TFT_CS
  );


  tft.begin();

  tft.setRotation(
    0
  );


  // Full 240x240 RGB565 framebuffer.
  // 115,200 bytes.

  fb =

    (uint16_t *)

    ps_malloc(

      W *
      H *
      sizeof(uint16_t)
    );


  if (!fb) {

    fb =

      (uint16_t *)

      malloc(

        W *
        H *
        sizeof(uint16_t)
      );
  }


  if (!fb) {

    while (true) {

      delay(1000);
    }
  }


  buildCircleMask();


  // Clear framebuffer

  memset(

    fb,

    0,

    W *
    H *
    sizeof(uint16_t)
  );


  tft.drawRGBBitmap(

    0,
    0,

    fb,

    W,
    H
  );


  Serial1.begin(

    115200,

    SERIAL_8N1,

    FC_RX,

    FC_TX
  );
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

#if !TEST_MODE

  readMSP();

#endif


#if TEST_MODE

  // ----------------------------------------------------------
  // Animated demo
  // ----------------------------------------------------------

  float time =

    millis() *
    0.001f;


  flight.rollDeg =

    45.0f *

    sinf(
      time *
      0.65f
    );


  flight.pitchDeg =

    28.0f *

    sinf(

      time *
      0.43f +

      1.0f
    );

  flight.headingDeg =
    (int)(
      fmodf(
        time * 14.0f,
        360.0f
      )
    );


  // Animated altitude

  flight.altitudeCm =

    (
      int32_t
    )(

      125.0f +

      80.0f *

      sinf(
        time *
        0.12f
      )

    ) *
    100;


  // Animated speed

  flight.speedCms =

    (
      uint16_t
    )(

      55.0f +

      25.0f *

      sinf(

        time *
        0.22f +

        2.0f
      )

    ) *
    1000 /
    36;

#endif


  static uint32_t lastFrame =
    0;

  static uint32_t lastAttRequest =
    0;

  static uint32_t lastGPSRequest =
    0;


  uint32_t now =
    millis();


#if !TEST_MODE

  // Request attitude

  if (

    now -
    lastAttRequest >=
    25

  ) {

    lastAttRequest =
      now;

    requestMSP(
      MSP_ATT
    );
  }


  // Request GPS data

  if (

    now -
    lastGPSRequest >=
    200

  ) {

    lastGPSRequest =
      now;

    requestMSP(
      MSP_RAW_GPS
    );
  }

#endif


  // Around 50 FPS

  if (

    now -
    lastFrame >=
    20

  ) {

    lastFrame =
      now;


    // Smooth real or test attitude

    smoothRoll +=

      (
        flight.rollDeg -
        smoothRoll
      ) *
      0.28f;


    smoothPitch +=

      (
        flight.pitchDeg -
        smoothPitch
      ) *
      0.28f;


    renderFrame();
  }
}
