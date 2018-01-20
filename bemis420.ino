#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// These are hardwired pins.  Cannot be changed.
// Data (master out, slave in)
#define MOSI_PIN  11 // PB3
// SPI clock
#define SCK_PIN   13 // PB5
// SPI slave select (must be set output)
#define SS_PIN    10 // PB2

#define STRIP_LEN 320
#define NUM_RGB 3
#define NUM_SUBPIXELS (NUM_RGB * STRIP_LEN)

// Struct used for convenient manipulation of an RGB value
typedef union {
  struct {
    // Reorder these to change color output order
    uint8_t g;
    uint8_t r;
    uint8_t b;
  };
  uint8_t rgb[NUM_RGB];
} rgb_t;

// Strip buffer. We only have SRAM to store one Bemis-worth of pixels.
// Future workarounds could include just tesselating the output, so the state
// of the strip uses less data.
rgb_t strip[STRIP_LEN];

// Global variables for managing pushing data to the strips
volatile uint16_t pixel_pushing = STRIP_LEN;
volatile uint8_t subpixel = 0;

// Kicks off the SPI transfers for a whole strip, from the strip buffer
void init_strip_refresh() {
  if (pixel_pushing < STRIP_LEN) {
    Serial.println("Waiting for previous refresh...");
    while (pixel_pushing < STRIP_LEN);
  }
  pixel_pushing = 0;
  subpixel = 1;
  // Write first byte manually to start SPI transfer
  SPDR = strip[0].rgb[0];
}

// When an SPI transfer completes, start transferring the next byte (if there is one)
ISR(SPI_STC_vect) {
  if (pixel_pushing >= STRIP_LEN) return;
  SPDR = strip[pixel_pushing].rgb[subpixel];
  subpixel++;
  if (subpixel >= NUM_RGB) {
    subpixel = 0;
    pixel_pushing++;
  }
}

// Atomically get pixel_pushing count (idk if necessary?)
__attribute__((always_inline)) inline uint16_t get_pixel_pushing() {
  cbi(SPCR, SPIE);
  uint16_t pp = pixel_pushing;
  sbi(SPCR, SPIE);
  return pp;
}

// Maps an input byte to a color spectrum, and writes it to the provided rgb struct
#define SPEC_W 255
void spec(rgb_t* c, uint8_t i) {
  // Spectrum defined by 3 intervals. Each interval, one color is fading out,
  //   one fades in, one is off. First r->g, then g->b, then b->r
  uint8_t* out;
  uint8_t* in;
  uint8_t* off;
  if (i < SPEC_W / 3) {
    out = &c->r;
    in  = &c->g;
    off = &c->b;
  } else if (i < 2 * SPEC_W / 3) {
    i -= SPEC_W / 3;
    out = &c->g;
    in  = &c->b;
    off = &c->r;
  } else {
    i -= 2 * SPEC_W / 3;
    out = &c->b;
    in  = &c->r;
    off = &c->g;
  }
  *in  = 3 * i;
  *out = 255 - *in;
  *off = 0;
}

// Fills the strip buffer with a rainbow.
// If we're not done pushing a strip, then tag along behind the SPI transfers
void fill_rainbow(uint8_t offset = 0, uint8_t scale = 1) {
  for (uint16_t p = 0; p < STRIP_LEN; p++) {
    // Don't clobber a pixel till we've pushed it.
    while (p >= pixel_pushing);
    spec(&strip[p], scale * p + offset);
  }
}

// For debugging purposes.
void print_color(rgb_t* c) {
  Serial.print(c->r);
  Serial.print(' ');
  Serial.print(c->g);
  Serial.print(' ');
  Serial.print(c->b);
  Serial.print(' ');
}

void setup() {
  // Seed the pseudo-random number generator on a random floating analog input.
  randomSeed(analogRead(0));

  // ============= Configuring SPI ===============
  // Pin modes
  pinMode(SS_PIN,   OUTPUT);  // must be OUTPUT for proper master function
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(SCK_PIN,  OUTPUT);
  // Clock polarity.  (0 for idle low)
  cbi(SPCR, CPOL);
  // Clock phase.  (0 for sample on leading edge)
  cbi(SPCR, CPHA);
  // Data order (0 for MSB first)
  cbi(SPCR, DORD);
  // Master/slave (1 for master)
  sbi(SPCR, MSTR);
  // Clock rate (= f_osc/N = 16MHz/N)
  //    SPI2X:SPR1:SPR0
  //    100 000 101 001 110 010 111 011
  // N: 2   4   8   16  32  64  64  128
  // Too fast a rate causes erratic behavior on long strips
  sbi(SPCR, SPR0);
  cbi(SPCR, SPR1);
  sbi(SPSR, SPI2X);
  // Enable SPI
  sbi(SPCR, SPE);
  // Enable SPI interrupt
  sbi(SPCR, SPIE);

  // Other setup stuff
  Serial.begin(9600);
  /*
    uint8_t r = random(50);
    uint8_t g = random(50);
    uint8_t b = random(50);
    uint32_t t1 = micros();
    for (uint16_t i = 0; i < STRIP_LEN; i++) {
      spec(&strip[i], (255 / 20)*i);
    }
    uint32_t t2 = micros();
    init_strip_refresh();
    while (pixel_pushing < STRIP_LEN);
    uint32_t t3 = micros();
    Serial.println(t2 - t1);
    Serial.println(t3 - t2);

    Serial.println(sizeof(strip));
    Serial.println(sizeof(strip[0]));
  */
}

#define FRAME_PERIOD_MS 50
void loop() {
  static uint32_t last_refresh = 0;
  static uint8_t counter = 0;
  // Update frame at a steady rate
  while (millis() - last_refresh < FRAME_PERIOD_MS);
  last_refresh = millis();
  init_strip_refresh();
  counter++;
  fill_rainbow(3 * counter, 2);
}