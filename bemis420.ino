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
#define STRIP_SUBPIXELS (NUM_RGB * STRIP_LEN)

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

// The subpixel currently being pushed out over SPI
volatile uint16_t subpixel_pushing = STRIP_SUBPIXELS;
uint8_t inline __attribute__((always_inline)) safe_subpixel_pushing() {
  // Get high byte of subpixel_pushing , then low byte (avr-gcc is little-endian)
  uint8_t sp_high = *( (uint8_t*)(&subpixel_pushing) + 1);
  uint8_t sp_low  = *( (uint8_t*)(&subpixel_pushing) );
  uint16_t safe_sp = sp_low + ((uint16_t)sp_high) << 8;
  // We grabbed the high byte first to mitigate issues from a race condition:
  //   The ISR could increment subpixel_pushing in-between, with a carry.
  // Grabbing the high byte first means that our (erroneous) local value
  //   is less than the actual value, which is safer than possibly thinking
  //   we're further down the strip than we actually are.
  return safe_sp;
}
// Returns true if the specified pixel has been pushed
//   i.e. it's safe to overwrite it
uint8_t inline __attribute__((always_inline)) pixel_pushed(uint16_t pixel) {
  return safe_subpixel_pushing() > pixel * NUM_RGB;
}

// Kicks off pushing the strip buffer out to the strip
void init_strip_refresh() {
  if (safe_subpixel_pushing() < STRIP_SUBPIXELS) {
    Serial.println("Waiting for previous refresh...");
    while (safe_subpixel_pushing() < STRIP_SUBPIXELS);
  }
  subpixel_pushing = 0;
  // Write first subpixel manually to start SPI transfer
  SPDR = strip[0].rgb[0];
}

// When an SPI transfer completes, start transferring the next byte (if there is one)
ISR(SPI_STC_vect) {
  // If we've pushed the entire strip, we're done
  if (subpixel_pushing >= STRIP_SUBPIXELS) return;
  // Else push the next subpixel
  SPDR = ((uint8_t*)(&strip))[subpixel_pushing];
  subpixel_pushing++;
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
  // Simple linear fading for now
  *in  = 3 * i;
  *out = 255 - *in;
  *off = 0;
}

// Fills the strip buffer with a rainbow.
// If we're not done pushing a strip, then tag along behind the SPI transfers
void fill_rainbow(uint8_t offset = 0, uint8_t scale = 1) {
  for (uint16_t p = 0; p < STRIP_LEN; p++) {
    // Don't clobber a pixel till we've pushed it.
    while (!pixel_pushed(p));
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

  // ============ Other setup stuff =================
  Serial.begin(9600);
}

void loop() {
  static uint8_t counter = 0;
  #define FRAME_FPS 20
  static uint32_t last_strip_refresh = 0;
  if (millis() - last_strip_refresh < 1000UL / FRAME_FPS) {
    last_strip_refresh = millis();
    init_strip_refresh();
    counter++;
    fill_rainbow(3 * counter, 2);
  }
}
