// You need a lot of program space/flash space to embed a truetype font
#if (defined(ARDUINO_ARCH_SAMD) && defined(SEEED_GROVE_UI_WIRELESS)) || defined(ARDUINO_ARCH_ESP32)
#define USE_TRUETYPE
#endif

#define LCD_HOST    VSPI
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define PIN_NUM_DC   2
#define PIN_NUM_RST  4
#define PIN_NUM_BCKL 15

#define LCD_WIDTH 480
#define LCD_HEIGHT 800
#define LCD_ROTATION 1
#define PIN_CS 5
#define PIN_RST 15
#define PIN_INT 2

#include <Arduino.h>
#include <stdio.h>
#include <tft_io.hpp>
#include <ra8875.hpp>
#include <gfx_cpp14.hpp>
#ifdef USE_TRUETYPE
#include "Maziro.h"
#endif
#include "Bm437_ATI_9x16.h"

using namespace arduino;
using namespace gfx;
using bus_type = tft_spi_ex<LCD_HOST,PIN_NUM_CS,PIN_NUM_MOSI,PIN_NUM_MISO,PIN_NUM_CLK,SPI_MODE0,false
#ifdef OPTIMIZE_DMA
,(LCD_WIDTH*LCD_HEIGHT)/8+8
#endif
>;

using lcd_type = ra8875<LCD_WIDTH,LCD_HEIGHT,bus_type,PIN_RST, PIN_INT,LCD_ROTATION>;

lcd_type lcd;

using lcd_color = color<typename lcd_type::pixel_type>;

// prints a draw source as 4-bit grayscale ASCII
template <typename Source>
void print_source(const Source& src) {
    static const char *col_table = " .,-~;+=x!1%$O@#";
    for(int y = 0;y<src.dimensions().height;++y) {
        for(int x = 0;x<src.dimensions().width;++x) {
            typename Source::pixel_type px;
            src.point(point16(x,y),&px);
            const auto px2 = convert<typename Source::pixel_type,gsc_pixel<4>>(px);
            size_t i =px2.template channel<0>();
            char sz[2] = {col_table[i],0};
            Monitor.print(sz);
        }
        Monitor.println();
    }
}

constexpr static const size16 bmp_size(16,16);
// you can use YbCbCr for example. It's lossy, so you'll want extra bits
//using bmp_type = bitmap<ycbcr_pixel<HTCW_MAX_WORD>>;
using bmp_type = bitmap<typename lcd_type::pixel_type>;
using bmp_color = color<typename bmp_type::pixel_type>;
using bmpa_pixel_type = rgba_pixel<HTCW_MAX_WORD>;
using bmpa_color = color<bmpa_pixel_type>;
// declare the bitmap
uint8_t bmp_buf[bmp_type::sizeof_buffer(bmp_size)];
bmp_type bmp(bmp_size,bmp_buf);

// produced by request
void scroll_text_demo() {
    lcd.clear(lcd.bounds());
    // draw stuff
    bmp.clear(bmp.bounds()); // comment this out and check out the uninitialized RAM. It looks neat.
    bmpa_pixel_type col = bmpa_color::yellow;
    col.channelr<channel_name::A>(.5);
    // bounding info for the face
    srect16 bounds(0,0,bmp_size.width-1,(bmp_size.height-1));
    rect16 ubounds(0,0,bounds.x2,bounds.y2);

    // draw the face
    draw::filled_ellipse(bmp,bounds,col);
    
    // draw the left eye
    srect16 eye_bounds_left(spoint16(bounds.width()/5,bounds.height()/5),ssize16(bounds.width()/5,bounds.height()/3));
    draw::filled_ellipse(bmp,eye_bounds_left,bmp_color::black);
    
    // draw the right eye
    srect16 eye_bounds_right(
        spoint16(
            bmp_size.width-eye_bounds_left.x1-eye_bounds_left.width(),
            eye_bounds_left.y1
        ),eye_bounds_left.dimensions());
    draw::filled_ellipse(bmp,eye_bounds_right,bmp_color::black);
    
    // draw the mouth
    srect16 mouth_bounds=bounds.inflate(-bounds.width()/7,-bounds.height()/8).normalize();
    // we need to clip part of the circle we'll be drawing
    srect16 mouth_clip(mouth_bounds.x1,mouth_bounds.y1+mouth_bounds.height()/(float)1.6,mouth_bounds.x2,mouth_bounds.y2);
    draw::ellipse(bmp,mouth_bounds,bmp_color::black,&mouth_clip);

    // do some alpha blended rectangles
    col = bmpa_color::red;
    col.channelr<channel_name::A>(.5);
    draw::filled_rectangle(bmp,srect16(spoint16(0,0),ssize16(bmp.dimensions().width,bmp.dimensions().height/4)),col);
    col = bmpa_color::blue;
    col.channelr<channel_name::A>(.5);
    draw::filled_rectangle(bmp,srect16(spoint16(0,0),ssize16(bmp.dimensions().width/4,bmp.dimensions().height)),col);
    col = bmpa_color::green;
    col.channelr<channel_name::A>(.5);
    draw::filled_rectangle(bmp,srect16(spoint16(0,bmp.dimensions().height-bmp.dimensions().height/4),ssize16(bmp.dimensions().width,bmp.dimensions().height/4)),col);
    col = bmpa_color::purple;
    col.channelr<channel_name::A>(.5);
    draw::filled_rectangle(bmp,srect16(spoint16(bmp.dimensions().width-bmp.dimensions().width/4,0),ssize16(bmp.dimensions().width/4,bmp.dimensions().height)),col);
    // uncomment to convert it to grayscale
    // resample<bmp_type,gsc_pixel<8>>(bmp);
    // uncomment to downsample
    // resample<bmp_type,rgb_pixel<8>>(bmp);
    srect16 new_bounds(0,0,63,63);

    // try using different values here. Bicubic yields the best visual result, but it's pretty slow. 
    // Bilinear is faster but better for shrinking images or changing sizes small amounts
    // Fast uses a nearest neighbor algorithm and is performant but looks choppy
    const bitmap_resize resize_type = 
         bitmap_resize::resize_bicubic;
        // bitmap_resize::resize_bilinear;
        //bitmap_resize::resize_fast;
    draw::bitmap(lcd,new_bounds.center_horizontal((srect16)lcd.bounds()).flip_vertical(),bmp,bmp.bounds(),resize_type);
    const font& f = Bm437_ATI_9x16_FON;
    const char* text = "copyright (C) 2021\r\nby honey the codewitch";
    ssize16 text_size = f.measure_text((ssize16)lcd.dimensions(),text);
    srect16 text_rect = text_size.bounds().center((srect16)lcd.bounds());
    int16_t text_start = text_rect.x1;
    
    // draw a polygon (a triangle in this case)
    // find the origin:
    const spoint16 porg = srect16(0,0,31,31)
                            .center_horizontal((srect16)lcd.bounds())
                                .offset(0,
                                    lcd.dimensions().height-32)
                                        .top_left();
    // draw a 32x32 triangle by creating a path
    spoint16 path_points[] = {spoint16(0,31),spoint16(15,0),spoint16(31,31)};
    spath16 path(3,path_points);
    // offset it so it starts at the origin
    path.offset_inplace(porg.x,porg.y); 
    // draw it
    draw::filled_polygon(lcd,path,lcd_color::coral);

    bool first=true;
    print_source(bmp);
    while(true) {

       draw::filled_rectangle(lcd,text_rect,lcd_color::black);
        if(text_rect.x2>=320) {
           draw::filled_rectangle(lcd,text_rect.offset(-320,0),lcd_color::black);
        }

        text_rect=text_rect.offset(2,0);
        draw::text(lcd,text_rect,text,f,lcd_color::old_lace,lcd_color::black,false);
        if(text_rect.x2>=320){
            draw::text(lcd,text_rect.offset(-320,0),text,f,lcd_color::old_lace,lcd_color::black,false);
        }
        if(text_rect.x1>=320) {
            text_rect=text_rect.offset(-320,0);
            first=false;
        }
        
        if(!first && text_rect.x1>=text_start)
            break;
    }
}
void lines_demo() {
    
    draw::filled_rectangle(lcd,(srect16)lcd.bounds(),lcd_color::white);
    const char* text = "GFX DEMO";
#ifdef USE_TRUETYPE
    const open_font& f = Maziro_ttf;
    float scale = f.scale(60);
    srect16 text_rect = f.measure_text((ssize16)lcd.dimensions(),{0,0},
                            text,scale).bounds();
    draw::text(lcd,
            text_rect.center((srect16)lcd.bounds()),
            {0,0},
            text,
            f,scale,
            lcd_color::dark_blue,lcd_color::white,false);
#else
  const font& f = Bm437_ATI_9x16_FON;
  srect16 text_rect = f.measure_text((ssize16)lcd.dimensions(),
                            text).bounds();
    draw::text(lcd,
            text_rect.center((srect16)lcd.bounds()),
            text,
            f,
            lcd_color::dark_blue,lcd_color::white,false);
#endif
    for(int i = 1;i<100;i+=2) {
        // calculate our extents
        srect16 r(i*(lcd.dimensions().width/100.0),
                i*(lcd.dimensions().height/100.0),
                lcd.dimensions().width-i*(lcd.dimensions().width/100.0)-1,
                lcd.dimensions().height-i*(lcd.dimensions().height/100.0)-1);
        // draw the four lines
        draw::line(lcd,srect16(0,r.y1,r.x1,lcd.dimensions().height-1),lcd_color::light_blue);
        draw::line(lcd,srect16(r.x2,0,lcd.dimensions().width-1,r.y2),lcd_color::hot_pink);
        draw::line(lcd,srect16(0,r.y2,r.x1,0),lcd_color::pale_green);
        draw::line(lcd,srect16(lcd.dimensions().width-1,r.y1,r.x2,lcd.dimensions().height-1),lcd_color::yellow);
        
    }
    
}

void alpha_demo() {
  draw::filled_rectangle(lcd,(srect16)lcd.bounds(),lcd_color::black);
  
  for(int y = 0;y<lcd.dimensions().height;y+=16) {
      for(int x = 0;x<lcd.dimensions().width;x+=16) {
          if(0!=((x+y)%32)) {
              draw::filled_rectangle(lcd,
                                  srect16(
                                      spoint16(x,y),
                                      ssize16(16,16)),
                                  lcd_color::white);
          }
      }
  }
  randomSeed(millis());
  
  rgba_pixel<32> px;
  spoint16 tpa[3];
  for(int i = 0;i<100;++i) {
    px.channel<channel_name::R>((rand()%256));
    px.channel<channel_name::G>((rand()%256));
    px.channel<channel_name::B>((rand()%256));
    px.channel<channel_name::A>(50+rand()%156);
    srect16 sr(0,0,rand()%50+50,rand()%50+50);
    sr.offset_inplace(rand()%(lcd.dimensions().width-sr.width()),
                    rand()%(lcd.dimensions().height-sr.height()));
    switch(rand()%4) {
      case 0:
        draw::filled_rectangle(lcd,sr,px);
        break;
      case 1:
        draw::filled_rounded_rectangle(lcd,sr,.1,px);
        break;
      case 2:
        draw::filled_ellipse(lcd,sr,px);
        break;
      case 3:
        tpa[0]={int16_t(((sr.x2-sr.x1)/2)+sr.x1),sr.y1};
        tpa[1]={sr.x2,sr.y2};
        tpa[2]={sr.x1,sr.y2};
        spath16 path(3,tpa);
        draw::filled_polygon(lcd,path,px);
        break;
    }  
  }
  
  
}

void setup() {
    Monitor.begin(115200);
   

}
void loop() {
  lines_demo();
  scroll_text_demo();
  alpha_demo();
}
