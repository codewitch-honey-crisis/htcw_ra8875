#pragma once
#include "gfx_core.hpp"
#include "gfx_positioning.hpp"
#include "gfx_pixel.hpp"
#include "gfx_palette.hpp"
#include <Arduino.h>
#include <SPI.h>
// We use the decoupled bus system, but not really.
// This driver only works with SPI, and sidesteps
// most of the tft_spi internals because they do
// not seem to work with this driver. Maybe a 
// timing issue
#include <tft_core.hpp>

namespace arduino {
    template<uint16_t Width,
            uint16_t Height,
            typename Bus,
            int8_t PinRst,
            int8_t PinInt = -1,
            uint8_t Rotation = 0,
            uint32_t WriteSpeedPercent = 200,
            uint32_t ReadSpeedPercent = 200,
            uint32_t InitSpeedPercent = 10,
            uint32_t TouchSpeedPercent = 100>
    class ra8875 {
        static_assert(Bus::type==tft_io_type::spi,"This driver only supports SPI");
    public:
        // indicates the type, itself
        using type = ra8875;
        // indicates the pixel type
        using pixel_type = gfx::rgb_pixel<16>;
        // indicates the capabilities of the driver
        using caps = gfx::gfx_caps<false,false,true,true,false,false,false>;
        // indicates the attached bus
        using bus = Bus;
        constexpr static const uint8_t rotation = Rotation & 3;
        constexpr static const float write_speed_multiplier = WriteSpeedPercent / 100.0;
        constexpr static const float read_speed_multiplier = ReadSpeedPercent / 100.0;
        constexpr static const float init_speed_multiplier = InitSpeedPercent / 100.0;
        constexpr static const float touch_speed_multiplier = TouchSpeedPercent / 100.0;
        constexpr static const int8_t pin_rst = PinRst;
        constexpr static const int8_t pin_int = PinInt;
    private:
        constexpr static const uint16_t native_width = Width;
        constexpr static const uint16_t native_height = Height;
        constexpr static const uint16_t voffset = (native_width == 80) * 192;
        constexpr static const uint8_t RA8875_MWCR0_LRTD = 0x00;
        constexpr static const uint8_t RA8875_MWCR0_RLTD = 0x04;
        constexpr static const uint8_t RA8875_MWCR0_TDLR = 0x08;
        constexpr static const uint8_t RA8875_MWCR0_DTLR = 0x0C;
        uint8_t reg(uint8_t reg) const {
            send_command(reg);
            return recv_data();
        }
        void reg(uint8_t reg, uint8_t value) const {
            send_command(reg);
            send_data(value);
        }
        void sendx(uint8_t x, uint8_t value) const {
            digitalWrite(bus::pin_cs, LOW);
            spi().beginTransaction(m_spi_settings);
            spi().transfer(x);
            spi().transfer(value);
            spi().endTransaction();
            digitalWrite(bus::pin_cs, HIGH);
        }
        uint8_t recvx(uint8_t x) const {
            digitalWrite(bus::pin_cs, LOW);
            spi().beginTransaction(m_spi_settings);
            spi().transfer(x);
            uint8_t result = spi().transfer(0);
            spi().endTransaction();
            digitalWrite(bus::pin_cs, HIGH);
            return result;
        }
        inline void send_command(uint8_t value) const {
            static const uint8_t RA8875_CMDWRITE = 0x80;
            sendx(RA8875_CMDWRITE,value);
        }
        inline void send_data(uint8_t value) const {
            static const uint8_t RA8875_DATAWRITE = 0x00;
            sendx(RA8875_DATAWRITE,value);
        }
        inline uint8_t recv_data() const {
            static const uint8_t RA8875_DATAREAD = 0x40;
            return recvx(RA8875_DATAREAD);
        }
        inline uint8_t recv_command() const {
            static const uint8_t RA8875_CMDREAD = 0xC0;
            return recvx(RA8875_CMDREAD);
        }
        // waits for an operation to complete
        void poll(uint8_t rg, uint8_t flag) const {
            while (true) {
                uint8_t result = reg(rg);
                if (!(result & flag))
                return;
            }
        }
        bool initialize_pll() {
            static const uint8_t RA8875_PLLC1 = 0x88;
            static const uint8_t  RA8875_PLLC1_PLLDIV1 = 0x00;
            static const uint8_t  RA8875_PLLC2 = 0x89;
            static const uint8_t  RA8875_PLLC2_DIV4 = 0x02;
            if ((native_height == 480 && native_width == 80) || (native_height==480 && native_width==128) || (native_height==480 && native_width==272))
            {
                reg(RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 10);
                delay(1);
                reg(RA8875_PLLC2, RA8875_PLLC2_DIV4);
                delay(1);
                return true;
            }
            else if(native_height==800 && native_width==480)
            {
                reg(RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 11);
                delay(1);
                reg(RA8875_PLLC2, RA8875_PLLC2_DIV4);
                delay(1);
                return true;
            }
            return false;
        }
        bool initialize_display() {
            static const uint8_t RA8875_SYSR = 0x10;
            static const uint8_t RA8875_SYSR_16BPP = 0x0C;
            static const uint8_t RA8875_SYSR_MCU8 = 0x00;
            static const uint8_t RA8875_PCSR = 0x04;
            static const uint8_t RA8875_PCSR_PDATL = 0x80;
            static const uint8_t RA8875_PCSR_2CLK = 0x01;
            static const uint8_t RA8875_PCSR_4CLK = 0x02;
            static const uint8_t RA8875_HDWR = 0x14;
            static const uint8_t RA8875_HNDFTR = 0x15;
            static const uint8_t RA8875_HNDFTR_DE_HIGH = 0x00;
            static const uint8_t RA8875_HNDR = 0x16;
            static const uint8_t RA8875_HSTR = 0x17;
            static const uint8_t RA8875_HPWR = 0x18;
            static const uint8_t RA8875_HPWR_LOW = 0x00;
            static const uint8_t RA8875_VDHR0 = 0x19;
            static const uint8_t RA8875_VDHR1 = 0x1A;
            static const uint8_t RA8875_VNDR0 = 0x1B;
            static const uint8_t RA8875_VNDR1 = 0x1C;
            static const uint8_t RA8875_VSTR0 = 0x1D;
            static const uint8_t RA8875_VSTR1 = 0x1E;
            static const uint8_t RA8875_VPWR = 0x1F;
            static const uint8_t RA8875_VPWR_LOW = 0x00;
            static const uint8_t RA8875_MCLR = 0x8E;         
            static const uint8_t RA8875_MCLR_START = 0x80;
            static const uint8_t RA8875_MCLR_FULL = 0x00;
            static const uint8_t RA8875_DPCR = 0x20;
            static const uint8_t RA8875_VDIR = 0x04;
            static const uint8_t RA8875_HDIR = 0x08;
            reg(RA8875_SYSR, RA8875_SYSR_16BPP | RA8875_SYSR_MCU8);

            uint8_t pixclk;
            uint8_t hsync_start;
            uint8_t hsync_pw;
            uint8_t hsync_finetune;
            uint8_t hsync_nondisp;
            uint8_t vsync_pw;
            uint16_t vsync_nondisp;
            uint16_t vsync_start;

            if (native_height == 480 && native_width == 80) {
                pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_4CLK;
                hsync_nondisp = 10;
                hsync_start = 8;
                hsync_pw = 48;
                hsync_finetune = 0;
                vsync_nondisp = 3;
                vsync_start = 8;
                vsync_pw = 10;
                // This uses the bottom 80 pixels of a 272 pixel controller
            } else if (native_height == 480 && (native_width == 128 || native_width == 272)) {
                pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_4CLK;
                hsync_nondisp = 10;
                hsync_start = 8;
                hsync_pw = 48;
                hsync_finetune = 0;
                vsync_nondisp = 3;
                vsync_start = 8;
                vsync_pw = 10;
            } else if (native_height == 800 && native_width == 480) {
                pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_2CLK;
                hsync_nondisp = 26;
                hsync_start = 32;
                hsync_pw = 96;
                hsync_finetune = 0;
                vsync_nondisp = 32;
                vsync_start = 23;
                vsync_pw = 2;
            } else {
                return false;
            }
            // TODO:: check this
            constexpr static const bool flip_x = rotation==0||rotation==3;
            constexpr static const bool flip_y =  rotation==2||rotation==3;
            if(flip_x) {
                uint8_t x = reg(RA8875_DPCR);
                x=x&~(RA8875_HDIR | RA8875_VDIR);
                x|=RA8875_HDIR;
                if(flip_y) {
                    x|=RA8875_VDIR;
                }
                reg(RA8875_DPCR,x);
            } else if(flip_y) {
                uint8_t x = reg(RA8875_DPCR);
                x=x&~(RA8875_HDIR | RA8875_VDIR);
                x|=RA8875_VDIR;
                reg(RA8875_DPCR,x);
            }
            reg(RA8875_PCSR, pixclk);
            delay(1);


            reg(RA8875_HDWR, (native_height / 8) - 1); // H width: (HDWR + 1) * 8 = 480
            reg(RA8875_HNDFTR, RA8875_HNDFTR_DE_HIGH + hsync_finetune);
            reg(RA8875_HNDR, (hsync_nondisp - hsync_finetune - 2) /
                                    8); // H non-display: HNDR * 8 + HNDFTR + 2 = 10
            reg(RA8875_HSTR, hsync_start / 8 - 1); // Hsync start: (HSTR + 1)*8
            reg(RA8875_HPWR,
                    RA8875_HPWR_LOW +
                        (hsync_pw / 8 - 1)); // HSync pulse width = (HPWR+1) * 8

            reg(RA8875_VDHR0, (uint16_t)(native_width - 1 + voffset) & 0xFF);
            reg(RA8875_VDHR1, (uint16_t)(native_width - 1 + voffset) >> 8);
            reg(RA8875_VNDR0, vsync_nondisp - 1); // V non-display period = VNDR + 1
            reg(RA8875_VNDR1, vsync_nondisp >> 8);
            reg(RA8875_VSTR0, vsync_start - 1); // Vsync start position = VSTR + 1
            reg(RA8875_VSTR1, vsync_start >> 8);
            reg(RA8875_VPWR,
                    RA8875_VPWR_LOW + vsync_pw - 1); // Vsync pulse width = VPWR + 1

            set_active_window({0,0,native_height-1,native_width-1});
            /* ToDo: Setup touch panel? */

            /* Clear the entire window */
            reg(RA8875_MCLR, RA8875_MCLR_START | RA8875_MCLR_FULL);
            delay(500);
            
            return true;
        }
        bool initialize_touch() {
            static const uint8_t RA8875_TPCR0 = 0x70;
            static const uint8_t RA8875_TPCR0_ENABLE = 0x80;
            static const uint8_t RA8875_TPCR0_WAIT_4096CLK = 0x30;
            static const uint8_t RA8875_TPCR0_ADCCLK_DIV4 = 0x02;
            static const uint8_t RA8875_TPCR0_WAKEENABLE = 0x08;
            static const uint8_t RA8875_TPCR1 = 0x71;
            static const uint8_t RA8875_TPCR1_AUTO = 0x00;
            static const uint8_t RA8875_TPCR1_DEBOUNCE = 0x04;
            static const uint8_t RA8875_INTC1 = 0xF0;
            static const uint8_t RA8875_INTC1_TP = 0x04;
            static const uint8_t RA8875_TPCR0_ADCCLK_DIV16 = 0x04;
            if(pin_int!=-1) {
                pinMode(pin_int, INPUT);
                digitalWrite(pin_int, HIGH);
            }
            uint8_t adcClk = (uint8_t)RA8875_TPCR0_ADCCLK_DIV4;

            if (native_height==800) // match up touch size with LCD size
                adcClk = (uint8_t)RA8875_TPCR0_ADCCLK_DIV16;

            
            // Enable Touch Panel (Reg 0x70) 
            reg(RA8875_TPCR0, RA8875_TPCR0_ENABLE | RA8875_TPCR0_WAIT_4096CLK |
                                    RA8875_TPCR0_WAKEENABLE | adcClk); // 10mhz max!
            // Set Auto Mode      (Reg 0x71)
            reg(RA8875_TPCR1, RA8875_TPCR1_AUTO |
                                    RA8875_TPCR1_DEBOUNCE);
            // Enable TP INT
            reg(RA8875_INTC1, reg(RA8875_INTC1) | RA8875_INTC1_TP);

            return true;
        
        }
        bool set_active_window(const gfx::rect16& rect) {
            static const uint8_t RA8875_HSAW0 = 0x30;
            static const uint8_t RA8875_HSAW1 = 0x31;
            static const uint8_t RA8875_VSAW0 = 0x32;
            static const uint8_t RA8875_VSAW1 = 0x33;
            static const uint8_t RA8875_HEAW0 = 0x34;
            static const uint8_t RA8875_HEAW1 = 0x35;
            static const uint8_t RA8875_VEAW0 = 0x36;
            static const uint8_t RA8875_VEAW1 = 0x37;
            
            gfx::rect16 r=rect.normalize();
            //r= apply_rotation(rect);
            reg(RA8875_HSAW0, r.x1); // horizontal start point
            reg(RA8875_HSAW1, r.x1 >> 8);
            reg(RA8875_HEAW0, r.x2); // horizontal end point
            reg(RA8875_HEAW1, r.x2 >> 8);

            /* Set active window Y */
            reg(RA8875_VSAW0, (uint16_t)(r.y1 + voffset) & 0xFF); // vertical start point
            reg(RA8875_VSAW1, (uint16_t)(r.y1 + voffset)>>8);
            reg(RA8875_VEAW0,
                    (uint16_t)(r.y2 + voffset) & 0xFF); // vertical end point
            reg(RA8875_VEAW1, (uint16_t)(r.y2 + voffset) >> 8);
            return true;
        }
        bool recv_touch(gfx::point16* out_point) {
            static const uint8_t RA8875_INTC2 = 0xF1;
            static const uint8_t RA8875_INTC2_TP = 0x04;
            static const uint8_t RA8875_TPXH = 0x72;
            static const uint8_t RA8875_TPYH = 0x73;
            static const uint8_t RA8875_TPXYL = 0x74;
            uint16_t tx, ty;
            uint8_t temp;

            tx = reg(RA8875_TPXH);
            ty = reg(RA8875_TPYH);
            temp = reg(RA8875_TPXYL);
            tx <<= 2;
            ty <<= 2;
            tx |= temp & 0x03;        // get the bottom x bits
            ty |= (temp >> 2) & 0x03; // get the bottom y bits
            *out_point = type::apply_rotation(gfx::point16(tx,ty),true);
            // Clear TP INT Status
            reg(RA8875_INTC2, RA8875_INTC2_TP);
            return true;
        }
        bool m_initialized;
        bool m_in_batch;
        size_t m_batch_offset;
        gfx::rect16 m_batch_bounds;
        SPISettings m_spi_settings;
        constexpr static uint8_t get_write_orientation() {
            if(rotation&1) {
                // portrait
               return RA8875_MWCR0_LRTD; 
            } else {
                // landscape
                return RA8875_MWCR0_TDLR;
               
            }
        }
        constexpr static const uint8_t write_orientation = get_write_orientation();
                template<typename Source, bool Blt> struct draw_helper {
            static gfx::gfx_result do_draw(SPIClass& s, const Source& src, const gfx::rect16& srcr,const gfx::rect16& dstr) {
                uint16_t w = dstr.dimensions().width;
                uint16_t h = dstr.dimensions().height;
                for(uint16_t y=0;y<h;++y) {
                    for(uint16_t x=0;x<w;++x) {
                        typename Source::pixel_type pp;
                        gfx::gfx_result rr=src.point(gfx::point16(x+srcr.x1,y+srcr.y1), &pp);
                        if(rr!=gfx::gfx_result::success) {
                            return rr;
                        }
                        pixel_type p;
                        rr=gfx::convert_palette_to(src,pp,&p);
                        if(gfx::gfx_result::success!=rr) {
                            return rr;
                        }
                        uint16_t v = p.channel_unchecked<2>() | (p.channel_unchecked<1>()<<5) | (p.channel_unchecked<0>()<<11);
                        s.transfer16(v);
                    }
                }
                return gfx::gfx_result::success;
            }
        };
        template<typename Source> struct draw_helper<Source, true> {
            static gfx::gfx_result do_draw(SPIClass& s, const Source& src, const gfx::rect16& srcr,const gfx::rect16& dstr) {
                uint16_t w = dstr.dimensions().width;
                
                if(srcr.x1==0 && w == src.dimensions().width && w == srcr.width()) {
                    const uint8_t* p = src.begin()+(w*srcr.y1);
                    size_t len = w*srcr.height()*2;
                    s.writeBytes( p, (uint32_t)len);
                    return gfx::gfx_result::success;
                }
                return draw_helper<Source, false>::do_draw(s,src,srcr,dstr);
                
                
            }
        };
        constexpr static gfx::point16 translate_rotation(gfx::point16 pt, bool translate) {
            switch(write_orientation) {
                case RA8875_MWCR0_TDLR:
                    return {pt.y,pt.x};
                default:
                    return pt;
            }
        }
        constexpr inline static gfx::rect16 translate_rotation(const gfx::rect16& r,bool translate) {
            return gfx::rect16(translate_rotation(r.point1(),translate),translate_rotation(r.point2(),translate));
        }
#if defined(ESP32) || defined(ARDUINO_ARCH_STM32)
        static SPIClass m_spi;
#endif
        inline static SPIClass& spi() FORCE_INLINE {
#if defined(ESP32) || defined(ARDUINO_ARCH_STM32)
            return m_spi;
#else

 #if SPI_INTERFACES_COUNT > 1
            if(bus::spi_host==1) {
                return SPI1;
            }
#endif
#if SPI_INTERFACES_COUNT > 2
            if(bus::spi_host==2) {
                return SPI2;
            }
#endif
#if SPI_INTERFACES_COUNT > 3
            if(bus::spi_host==3) {
                return SPI3;
            }
#endif
#if SPI_INTERFACES_COUNT > 4
            if(bus::spi_host==4) {
                return SPI4;
            }
#endif
#if SPI_INTERFACES_COUNT > 5
            if(bus::spi_host==5) {
                return SPI5;
            }
#endif

#endif
            return SPI;
        }
    public:
        ra8875() : m_initialized(false),
                    m_in_batch(false),
                    m_batch_offset(0),
                    m_batch_bounds(0,0,0,0) {

        }
        gfx::gfx_result initialize() {
            static const uint8_t RA8875_PWRR = 0x01;
            static const uint8_t RA8875_PWRR_DISPON = 0x80;
            static const uint8_t RA8875_PWRR_NORMAL = 0x00;
            static const uint8_t RA8875_GPIOX = 0xC7;
            static const uint8_t RA8875_PWM_CLK_DIV1024 = 0x0A;
            static const uint8_t RA8875_P1CR = 0x8A;
            static const uint8_t RA8875_P1CR_ENABLE = 0x80;
            static const uint8_t RA8875_P1DCR = 0x8B;
            static const uint8_t RA8875_MRWC = 0x02;
            static const uint8_t RA8875_MWCR0 = 0x40;
            static const uint8_t RA8875_MWCR0_DIRMASK = 0x0C;
            if(!m_initialized) {
                pinMode(bus::pin_cs,OUTPUT);
                digitalWrite(bus::pin_cs,HIGH);
#ifdef ASSIGNABLE_SPI_PINS
                spi().begin(bus::pin_sclk,bus::pin_miso,bus::pin_mosi,-1);
#else // !ASSIGNABLE_SPI_PINS
            spi().begin();
#endif // !ASSIGNABLE_SPI_PINS
                m_spi_settings = SPISettings(init_speed_multiplier*10*1000*1000,SPI_MSBFIRST,bus::spi_mode);
        
                pinMode(pin_rst, OUTPUT);

                digitalWrite(pin_rst, LOW);
                delay(100);
                digitalWrite(pin_rst, HIGH);
                delay(100);
                
                uint8_t result = reg(0);
                if (result != 0x75) {
                    Serial.println("RA8875 not detected");
                    return gfx::gfx_result::device_error;
                }
                if(!initialize_pll()) {
                    
                    return gfx::gfx_result::device_error;
                }
                
                if(!initialize_display()) {
                    return gfx::gfx_result::device_error;
                }
                // turn on the display
                reg(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPON);

                // GPIOX on
                reg(RA8875_GPIOX, 1);

                // enable backlight
                reg(RA8875_P1CR, RA8875_P1CR_ENABLE | (RA8875_PWM_CLK_DIV1024 & 0xF));
                reg(RA8875_P1DCR, 0xFF);

                // set rotation
                reg(RA8875_MWCR0, (reg(RA8875_MWCR0) & ~RA8875_MWCR0_DIRMASK) | write_orientation);
                send_command(RA8875_MRWC);
                
                if(!initialize_touch()) {
                    return gfx::gfx_result::device_error;
                }
                m_spi_settings = SPISettings(write_speed_multiplier*10*1000*1000,SPI_MSBFIRST,bus::spi_mode);
                
                m_initialized = true;
            
            }
            return gfx::gfx_result::success;
        }

        bool touched(gfx::point16* out_point) {
            static const uint8_t RA8875_INTC2 = 0xF1;
            static const uint8_t RA8875_INTC2_TP = 0x04;
            if(pin_int == -1 || !digitalRead(pin_int)) {
                m_spi_settings = SPISettings(touch_speed_multiplier*10*1000*1000,SPI_MSBFIRST,bus::spi_mode);
                //bus::set_speed_multiplier(init_speed_multiplier);
                //Serial.println("Passed INT pin check");
                if(reg(RA8875_INTC2) & RA8875_INTC2_TP) {
                    //Serial.println("Passed touched reg check");
                    bool res= recv_touch(out_point);
                    m_spi_settings = SPISettings(write_speed_multiplier*10*1000*1000,SPI_MSBFIRST,bus::spi_mode);
                    return res;
                }
                m_spi_settings = SPISettings(write_speed_multiplier*10*1000*1000,SPI_MSBFIRST,bus::spi_mode);
            }
            return false;
        }
        
        // retrieves the dimensions of the screen
        constexpr inline gfx::size16 dimensions() const {
            if(rotation&1) { // landscape
                return {native_height,native_width};
            }
            return {native_width,native_height};
        }
        // retrieves the bounds of the screen
        constexpr inline gfx::rect16 bounds() const {
            return dimensions().bounds();
        }
        // sets a point to the specified pixel
        gfx::gfx_result point(gfx::point16 location,pixel_type pixel) {
            static const uint8_t RA8875_CURH0 = 0x46;
            static const uint8_t RA8875_CURH1 = 0x47;
            static const uint8_t RA8875_CURV0 = 0x48;
            static const uint8_t RA8875_CURV1 = 0x49;
            static const uint8_t RA8875_MRWC = 0x02;
            static const uint8_t RA8875_DATAWRITE = 0x00;
            gfx::gfx_result r;
            if(location.x>=dimensions().width || location.y>=dimensions().height)
                return gfx::gfx_result::success;
            if(!m_initialized) {
                r=initialize();
                if(r!=gfx::gfx_result::success)
                    return r;
            }
            location = translate_rotation(location,true);
            reg(RA8875_CURH0, location.x);
            reg(RA8875_CURH1, location.x >> 8);
            reg(RA8875_CURV0, location.y);
            reg(RA8875_CURV1, location.y >> 8);
            send_command(RA8875_MRWC);
            digitalWrite(bus::pin_cs, LOW);
            auto v = pixel.value();
            m_spi.transfer(RA8875_DATAWRITE);
            m_spi.transfer(v);
            m_spi.transfer(v >> 8);
            digitalWrite(bus::pin_cs, HIGH);
            return gfx::gfx_result::success;
        }
        // fills the specified rectangle
        gfx::gfx_result fill(const gfx::rect16& rect,pixel_type pixel) {
            static const uint8_t RA8875_DCR = 0x90;
            static const uint8_t RA8875_DCR_LINESQUTRI_STATUS = 0x80;
            // it's a point
            if(rect.x1==rect.x2&&rect.y1==rect.y2) {
                return point(rect.point1(),pixel);
            }
            if(!rect.intersects(bounds())) return gfx::gfx_result::success;
            
            if(!m_initialized) {
                gfx::gfx_result r =initialize();
                if(r!=gfx::gfx_result::success) {
                    return gfx::gfx_result::device_error;
                }
            }
            
            gfx::rect16 r = translate_rotation(rect,false).crop({0,0,native_height-1,native_width-1});
            // it's a line
            if(r.x1==r.x2 || r.y1==r.y2) {    
                send_command(0x91);
                send_data(r.x1);
                send_command(0x92);
                send_data(r.x1 >> 8);

                send_command(0x93);
                send_data(r.y1);
                send_command(0x94);
                send_data(r.y1 >> 8);

                send_command(0x95);
                send_data(r.x2);
                send_command(0x96);
                send_data((r.x2) >> 8);

                send_command(0x97);
                send_data(r.y2);
                send_command(0x98);
                send_data((r.y2) >> 8);

                send_command(0x63);
                send_data(pixel.channel_unchecked<0>());
                send_command(0x64);
                send_data(pixel.channel_unchecked<1>());
                send_command(0x65);
                send_data(pixel.channel_unchecked<2>());

                send_command(RA8875_DCR);
                send_data(0x80);

                poll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
                return gfx::gfx_result::success;
            }
            // set x
            send_command(0x91);
            send_data(r.x1);
            send_command(0x92);
            send_data(r.x1 >> 8);
            // set y
            send_command(0x93);
            send_data(r.y1);
            send_command(0x94);
            send_data(r.y1 >> 8);

            // set width
            send_command(0x95);
            send_data(r.x2);
            send_command(0x96);
            send_data(r.x2 >> 8);

            // set height
            send_command(0x97);
            send_data(r.y2);
            send_command(0x98);
            send_data(r.y2 >> 8);

            // set color 
            send_command(0x63);
            send_data(pixel.channel_unchecked<0>());
            send_command(0x64);
            send_data(pixel.channel_unchecked<1>());
            send_command(0x65);
            send_data(pixel.channel_unchecked<2>());

            send_command(RA8875_DCR);
            send_data(0xB0);

            // wait
            poll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);

            return gfx::gfx_result::success;
        }
        // clears the target rectangle
        gfx::gfx_result clear(const gfx::rect16& rect) {
            return fill(rect,pixel_type());
        }
        // begins a batch operation
        gfx::gfx_result begin_batch(const gfx::rect16& rect) {
            static const uint8_t RA8875_DATAWRITE = 0x00;
            static const uint8_t RA8875_CURH0 = 0x46;
            static const uint8_t RA8875_CURH1 = 0x47;
            static const uint8_t RA8875_CURV0 = 0x48;
            static const uint8_t RA8875_CURV1 = 0x49;
            static const uint8_t RA8875_MRWC = 0x02;
            static const uint8_t RA8875_MWCR0 = 0x40;
            static const uint8_t RA8875_MWCR0_DIRMASK = 0x0C;
            
            //static const uint8_t RA8875_MWCR0_TDLR = 0x08;
            gfx::gfx_result rr = commit_batch();
            if(gfx::gfx_result::success != rr) {
                return rr;
            }
            gfx::rect16 r = translate_rotation(rect.normalize(),false).crop(bounds());
            set_active_window(r);
            //Serial.printf("window: (%d, %d)-(%d, %d)\r\n",(int)r.x1,(int)r.y1,(int)r.x2,(int)r.y2);
            r=translate_rotation(rect,true);
            //Serial.printf("translated: (%d, %d)-(%d, %d)\r\n",(int)r.x1,(int)r.y1,(int)r.x2,(int)r.y2);
            reg(RA8875_CURH0, r.x1);
            reg(RA8875_CURH1, r.x1 >> 8);
            reg(RA8875_CURV0, r.y1);
            reg(RA8875_CURV1, r.y1 >> 8);
        
            reg(RA8875_MWCR0, (reg(RA8875_MWCR0) & ~RA8875_MWCR0_DIRMASK) | write_orientation);
            send_command(RA8875_MRWC);
            digitalWrite(bus::pin_cs, LOW);
            m_spi.transfer(RA8875_DATAWRITE);
            m_batch_bounds = r;
            m_batch_offset = 0;
            m_in_batch = true;
            return gfx::gfx_result::success;
        }
        
        // writes a pixel to a pending batch
        gfx::gfx_result write_batch(pixel_type color) {
            if(!m_in_batch) return gfx::gfx_result::invalid_state;
            uint16_t v = color.channel_unchecked<2>() | (color.channel_unchecked<1>()<<5) | (color.channel_unchecked<0>()<<11);
            m_spi.transfer16(v);
            ++m_batch_offset;
            return gfx::gfx_result::success;
        }
        // commits a pending batch
        gfx::gfx_result commit_batch() {
            if(!m_in_batch) {
                return gfx::gfx_result::success;
            }
            digitalWrite(bus::pin_cs, HIGH);
            set_active_window({0,0,native_height-1,native_width-1});
            m_in_batch = false;
            return gfx::gfx_result::success;
        }
        template<typename Source>
        gfx::gfx_result copy_from(const gfx::rect16& src_rect,const Source& src,gfx::point16 location)  {
            static const uint8_t RA8875_DATAWRITE = 0x00;
            static const uint8_t RA8875_CURH0 = 0x46;
            static const uint8_t RA8875_CURH1 = 0x47;
            static const uint8_t RA8875_CURV0 = 0x48;
            static const uint8_t RA8875_CURV1 = 0x49;
            static const uint8_t RA8875_MRWC = 0x02;
            static const uint8_t RA8875_MWCR0 = 0x40;
            static const uint8_t RA8875_MWCR0_DIRMASK = 0x0C;
            gfx::gfx_result r = initialize();
            if(r!=gfx::gfx_result::success) {
                return r;
            }
            gfx::gfx_result rr = commit_batch();
            if(gfx::gfx_result::success!=rr) {
                return rr;
            }
            gfx::rect16 srcr = src_rect.normalize().crop(src.bounds());
            gfx::rect16 dstr(location,src_rect.dimensions());
            dstr=dstr.crop(bounds());
            if(srcr.width()>dstr.width()) {
                srcr.x2=srcr.x1+dstr.width()-1;
            }
            if(srcr.height()>dstr.height()) {
                srcr.y2=srcr.y1+dstr.height()-1;
            }
            set_active_window(translate_rotation(dstr,false));
            const gfx::rect16 rot = translate_rotation(dstr,true);
            
            digitalWrite(bus::pin_cs, HIGH);
            //const gfx::rect16 rot2 = rot.normalize();
            
            reg(RA8875_CURH0, rot.x1);
            reg(RA8875_CURH1, rot.x1>>8);
            reg(RA8875_CURV0, rot.y1);
            reg(RA8875_CURV1, rot.y1>>8);
        
            reg(RA8875_MWCR0, (reg(RA8875_MWCR0) & ~RA8875_MWCR0_DIRMASK) | write_orientation);
            send_command(RA8875_MRWC);
            digitalWrite(bus::pin_cs, LOW);
            m_spi.transfer(RA8875_DATAWRITE);
            rr=draw_helper<Source,Source::caps::blt && Source::pixel_type::template equals_exact<gfx::rgb_pixel<16>>::value>::do_draw(m_spi,src,srcr,dstr);
            digitalWrite(bus::pin_cs, HIGH);
            set_active_window({0,0,native_height-1,native_width-1});
            return rr;
        }
        gfx::gfx_result scroll_window(const gfx::rect16& rect, uint8_t mode) {
            static const uint8_t RA8875_HSBE0 = 0x38;
            static const uint8_t RA8875_HSBE1 = 0x39;
            static const uint8_t RA8875_VSBE0 = 0x3a;
            static const uint8_t RA8875_VSBE1 = 0x3b;
            static const uint8_t RA8875_HDBE0 = 0x3c;
            static const uint8_t RA8875_HDBE1 = 0x3d;
            static const uint8_t RA8875_VDBE0 = 0x3e;
            static const uint8_t RA8875_VDBE1 = 0x3f;
            static const uint8_t RA8875_LTPR0 = 0x52;
            gfx::gfx_result rr = initialize();
            if(rr!=gfx::gfx_result::success) {
                return rr;
            }
            const gfx::rect16 r = rect.crop(bounds()).normalize();
            r=translate_rotation(r,true).normalize();
            // Horizontal Start point of Scroll Window
            send_command(RA8875_HSBE0);
            send_data(r.x1);
            send_command(RA8875_HSBE1);
            send_data(r.x1 >> 8);

            // Vertical Start Point of Scroll Window
            send_command(RA8875_VSBE0);
            send_data(r.y1);
            send_command(RA8875_VSBE1);
            send_data(r.y1 >> 8);

            // Horizontal End Point of Scroll Window
            send_command(RA8875_HDBE0);
            send_data(r.x2);
            send_command(RA8875_HDBE1);
            send_data(r.x2 >> 8);

            // Vertical End Point of Scroll Window
            send_command(RA8875_VDBE0);
            send_data(r.y2);
            send_command(RA8875_VDBE1);
            send_data(r.y2 >> 8);

            // Scroll function setting
            send_command(RA8875_LTPR0);
            send_data(mode);
            return gfx::gfx_result::success;
        }

        gfx::gfx_result scroll_x(int16_t dist) {
            static const uint8_t RA8875_HOFS0 = 0x24;
            static const uint8_t RA8875_HOFS1 = 0x25;
            static const uint8_t RA8875_VOFS0 = 0x26;
            static const uint8_t RA8875_VOFS1 = 0x27;
            if(!initialize()) {
                return gfx::gfx_result::device_error;
            }
            if(write_orientation==RA8875_MWCR0_TDLR || write_orientation == RA8875_MWCR0_DTLR) {
                send_command(RA8875_VOFS0);
                send_data(dist);
                send_command(RA8875_VOFS1);
                send_data(dist >> 8);
            } else {
                send_command(RA8875_HOFS0);
                send_data(dist);
                send_command(RA8875_HOFS1);
                send_data(dist >> 8);
            }
            return gfx::gfx_result::success;
        }
        gfx::gfx_result scroll_y(int16_t dist) {
            static const uint8_t RA8875_HOFS0 = 0x24;
            static const uint8_t RA8875_HOFS1 = 0x25;
            static const uint8_t RA8875_VOFS0 = 0x26;
            static const uint8_t RA8875_VOFS1 = 0x27;
            if(!initialize()) {
                return gfx::gfx_result::device_error;
            }
            if(write_orientation==RA8875_MWCR0_TDLR || write_orientation == RA8875_MWCR0_DTLR) {
                send_command(RA8875_HOFS0);
                send_data(dist);
                send_command(RA8875_HOFS1);
                send_data(dist >> 8);
            } else {
                send_command(RA8875_VOFS0);
                send_data(dist);
                send_command(RA8875_VOFS1);
                send_data(dist >> 8);
            }
            return gfx::gfx_result::success;
        }
    };
#if defined(ESP32) || defined(ARDUINO_ARCH_STM32)
    template<uint16_t Width,
            uint16_t Height,
            typename Bus,
            int8_t PinRst,
            int8_t PinInt,
            uint8_t Rotation,
            uint32_t WriteSpeedPercent,
            uint32_t ReadSpeedPercent,
            uint32_t InitSpeedPercent,
            uint32_t TouchSpeedPercent> SPIClass ra8875<Width,
                                                        Height,
                                                        Bus,
                                                        PinRst,
                                                        PinInt,
                                                        Rotation,
                                                        WriteSpeedPercent,
                                                        ReadSpeedPercent,
                                                        InitSpeedPercent,
                                                        TouchSpeedPercent>
                                                        ::m_spi(Bus::spi_host);
                        
#endif
}
