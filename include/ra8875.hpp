#include <Arduino.h>

#include <gfx_core.hpp>
#include <gfx_palette.hpp>
#include <gfx_pixel.hpp>
#include <gfx_positioning.hpp>
#include <tft_core.hpp>
namespace arduino {
template <uint16_t Width, uint16_t Height, int8_t PinRst, typename Bus,
          int8_t PinInt = -1, uint8_t Rotation = 0,
          uint32_t WriteSpeedPercent = 200, uint32_t ReadSpeedPercent = 200,
          uint32_t InitSpeedPercent = 10, uint32_t TouchSpeedPercent = 100>
struct ra8875 {
    using type = ra8875;
    using bus = Bus;
    constexpr static const int8_t pin_rst = PinRst;
    constexpr static const int8_t pin_int = PinInt;
    constexpr static const float write_speed_multiplier =
        WriteSpeedPercent / 100.0;
    constexpr static const float read_speed_multiplier =
        ReadSpeedPercent / 100.0;
    constexpr static const float init_speed_multiplier =
        InitSpeedPercent / 100.0;
    constexpr static const float touch_speed_multiplier =
        TouchSpeedPercent / 100.0;
    constexpr static const uint8_t rotation = Rotation & 3;

   private:
    constexpr static const uint16_t native_width = Height;
    constexpr static const uint16_t native_height = Width;
    constexpr static const uint16_t voffset = (native_height == 80) * 192;
    
    static void sendx(uint8_t x, uint8_t value) {
        bus::cs_low();
        bus::begin_transaction();
        bus::begin_write();
        bus::write_raw8(x);
        bus::write_raw8(value);
        bus::end_write();
        bus::end_transaction();
        bus::cs_high();
    }
    static uint8_t recvx(uint8_t x) {
        bus::cs_low();
        bus::begin_transaction();
        bus::begin_write();
        bus::write_raw8(x);
        bus::end_write();
        bus::begin_read();
        uint8_t result = bus::read_raw8();
        bus::end_read();
        bus::end_transaction();
        bus::cs_high();
        return result;
    }
    static inline void send_command(uint8_t value) {
        constexpr static const uint8_t RA8875_CMDWRITE = 0x80;
        sendx(RA8875_CMDWRITE, value);
    }
    static inline void send_data(uint8_t value) {
        constexpr static const uint8_t RA8875_DATAWRITE = 0x00;
        sendx(RA8875_DATAWRITE, value);
    }
    static inline uint8_t recv_data() {
        constexpr static const uint8_t RA8875_DATAREAD = 0x40;
        return recvx(RA8875_DATAREAD);
    }
    static inline uint8_t recv_command() {
        constexpr static const uint8_t RA8875_CMDREAD = 0xC0;
        return recvx(RA8875_CMDREAD);
    }
    static uint8_t reg(uint8_t reg) {
        send_command(reg);
        return recv_data();
    }
    static void reg(uint8_t reg, uint8_t value) {
        send_command(reg);
        send_data(value);
    }
    // waits for an operation to complete
    static void poll(uint8_t rg, uint8_t flag) {
        while (true) {
            uint8_t result = reg(rg);
            if (!(result & flag)) return;
        }
    }
    static gfx::gfx_result initialize_pll() {
        constexpr static const uint8_t RA8875_PLLC1 = 0x88;
        constexpr static const uint8_t RA8875_PLLC1_PLLDIV1 = 0x00;
        constexpr static const uint8_t RA8875_PLLC2 = 0x89;
        constexpr static const uint8_t RA8875_PLLC2_DIV4 = 0x02;
        if ((native_width == 480 && native_height == 80) ||
            (native_width == 480 && native_height == 128) ||
            (native_width == 480 && native_height == 272)) {
            reg(RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 10);
            delay(1);
            reg(RA8875_PLLC2, RA8875_PLLC2_DIV4);
            delay(1);
            return gfx::gfx_result::success;
        } else if (native_width == 800 && native_height == 480) {
            reg(RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 11);
            delay(1);
            reg(RA8875_PLLC2, RA8875_PLLC2_DIV4);
            delay(1);
            return gfx::gfx_result::success;
        }
        return gfx::gfx_result::invalid_argument;
    }
    gfx::gfx_result initialize_display() {
        constexpr static const uint8_t RA8875_SYSR = 0x10;
        constexpr static const uint8_t RA8875_SYSR_16BPP = 0x0C;
        constexpr static const uint8_t RA8875_SYSR_MCU8 = 0x00;
        constexpr static const uint8_t RA8875_PCSR = 0x04;
        constexpr static const uint8_t RA8875_PCSR_PDATL = 0x80;
        constexpr static const uint8_t RA8875_PCSR_2CLK = 0x01;
        constexpr static const uint8_t RA8875_PCSR_4CLK = 0x02;
        constexpr static const uint8_t RA8875_HDWR = 0x14;
        constexpr static const uint8_t RA8875_HNDFTR = 0x15;
        constexpr static const uint8_t RA8875_HNDFTR_DE_HIGH = 0x00;
        constexpr static const uint8_t RA8875_HNDR = 0x16;
        constexpr static const uint8_t RA8875_HSTR = 0x17;
        constexpr static const uint8_t RA8875_HPWR = 0x18;
        constexpr static const uint8_t RA8875_HPWR_LOW = 0x00;
        constexpr static const uint8_t RA8875_VDHR0 = 0x19;
        constexpr static const uint8_t RA8875_VDHR1 = 0x1A;
        constexpr static const uint8_t RA8875_VNDR0 = 0x1B;
        constexpr static const uint8_t RA8875_VNDR1 = 0x1C;
        constexpr static const uint8_t RA8875_VSTR0 = 0x1D;
        constexpr static const uint8_t RA8875_VSTR1 = 0x1E;
        constexpr static const uint8_t RA8875_VPWR = 0x1F;
        constexpr static const uint8_t RA8875_VPWR_LOW = 0x00;
        constexpr static const uint8_t RA8875_MCLR = 0x8E;
        constexpr static const uint8_t RA8875_MCLR_START = 0x80;
        constexpr static const uint8_t RA8875_MCLR_FULL = 0x00;
        constexpr static const uint8_t RA8875_DPCR = 0x20;
        constexpr static const uint8_t RA8875_VDIR = 0x04;
        constexpr static const uint8_t RA8875_HDIR = 0x08;
        reg(RA8875_SYSR, RA8875_SYSR_16BPP | RA8875_SYSR_MCU8);

        uint8_t pixclk;
        uint8_t hsync_start;
        uint8_t hsync_pw;
        uint8_t hsync_finetune;
        uint8_t hsync_nondisp;
        uint8_t vsync_pw;
        uint16_t vsync_nondisp;
        uint16_t vsync_start;

        if (native_width == 480 && native_height == 80) {
            pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_4CLK;
            hsync_nondisp = 10;
            hsync_start = 8;
            hsync_pw = 48;
            hsync_finetune = 0;
            vsync_nondisp = 3;
            vsync_start = 8;
            vsync_pw = 10;
            // This uses the bottom 80 pixels of a 272 pixel controller
        } else if (native_width == 480 &&
                   (native_height == 128 || native_height == 272)) {
            pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_4CLK;
            hsync_nondisp = 10;
            hsync_start = 8;
            hsync_pw = 48;
            hsync_finetune = 0;
            vsync_nondisp = 3;
            vsync_start = 8;
            vsync_pw = 10;
        } else if (native_width == 800 && native_height == 480) {
            pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_2CLK;
            hsync_nondisp = 26;
            hsync_start = 32;
            hsync_pw = 96;
            hsync_finetune = 0;
            vsync_nondisp = 32;
            vsync_start = 23;
            vsync_pw = 2;
        } else {
            return gfx::gfx_result::invalid_argument;
        }
        uint8_t x = reg(RA8875_DPCR);
        switch(rotation) {
            case 0:
                x&=~(RA8875_HDIR | RA8875_VDIR);
                x|=RA8875_HDIR;
                reg(RA8875_DPCR, x);
                break;
            case 1:
                x&=~(RA8875_HDIR | RA8875_VDIR);
                reg(RA8875_DPCR, x);
                break;
            case 2:
                x&=~(RA8875_HDIR | RA8875_VDIR);
                x|=RA8875_VDIR;
                reg(RA8875_DPCR, x);
                break;
            case 3:
                // do nothing
                break;
        }

        reg(RA8875_PCSR, pixclk);
        delay(1);

        reg(RA8875_HDWR,
            (native_width / 8) - 1);  // H width: (HDWR + 1) * 8 = 480
        reg(RA8875_HNDFTR, RA8875_HNDFTR_DE_HIGH + hsync_finetune);
        reg(RA8875_HNDR, (hsync_nondisp - hsync_finetune - 2) /
                             8);  // H non-display: HNDR * 8 + HNDFTR + 2 = 10
        reg(RA8875_HSTR, hsync_start / 8 - 1);  // Hsync start: (HSTR + 1)*8
        reg(RA8875_HPWR,
            RA8875_HPWR_LOW +
                (hsync_pw / 8 - 1));  // HSync pulse width = (HPWR+1) * 8

        reg(RA8875_VDHR0, (uint16_t)(native_height - 1 + voffset) & 0xFF);
        reg(RA8875_VDHR1, (uint16_t)(native_height - 1 + voffset) >> 8);
        reg(RA8875_VNDR0,
            vsync_nondisp - 1);  // V non-display period = VNDR + 1
        reg(RA8875_VNDR1, vsync_nondisp >> 8);
        reg(RA8875_VSTR0, vsync_start - 1);  // Vsync start position = VSTR + 1
        reg(RA8875_VSTR1, vsync_start >> 8);
        reg(RA8875_VPWR,
            RA8875_VPWR_LOW + vsync_pw - 1);  // Vsync pulse width = VPWR + 1

        set_active_window({0, 0, native_width - 1, native_height - 1});
        /* ToDo: Setup touch panel? */

        /* Clear the entire window */
        reg(RA8875_MCLR, RA8875_MCLR_START | RA8875_MCLR_FULL);
        delay(500);

        return gfx::gfx_result::success;
    }
    gfx::gfx_result initialize_touch() {
        constexpr static const uint8_t RA8875_TPCR0 = 0x70;
        constexpr static const uint8_t RA8875_TPCR0_ENABLE = 0x80;
        constexpr static const uint8_t RA8875_TPCR0_WAIT_4096CLK = 0x30;
        constexpr static const uint8_t RA8875_TPCR0_ADCCLK_DIV4 = 0x02;
        constexpr static const uint8_t RA8875_TPCR0_WAKEENABLE = 0x08;
        constexpr static const uint8_t RA8875_TPCR1 = 0x71;
        constexpr static const uint8_t RA8875_TPCR1_AUTO = 0x00;
        constexpr static const uint8_t RA8875_TPCR1_DEBOUNCE = 0x04;
        constexpr static const uint8_t RA8875_INTC1 = 0xF0;
        constexpr static const uint8_t RA8875_INTC1_TP = 0x04;
        constexpr static const uint8_t RA8875_TPCR0_ADCCLK_DIV16 = 0x04;
        if (pin_int != -1) {
            pinMode(pin_int, INPUT);
            digitalWrite(pin_int, HIGH);
        }
        uint8_t adcClk = (uint8_t)RA8875_TPCR0_ADCCLK_DIV4;

        if (native_width == 800)  // match up touch size with LCD size
            adcClk = (uint8_t)RA8875_TPCR0_ADCCLK_DIV16;

        // Enable Touch Panel (Reg 0x70)
        reg(RA8875_TPCR0, RA8875_TPCR0_ENABLE | RA8875_TPCR0_WAIT_4096CLK |
                              RA8875_TPCR0_WAKEENABLE | adcClk);  // 10mhz max!
        // Set Auto Mode      (Reg 0x71)
        reg(RA8875_TPCR1, RA8875_TPCR1_AUTO | RA8875_TPCR1_DEBOUNCE);
        // Enable TP INT
        reg(RA8875_INTC1, reg(RA8875_INTC1) | RA8875_INTC1_TP);

        return gfx::gfx_result::success;
    }
    static gfx::gfx_result set_active_window(const gfx::rect16& rect) {
        constexpr static const uint8_t RA8875_HSAW0 = 0x30;
        constexpr static const uint8_t RA8875_HSAW1 = 0x31;
        constexpr static const uint8_t RA8875_VSAW0 = 0x32;
        constexpr static const uint8_t RA8875_VSAW1 = 0x33;
        constexpr static const uint8_t RA8875_HEAW0 = 0x34;
        constexpr static const uint8_t RA8875_HEAW1 = 0x35;
        constexpr static const uint8_t RA8875_VEAW0 = 0x36;
        constexpr static const uint8_t RA8875_VEAW1 = 0x37;

        gfx::rect16 r = rect.normalize();
        // r= apply_rotation(rect);
        reg(RA8875_HSAW0, r.x1);  // horizontal start point
        reg(RA8875_HSAW1, r.x1 >> 8);
        reg(RA8875_HEAW0, r.x2);  // horizontal end point
        reg(RA8875_HEAW1, r.x2 >> 8);

        /* Set active window Y */
        reg(RA8875_VSAW0,
            (uint16_t)(r.y1 + voffset) & 0xFF);  // vertical start point
        reg(RA8875_VSAW1, (uint16_t)(r.y1 + voffset) >> 8);
        reg(RA8875_VEAW0,
            (uint16_t)(r.y2 + voffset) & 0xFF);  // vertical end point
        reg(RA8875_VEAW1, (uint16_t)(r.y2 + voffset) >> 8);
        return gfx::gfx_result::success;
    }
    static bool recv_touch(gfx::point16* out_point) {
        constexpr static const uint8_t RA8875_INTC2 = 0xF1;
        constexpr static const uint8_t RA8875_INTC2_TP = 0x04;
        constexpr static const uint8_t RA8875_TPXH = 0x72;
        constexpr static const uint8_t RA8875_TPYH = 0x73;
        constexpr static const uint8_t RA8875_TPXYL = 0x74;
        uint16_t tx, ty;
        uint8_t temp;

        tx = reg(RA8875_TPXH);
        ty = reg(RA8875_TPYH);
        temp = reg(RA8875_TPXYL);
        tx <<= 2;
        ty <<= 2;
        tx |= temp & 0x03;         // get the bottom x bits
        ty |= (temp >> 2) & 0x03;  // get the bottom y bits
        *out_point = type::apply_rotation(gfx::point16(tx, ty), true);
        // Clear TP INT Status
        reg(RA8875_INTC2, RA8875_INTC2_TP);
        return true;
    }
    constexpr static uint8_t get_write_orientation() {
        constexpr const uint8_t RA8875_MWCR0_LRTD = 0x00;
        constexpr const uint8_t RA8875_MWCR0_TDLR = 0x08;
        if (!(rotation & 1)) {
            // portrait
            return RA8875_MWCR0_TDLR;
        } else {
            // landscape
            return RA8875_MWCR0_LRTD;
        }
    }
    constexpr static inline gfx::point16 apply_rotation(gfx::point16 pt,
                                                        bool translate) {
        if (!(rotation & 1)) {
            // portrait
            return {pt.y, pt.x};
        }
        return pt;
    }
    constexpr inline static gfx::rect16 apply_rotation(const gfx::rect16& r,
                                                       bool translate) {
        return gfx::rect16(apply_rotation(r.point1(), translate),
                           apply_rotation(r.point2(), translate));
    }
    constexpr static const uint8_t write_orientation = get_write_orientation();

    template <typename Source, bool Blt>
    struct draw_helper {
        static gfx::gfx_result do_draw(const Source& src,
                                       const gfx::rect16& srcr,
                                       const gfx::rect16& dstr) {
            uint16_t w = dstr.dimensions().width;
            uint16_t h = dstr.dimensions().height;
            for (uint16_t y = 0; y < h; ++y) {
                for (uint16_t x = 0; x < w; ++x) {
                    typename Source::pixel_type pp;
                    gfx::gfx_result rr =
                        src.point(gfx::point16(x + srcr.x1, y + srcr.y1), &pp);
                    if (rr != gfx::gfx_result::success) {
                        return rr;
                    }
                    pixel_type p;
                    rr = gfx::convert_palette_to(src, pp, &p);
                    if (gfx::gfx_result::success != rr) {
                        return rr;
                    }
                    uint16_t v = p.channel_unchecked<2>() |
                                 (p.channel_unchecked<1>() << 5) |
                                 (p.channel_unchecked<0>() << 11);
                    bus::write_raw16(v);
                }
            }
            return gfx::gfx_result::success;
        }
    };
    template <typename Source>
    struct draw_helper<Source, true> {
        static gfx::gfx_result do_draw(const Source& src,
                                       const gfx::rect16& srcr,
                                       const gfx::rect16& dstr) {
            uint16_t w = dstr.dimensions().width;

            if (srcr.x1 == 0 && w == src.dimensions().width &&
                w == srcr.width()) {
                const uint8_t* p = src.begin() + (w * srcr.y1);
                size_t len = w * srcr.height() * 2;
                bus::write_raw(p, len);
                return gfx::gfx_result::success;
            }
            return draw_helper<Source, false>::do_draw(src, srcr, dstr);
        }
    };

    bool m_initialized;
    bool m_in_batch;

   public:
    using pixel_type = gfx::rgb_pixel<16>;
    using caps = gfx::gfx_caps<false, false, true, true, false, false, false>;
    ra8875() : m_initialized(false), m_in_batch(false) {}
    inline bool initialized() const { return m_initialized; }
    gfx::gfx_result initialize() {
        constexpr static const uint8_t RA8875_PWRR = 0x01;
        constexpr static const uint8_t RA8875_PWRR_DISPON = 0x80;
        constexpr static const uint8_t RA8875_PWRR_NORMAL = 0x00;
        constexpr static const uint8_t RA8875_GPIOX = 0xC7;
        constexpr static const uint8_t RA8875_PWM_CLK_DIV1024 = 0x0A;
        constexpr static const uint8_t RA8875_P1CR = 0x8A;
        constexpr static const uint8_t RA8875_P1CR_ENABLE = 0x80;
        constexpr static const uint8_t RA8875_P1DCR = 0x8B;
        constexpr static const uint8_t RA8875_MRWC = 0x02;
        constexpr static const uint8_t RA8875_MWCR0 = 0x40;
        constexpr static const uint8_t RA8875_MWCR0_DIRMASK = 0x0C;
        if (!m_initialized) {
            m_initialized = true;
            pinMode(pin_rst, OUTPUT);
            digitalWrite(pin_rst, LOW);
            delay(100);
            digitalWrite(pin_rst, HIGH);
            delay(100);
            if (!bus::initialize()) {
                return gfx::gfx_result::device_error;
            }
            bus::set_speed_multiplier(init_speed_multiplier);
            bus::begin_initialization();
            uint8_t result = reg(0);
            if (result != 0x75) {
                return gfx::gfx_result::device_error;
            }
            gfx::gfx_result r = type::initialize_pll();
            if (r != gfx::gfx_result::success) {
                return r;
            }
            r = type::initialize_display();
            if (r != gfx::gfx_result::success) {
                return r;
            }
            // turn on the display
            reg(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPON);

            // GPIOX on
            reg(RA8875_GPIOX, 1);

            // enable backlight
            reg(RA8875_P1CR,
                RA8875_P1CR_ENABLE | (RA8875_PWM_CLK_DIV1024 & 0xF));
            reg(RA8875_P1DCR, 0xFF);

            // set rotation
            reg(RA8875_MWCR0, (reg(RA8875_MWCR0) & ~RA8875_MWCR0_DIRMASK) |
                                  write_orientation);
            send_command(RA8875_MRWC);
            r = initialize_touch();
            if (r != gfx::gfx_result::success) {
                return r;
            }
            bus::end_initialization();
            bus::set_speed_multiplier(write_speed_multiplier);

            m_initialized = true;
        }
        return gfx::gfx_result::success;
    }
    bool touched(gfx::point16* out_point) {
        constexpr static const uint8_t RA8875_INTC2 = 0xF1;
        constexpr static const uint8_t RA8875_INTC2_TP = 0x04;
        if (pin_int == -1 || !digitalRead(pin_int)) {
            bus::set_speed_multiplier(touch_speed_multiplier);
            bus::begin_write();
            if (reg(RA8875_INTC2) & RA8875_INTC2_TP) {
                bool res = recv_touch(out_point);
                bus::end_write();
                bus::set_speed_multiplier(write_speed_multiplier);
                return res;
            }
            bus::end_write();
            bus::set_speed_multiplier(write_speed_multiplier);
        }
        return false;
    }
    // retrieves the dimensions of the screen
    constexpr inline gfx::size16 dimensions() const {
        if (rotation & 1) {
            return {native_width, native_height};
        }
        return {native_height, native_width};
    }
    constexpr inline gfx::rect16 bounds() const {
        return dimensions().bounds();
    }
    // sets a point to the specified pixel
    gfx::gfx_result point(gfx::point16 location, pixel_type pixel) {
        constexpr static const uint8_t RA8875_CURH0 = 0x46;
        constexpr static const uint8_t RA8875_CURH1 = 0x47;
        constexpr static const uint8_t RA8875_CURV0 = 0x48;
        constexpr static const uint8_t RA8875_CURV1 = 0x49;
        constexpr static const uint8_t RA8875_MRWC = 0x02;
        constexpr static const uint8_t RA8875_DATAWRITE = 0x00;

        if (location.x >= dimensions().width ||
            location.y >= dimensions().height)
            return gfx::gfx_result::success;
        gfx::gfx_result rr = commit_batch();
        if (rr != gfx::gfx_result::success) {
            return rr;
        }
        location = apply_rotation(location, true);
        reg(RA8875_CURH0, location.x);
        reg(RA8875_CURH1, location.x >> 8);
        reg(RA8875_CURV0, location.y);
        reg(RA8875_CURV1, location.y >> 8);
        send_command(RA8875_MRWC);
        bus::cs_low();
        bus::begin_write();
        auto v = pixel.value();
        bus::write_raw8(RA8875_DATAWRITE);
        bus::write_raw8(v);
        bus::write_raw8(v >> 8);
        bus::cs_high();
        bus::end_write();
        return gfx::gfx_result::success;
    }
    // fills the specified rectangle
    gfx::gfx_result fill(const gfx::rect16& rect, pixel_type pixel) {
        constexpr static const uint8_t RA8875_DCR = 0x90;
        constexpr static const uint8_t RA8875_DCR_LINESQUTRI_STATUS = 0x80;
        // it's a point
        if (rect.x1 == rect.x2 && rect.y1 == rect.y2) {
            return point(rect.point1(), pixel);
        }
        if (!rect.intersects(bounds())) return gfx::gfx_result::success;
        gfx::gfx_result rr = commit_batch();
        if (rr != gfx::gfx_result::success) {
            return rr;
        }
        gfx::rect16 r = apply_rotation(rect, false)
                            .crop({0, 0, native_width - 1, native_height - 1});
        // it's a line
        if (r.x1 == r.x2 || r.y1 == r.y2) {
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
        // it's a filled rect:

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
        return fill(rect, pixel_type());
    }

    // begins a batch operation
    gfx::gfx_result begin_batch(const gfx::rect16& rect) {
        constexpr static const uint8_t RA8875_DATAWRITE = 0x00;
        constexpr static const uint8_t RA8875_CURH0 = 0x46;
        constexpr static const uint8_t RA8875_CURH1 = 0x47;
        constexpr static const uint8_t RA8875_CURV0 = 0x48;
        constexpr static const uint8_t RA8875_CURV1 = 0x49;
        constexpr static const uint8_t RA8875_MRWC = 0x02;
        constexpr static const uint8_t RA8875_MWCR0 = 0x40;
        constexpr static const uint8_t RA8875_MWCR0_DIRMASK = 0x0C;

        // static const uint8_t RA8875_MWCR0_TDLR = 0x08;
        gfx::gfx_result rr = commit_batch();
        if (gfx::gfx_result::success != rr) {
            return rr;
        }
        gfx::rect16 r = apply_rotation(rect.normalize(), false)
                            .crop({0, 0, native_width - 1, native_height - 1});
        set_active_window(r);
        // Serial.printf("window: (%d, %d)-(%d,
        // %d)\r\n",(int)r.x1,(int)r.y1,(int)r.x2,(int)r.y2);
        r = apply_rotation(rect, true);
        // Serial.printf("translated: (%d, %d)-(%d,
        // %d)\r\n",(int)r.x1,(int)r.y1,(int)r.x2,(int)r.y2);
        reg(RA8875_CURH0, r.x1);
        reg(RA8875_CURH1, r.x1 >> 8);
        reg(RA8875_CURV0, r.y1);
        reg(RA8875_CURV1, r.y1 >> 8);

        reg(RA8875_MWCR0,
            (reg(RA8875_MWCR0) & ~RA8875_MWCR0_DIRMASK) | write_orientation);
        send_command(RA8875_MRWC);
        bus::cs_low();
        bus::begin_write();
        bus::write_raw8(RA8875_DATAWRITE);
        m_in_batch = true;
        return gfx::gfx_result::success;
    }

    // writes a pixel to a pending batch
    gfx::gfx_result write_batch(pixel_type color) {
        if (!m_in_batch) return gfx::gfx_result::invalid_state;
        uint16_t v = color.channel_unchecked<2>() |
                     (color.channel_unchecked<1>() << 5) |
                     (color.channel_unchecked<0>() << 11);
        bus::write_raw16(v);
        return gfx::gfx_result::success;
    }
    // commits a pending batch
    gfx::gfx_result commit_batch() {
        if (!m_in_batch) {
            return initialize();
        }
        bus::cs_high();
        bus::end_write();
        set_active_window({0, 0, native_width - 1, native_height - 1});
        m_in_batch = false;
        return gfx::gfx_result::success;
    }
    template <typename Source>
    gfx::gfx_result copy_from(const gfx::rect16& src_rect, const Source& src,
                              gfx::point16 location) {
        constexpr static const uint8_t RA8875_DATAWRITE = 0x00;
        constexpr static const uint8_t RA8875_CURH0 = 0x46;
        constexpr static const uint8_t RA8875_CURH1 = 0x47;
        constexpr static const uint8_t RA8875_CURV0 = 0x48;
        constexpr static const uint8_t RA8875_CURV1 = 0x49;
        constexpr static const uint8_t RA8875_MRWC = 0x02;
        constexpr static const uint8_t RA8875_MWCR0 = 0x40;
        constexpr static const uint8_t RA8875_MWCR0_DIRMASK = 0x0C;
        gfx::gfx_result rr = commit_batch();
        if (gfx::gfx_result::success != rr) {
            return rr;
        }
        gfx::rect16 srcr = src_rect.normalize().crop(src.bounds());
        gfx::rect16 dstr(location, src_rect.dimensions());
        dstr = dstr.crop(bounds());
        if (srcr.width() > dstr.width()) {
            srcr.x2 = srcr.x1 + dstr.width() - 1;
        }
        if (srcr.height() > dstr.height()) {
            srcr.y2 = srcr.y1 + dstr.height() - 1;
        }
        set_active_window(apply_rotation(dstr, false));
        const gfx::rect16 rot = apply_rotation(dstr, true);
        bus::cs_high();

        reg(RA8875_CURH0, rot.x1);
        reg(RA8875_CURH1, rot.x1 >> 8);
        reg(RA8875_CURV0, rot.y1);
        reg(RA8875_CURV1, rot.y1 >> 8);

        reg(RA8875_MWCR0,
            (reg(RA8875_MWCR0) & ~RA8875_MWCR0_DIRMASK) | write_orientation);
        send_command(RA8875_MRWC);
        bus::cs_low();
        bus::begin_write();
        bus::write_raw8(RA8875_DATAWRITE);
        rr = draw_helper < Source,
        Source::caps::blt &&
            Source::pixel_type::template equals_exact<
                gfx::rgb_pixel<16>>::value > ::do_draw(src, srcr, dstr);
        bus::end_write();
        bus::cs_high();
        set_active_window({0, 0, native_width - 1, native_height - 1});
        return rr;
    }
};
}  // namespace arduino