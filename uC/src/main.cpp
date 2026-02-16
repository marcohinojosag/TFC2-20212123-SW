#include <Arduino.h>
#include <SPI.h>

#include "TMAG5170.h"
#include "VNH7070.h"

// ===================== Pines =====================

static constexpr uint8_t TMAG_SCK  = 18U;
static constexpr uint8_t TMAG_MISO = 19U;
static constexpr uint8_t TMAG_MOSI = 23U;
static constexpr uint8_t TMAG_CS   = 22U;

static constexpr uint8_t VNH_INA   = 21U;
static constexpr uint8_t VNH_INB   = 4U;
static constexpr uint8_t VNH_PWM   = 17U;
static constexpr uint8_t VNH_SEL0  = 16U;
static constexpr uint8_t VNH_CSADC = 34U;

// ===================== Objetos =====================

TMAG5170SPI tmag;
VNH7070 vnh;

static float g_duty = 0.0f;

// ===================== Serial Commands =====================

static void handleCmd(const String &cmd)
{
    if (cmd.length() == 0U) return;

    if (cmd == "S")
    {
        g_duty = 0.0f;
        vnh.setHiZ();
        Serial.println("OK: STOP");
        return;
    }

    if (cmd.startsWith("M"))
    {
        int32_t pct = (int32_t)cmd.substring(1).toInt();
        pct = constrain(pct, 0L, 100L);

        g_duty = (float)pct / 100.0f;
        vnh.setForwardDuty(g_duty);

        Serial.printf("OK: duty=%.2f\n", g_duty);
        return;
    }

    if (cmd == "R")
    {
        float ang = tmag.readAngleDeg();
        float iA  = vnh.readCurrent_A();
        float vcs = vnh.readCsVoltage_V();

        Serial.printf(
            "angle=%.2f deg | Vcs=%.3fV | I=%.2fA | duty=%.2f\n",
            ang, vcs, iA, g_duty
        );
        return;
    }

    Serial.println("Comandos: Mxx (0-100), S, R");
}

// ===================== SETUP =====================

void setup()
{
    Serial.begin(115200UL);
    delay(200U);

    // ---------- SPI ----------
    SPI.begin(TMAG_SCK, TMAG_MISO, TMAG_MOSI, (uint8_t)-1);

    TMAG5170SPI::Pins tmagPins{ TMAG_CS };
    tmag.begin(SPI, tmagPins, 1000000UL, SPI_MODE0);
    tmag.configAngleXY_A1();

    // ---------- Driver ----------
    VNH7070::Pins vnhPins{
        VNH_INA,
        VNH_INB,
        VNH_PWM,
        VNH_SEL0,
        VNH_CSADC
    };

    vnh.begin(vnhPins, 0, 20000, 10);
    vnh.setRsenseOhms(1000.0f);
    vnh.setKtyp(1540.0f);
    vnh.setHiZ();

    Serial.println("Listo. Comandos: Mxx (0-100), S, R");
}

// ===================== LOOP =====================

void loop()
{
    vnh.setForwardDuty(g_duty);

    static String buf;

    while (Serial.available() > 0U)
    {
        char c = (char)Serial.read();

        if ((c == '\n') || (c == '\r'))
        {
            buf.trim();
            handleCmd(buf);
            buf = "";
        }
        else
        {
            buf += c;
        }
    }

    delay(10U);
}
