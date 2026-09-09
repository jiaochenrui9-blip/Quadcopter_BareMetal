#include "OLED.h"
#include "main.h"
#include "i2c.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define OLED_WIDTH              128
#define OLED_HEIGHT             64
#define OLED_PAGE_COUNT         8
#define OLED_I2C_ADDRESS        0x78
#define OLED_PI                 3.14159265358979323846

uint8_t OLED_DisplayBuf[OLED_PAGE_COUNT][OLED_WIDTH];

/* Main-loop DMA state machine. Drawing uses DisplayBuf, DMA uses a snapshot. */
#define OLED_TX_TIMEOUT_MS 50U
static uint8_t oled_frame[OLED_PAGE_COUNT][OLED_WIDTH];
static uint8_t oled_tx_buffer[OLED_WIDTH + 1U];
static volatile uint8_t oled_done, oled_error;
static uint8_t oled_active, oled_pending, oled_initialized, oled_stage, oled_page;
static uint32_t oled_started_ms;

static HAL_StatusTypeDef OLED_Transmit(uint8_t control, const uint8_t *data, uint8_t count)
{
    /* Avoid the HAL DMA start path's bus-busy polling when a panel holds SDA. */
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY)) return HAL_BUSY;
    oled_tx_buffer[0] = control;
    memcpy(&oled_tx_buffer[1], data, count);
    oled_done = oled_error = 0U;
    oled_started_ms = HAL_GetTick();
    oled_active = 1U;
    return HAL_I2C_Master_Transmit_DMA(&hi2c1, OLED_I2C_ADDRESS,
                                      oled_tx_buffer, (uint16_t)count + 1U);
}

void OLED_I2CTxCpltCallback(void) { oled_done = 1U; }
void OLED_I2CErrorCallback(void) { oled_error = 1U; }
uint8_t OLED_IsBusy(void) { return oled_pending || oled_active; }

void OLED_Init(void)
{
    oled_active = oled_pending = oled_initialized = 0U;
    oled_done = oled_error = 0U;
    OLED_Clear();
}

void OLED_Poll(void)
{
    static const uint8_t commands[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1,
        0xDB, 0x30, 0xA4, 0xA6, 0x8D, 0x14, 0xAF
    };
    if (!oled_pending) return;
    if (oled_active)
    {
        if (oled_error || (uint32_t)(HAL_GetTick() - oled_started_ms) >= OLED_TX_TIMEOUT_MS)
        {
            /* Quiesce DMA before allowing the snapshot or packet to be reused. */
            (void)HAL_I2C_DeInit(&hi2c1);
            (void)HAL_I2C_Init(&hi2c1);
            oled_active = oled_pending = oled_initialized = 0U;
            oled_done = oled_error = 0U;
            return;
        }
        if (!oled_done) return;
        oled_active = 0U;
        oled_done = 0U;
        if (oled_stage == 0U) { oled_initialized = 1U; oled_stage = 1U; }
        else if (oled_stage == 1U) oled_stage = 2U;
        else
        {
            if (++oled_page == OLED_PAGE_COUNT) { oled_pending = 0U; return; }
            oled_stage = 1U;
        }
    }
    HAL_StatusTypeDef result;
    if (oled_stage == 0U)
        result = OLED_Transmit(0x00U, commands, sizeof(commands));
    else if (oled_stage == 1U)
    {
        uint8_t cursor[] = {(uint8_t)(0xB0U | oled_page), 0x10U, 0x00U};
        result = OLED_Transmit(0x00U, cursor, sizeof(cursor));
    }
    else result = OLED_Transmit(0x40U, oled_frame[oled_page], OLED_WIDTH);
    if (result == HAL_BUSY && !oled_active)
    {
        /* Bus unavailable: drop this refresh, retry on the next display period. */
        oled_pending = 0U;
    }
    else if (result != HAL_OK) oled_error = 1U;
}

static uint8_t OLED_ClipArea(int16_t *X0, int16_t *Y0, int16_t *X1, int16_t *Y1,
                             int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
    int16_t x1;
    int16_t y1;

    if (Width == 0U || Height == 0U)
    {
        return 0;
    }

    x1 = (int16_t)(X + Width - 1);
    y1 = (int16_t)(Y + Height - 1);

    if (x1 < 0 || y1 < 0 || X >= OLED_WIDTH || Y >= OLED_HEIGHT)
    {
        return 0;
    }

    *X0 = X < 0 ? 0 : X;
    *Y0 = Y < 0 ? 0 : Y;
    *X1 = x1 >= OLED_WIDTH ? (OLED_WIDTH - 1) : x1;
    *Y1 = y1 >= OLED_HEIGHT ? (OLED_HEIGHT - 1) : y1;
    return 1;
}

void OLED_Update(void)
{
    if (OLED_IsBusy()) return;
    memcpy(oled_frame, OLED_DisplayBuf, sizeof(oled_frame));
    oled_page = 0U;
    oled_stage = oled_initialized ? 1U : 0U;
    oled_pending = 1U;
}

/* Area updates share the same asynchronous full-frame transport. */
void OLED_UpdateArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
    int16_t x0, y0, x1, y1;
    if (OLED_ClipArea(&x0, &y0, &x1, &y1, X, Y, Width, Height)) OLED_Update();
}

void OLED_Clear(void)
{
    memset(OLED_DisplayBuf, 0, sizeof(OLED_DisplayBuf));
}

void OLED_ClearArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
    int16_t x0, y0, x1, y1;

    if (OLED_ClipArea(&x0, &y0, &x1, &y1, X, Y, Width, Height) == 0U)
    {
        return;
    }

    for (int16_t y = y0; y <= y1; y++)
    {
        uint8_t mask = (uint8_t)(1U << (y & 0x07));
        uint8_t page = (uint8_t)(y >> 3);
        for (int16_t x = x0; x <= x1; x++)
        {
            OLED_DisplayBuf[page][x] &= (uint8_t)~mask;
        }
    }
}

void OLED_Reverse(void)
{
    for (uint8_t page = 0; page < OLED_PAGE_COUNT; page++)
    {
        for (uint8_t x = 0; x < OLED_WIDTH; x++)
        {
            OLED_DisplayBuf[page][x] = (uint8_t)~OLED_DisplayBuf[page][x];
        }
    }
}

void OLED_ReverseArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
    int16_t x0, y0, x1, y1;

    if (OLED_ClipArea(&x0, &y0, &x1, &y1, X, Y, Width, Height) == 0U)
    {
        return;
    }

    for (int16_t y = y0; y <= y1; y++)
    {
        uint8_t mask = (uint8_t)(1U << (y & 0x07));
        uint8_t page = (uint8_t)(y >> 3);
        for (int16_t x = x0; x <= x1; x++)
        {
            OLED_DisplayBuf[page][x] ^= mask;
        }
    }
}

void OLED_DrawPoint(int16_t X, int16_t Y)
{
    if (X >= 0 && X < OLED_WIDTH && Y >= 0 && Y < OLED_HEIGHT)
    {
        OLED_DisplayBuf[Y >> 3][X] |= (uint8_t)(1U << (Y & 0x07));
    }
}

uint8_t OLED_GetPoint(int16_t X, int16_t Y)
{
    if (X >= 0 && X < OLED_WIDTH && Y >= 0 && Y < OLED_HEIGHT)
    {
        return (uint8_t)((OLED_DisplayBuf[Y >> 3][X] & (uint8_t)(1U << (Y & 0x07))) != 0U);
    }
    return 0;
}

void OLED_ShowImage(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, const uint8_t *Image)
{
    for (uint8_t y = 0; y < Height; y++)
    {
        for (uint8_t x = 0; x < Width; x++)
        {
            uint16_t index = (uint16_t)(y / 8U) * Width + x;
            if ((Image[index] & (uint8_t)(1U << (y & 0x07))) != 0U)
            {
                OLED_DrawPoint((int16_t)(X + x), (int16_t)(Y + y));
            }
        }
    }
}

void OLED_ShowChar(int16_t X, int16_t Y, char Char, uint8_t FontSize)
{
    if (Char < ' ' || Char > '~')
    {
        Char = '?';
    }

    if (FontSize == OLED_8X16)
    {
        OLED_ShowImage(X, Y, 8, 16, OLED_F8x16[Char - ' ']);
    }
    else if (FontSize == OLED_6X8)
    {
        OLED_ShowImage(X, Y, 6, 8, OLED_F6x8[Char - ' ']);
    }
}

static uint8_t OLED_UTF8CharLength(uint8_t first)
{
    if ((first & 0x80U) == 0U) { return 1; }
    if ((first & 0xE0U) == 0xC0U) { return 2; }
    if ((first & 0xF0U) == 0xE0U) { return 3; }
    if ((first & 0xF8U) == 0xF0U) { return 4; }
    return 1;
}

void OLED_ShowString(int16_t X, int16_t Y, char *String, uint8_t FontSize)
{
    uint16_t i = 0;
    int16_t xOffset = 0;

    while (String[i] != '\0')
    {
        uint8_t first = (uint8_t)String[i];
        if (first < 0x80U)
        {
            OLED_ShowChar((int16_t)(X + xOffset), Y, String[i], FontSize);
            xOffset = (int16_t)(xOffset + FontSize);
            i++;
        }
        else
        {
            char singleChar[5] = {0};
            uint8_t charLength = OLED_UTF8CharLength(first);
            if ( charLength > 4U )
            {
                charLength = 4U;
            }
            memcpy(singleChar, &String[i], charLength);

            uint16_t index = 0;
            while (OLED_CF16x16[index].Index[0] != '\0')
            {
                if (strcmp(OLED_CF16x16[index].Index, singleChar) == 0)
                {
                    OLED_ShowImage((int16_t)(X + xOffset), Y, 16, 16, OLED_CF16x16[index].Data);
                    break;
                }
                index++;
            }

            xOffset = (int16_t)(xOffset + 16);
            i = (uint16_t)(i + charLength);
        }
    }
}

static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t result = 1;
    while (Y-- != 0U)
    {
        result *= X;
    }
    return result;
}

void OLED_ShowNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
    for (uint8_t i = 0; i < Length; i++)
    {
        uint32_t div = OLED_Pow(10, (uint32_t)(Length - i - 1U));
        OLED_ShowChar((int16_t)(X + i * FontSize), Y, (char)(Number / div % 10U + '0'), FontSize);
    }
}

void OLED_ShowSignedNum(int16_t X, int16_t Y, int32_t Number, uint8_t Length, uint8_t FontSize)
{
    uint32_t absNumber;

    if (Number >= 0)
    {
        OLED_ShowChar(X, Y, '+', FontSize);
        absNumber = (uint32_t)Number;
    }
    else
    {
        OLED_ShowChar(X, Y, '-', FontSize);
        absNumber = (uint32_t)(-Number);
    }

    OLED_ShowNum((int16_t)(X + FontSize), Y, absNumber, Length, FontSize);
}

void OLED_ShowHexNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
    const char hex[] = "0123456789ABCDEF";

    for (uint8_t i = 0; i < Length; i++)
    {
        uint8_t shift = (uint8_t)((Length - i - 1U) * 4U);
        OLED_ShowChar((int16_t)(X + i * FontSize), Y, hex[(Number >> shift) & 0x0FU], FontSize);
    }
}

void OLED_ShowBinNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
    for (uint8_t i = 0; i < Length; i++)
    {
        uint8_t shift = (uint8_t)(Length - i - 1U);
        OLED_ShowChar((int16_t)(X + i * FontSize), Y, (char)(((Number >> shift) & 0x01U) + '0'), FontSize);
    }
}

void OLED_ShowFloatNum(int16_t X, int16_t Y, float Number, uint8_t IntLength, uint8_t FraLength, uint8_t FontSize)
{
    uint32_t intPart;
    float fracPart;
    int16_t xOffset = 0;

    if (Number < 0)
    {
        OLED_ShowChar(X, Y, '-', FontSize);
        Number = -Number;
    }
    else
    {
        OLED_ShowChar(X, Y, '+', FontSize);
    }

    xOffset = (int16_t)(xOffset + FontSize);
    intPart = (uint32_t)Number;
    fracPart = Number - (float)intPart;

    OLED_ShowNum((int16_t)(X + xOffset), Y, intPart, IntLength, FontSize);
    xOffset = (int16_t)(xOffset + IntLength * FontSize);
    OLED_ShowChar((int16_t)(X + xOffset), Y, '.', FontSize);
    xOffset = (int16_t)(xOffset + FontSize);

    for (uint8_t i = 0; i < FraLength; i++)
    {
        fracPart *= 10.0f;
        OLED_ShowChar((int16_t)(X + xOffset + i * FontSize), Y, (char)((uint8_t)fracPart % 10U + '0'), FontSize);
    }
}


void OLED_DrawLine(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1)
{
    int16_t dx = (int16_t)abs(X1 - X0);
    int16_t sx = X0 < X1 ? 1 : -1;
    int16_t dy = (int16_t)-abs(Y1 - Y0);
    int16_t sy = Y0 < Y1 ? 1 : -1;
    int16_t err = (int16_t)(dx + dy);

    while (1)
    {
        OLED_DrawPoint(X0, Y0);
        if (X0 == X1 && Y0 == Y1)
        {
            break;
        }

        int16_t e2 = (int16_t)(2 * err);
        if (e2 >= dy)
        {
            err = (int16_t)(err + dy);
            X0 = (int16_t)(X0 + sx);
        }
        if (e2 <= dx)
        {
            err = (int16_t)(err + dx);
            Y0 = (int16_t)(Y0 + sy);
        }
    }
}

void OLED_DrawRectangle(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, uint8_t IsFilled)
{
    if (Width == 0U || Height == 0U)
    {
        return;
    }

    if (IsFilled)
    {
        for (uint8_t y = 0; y < Height; y++)
        {
            OLED_DrawLine(X, (int16_t)(Y + y), (int16_t)(X + Width - 1), (int16_t)(Y + y));
        }
    }
    else
    {
        OLED_DrawLine(X, Y, (int16_t)(X + Width - 1), Y);
        OLED_DrawLine(X, (int16_t)(Y + Height - 1), (int16_t)(X + Width - 1), (int16_t)(Y + Height - 1));
        OLED_DrawLine(X, Y, X, (int16_t)(Y + Height - 1));
        OLED_DrawLine((int16_t)(X + Width - 1), Y, (int16_t)(X + Width - 1), (int16_t)(Y + Height - 1));
    }
}

static int32_t OLED_Edge(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1, int16_t X, int16_t Y)
{
    return (int32_t)(X - X0) * (Y1 - Y0) - (int32_t)(Y - Y0) * (X1 - X0);
}

void OLED_DrawTriangle(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint8_t IsFilled)
{
    if (!IsFilled)
    {
        OLED_DrawLine(X0, Y0, X1, Y1);
        OLED_DrawLine(X1, Y1, X2, Y2);
        OLED_DrawLine(X2, Y2, X0, Y0);
        return;
    }

    int16_t minX = X0 < X1 ? X0 : X1;
    int16_t maxX = X0 > X1 ? X0 : X1;
    int16_t minY = Y0 < Y1 ? Y0 : Y1;
    int16_t maxY = Y0 > Y1 ? Y0 : Y1;

    if (X2 < minX) { minX = X2; }
    if (X2 > maxX) { maxX = X2; }
    if (Y2 < minY) { minY = Y2; }
    if (Y2 > maxY) { maxY = Y2; }

    for (int16_t y = minY; y <= maxY; y++)
    {
        for (int16_t x = minX; x <= maxX; x++)
        {
            int32_t e0 = OLED_Edge(X0, Y0, X1, Y1, x, y);
            int32_t e1 = OLED_Edge(X1, Y1, X2, Y2, x, y);
            int32_t e2 = OLED_Edge(X2, Y2, X0, Y0, x, y);
            if ((e0 >= 0 && e1 >= 0 && e2 >= 0) || (e0 <= 0 && e1 <= 0 && e2 <= 0))
            {
                OLED_DrawPoint(x, y);
            }
        }
    }
}

static void OLED_DrawCirclePoints(int16_t X, int16_t Y, int16_t x, int16_t y)
{
    OLED_DrawPoint((int16_t)(X + x), (int16_t)(Y + y));
    OLED_DrawPoint((int16_t)(X - x), (int16_t)(Y + y));
    OLED_DrawPoint((int16_t)(X + x), (int16_t)(Y - y));
    OLED_DrawPoint((int16_t)(X - x), (int16_t)(Y - y));
    OLED_DrawPoint((int16_t)(X + y), (int16_t)(Y + x));
    OLED_DrawPoint((int16_t)(X - y), (int16_t)(Y + x));
    OLED_DrawPoint((int16_t)(X + y), (int16_t)(Y - x));
    OLED_DrawPoint((int16_t)(X - y), (int16_t)(Y - x));
}

void OLED_DrawCircle(int16_t X, int16_t Y, uint8_t Radius, uint8_t IsFilled)
{
    int16_t x = 0;
    int16_t y = Radius;
    int16_t d = (int16_t)(3 - 2 * Radius);

    while (x <= y)
    {
        if (IsFilled)
        {
            OLED_DrawLine((int16_t)(X - x), (int16_t)(Y + y), (int16_t)(X + x), (int16_t)(Y + y));
            OLED_DrawLine((int16_t)(X - x), (int16_t)(Y - y), (int16_t)(X + x), (int16_t)(Y - y));
            OLED_DrawLine((int16_t)(X - y), (int16_t)(Y + x), (int16_t)(X + y), (int16_t)(Y + x));
            OLED_DrawLine((int16_t)(X - y), (int16_t)(Y - x), (int16_t)(X + y), (int16_t)(Y - x));
        }
        else
        {
            OLED_DrawCirclePoints(X, Y, x, y);
        }

        if (d < 0)
        {
            d = (int16_t)(d + 4 * x + 6);
        }
        else
        {
            d = (int16_t)(d + 4 * (x - y) + 10);
            y--;
        }
        x++;
    }
}

void OLED_DrawEllipse(int16_t X, int16_t Y, uint8_t A, uint8_t B, uint8_t IsFilled)
{
    if (A == 0U || B == 0U)
    {
        return;
    }

    if (IsFilled)
    {
        for (int16_t y = -(int16_t)B; y <= (int16_t)B; y++)
        {
            double ratio = (double)y / (double)B;
            int16_t xLimit = (int16_t)((double)A * sqrt(1.0 - ratio * ratio));
            OLED_DrawLine((int16_t)(X - xLimit), (int16_t)(Y + y), (int16_t)(X + xLimit), (int16_t)(Y + y));
        }
    }
    else
    {
        for (uint16_t angle = 0; angle < 360; angle++)
        {
            double rad = (double)angle * OLED_PI / 180.0;
            int16_t x = (int16_t)((double)A * cos(rad));
            int16_t y = (int16_t)((double)B * sin(rad));
            OLED_DrawPoint((int16_t)(X + x), (int16_t)(Y + y));
        }
    }
}

static int16_t OLED_NormalizeAngle(int16_t angle)
{
    while (angle < 0)
    {
        angle = (int16_t)(angle + 360);
    }
    while (angle >= 360)
    {
        angle = (int16_t)(angle - 360);
    }
    return angle;
}

void OLED_DrawArc(int16_t X, int16_t Y, uint8_t Radius, int16_t StartAngle, int16_t EndAngle, uint8_t IsFilled)
{
    StartAngle = OLED_NormalizeAngle(StartAngle);
    EndAngle = OLED_NormalizeAngle(EndAngle);

    int16_t span = (int16_t)(EndAngle - StartAngle);
    if (span < 0)
    {
        span = (int16_t)(span + 360);
    }

    for (int16_t i = 0; i <= span; i++)
    {
        double rad = (double)OLED_NormalizeAngle((int16_t)(StartAngle + i)) * OLED_PI / 180.0;
        int16_t x = (int16_t)((double)Radius * cos(rad));
        int16_t y = (int16_t)((double)Radius * sin(rad));
        if (IsFilled)
        {
            OLED_DrawLine(X, Y, (int16_t)(X + x), (int16_t)(Y - y));
        }
        else
        {
            OLED_DrawPoint((int16_t)(X + x), (int16_t)(Y - y));
        }
    }
}
