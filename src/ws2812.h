#ifndef __WS2812_H
#define __WS2812_H

#define WS2812_NUM_CHANS 4

#define COLOR(r,g,b,w) (((uint32_t)(w) << 24) | ((uint32_t)(r) << 16) | \
                        ((uint32_t)(g) << 8) | (b))
#define White(c) ((uint8_t)((c >> 24) & 0xFF))
#define Red(c) ((uint8_t)((c >> 16) & 0xFF))
#define Green(c) ((uint8_t)((c >> 8) & 0xFF))
#define Blue(c) ((uint8_t)(c & 0xFF))

void ws2812_set_rgbw(uint8_t * p, uint8_t r, uint8_t g, uint8_t b, uint8_t w);
void ws2812_set_u32(uint8_t * p, uint32_t c);
void ws2812_init(void);
void ws2812_send(uint8_t * p , uint16_t len);


#endif /* __WS2812_H */
