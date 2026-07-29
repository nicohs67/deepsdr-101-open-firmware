#include <string.h>
#include "waterfall.h"
#include "gfx.h"

/* .bss, RAM principal (0x20000000). WATERFALL_WIDTH*WATERFALL_ROWS*2 bytes,
 * ver presupuesto documentado en waterfall.h antes de subir WATERFALL_ROWS. */
static uint16_t s_buf[WATERFALL_ROWS][WATERFALL_WIDTH];

void waterfall_init(void)
{
    memset(s_buf, 0, sizeof(s_buf));
}

void waterfall_push_line(const uint16_t *line)
{
    /* Desplaza filas 0..ROWS-2 a 1..ROWS-1 (la mas antigua, ROWS-1, se
     * pierde) y deja la fila 0 libre para la nueva. memmove porque los
     * rangos origen/destino se solapan. */
    memmove(&s_buf[1][0], &s_buf[0][0],
            (size_t)(WATERFALL_ROWS - 1) * WATERFALL_WIDTH * sizeof(uint16_t));
    memcpy(&s_buf[0][0], line, WATERFALL_WIDTH * sizeof(uint16_t));
}

void waterfall_blit(uint16_t x, uint16_t y)
{
    gfx_blit(x, y, WATERFALL_WIDTH, WATERFALL_ROWS, &s_buf[0][0]);
}

uint16_t *waterfall_row(uint16_t row)
{
    if (row >= WATERFALL_ROWS) {
        return NULL;
    }
    return &s_buf[row][0];
}
