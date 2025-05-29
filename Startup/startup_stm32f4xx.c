#include <stdint.h>

typedef void (*vector_handler)(void);

extern uint32_t _stext;
extern uint32_t _etext;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern void _stack_top(void);
extern int main(void);

void ResetHandler(void);

__attribute__((used, section(".vector")))
const vector_handler vector_table[] = {
    &_stack_top,
    ResetHandler,
    
};

void ResetHandler(void)
{
    uint32_t *pSrc, *pDst;
    // Copy the data segment from FLASH to SRAM
    pSrc = (uint32_t*)&_etext;
    pDst = (uint32_t*)&_sdata;
    while (pDst < &_edata) {
        *(pDst++) = *(pSrc++);
    }
    
    // Fill BSS segment zero
    pSrc = (uint32_t*)&_sbss;
    pDst = (uint32_t*)&_ebss;
    while (pSrc < pDst) {
        *(pSrc++) = 0;
    }

    // Enter to main
    main();
}