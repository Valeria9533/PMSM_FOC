/* MODULE Serial. */

#include "stdint.h"
#include "PE_freemaster_56F8xxx.h"

extern void FMSTR_SCI_PUTCHAR(uint8_t _data);
extern uint16_t FMSTR_SCI_GETCHAR(void);
extern void FMSTR_SCI_RE(void);
extern void FMSTR_SCI_RD(void);
extern void FMSTR_SCI_TE(void);
extern void FMSTR_SCI_TD(void);
extern FMSTR_SCISR FMSTR_SCI_RDCLRSR(void);

extern uint8_t DataInFreemasterCAN[256];
extern uint8_t PointerDataInFreemasterCAN;
extern void FMSTR_CAN_Poll(void);
extern void FMSTR_USB_Poll(void);

/* END Serial. */
