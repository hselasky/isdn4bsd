/* this file is necessary to set the
 * end of sections, hence the compiler
 * will re-order the structures defined
 * in a C-file
 */
#define SECTION_END(name,start,end)\
extern unsigned name##end[0];\
unsigned __attribute__((__aligned__(1),__section__(#name #start),__used__)) name##end[0] = { };\
/**/

SECTION_END(ihfc_usb_id,_start,_end)
SECTION_END(ihfc_pci_id,_start,_end)
SECTION_END(ihfc_pnp_id,_start,_end)
SECTION_END(ihfc_isa_id,_start,_end)

/* SECTION_END(ihfc_fifo_maps,,_end) */
SECTION_END(ihfc_filter_info,_start,_end)



