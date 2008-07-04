/*
 * $FreeBSD: $
 *
 * Direct Register Access Guide (AM79C3XX)
 * =======================================
 *
 * 0x00 : Command  Register  (CR)   W 
 * 0x00 : Interrupt  Register  (IR)   R 
 * 0x01 : Data  Register  (DR)   W 
 * 0x01 : Data  Register  (DR)   R 
 * 0x02 : D-channel Status Register 1 (DSR1)    R 
 * 0x03 : D-channel Error Register (DER) (2-byte FIFO)    R 
 * 0x04 : D-channel Transmit buffer (DCTB) (8- or 16-byte FIFO)    W 
 * 0x04 : D-channel Receive buffer (DCRB) (8- or 32-byte FIFO)    R 
 * 0x05 : Bb-channel Transmit buffer (BBTB)    W 
 * 0x05 : Bb-channel Receive buffer (BBRB)    R 
 * 0x06 : Bc-channel Transmit buffer (BCTB)    W 
 * 0x06 : Bc-channel Receive buffer (BCRB)    R 
 * 0x07 : D-channel Status Register 2 (DSR2)    R 
 *
 * NOTE: To have the IOM-2 D-channel data sent to the ISDN line,
 * the TIC bus must be enabled!
 *
 * NOTE: AM79C32A does not have MAP (builtin phone support).
 *
 * Indirect Register Address Guide using DR and CR:
 * ================================================
 *
 * DLC Address Status Register                    ASR             R       91H One byte transferred
 * DLC D-channel Mode Register 1                  DMR1            R/W     86H One byte transferred
 * DLC D-channel Mode Register 2                  DMR2            R/W     87H One byte transferred
 * DLC D-channel Mode Register 3                  DMR3            R/W     8EH One byte transferred
 * DLC D-channel Mode Register 4                  DMR4            R/W     8FH One byte transferred
 * DLC D-channel Receive Byte Count Register      DRCR            R       89H LSB, MSB
 * DLC D-channel Receive Byte Limit Register      DRLR            R/W     84H LSB, MSB
 * DLC D-channel Transmit Byte Count Register     DTCR            R/W     85H LSB, MSB
 * DLC Extended FIFO Control Register             EFCR            R/W     92H One byte transferred
 * DLC First Received Byte Address Reg. 1,2,3     FRAR1, 2, 3     R/W     81H FRAR1, 2
 * DLC First Received Byte Address Register 4     FRAR4           R/W     8CH One byte transferred
 * DLC Random Number Generator Register           RNGR1 (LSB)     R/W     8AH One byte transferred
 * DLC Random Number Generator Register           RNGR2 (MSB)     R/W     8BH One byte transferred
 * DLC Second Received Byte Address Reg. 1,2,3    SRAR1, 2, 3     R/W     82H SRAR1, 2
 * DLC Second Received Byte Address Register 4    SRAR4           R/W     8DH One byte transferred
 * DLC Transmit Address Register                  TAR             R/W     83H LSB, MSB
 * INIT Initialization Register                   INIT            R/W     21H One byte transferred
 * INIT Initialization Register 2                 INIT2           R/W     20H One byte transferred (XXX special)
 * LIU LIU Mode Register 1                        LMR1            R/W     A3H One byte transferred
 * LIU LIU Mode Register 2                        LMR2            R/W     A4H One byte transferred
 * LIU LIU Priority Register                      LPR             R/W     A2H One byte transferred
 * LIU LIU Status Register                        LSR             R       A1H One byte transferred
 * LIU Multiframe Q-bit buffer                    MFQB            W       A8H One byte transferred
 * LIU Multiframe Register                        MF              R/W     A6H One byte transferred
 * LIU Multiframe S-bit/Status Register           MFSB            R       A7H One byte transferred
 * MAP Amplitude Tone Generator Register 1, 2     ATGR1, 2        R/W     68H ATGR1, 2
 * MAP Frequency Tone Generator Register 1, 2     FTGR1, 2        R/W     67H FTGR1, 2
 * MAP GER Gain Coefficient Register              GER Coeff.      R/W     65H LSB, MSB
 * MAP GR Gain Coefficient Register               GR Coeff.       R/W     64H LSB, MSB
 * MAP GX Gain Coefficient Register               GX Coeff.       R/W     63H LSB, MSB
 * MAP MAP Mode Register 1                        MMR1            R/W     69H One byte transferred
 * MAP MAP Mode Register 2                        MMR2            R/W     6AH One byte transferred
 * MAP MAP Mode Register 3                        MMR3            R/W     6CH One byte transferred
 * MAP R filter Coefficient Register              R Coeff.        R/W     62H h0 LSB, h0 MSB...h7 MSB
 * MAP Receive Peak Register                      PEAKR           R       71H One byte transferred
 * MAP Secondary Tone Ringer Amplitude            STRA            R/W     6DH One byte transferred
 * MAP Secondary Tone Ringer Frequency            STRF            R/W     6EH One byte transferred
 * MAP Sidetone Gain Coefficient Register         STG Coeff.      R/W     66H LSB, MSB
 * MAP Transmit Peak Register                     PEAKX           R       70H One byte transferred
 * MAP X filter Coefficient Register              X Coeff.        R/W     61H h0 LSB, h0 MSB...h7 MSB
 * MUX MUX Control Register 1                     MCR1            R/W     41H One byte transferred
 * MUX MUX Control Register 2                     MCR2            R/W     42H One byte transferred
 * MUX MUX Control Register 3                     MCR3            R/W     43H One byte transferred
 * MUX MUX Control Register 4                     MCR4            R/W     44H One byte transferred
 * PP C/I Receive Data Register 0                 CIRDR0          R       C4H One byte transferred
 * PP C/I Receive Data Register 1                 CIRDR1          R       C5H One byte transferred
 * PP C/I Transmit Data Register 0                CITDR0          W       C4H One byte transferred
 * PP C/I Transmit Data Register 1                CITDR1          W       C5H One byte transferred
 * PP Monitor Receive Data Register               MRDR            R       C3H One byte transferred
 * PP Monitor Transmit Data Register              MTDR            W       C3H One byte transferred
 * PP Peripheral Port Control Register 1          PPCR1           R/W     C0H One byte transferred
 * PP Peripheral Port Control Register 2          PPCR2           R/W     C8H One byte transferred
 * PP Peripheral Port Control Register 3          PPCR3           R/W     C9H One byte transferred
 * PP Peripheral Port Interrupt Enable Register   PPIER           R/W     C2H One byte transferred
 * PP Peripheral Port Status Register             PPSR            R       C1H One byte transferred
 */
