#ifndef SNESTISTICS_H
#define SNESTISTICS_H

#include <stdint.h>

struct SRegisters;

enum class SnestisticsEvent {
	EVENT_NMI, EVENT_RESET, EVENT_IRQ
};

struct SnesisticsRegs {
	uint32_t PC;
	uint32_t WRAM;	
	uint16_t P, A, X, Y, S, D;
	uint8_t	DB;	
};

SnesisticsRegs snestistics_capture_regs();

void snestistics_event(const SnestisticsEvent event, const SnesisticsRegs &reg_before, const SnesisticsRegs &reg_after);
void snestistics_op(const SnesisticsRegs &reg_before, const SnesisticsRegs &reg_after);
void snestistics_read_byte(const uint32_t adress, const uint8_t value);
void snestistics_read_word(const uint32_t adress, const uint16_t value);
void snestistics_pause_read_byte(const bool pause);
void snestistics_save();

#endif // SNESTISTICS_H
