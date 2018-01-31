#include "snestistics.h"
#include <stdint.h>
#include <unordered_map>
#include <cassert>
#include "rpc.h"

#define NUM_TESTS 4

#pragma comment(lib, "Rpcrt4")

// Note this link goes outside the snes9x source code base and reach into snestistics
#include "../../../source/trace_format.h"
#include "../../../source/state_compression.h"

// Access to internal snes9x structures
#include "port.h"
#include "65c816.h"
#include "dma.h"
#include "snes9x.h"
#include "memmap.h"

/*
	NOTE: This class just assumes same endian on snestistics and the emulator.
*/
class SnesisticsState {
public:

	SnesisticsState() {
	}

	void reopen() {
		/// This cancels ongoing log and starts a new one (with same name). Clears all state.
		if (trace_log) {
			save();
		}

		char name[2048];
		sprintf(name, "trace%d.trace", _reopens);

		trace_log = fopen(name, "wb");
		if (_save_trace_helper) {
			for (int i=0; i<NUM_TESTS; i++) {
				sprintf(name, "trace%d.trace_helper_%d", _reopens, i);
				trace_log_helper_new[i] = fopen(name, "wb");
				state_compression[i] = state_compression_create(sizeof(snestistics::TraceRegisters));
			}
		}

		snestistics::TraceHeader header;
		header.version = TRACE_VERSION_NUMBER;
		header.rom_mode = snestistics::TraceHeader::ROMMODE_UNKNOWN;
		if (Memory.LoROM) header.rom_mode = snestistics::TraceHeader::ROMMODE_LOROM;
		else if (Memory.HiROM) header.rom_mode = snestistics::TraceHeader::ROMMODE_HIROM;
		header.rom_size = Memory.CalculatedSize;
		header.rom_checksum = Memory.CalculatedChecksum;

		UUID id;
		RPC_STATUS status = UuidCreate(&id);
		assert(status == RPC_S_OK || status == RPC_S_UUID_LOCAL_ONLY);
		memcpy(header.content_guid, id.Data4, 8);
		fwrite(&header, sizeof(header), 1, trace_log);

		_op_counter = 0;
		_op_counter_last_written = 0;
		_reopens++;
	}

	static SnesisticsState* instance() {
		static SnesisticsState* state = new SnesisticsState;
		return state;
	}

	bool _save_trace_helper = false;
	void set_save_trace_helper(bool state) { _save_trace_helper = state; }
	bool get_save_trace_helper() const { return _save_trace_helper; }
	bool get_active() const { return trace_log != nullptr; }

	void write(snestistics::TraceEventType se) {
		snestistics::TraceEvent te;
		te.type = se;
		te.op_counter_delta = _op_counter - _op_counter_last_written;
		fwrite(&te, sizeof(te), 1, trace_log);
		_op_counter_last_written = _op_counter;
	}

	void save() {
		if (trace_log == nullptr)
			return;
		write(snestistics::TraceEventType::EVENT_FINISHED);
		fclose(trace_log);
		trace_log = nullptr;
		if (trace_log_helper_new[0]) {
			for (int i=0; i<NUM_TESTS; i++) {
				state_compression_destroy(state_compression[i], trace_log_helper_new[i]);
				fclose(trace_log_helper_new[i]);
				trace_log_helper_new[i] = nullptr;

				if (i==0)
					continue;

				char name[2048];
				sprintf(name, "trace%d.trace_helper_%d", _reopens - 1, 0);
				FILE *compare_file = fopen(name, "rb");

				// Reopen result and compare with first one
				sprintf(name, "trace%d.trace_helper_%d", _reopens-1, i);
				FILE *blah = fopen(name, "rb");

				state_compress_validate(blah, compare_file, i);

				fclose(compare_file);
				fclose(blah);
			}
		}
	}
	
	inline bool is_memory_mapped(uint8_t bank, uint16_t adr) {
		if (bank == 0x00 && adr >= 0x2000 && adr < 0x8000) {
			return true;
		}
		return false;
	}

	static void parse_reg(snestistics::TraceRegisters &out, const SnesisticsRegs &in) {
		out.pc_bank = in.PC >> 16;
		out.pc_address = in.PC & 0xFFFF;
		out.P = in.P;
		out.A = in.A;
		out.DP = in.D;
		out.S = in.S;
		out.X = in.X;
		out.Y = in.Y;
		out.DB = in.DB;
		out.wram_address = in.WRAM & 0xFFFF;
		out.wram_bank = in.WRAM >> 16;
	}

	void op(const SnesisticsRegs &reg_before, const SnesisticsRegs &reg_after) {
		if (trace_log_helper_new[0]) {
			snestistics::TraceRegisters t0, t1;
			parse_reg(t0, reg_before);
			parse_reg(t1, reg_after);
			for (int i=0; i<NUM_TESTS; i++) {
				state_compression_add(state_compression[i], (uint8_t*)&t0, trace_log_helper_new[i], i);
				state_compression_add(state_compression[i], (uint8_t*)&t1, trace_log_helper_new[i], i);
			}
		}
		_op_counter++;
	}

	void event(SnestisticsEvent e, const SnesisticsRegs &reg_before, const SnesisticsRegs &reg_after) {
		snestistics::TraceEventType se;
		if (e == SnestisticsEvent::EVENT_IRQ) se = snestistics::TraceEventType::EVENT_IRQ;
		if (e == SnestisticsEvent::EVENT_NMI) se = snestistics::TraceEventType::EVENT_NMI;
		if (e == SnestisticsEvent::EVENT_RESET) {
			se = snestistics::TraceEventType::EVENT_RESET;
			reopen(); // we only support the last reset so close current log and start a new one
		}

		write(se);

		if (e == SnestisticsEvent::EVENT_RESET) {
			// Then emit event
			snestistics::TraceEventReset r;
			parse_reg(r.regs_after, reg_after);
			fwrite(&r, sizeof(r), 1, trace_log);

			// Now emit RAM, 128kb of it
			int		block7E = (0x7E0000 & 0xffffff) >> MEMMAP_SHIFT;
			int		block7F = (0x7F0000 & 0xffffff) >> MEMMAP_SHIFT;

			uint8	*GetAddress7E = Memory.Map[block7E];
			assert(GetAddress7E > (uint8 *) CMemory::MAP_LAST);
			uint8	*GetAddress7F = Memory.Map[block7F];
			assert(GetAddress7F > (uint8 *) CMemory::MAP_LAST);

			fwrite(GetAddress7E, 64*1024, 1, trace_log);
			fwrite(GetAddress7F, 64*1024, 1, trace_log);
		}

		if (trace_log_helper_new[0]) {
			snestistics::TraceRegisters t0, t1;
			parse_reg(t0, reg_before);
			parse_reg(t1, reg_after);
			for (int i = 0; i<NUM_TESTS; i++) {
				state_compression_add(state_compression[i], (uint8_t*)&t0, trace_log_helper_new[i], i);
				state_compression_add(state_compression[i], (uint8_t*)&t1, trace_log_helper_new[i], i);
			}
		}
		_op_counter++;
	}

	bool cull(const uint32_t adress) {
		uint8_t bank = adress >> 16;
		uint16_t a16 = adress & 0xFFFF;
		if (a16 >= 0x8000)
			return true; // ROM

		// Do remapping of stuff below 0x8000
		if ((bank >= 0x00 && bank <= 0x3f) || (bank >= 0x80 && bank <= 0xBF)) {
			bank = (a16<0x2000) ? 0x7E : 0;
		}

		if (bank >= 0x70 && bank <= 0x7D) {
			// SRAM
		} else if (bank >= 0xF0 && bank <= 0xFF) {
			// SRAM
		} else if (bank == 0 && a16 >= 0x2000) {
			// MMIO
		} else {
			// Something else, maybe 7e or 7f
			return true;
		}
		return false;
	}

	void read_byte(const uint32_t adress, const uint8_t value) {
		if (_read_byte_paused)
			return;
		if (cull(adress))
			return;
		write(snestistics::TraceEventType::EVENT_READ_BYTE);
		snestistics::TraceEventReadByte rb;
		rb.adress = adress;
		rb.value = value;
		fwrite(&rb, sizeof(rb), 1, trace_log);
	}

	void read_word(const uint32_t adress, const uint16_t value) {
		if (cull(adress))
			return;
		write(snestistics::TraceEventType::EVENT_READ_WORD);
		snestistics::TraceEventReadWord rb;
		rb.adress = adress;
		rb.value = value;
		fwrite(&rb, sizeof(rb), 1, trace_log);
	}

	bool _read_byte_paused = false;
	void pause_read_byte(bool paused) {
		assert(!paused || !_read_byte_paused);
		_read_byte_paused = paused;
	}

	FILE *trace_log = nullptr;
	FILE *trace_log_helper_new[NUM_TESTS] = {nullptr,nullptr};

	StateCompression *state_compression[NUM_TESTS] = { nullptr,nullptr};
	uint32_t _reopens = 0;
	uint64_t _op_counter = 0, _op_counter_last_written = 0;
};

void snestistics_save() {
	return SnesisticsState::instance()->save();
}

SnesisticsRegs snestistics_capture_regs() {
	SnesisticsRegs regs;
	const SRegisters &sr = Registers;

	// NOTE: This is equivalent to unpack but I don't want to mess with global Registers
	uint16_t P = sr.P.W;
	P &= ~(Overflow|Carry|Negative|Zero);
	if (CheckOverflow()) P |= Overflow;
	if (CheckCarry())    P |= Carry;
	if (CheckNegative()) P |= Negative;
	if (CheckZero())     P |= Zero;

	regs.A = sr.A.W;
	regs.X = sr.X.W;
	regs.Y = sr.Y.W;
	regs.S = sr.S.W;
	regs.D = sr.D.W;
	regs.DB = sr.DB;
	regs.D = sr.D.W;
	regs.P = P;
	regs.PC = sr.PC.xPBPC;
	regs.WRAM = PPU.WRAM;
	return regs;
}

void snestistics_event(const SnestisticsEvent event, const SnesisticsRegs &reg_before, const SnesisticsRegs &reg_after) {
	SnesisticsState::instance()->event(event, reg_before, reg_after);
}

void snestistics_op(const SnesisticsRegs &reg_before, const SnesisticsRegs &reg_after) {
	SnesisticsState::instance()->op(reg_before, reg_after);
}

void snestistics_read_byte(const uint32_t adress, const uint8_t value) {
	SnesisticsState::instance()->read_byte(adress, value);
}
void snestistics_read_word(const uint32_t adress, const uint16_t value) {
	SnesisticsState::instance()->read_word(adress, value);
}

void snestistics_pause_read_byte(bool pause) {
	SnesisticsState::instance()->pause_read_byte(pause);
}

void snestistics_set_save_trace_helper(bool state) {
	SnesisticsState::instance()->set_save_trace_helper(state);
}

bool snestistics_get_save_trace_helper() {
	return SnesisticsState::instance()->get_save_trace_helper();
}

bool snestistics_get_active() {
	return SnesisticsState::instance()->get_active();
}
