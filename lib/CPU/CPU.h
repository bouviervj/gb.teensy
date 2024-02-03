/**
 * gb.teensy Emulation Software
 * Copyright (C) 2020  Raphael Stäbler
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 **/

#pragma once

#include <Arduino.h>
#include "Memory.h"

typedef void (*routing_func)(uint8_t& cyclesDelta, uint8_t& op);

class CPU {
   public:
    static volatile bool cpuEnabled;
    static volatile uint64_t totalCycles;
    static volatile uint64_t totalCyclesInCBInstruct;

    static void cpuStep();
    static void stopAndRestart();

    static void reset();

   protected:
    static inline uint8_t readOp(){
        /**
         * Read an opcode from the program
         * @return An 8 byte opcode
         */
        return Memory::readByte(PC++);
    }

    static inline uint16_t readNn(){
        /**
         * Read program data from where PC is currently pointing
         * Advances PC by two
         * @return Two bytes of big endian program data
         */
        uint8_t n1 = Memory::readByte(PC++);
        uint8_t n2 = Memory::readByte(PC++);
        return n1 | (n2 << 8);
    }
    
    static void pushStack(const uint16_t data);
    static uint16_t popStack();

 //   static routing_func routing_func_mapping[16]; 

   public:

    static int (*Inst0xAll[])(); // Regular CPU Instruction Set
    static int (*InstCB0xAll[])(); // Extended Instruction Set

    // Registers
    static uint16_t AF;
    static uint16_t BC;
    static uint16_t DE;
    static uint16_t HL;
    static uint16_t SP;
    static uint16_t PC;

    // Init OP
    static uint8_t op;

    // Init IME
    static bool IME;

    // Virtual HALT
    static bool halted;

    // Define benchmark and debugging options
    static double start, stop;

    // IRQ control
    static uint8_t enableIRQ, disableIRQ;

    // Divider interval
    static uint8_t divider;

    // Timer control
    static uint64_t timerCycles, timerTotalCycles;

    static uint8_t cyclesDelta;

    // Debug
    static void dumpRegister();
    static void dumpStack();

    // Run CB extension code
    static void cpuRunCB(uint8_t& cyclesDelta);

/*
    static void runInst0x00(uint8_t& cyclesDelta, uint8_t& op);
    static void runInst0x10(uint8_t& cyclesDelta, uint8_t& op);
    static void runInst0x20(uint8_t& cyclesDelta, uint8_t& op);
    static void runInst0x30(uint8_t& cyclesDelta, uint8_t& op);
    static void runInst0x40(uint8_t& cyclesDelta, uint8_t& op);
    static void runInst0x50(uint8_t& cyclesDelta, uint8_t& op);
    static void runInst0x60(uint8_t& cyclesDelta, uint8_t& op);
    static void runInst0x70(uint8_t& cyclesDelta, uint8_t& op);
    static void runInst0x80(uint8_t& cyclesDelta, uint8_t& op);
    static void runInst0x90(uint8_t& cyclesDelta, uint8_t& op);
    static void runInst0xA0(uint8_t& cyclesDelta, uint8_t& op);
    static void runInst0xB0(uint8_t& cyclesDelta, uint8_t& op);
    static void runInst0xC0(uint8_t& cyclesDelta, uint8_t& op);
    static void runInst0xD0(uint8_t& cyclesDelta, uint8_t& op);
    static void runInst0xE0(uint8_t& cyclesDelta, uint8_t& op);
    static void runInst0xF0(uint8_t& cyclesDelta, uint8_t& op);
*/

    // Internal CPU Instruction variables
    static uint8_t n, n1, n2, interrupt;
    static int8_t sn;
    static uint16_t nn, nn1, nn2;
    static bool c;

};
