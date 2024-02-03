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

#include "Joypad.h"

#include "Memory.h"

#define YP A4
#define XM A7
#define YM A6
#define XP A5

joypad_combined_t Joypad::previousValue = {.value = 0xFF};
#ifndef PLATFORM_NATIVE
TouchScreen Joypad::ts = TouchScreen(XP, YP, XM, YM, 300);
#endif
uint32_t Joypad::_cycle = 0;
bool Joypad::_press_value = false;

void Joypad::begin() {

    //pinMode(JOYPAD_START, INPUT_PULLUP);
    //pinMode(JOYPAD_SELECT, INPUT_PULLUP);
    //pinMode(JOYPAD_LEFT, INPUT_PULLUP);
    //pinMode(JOYPAD_RIGHT, INPUT_PULLUP);
    //pinMode(JOYPAD_UP, INPUT_PULLUP);
    //pinMode(JOYPAD_DOWN, INPUT_PULLUP);
    //pinMode(JOYPAD_B, INPUT_PULLUP);
    //pinMode(JOYPAD_A, INPUT_PULLUP);
}

void Joypad::joypadStep() {
    joypad_register_t joypad = {.value = Memory::readByte(MEM_JOYPAD)};

    #ifndef PLATFORM_NATIVE
    _cycle++; 
    int value = _cycle % 1000 == 0 ? ts.pressure() : 0;
    if (_cycle % 1000 == 0) {
        //Serial.printf("press:");Serial.println(value);
        _press_value = value>100 && value < 1000;
        if (_press_value) {
            Serial.printf("Button pressed:");Serial.println(joypad.value, BIN);
        }
    }
    #endif

    // Handle direction key input
    // Interrupts are also handled inside this condition!
    if (joypad.direction.selectDirection == 0) {
        joypad.direction.left = HIGH;
        joypad.direction.right = HIGH;
        joypad.direction.up = HIGH;
        joypad.direction.down = HIGH;

        Memory::writeByteInternal(MEM_JOYPAD, joypad.value);

        if ((joypad.value & 0xF) != 0xF && Joypad::previousValue.parts.direction != (joypad.value & 0xF)) {
            Memory::interrupt(IRQ_JOYPAD);
        }
        Joypad::previousValue.parts.direction = joypad.value & 0xF;
    }

    // Handle button key input
    // Interrupts are also handled inside this condition!
    if (joypad.button.selectButton == 0) {
        joypad.button.start = _press_value ? LOW : HIGH;
        joypad.button.select = HIGH;
        joypad.button.a = HIGH;
        joypad.button.b = HIGH;
        //joypad.button.start = press ? HIGH : LOW;

        Memory::writeByteInternal(MEM_JOYPAD, joypad.value);

        if ((joypad.value & 0xF) != 0xF && Joypad::previousValue.parts.button != (joypad.value & 0xF)) {
            Memory::interrupt(IRQ_JOYPAD);
        }
        Joypad::previousValue.parts.button = joypad.value & 0xF;
    }
}