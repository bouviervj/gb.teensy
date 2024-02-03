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

// This is the main entry point for the Teensy microcontroller.
// Compile and upload to the Teensy development board in order to run the emulator.

#include <APU.h>
#include <Arduino.h>
#include <CPU.h>
#include <Cartridge.h>
#include <Joypad.h>
#include <Memory.h>
#include <PPU.h>
#include <SerialDataTransfer.h>

#ifndef PLATFORM_NATIVE

#define TFT_D0        34 // Data bit 0 pin (MUST be on PORT byte boundary)
#define TFT_WR        26 // Write-strobe pin (CCL-inverted timer output)
#define TFT_DC        10 // Data/command pin
#define TFT_CS        11 // Chip-select pin
#define TFT_RST       24 // Reset pin
#define TFT_RD         9 // Read-strobe pin
#define TFT_BACKLIGHT 25


#define SCREENWIDTH  ILI9341_TFTHEIGHT // Native display orientation is
#define SCREENHEIGHT ILI9341_TFTWIDTH  // vertical, so swap width/height

// ILI9341 with 8-bit parallel interface:
Adafruit_ILI9341 tft(tft8bitbus, TFT_D0, TFT_WR, TFT_DC, TFT_CS, TFT_RST, TFT_RD);

#define BGCOLOR    0xAD75
#define GRIDCOLOR  0xA815
#define BGSHADOW   0x5285
#define GRIDSHADOW 0x600C
#define RED        0xF800
#define WHITE      0xFFFF

//FT81x ft81x = FT81x(10, 9, 8);

static char title[17];  // 16 chars for name, 1 for null terminator

static uint64_t start = 0;
static uint64_t previous_time = 0;

void setup() {
    Serial.begin(115200);

    delay(2000); // waits for a second

    //SPI.begin();

    Serial.println("Enable display");

    pinMode(TFT_BACKLIGHT, OUTPUT);
    digitalWrite(TFT_BACKLIGHT, HIGH);

    tft.begin(42000000L);
    tft.setRotation(3); // Landscape orientation, USB at bottom right

     // read diagnostics (optional but can help debug problems)
    uint8_t x = tft.readcommand8(ILI9341_RDMODE);
    Serial.print("Display Power Mode: 0x"); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDMADCTL);
    Serial.print("MADCTL Mode: 0x"); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDPIXFMT);
    Serial.print("Pixel Format: 0x"); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDIMGFMT);
    Serial.print("Image Format: 0x"); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDSELFDIAG);
    Serial.print("Self Diagnostic: 0x"); Serial.println(x, HEX); 

    tft.fillScreen(ILI9341_BLACK);
    tft.setCursor(0, 0);
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_WHITE);
    //tft.println(title);
    tft.println("Emulated speed: ...");

    tft.startWrite();
    tft.setAddrWindow(0, 0, 160, 144);
    tft.endWrite();

    Serial.printf("\nStart Gameboy...\n");
    Cartridge::begin("mario.gb");
    Cartridge::getGameName(title);

    Memory::initMemory();
    CPU::cpuEnabled = 1;

    //APU::begin();
    Joypad::begin();
    
}

void loop() {

    start = millis();
    previous_time = start;
    PPU::fps_count = 0;

    while (true) {
        
        //Serial.print("Execute CPU step");
        CPU::cpuStep();
        //Serial.print("End Execute CPU step");
        //Serial.print("Execute PPU step");
        PPU::ppuStep(tft);
        //Serial.print("End Execute PPU step");
       
        //APU::apuStep();
        SerialDataTransfer::serialStep();
        Joypad::joypadStep();

        if ((CPU::totalCycles % 1000000) == 0) {
            uint64_t milliseconds =  millis();
            uint64_t ellapsed = milliseconds - previous_time;
            previous_time = milliseconds;

            Serial.print("Ellapsed:"); Serial.println(ellapsed);

            uint64_t hz = 1000 * 1000000 / ellapsed; // # de cycles per seconds 
            uint32_t speed = hz;
            char buff[40];
            sprintf(buff, "Emulated speed: %ld kCycle/sec", speed);
             Serial.println(buff);
            
            sprintf(buff, "CB Instr: %ld Cycle", CPU::totalCyclesInCBInstruct);
            Serial.println(buff);
            CPU::totalCyclesInCBInstruct = 0;
            
            uint64_t fps = (PPU::fps_count *1000)/ ellapsed;
            PPU::fps_count = 0;
            sprintf(buff, "FPS: %ld fps", fps);
            Serial.println(buff);
        }

    }

}

#else

// The code down here is for automated runs of ROMs on a host computer (e.g. Linux)
// It depends on some mocked Arduino libraries currently placed inside the test directory
// as well as ROM data, also placed inside the test directory.
//
// Example usage:
// > pio run -e native
// > .pio/build/native/program 0 70000000
//
// The commands above will run the ROM data at ROM::getRom(0) for 70000000 cycles.
// All the Serial output is printed to stdout.

#include <Arduino.h>
#include <CPU.h>
#include <Memory.h>
#include <PPU.h>
#include <SD.h>
#include <SerialDataTransfer.h>
#include <rom.h>
#include <iostream>
#include <iomanip>

SDClass SD;
StdioSerial Serial;

int main(int argc, char **argv) {
    if (argc != 3) {
        printf("Invalid argument count %i instead of 3.\n", argc);
        printf("Usage: program [rom index] [cycle count]\n");
        return 1;
    }

    //const unsigned int romIndex = atoi(argv[1]);
    const unsigned long cycleCount = atol(argv[2]);

    long bufsize = 0;
    uint8_t *source = NULL;
    FILE *fp = fopen(argv[1], "r");
    if (fp != NULL) {
        /* Go to the end of the file. */
        if (fseek(fp, 0L, SEEK_END) == 0) {
            /* Get the size of the file. */
            bufsize = ftell(fp);
            if (bufsize == -1) { /* Error */ }

            std::cout << "ROM data size:" << bufsize << std::endl;

            /* Allocate our buffer to that size. */
            source = (uint8_t*) malloc(sizeof(uint8_t) * (bufsize));

            /* Go back to the start of the file. */
            if (fseek(fp, 0L, SEEK_SET) != 0) { /* Error */ }

            /* Read the entire file into memory. */
            size_t newLen = fread(source, sizeof(uint8_t), bufsize, fp);
            if ( ferror( fp ) != 0 ) {
                fputs("Error reading file", stderr);
            } 
        }
        fclose(fp);
    } else {
        std::cout << "File not found:" << argv[1] << std::endl;
    }

    for (int i=0; i< bufsize; i++){
        if (i%32==0) std::cout << std::endl << std::hex <<  std::setw(4) << i << " " ;
        std::cout << " " << std::hex << std::uppercase <<  std::setw(2) << std::setfill('0') << (uint32_t) source[i] ;
    }
    std::cout << std::endl;

    Cartridge::begin(source);

    //Cartridge::begin(ROM::getRom(romIndex));
    Memory::initMemory();
    CPU::cpuEnabled = 1;

    CPU::reset();

    while (CPU::totalCycles < cycleCount) {
        CPU::cpuStep();
        PPU::ppuStep();
        SerialDataTransfer::serialStep();
    }

    free(source); /* Don't forget to call free() later! */

    return 0;
}

#endif
