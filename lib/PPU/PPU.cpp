/**
 * gb.teensy Emulation Software
 * Copyright (C) 2020  Raphael Stäbler, Grant Haack
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

/**
 * How the PPU Works
 *
 * Rendering a frame:
 * Graphics are loaded to the LCD via Scanline Rendering. This allows for
 * a much smaller memory footprint since the system only needs a buffer
 * the size of a single row of pixels. This buffer is called the line buffer
 * The graphics are rendered from the top down in a raster pattern, just
 * like a CRT display.
 *
 * Each line is loaded like this:
 * 1) Load the current row of the background tiles or window tiles to the
 * line buffer
 * 2) Sprite engine places the sprites with 1 pixel accuracy
 * 3) Overwrite line buffer with data from sprites, if they are on the
 * current line
 *
 * This routine is repeated 144 times to form a single frame
 *
 */

// PPU TODOs:
//  Make sure all bits in LCDC are being acted upon
//      DONE Bit 7: LCD Display Enable
//      Bit 6: Window tile map display select
//      Bit 5: Window display enable
//      Bit 4: BG & Window tile data select
//      DONE Bit 3: BG Tile Map Display Select
//      Bit 2: Sprite size
//      DONE Bit 1: Sprite display enable
//      Bit 0: BG/Window Display/Priority
//  Look in to how windows work, implement that behavior
//  Implement Background and sprite (OPB0, OBP1) color palettes

#include "PPU.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "CPU.h"
#include "Memory.h"

#define COLOR1 0x0000
#define COLOR2 0x4BC4
#define COLOR3 0x968B
#define COLOR4 0xFFFF

uint16_t PPU::frames[2][160 * 144] = {{0}, {0}};
uint64_t PPU::ticks = 0;
uint8_t PPU::originX = 0, PPU::originY = 0, PPU::lcdc = 0, PPU::lcdStatus = 0;
uint64_t PPU::fps_count = 0;

void PPU::getBackgroundForLine(const uint8_t y, uint16_t *frame, const uint8_t originX, const uint8_t originY) {
    //memset(frame + y * 160, 0x33, sizeof(uint16_t) * 160);
    uint8_t lcdc = Memory::readByteIORegFast(MEM_LCDC);
    uint8_t tileIndex, tileLineU, tileLineL;
    int8_t signedTileIndex;

    uint8_t tilePosY = (y >> 3 ) << 3;
    uint8_t tileLineY = y - tilePosY;

    // Check to see which Background Tile Map is selected
    uint16_t bgTileMap = MEM_VRAM_MAP1;
    // Read bit 3 of LCDC to get the Background Tile Map
    if ((lcdc & 0x08) == 0x08) {
        // If it's set, use the second tilemap
        bgTileMap = MEM_VRAM_MAP2;
    }

    // Check to see which addressing method is being used for VRAM
    uint16_t baseTilePtr = MEM_VRAM_TILES_B2;
    bool convertTileIndex = true;
    // If LCDC bit 4 is set, use VRAM Tiles Block0 as a base pointer
    // for the tiles and access them with an unsigned index (0 - 255)
    // Otherwise, use VRAM Tiles Block1 as a base pointer for the
    // tiles and access them with a signed index (-128 to 127)
    if ((lcdc & 0x10) == 0x10) {
        baseTilePtr = MEM_VRAM_TILES;
        convertTileIndex = false;
    }

    uint16_t base = y * 160;
    uint16_t indexed_base = base;
    uint32_t tileBase = bgTileMap + ((tilePosY >> 3) << 5);
    uint16_t tileAddress = 0;
    uint16_t word = 0;
    for (uint8_t i = 0; i < 20; i++) {
        tileIndex = Memory::readByte(i + tileBase);
        // Check to see if the tile index needs to be converted to a signed number
        tileAddress = baseTilePtr + (tileLineY << 1);
        if (convertTileIndex) {
            signedTileIndex = (int8_t)tileIndex;
            tileAddress = tileAddress + signedTileIndex * 16 ;
        } else {
            tileAddress = tileAddress + tileIndex * 16 ;
        }
        
        // Convert the tile index and use it     
        //tileLineL = Memory::readByte(tileAddress);
        //tileLineU = Memory::readByte(tileAddress + 1);
        word = Memory::readWord(tileAddress);
        tileLineL = word & 0x00FF;
        tileLineU = word >> 8;
       
        indexed_base = base + i * 8;
        for (int8_t c = 0; c < 8; c++) {
            frame[indexed_base + c] = (((tileLineU >> (7 - c)) << 1) & 0x2) | ((tileLineL >> (7 - c)) & 0x1);
        }
    }
}

void PPU::getSpritesForLine(const uint8_t y, uint16_t *frame) {
    uint8_t spritePosX, spritePosY;
    uint8_t tileIndex, attributes, tileLineU, tileLineL, pixel;
    int16_t spriteLineY, x;
    uint16_t memIndex;

    uint16_t const_line = y * 160;

    bool flag = false;
    uint16_t word = 0;

    for (uint16_t i = 0xFE00; i < 0xFEA0; i += 4) {
        spritePosY = Memory::readByteIORegFast(i) - 16;
        spriteLineY = y - spritePosY;
        if (spriteLineY >= 0 && spriteLineY < 8) {
            spritePosX = Memory::readByteIORegFast(i + 1) - 8;
            tileIndex = Memory::readByteIORegFast(i + 2);
            attributes = Memory::readByteIORegFast(i + 3);

            memIndex = MEM_VRAM_TILES + tileIndex * 16 + spriteLineY * 2;

            //tileLineL = Memory::readByte(memIndex);
            //tileLineU = Memory::readByte(memIndex+1);

            word = Memory::readWord(memIndex);
            tileLineL = word & 0x00FF;
            tileLineU = word >> 8;

            flag = (attributes & 0x80) == 0;

            for (int8_t c = 0; c < 8; c++) {
                x = const_line + spritePosX + c;
                //if (x >= 0) {
                    if (flag || frame[x] == 0) {
                        pixel = (((tileLineU >> (7 - c)) << 1) & 0x2) | ((tileLineL >> (7 - c)) & 0x1);
                        if (pixel != 0) frame[x] = pixel;
                    }
                //}
            }
        }
    }
}

uint16_t COLORS[4] = { COLOR4, COLOR3, COLOR2, COLOR1 }; 

void PPU::mapColorsForFrame(uint16_t *frame) {
    for (uint16_t i = 0; i < 160 * 144; i++) {
        frame[i] = COLORS[frame[i]];
    }
}

#ifndef PLATFORM_NATIVE
void PPU::ppuStep(Adafruit_ILI9341 &tft) {
#else
void PPU::ppuStep() {
#endif
    uint8_t y = Memory::readByteIORegFast(MEM_LCD_Y) % 152; // 152
    //uint8_t y = 0;
    static uint8_t sendingFrame = 1;
    static uint8_t calculatingFrame = 0;
    uint16_t word = 0;

    while (ticks < CPU::totalCycles) {
        ticks++;
        const uint8_t cycleTicks = ticks % 114; //114

        switch (cycleTicks) {
            case 0:  // reading from OAM memory
                // TODO: Disable access to OAM during this time
                lcdStatus = Memory::readByteIORegFast(MEM_LCD_STATUS);
                // Set LCD STAT to Mode 2: Searching OAM
                Memory::writeByteInternal(MEM_LCD_STATUS, (lcdStatus & 0xFC) | 0x02);
                // Trigger an OAM interrupt through LCD STAT if enabled
                if ((lcdStatus & 0x20) == 0x20) {
                    Memory::interrupt(IRQ_LCD_STAT);
                }
                break;

            case 20:  // reading from both OAM and VRAM //20
                // TODO: Disable access to all video memory during this time
                lcdStatus = Memory::readByteIORegFast(MEM_LCD_STATUS);
                // Set LCD STAT to mode 0x3, Transfer Data to LCD Driver
                Memory::writeByteInternal(MEM_LCD_STATUS, (lcdStatus & 0xFC) | 0x03);  // BUG: Shouldn't this be 0x03? If things break, return 0x03 to 0x02
                // Check if we the current line is the same as what's in LY Compare (LYC)
                //y = Memory::readByteIORegFast(MEM_LCD_Y) % 152; // 152
                if (y == Memory::readByteIORegFast(MEM_LCD_YC)) {
                    // Set coincidence flag
                    Memory::writeByteInternal(MEM_LCD_STATUS, (lcdStatus & 0xFB) | 0x04);
                    // Trigger coincidence interrupt through LCD STAT if enabled
                    if ((lcdStatus & 0x40) == 0x40) {
                        Memory::interrupt(IRQ_LCD_STAT);
                    }
                } else {
                    // Otherwise, clear the coincidence flag
                    Memory::writeByteInternal(MEM_LCD_STATUS, (lcdStatus & 0xFB) | 0x00);
                }
                break;

            case 43:  // H-Blank and V-Blank period //43
                
                word = Memory::readWord(MEM_LCDC); // + MEM_LCD_STATUS
                lcdc = word & 0x00FF;
                lcdStatus = word >> 8;
                //lcdc =  Memory::readByte(MEM_LCDC);
                //lcdStatus = Memory::readByte(MEM_LCD_STATUS);
                // Check if LCD is enabled
                if ((lcdc & 0x80) == 0x80) {
                    y = (y + 1) % 152;
                    // Update the current LCD Y coordinate
                    Memory::writeByteInternal(MEM_LCD_Y, y);
                    // Get the X and Y posision of the background map
                    word = Memory::readWord(MEM_LCD_SCROLL_Y); // + MEM_LCD_SCROLL_X
                    originY = word & 0x00FF;
                    originX = word >> 8;

                    //originY = Memory::readByte(MEM_LCD_SCROLL_Y);
                    //originX = Memory::readByte(MEM_LCD_SCROLL_X);
                    // Make sure we're in the visible portion of the screen
                    if (y < 144) {
                        // Because of how our screen works in the emulator, we
                        // perform the entire line transfer immediately after
                        // the Game Boy's "transfer" phase.
                        // A cycle accurate system would transfer data over,
                        // pixel by pixel, during the transfer phase. Instead,
                        // We get the whole line at once as soon as we hit H-Blank
                        // This will need to be rewritten if we ever need to
                        // emulate some behavior that takes place mid-scanline

                        // Check if background is enabled
                        if ((lcdc & 0x01) == 0x01) {
                            // Get the background for the current line
                            getBackgroundForLine(y, frames[calculatingFrame], originX, originY);
                        }
                        // Check if sprites are enabled
                        if ((lcdc & 0x02) == 0x02) {
                            // Get the sprite for the current line
                            getSpritesForLine(y, frames[calculatingFrame]);
                        }
                        // Set LCD STAT to mode 0, During H-Blank
                        Memory::writeByteInternal(MEM_LCD_STATUS, (lcdStatus & 0xFC) | 0x00);
                        // Trigger H-Blank interrupt through LCD STAT if enabled
                        if ((lcdStatus & 0x08) == 0x08) {
                            Memory::interrupt(IRQ_LCD_STAT);
                        }
                        // If we're outside viewable area, we're in VBLANK
                    } else if (y == 144) {
                        // Set LCD STAT to mode 1, VBlank
                        Memory::writeByteInternal(MEM_LCD_STATUS, (lcdStatus & 0xFC) | 0x01);
                        // Trigger a VBLANK interrupt
                        Memory::interrupt(IRQ_VBLANK);

                        // Map colors for the frame
                        mapColorsForFrame(frames[calculatingFrame]);

                        // Swap the sending and calculating frame
                        sendingFrame = calculatingFrame;

#ifndef PLATFORM_NATIVE
                        tft.dmaWait();
                        tft.endWrite();
#endif

                        fps_count++;

#ifndef PLATFORM_NATIVE
                        tft.startWrite();
                        //tft.setAddrWindow(0, 0, 160, 144);
                        tft.writePixels(&frames[sendingFrame][0], 160 * 144, false); // Non-blocking write
#endif
                        

                        calculatingFrame = (calculatingFrame + 1) % 2;
                        // Write the sending frame to the screen
                        //tft.writeGRAM(0, 2 * 160 * 144, (uint8_t *)frames[sendingFrame]);
                        //tft.drawRGBBitmap(0,0, (uint16_t *)frames[sendingFrame], 160, 144);

                    }
                } else {
                    // If LCD is not enabled, always set LCD STAT to mode 0, Vertical Blanking
                    Memory::writeByteInternal(MEM_LCD_STATUS, (lcdStatus & 0xFC) | 0x00);

                }
                break;

            default:
                break;
        }
    }

/*
    lcdStatus = Memory::readByte(MEM_LCD_STATUS);
    // Check if LCD is enabled
    if ((lcdc & 0x80) != 0x80) {
        Serial.print("LCD not enabled ");Serial.print("PC:");Serial.print(CPU::PC, HEX); Serial.print(" IME:");Serial.print(CPU::IME, HEX);Serial.print(" op: ");Serial.print(CPU::op, HEX);Serial.print(" LCD: ");Serial.println( Memory::readByte(MEM_LCD_STATUS), HEX);
    }
*/
}
