#include "MBC2.h"

MBC2::MBC2(const char *romFile) : Cartridge(romFile){
    // Initialize the control registers
    ramEnable = 0x0;
    romBankSelect = 0x1; // Defaults to bank 1 on PoR

    // Allocate memory for ROM banks
    romBanks = (uint8_t**)malloc(romBankCount * sizeof(uint8_t *));
    for (uint8_t i = 0; i < romBankCount; i++){
        romBanks[i] = (uint8_t*)malloc(ROM_BANK_SIZE * sizeof(uint8_t));
    }

    // Allocate memory for the only RAM bank
    // Technically this could be cut in half since the MBC2 only uses 4
    // bits of ROM per address, but that would probably slow things down
    // and we have plenty of RAM.
    ramBank = (uint8_t*)malloc(MBC2_CART_RAM_TOP - CART_RAM * sizeof(uint8_t))
}

MBC2::~MBC2(){
    Serial.println("Deleting MBC2");
}

uint8_t MBC2::readByte(uint16_t addr){
    // Handle reads from RAM
    if(addr >= CART_RAM && addr <= MBC2_CART_RAM_TOP){
        
    }
    // Handle reads from banked cartridge ROM
    else if(addr >= CART_ROM_BANKED){

    }
    // Handle reads from ROM bank zero
    else if(addr >= CART_ROM_ZERO){

    }
    // MISRA
    else{
        // Assume invalid reads return 0xFF
        // TODO: Look this up
        Serial.printf("ERROR: Attempted to read from invalid address in MBC2 cartridge: 0x%04x\n", addr);
        return 0xFF;
    } 
}

void MBC2::writeByte(uint16_t addr, uint8_t data){
    // Handle writes to RAM
    if(addr >= CART_RAM && addr <= MBC2_CART_RAM_TOP){
        // Make sure RAM is enabled and it exists
        if(ramEnable && ramBankCount > 0){
            // Only the bottom four bits can be written to RAM
            ramBank[addr] = data & 0xF;
            return;
        }
        else{
            return;
        }
    }

    // Handle writes to control registers
    // This write function ensures that all data written to control registers
    // is valid. Additional checking elsewhere is not needed
    // Manipulate the bank select register
    else if(addr >= MBC2_PRIMARY_BANK_REG && addr <= MBC2_PRIMARY_BANK_REG_TOP){
        // LSb of upper address byte must be 1 to select a ROM bank
        if(data & 0x100){
            // Get the bank select bits from the lower 4 bits
            romBankSelect = data & 0xf;
            return
        }
        else{
            return
        }
    }
    // Manipulate the RAM enable register
    else if(addr >= MBC2_RAM_ENABLE_REG){
        // The docs are a little unclear on how this works. I assume that
        // 0x0 will disable the RAM, any other value enables RAM, and in order
        // to change states the 0x100 bit must not be set
        if(data & 0x100){
            return;
        }
        else if(data){
            ramEnable = 1;
        }
        else{
            ramEnable = 0;
        }
    }
    // MISRA
    else{
        Serial.printf("ERROR: Attempted to write 0x%04x to invalid address 0x%04x in MBC2 cartridge\n", data, addr);
    } 
}