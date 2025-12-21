/**
 * Marlin 3D Printer Firmware - GD32 MFL EEPROM emulation with wear-leveling
 * Строгая логика старого STM32-проекта:
 * - фиксированный регион из HAL/shared/eeprom_api.h
 * - поиск непустого DWORD по всему региону → вычисление слота
 * - запись в новый слот вниз по номерам
 * - стирание всего региона при исчерпании слотов
 * - реализации FLASH_If_Erase / FLASH_If_Write под MFL
 */

#include "../../platforms.h"

#include "../../../inc/MarlinConfig.h"
#include "../../../libs/crc16.h"
#include <cstring>

#if ENABLED(FLASH_EEPROM_EMULATION)

#include "../../shared/eeprom_api.h"
#include "FMC.hpp"

using namespace fmc;

// -------------------------------
// Конфигурация
// -------------------------------
#ifndef MARLIN_EEPROM_SIZE
  #define MARLIN_EEPROM_SIZE 0x1000U  // 4KB
#endif

#ifndef FLASH_PAGE_SIZE
  #define FLASH_PAGE_SIZE 0x800U      // 2KB
#endif

#define EMPTY_UINT32 0xFFFFFFFFU
#define EMPTY_UINT8  0xFFU

// -------------------------------
//Костыль, пересмотреть позже реализацию! 
// Регион EEPROM из shared
static constexpr uint32_t REGION_BEGIN    = FLASH_OUTAGE_DATA_ADDR;
static constexpr uint32_t REGION_SIZE     = FLASH_OUTAGE_DATA_SIZE;
static constexpr uint32_t REGION_END_EXCL = REGION_BEGIN + REGION_SIZE;

static_assert((MARLIN_EEPROM_SIZE % FLASH_PAGE_SIZE) == 0, "MARLIN_EEPROM_SIZE must align to FLASH_PAGE_SIZE");
static_assert((REGION_SIZE % MARLIN_EEPROM_SIZE) == 0, "REGION_SIZE must be multiple of MARLIN_EEPROM_SIZE");

static constexpr int EEPROM_SLOTS = int(REGION_SIZE / MARLIN_EEPROM_SIZE);
#define SLOT_ADDRESS(slot) (REGION_BEGIN + (uint32_t(slot) * MARLIN_EEPROM_SIZE))
static_assert(EEPROM_SLOTS >= 2, "Use at least 2 slots for wear-leveling");

// -------------------------------
// Буфер и состояние
// -------------------------------
static uint8_t ram_eeprom[MARLIN_EEPROM_SIZE] __attribute__((aligned(4))) = {0};
static bool eeprom_data_written = false;
static int current_slot = -1;

//PersistentStore persistentStore;

#ifndef MARLIN_EEPROM_SIZE
  #define MARLIN_EEPROM_SIZE size_t(E2END + 1)
#endif
size_t PersistentStore::capacity() { return MARLIN_EEPROM_SIZE; }

// -------------------------------
// Старт доступа (поиск слота как в оригинале)
// -------------------------------
bool PersistentStore::access_start() {

  // В STM32-оригинале был EEPROM.begin() для совместимости; в MFL не нужен.

  if (current_slot == -1 || eeprom_data_written) {
    if (eeprom_data_written)
      SERIAL_ECHOLNPGM("Dangling EEPROM write_data");

    current_slot = -1;

    // Оригинальная логика: сканируем по DWORD весь регион
    uint32_t address = REGION_BEGIN;
    while (address < REGION_END_EXCL) {
      const uint32_t v = *(__IO const uint32_t*)address;
      if (v != EMPTY_UINT32) {
        current_slot = int((address - REGION_BEGIN) / MARLIN_EEPROM_SIZE);
        break;
      }
      address += sizeof(uint32_t);
    }

    if (current_slot == -1) {
      // Пусто — инициализируем RAM FF, первая запись пойдёт в "последний" слот
      memset(ram_eeprom, EMPTY_UINT8, MARLIN_EEPROM_SIZE);
      current_slot = EEPROM_SLOTS;
    }
    else {
      // Загружаем текущие настройки
      const uint8_t* src = (const uint8_t*)SLOT_ADDRESS(current_slot);
      memcpy(ram_eeprom, src, MARLIN_EEPROM_SIZE);
      SERIAL_ECHOPGM("EEPROM loaded from slot "); SERIAL_ECHOLN(current_slot);
    }

    eeprom_data_written = false;
  }

  return true;
}

// -------------------------------
// Завершение доступа (запись/стирание)
// -------------------------------
bool PersistentStore::access_finish() {

  if (!eeprom_data_written) return true;

  auto& flash = FMC::get_instance();
  bool success = true;

  // Двигаемся "вниз" по слотам
  if (--current_slot < 0) {
    // Все слоты использованы — стираем весь регион
    SERIAL_ECHOLNPGM("Erasing EEPROM region");

    TERN_(HAS_PAUSE_SERVO_OUTPUT, PAUSE_SERVO_OUTPUT());
    hal.isr_off();

    flash.unlock();

    for (uint32_t addr = REGION_BEGIN; addr < REGION_END_EXCL; addr += FLASH_PAGE_SIZE) {
      FMC_Error_Type er = flash.erase_page(addr);
      if ( er != FMC_Error_Type::READY ) {
        SERIAL_ECHO("Erase failed at 0x"); SERIAL_ECHO(addr, HEX); SERIAL_EOL();
        success = false; break;
      }
      // Верификация стирания
      for (uint32_t i = 0; i < FLASH_PAGE_SIZE; i += 4) {
        if (*(__IO const uint32_t*)(addr + i) != EMPTY_UINT32) {
          SERIAL_ECHO("Erase verify failed at 0x"); SERIAL_ECHO(addr + i, HEX); SERIAL_EOL();
          success = false; break;
        }
      }
      if (!success) break;
    }

    flash.lock();

    hal.isr_on();
    TERN_(HAS_PAUSE_SERVO_OUTPUT, RESUME_SERVO_OUTPUT());

    if (!success) {
      SERIAL_ECHOLNPGM("Save failed");
      return false;
    }

    current_slot = EEPROM_SLOTS - 1;
  }

  // Запись RAM → новый слот (полусловами, с верификацией)
  {
    const uint32_t dest_begin    = SLOT_ADDRESS(current_slot);
    const uint32_t dest_end_excl = dest_begin + MARLIN_EEPROM_SIZE;

    // Строгая защита границ
    if (dest_begin < REGION_BEGIN || dest_end_excl > REGION_END_EXCL) {
      SERIAL_ECHOLNPGM("EEPROM: slot bounds violation");
      return false;
    }

    TERN_(HAS_PAUSE_SERVO_OUTPUT, PAUSE_SERVO_OUTPUT());
    hal.isr_off();

    flash.unlock();

    uint32_t addr = dest_begin;
    for (uint32_t off = 0; off < MARLIN_EEPROM_SIZE; off += 2, addr += 2) {
      uint16_t data16 = ram_eeprom[off];
      if ((off + 1) < MARLIN_EEPROM_SIZE)
        data16 |= uint16_t(ram_eeprom[off + 1]) << 8;
      else
        data16 |= 0x00FF; // добивка для нечётного байта

      const FMC_Error_Type wr = flash.program_halfword(addr, data16);
      if (wr != FMC_Error_Type::READY) {
        SERIAL_ECHO("Write failed at 0x"); SERIAL_ECHO(addr, HEX); SERIAL_EOL();
        success = false; break;
      }
      if (*(__IO const uint16_t*)addr != data16) {
        SERIAL_ECHO("Write verify failed at 0x"); SERIAL_ECHO(addr, HEX); SERIAL_EOL();
        success = false; break;
      }
    }

    flash.lock();

    hal.isr_on();
    TERN_(HAS_PAUSE_SERVO_OUTPUT, RESUME_SERVO_OUTPUT());

    if (!success) {
      SERIAL_ECHOLNPGM("Save failed");
      return false;
    }

    eeprom_data_written = false;
    SERIAL_ECHOPGM("EEPROM saved to slot "); SERIAL_ECHOLN(current_slot);
  }

  return true;
}

// -------------------------------
// RAM-буфер
// -------------------------------
bool PersistentStore::write_data(int &pos, const uint8_t *value, size_t size, uint16_t *crc) {
  while (size--) {
    const uint8_t v = *value;
    if (v != ram_eeprom[pos]) { ram_eeprom[pos] = v; eeprom_data_written = true; }
    if (crc) crc16(crc, &v, 1);
    pos++; value++;
  }
  return false;
}

bool PersistentStore::read_data(int &pos, uint8_t *value, size_t size, uint16_t *crc, const bool writing/*=true*/) {
  do {
    const uint8_t c = ram_eeprom[pos];
    if (writing) *value = c;
    if (crc) crc16(crc, &c, 1);
    pos++; value++;
  } while (--size);
  return false;
}

// -------------------------------
// Реализации shared API под MFL
// -------------------------------
uint32_t PersistentStore::FLASH_If_Erase(uint32_t addr_start, uint32_t addr_end) {

  // Граница региона: строго внутри [REGION_BEGIN, REGION_END_EXCL)
  if (addr_start < REGION_BEGIN || addr_end > (REGION_END_EXCL - 1)) {
    return FLASHIF_ERASEKO;
  }
  if (addr_end < addr_start) {
    return FLASHIF_ERASEKO;
  }

  auto& flash = FMC::get_instance();
  flash.unlock();

  bool ok = true;

  // Считаем страницы и стираем постранично
  const uint32_t first_page = (addr_start - REGION_BEGIN) / FLASH_PAGE_SIZE;
  const uint32_t last_page  = (addr_end   - REGION_BEGIN) / FLASH_PAGE_SIZE;

  uint32_t page_addr = REGION_BEGIN + first_page * FLASH_PAGE_SIZE;
  for (uint32_t p = first_page; p <= last_page; ++p, page_addr += FLASH_PAGE_SIZE) {
    FMC_Error_Type er = flash.erase_page(page_addr);
    if (er != FMC_Error_Type::READY) { ok = false; break; }
    // Верификация
    for (uint32_t i = 0; i < FLASH_PAGE_SIZE; i += 4) {
      if (*(__IO const uint32_t*)(page_addr + i) != EMPTY_UINT32) { ok = false; break; }
    }
    if (!ok) break;
  }

  flash.lock();

  return ok ? FLASHIF_OK : FLASHIF_ERASEKO;
}

uint32_t PersistentStore::FLASH_If_Write(uint32_t destination, uint32_t *p_source, uint32_t length_words) {

  // Должны писать только внутрь региона, по словам
  const uint32_t end_addr = destination + (length_words * 4);
  if (destination < REGION_BEGIN || end_addr > REGION_END_EXCL) {
    return FLASHIF_WRITING_ERROR;
  }

  auto& flash = FMC::get_instance();
  flash.unlock();

  bool ok = true;

  uint32_t addr = destination;
  for (uint32_t i = 0; i < length_words; ++i, addr += 4) {
    // Пишем двумя полусловами (16-бит), как требует FMC
    const uint32_t word = p_source[i];

    // Нижнее полуслово
    {
      const uint16_t hw = uint16_t(word & 0xFFFF);
      const FMC_Error_Type wr = flash.program_halfword(addr + 0, hw);
      if (wr != FMC_Error_Type::READY) { ok = false; break; }
      if (*(__IO const uint16_t*)(addr + 0) != hw) { ok = false; break; }
    }

    // Верхнее полуслово
    {
      const uint16_t hw = uint16_t((word >> 16) & 0xFFFF);
      const FMC_Error_Type wr = flash.program_halfword(addr + 2, hw);
      if (wr != FMC_Error_Type::READY) { ok = false; break; }
      if (*(__IO const uint16_t*)(addr + 2) != hw) { ok = false; break; }
    }
  }

  flash.lock();

  if (!ok) return FLASHIF_WRITING_ERROR;

  // Доп. контроль, как в STM32: сверка слова
  addr = destination;
  for (uint32_t i = 0; i < length_words; ++i, addr += 4) {
    if (*(__IO const uint32_t*)addr != p_source[i]) {
      return FLASHIF_WRITINGCTRL_ERROR;
    }
  }

  return FLASHIF_OK;
}

#endif // FLASH_EEPROM_EMULATION

