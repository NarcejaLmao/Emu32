// SD-card mounting, ROM discovery, filename helpers and ROM-list state.
// Keep this module included after the system enum, directory defines and Arduino/SD headers.
#pragma once

static const char *const sysName[SYS_COUNT]  = { "NES", "GAME BOY", "GAME BOY COLOR", "ATARI 2600", "SEGA GENESIS" };
static const char *const sysShort[SYS_COUNT] = { "NES", "GB", "GBC", "2600", "GEN" };
static const char *const sysSub[SYS_COUNT]   = { "8-BIT CONSOLE", "HANDHELD", "COLOR HANDHELD", "VCS CONSOLE", "16-BIT CONSOLE" };
static const char *const sysDir[SYS_COUNT]   = { NES_DIR, GB_DIR, GBC_DIR, A2600_DIR, GENESIS_DIR };

// Longest supported FAT filename (255 bytes) plus system directory and terminator.
static const size_t ROM_PATH_LEN = 288;

static char loadErr1[24], loadErr2[24];          // last load / scan error, shown on screen

static char **romNames = nullptr;
static int   romCount = 0;
static int   romCapacity = 0;
static bool  romListTruncated = false;

// Store filename bytes in stable, growable blocks instead of one malloc per
// ROM. This sharply reduces heap fragmentation on memory-constrained boards.
static const size_t ROM_NAME_BLOCK_BYTES = 2048;
struct RomNameBlock {
  RomNameBlock *next;
  size_t used;
  size_t capacity;
  char data[1];
};
static RomNameBlock *romNameBlocks = nullptr;
static RomNameBlock *romNameTail = nullptr;

static char *copyRomName(const char *name) {
  if (!name) return nullptr;
  const size_t bytes = strlen(name) + 1;
  if (bytes > ROM_NAME_BLOCK_BYTES) return nullptr;
  RomNameBlock *block = romNameTail;
  if (!block || block->capacity - block->used < bytes) {
    const size_t capacity = ROM_NAME_BLOCK_BYTES;
    if (capacity > SIZE_MAX - (sizeof(RomNameBlock) - 1)) return nullptr;
    block = (RomNameBlock *)malloc(sizeof(RomNameBlock) - 1 + capacity);
    if (!block) return nullptr;
    block->next = nullptr;
    block->used = 0;
    block->capacity = capacity;
    if (romNameTail) romNameTail->next = block;
    else romNameBlocks = block;
    romNameTail = block;
  }
  char *copy = block->data + block->used;
  memcpy(copy, name, bytes);
  block->used += bytes;
  return copy;
}
static int   menuSel[SYS_COUNT];                 // remembered per system between visits
static int   sysSel   = 0;                       // remembered main-menu choice

// ---- SD card: mounted on demand. The NES unmounts it once the ROM is in RAM;
// ---- the Game Boy keeps it mounted (battery saves, streaming big ROMs).

// ---- memory helper: internal RAM first, PSRAM as a fallback -------------------
// The board has 8 MB of octal PSRAM (enable it: Tools > PSRAM > "OPI PSRAM"). Hot, small data stays in
// fast internal RAM; when internal RAM is short (BLE stack, SD buffers...) bigger buffers spill into PSRAM
// instead of failing with NOT ENOUGH RAM. `internalReserve` = internal heap to leave free (0 = none).
#include <esp_heap_caps.h>
static void *emuAlloc(size_t n, size_t internalReserve = 0, bool zero = false) {
  // Guard the reserve calculation against size_t overflow. Without this,
  // internalReserve + n could wrap and incorrectly allow an internal-RAM
  // allocation that violates the caller's reserve requirement.
  if (n > SIZE_MAX - internalReserve) return nullptr;
  void *p = nullptr;
  if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) > internalReserve + n)
    p = heap_caps_malloc(n, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  if (!p && psramFound()) p = heap_caps_malloc(n, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  if (p && zero) memset(p, 0, n);
  return p;                                      // release with free() as usual
}

static bool sdMounted = false;
static char currentRomPath[ROM_PATH_LEN] = "";
// Set after an SD operation fails; the main loop uses this to leave streaming
// cores safely and attempt a clean remount when the user returns to the menu.
static bool sdIoFault = false;
static void sdMarkIoFault(const char *where) {
  if (!sdIoFault) Serial.printf("SD: I/O failure during %s; stopping SD-backed emulation\n", where ? where : "operation");
  sdIoFault = true;
}
// Last save failure reason for serial diagnostics and user-facing messages.
static char sdSaveLastError[64] = "";
// Forward declaration: save helpers can request a clean mount before I/O.
static bool sdMount();
static void sdSaveSetError(const char *reason) {
  strlcpy(sdSaveLastError, reason ? reason : "SAVE FAILED", sizeof(sdSaveLastError));
  Serial.printf("SAVE: %s\n", sdSaveLastError);
}

// Battery-save safety helpers. Write a complete replacement beside the save,
// verify its length, then rotate the old save to .bak before promoting .tmp.
// Keeping the previous generation means a reset or card-write failure cannot
// destroy the last known-good save. FILE_WRITE appends on Arduino SD, so the
// temporary file is explicitly removed before every write.
static bool sdAtomicSaveWrite(const char *path, const uint8_t *data, size_t len) {
  sdSaveLastError[0] = '\0';
  if (!path || !path[0] || !data || !len) { sdSaveSetError("INVALID SAVE REQUEST"); return false; }
  if (!sdMount()) { sdSaveSetError("SD CARD UNAVAILABLE"); return false; }

  // Save paths can be as long as ROM_PATH_LEN; include room for .tmp/.bak/.bak.prev.
  char tmp[ROM_PATH_LEN + 16], bak[ROM_PATH_LEN + 16], prev[ROM_PATH_LEN + 16];
  if (snprintf(tmp, sizeof(tmp), "%s.tmp", path) >= (int)sizeof(tmp) ||
      snprintf(bak, sizeof(bak), "%s.bak", path) >= (int)sizeof(bak) ||
      snprintf(prev, sizeof(prev), "%s.bak.prev", path) >= (int)sizeof(prev)) {
    sdSaveSetError("SAVE PATH TOO LONG"); return false;
  }

  // Arduino FILE_WRITE appends, so always start with an empty temporary file.
  SD_MMC.remove(tmp);
  File f = SD_MMC.open(tmp, FILE_WRITE);
  if (!f) {
    sdSaveSetError("SD WRITE FAILED / CARD FULL");
    sdMarkIoFault("save open for write");
    Serial.printf("SAVE: cannot open temp %s\n", tmp);
    return false;
  }
  size_t written = f.write(data, len);
  f.flush();
  f.close();
  if (written != len) {
    sdSaveSetError("SD CARD FULL OR WRITE FAILED");
    Serial.printf("SAVE: short write %u/%u bytes: %s\n", (unsigned)written, (unsigned)len, tmp);
    sdMarkIoFault("save short write");
    SD_MMC.remove(tmp);
    return false;
  }

  File check = SD_MMC.open(tmp, FILE_READ);
  bool verified = check && check.size() == len;
  if (check) check.close();
  if (!verified) {
    sdSaveSetError("SAVE VERIFY FAILED");
    Serial.printf("SAVE: verify failed: %s\n", tmp);
    sdMarkIoFault("save verification read");
    SD_MMC.remove(tmp);
    return false;
  }

  // Recover an interrupted backup rotation before starting another transaction.
  // A missing primary with both backup generations means the old primary had
  // already moved to .bak; restore it, then restore the older backup to .bak.prev's
  // former role. If the primary exists, a stale .bak.prev is safe to retire only
  // after the primary is known to exist.
  bool hadMain = SD_MMC.exists(path);
  if (SD_MMC.exists(prev)) {
    if (!hadMain && SD_MMC.exists(bak)) {
      if (!SD_MMC.rename(bak, path)) {
        sdSaveSetError("SAVE RECOVERY FAILED");
        sdMarkIoFault("save recovery primary rename");
        SD_MMC.remove(tmp);
        return false;
      }
      hadMain = true;
    }
    if (hadMain && !SD_MMC.exists(bak)) {
      if (!SD_MMC.rename(prev, bak)) {
        sdSaveSetError("SAVE RECOVERY FAILED");
        sdMarkIoFault("save recovery backup rename");
        SD_MMC.remove(tmp);
        return false;
      }
    } else if (hadMain && SD_MMC.exists(bak)) {
      // Primary + backup are already present, so this is a committed save whose
      // cleanup was interrupted. The older .bak.prev generation is now obsolete.
      if (!SD_MMC.remove(prev))
        Serial.printf("SAVE: warning: could not remove obsolete backup %s\n", prev);
    }
  }

  hadMain = SD_MMC.exists(path);
  bool movedBackup = false;
  if (SD_MMC.exists(bak)) {
    if (!SD_MMC.rename(bak, prev)) {
      sdSaveSetError("CANNOT PRESERVE OLD BACKUP");
      sdMarkIoFault("save backup preservation rename");
      SD_MMC.remove(tmp);
      return false;
    }
    movedBackup = true;
  }

  if (hadMain && !SD_MMC.rename(path, bak)) {
    sdSaveSetError("CANNOT BACK UP OLD SAVE");
    sdMarkIoFault("save backup rename");
    if (movedBackup && !SD_MMC.rename(prev, bak))
      Serial.printf("SAVE: WARNING: backup remains at %s\n", prev);
    SD_MMC.remove(tmp);
    return false;
  }

  if (!SD_MMC.rename(tmp, path)) {
    sdSaveSetError("CANNOT INSTALL SAVE");
    sdMarkIoFault("save promote rename");
    // Restore the old primary first, then put the prior backup back in place.
    if (hadMain && !SD_MMC.rename(bak, path))
      Serial.printf("SAVE: WARNING: old primary remains at %s\n", bak);
    if (movedBackup && !SD_MMC.rename(prev, bak))
      Serial.printf("SAVE: WARNING: previous backup remains at %s\n", prev);
    SD_MMC.remove(tmp);
    return false;
  }

  // Commit succeeded: the new primary is verified and installed. Only now is
  // it safe to discard the older backup generation.
  if (SD_MMC.exists(prev) && !SD_MMC.remove(prev))
    Serial.printf("SAVE: warning: could not remove obsolete backup %s\n", prev);
  sdSaveLastError[0] = '\0';
  Serial.printf("SAVE: verified %s (%u bytes)\n", path, (unsigned)len);
  return true;
}

// Load a save only when its complete length matches the cartridge RAM size.
// If the primary save is missing/corrupt, recover from the last known-good .bak.
static bool sdAtomicSaveLoad(const char *path, uint8_t *data, size_t len) {
  if (!path || !path[0] || !data || !len) return false;
  if (!sdMount()) { sdSaveSetError("SD CARD UNAVAILABLE"); return false; }
  char bak[ROM_PATH_LEN + 16], prev[ROM_PATH_LEN + 16];
  if (snprintf(bak, sizeof(bak), "%s.bak", path) >= (int)sizeof(bak) ||
      snprintf(prev, sizeof(prev), "%s.bak.prev", path) >= (int)sizeof(prev)) return false;
  // The save writer keeps a second-generation backup during atomic rotation.
  // Try it as a last resort if both the primary and .bak are missing or invalid.
  const char *candidates[3] = { path, bak, prev };
  for (uint8_t i = 0; i < 3; ++i) {
    File f = SD_MMC.open(candidates[i], FILE_READ);
    if (!f) continue;
    size_t size = f.size();
    if (size != len) { f.close(); Serial.printf("SAVE: invalid size %u (expected %u): %s\n", (unsigned)size, (unsigned)len, candidates[i]); continue; }
    size_t got = f.read(data, len);
    f.close();
    if (got == len) {
      sdSaveLastError[0] = '\0';
      Serial.printf("SAVE: loaded %s (%u bytes)%s\n", candidates[i], (unsigned)len, i == 1 ? " [backup recovery]" : (i == 2 ? " [previous backup recovery]" : ""));
      return true;
    }
    Serial.printf("SAVE: short read %u/%u: %s\n", (unsigned)got, (unsigned)len, candidates[i]);
    sdSaveSetError("SAVE READ FAILED");
    sdMarkIoFault("battery save short read");
  }
  return false;
}

// ROM streaming diagnostics. Counters are deliberately lightweight: no per-read
// Serial output, so diagnostics do not themselves cause frame-time spikes.
struct SdRomDiag {
  uint32_t reads, seeks, failures, shortReads, slowReads, retries, cacheHits, cacheMisses;
  uint64_t bytesRead, readUsTotal, seekUsTotal;
  uint32_t maxReadUs, maxSeekUs;
  char label[48];
};
static SdRomDiag sdRomDiag = {};
static void sdDiagReset(const char *label) {
  memset(&sdRomDiag, 0, sizeof(sdRomDiag));
  if (label) strlcpy(sdRomDiag.label, label, sizeof(sdRomDiag.label));
}
static int sdDiagReadAt(File &f, uint32_t offset, uint8_t *dst, size_t len) {
  if (!f || !dst || !len) { sdRomDiag.failures++; sdMarkIoFault("ROM read (invalid file handle)"); return 0; }
  // Retry one transient seek/read failure. Always restart from the requested
  // offset so a partial first read can never leave a mixed/corrupt cache page.
  for (uint8_t attempt = 0; attempt < 2; ++attempt) {
    if ((uint32_t)f.position() != offset) {
      uint32_t t0 = micros();
      bool ok = f.seek(offset);
      uint32_t elapsed = micros() - t0;
      sdRomDiag.seeks++;
      sdRomDiag.seekUsTotal += elapsed;
      if (elapsed > sdRomDiag.maxSeekUs) sdRomDiag.maxSeekUs = elapsed;
      if (!ok) {
        if (attempt == 0) { sdRomDiag.retries++; delay(2); continue; }
        sdRomDiag.failures++;
        sdMarkIoFault("ROM seek");
        return 0;
      }
    }
    uint32_t t0 = micros();
    int n = f.read(dst, len);
    uint32_t elapsed = micros() - t0;
    sdRomDiag.reads++;
    sdRomDiag.readUsTotal += elapsed;
    if (elapsed > sdRomDiag.maxReadUs) sdRomDiag.maxReadUs = elapsed;
    if (elapsed >= 5000) sdRomDiag.slowReads++;
    if (n > 0) sdRomDiag.bytesRead += (uint32_t)n;
    if (n >= 0 && (size_t)n == len) return n;
    if (n >= 0 && (size_t)n < len) sdRomDiag.shortReads++;
    if (attempt == 0) { sdRomDiag.retries++; delay(2); continue; }
    sdRomDiag.failures++;
    sdMarkIoFault("ROM read/short read");
    return 0; // normalize negative reads; callers treat this as a short read
  }
  sdRomDiag.failures++;
  sdMarkIoFault("ROM read");
  return 0;
}
static void sdDiagSummary(const char *reason) {
  const uint32_t avgRead = sdRomDiag.reads ? (uint32_t)(sdRomDiag.readUsTotal / sdRomDiag.reads) : 0;
  const uint32_t avgSeek = sdRomDiag.seeks ? (uint32_t)(sdRomDiag.seekUsTotal / sdRomDiag.seeks) : 0;
  Serial.printf("SDDIAG %s (%s): reads=%lu bytes=%llu avg/max-read=%lu/%lu us seeks=%lu avg/max-seek=%lu/%lu us slow(>=5ms)=%lu short=%lu retries=%lu errors=%lu cache-hit/miss=%lu/%lu heap=%u min=%u\n",
    sdRomDiag.label, reason ? reason : "summary", (unsigned long)sdRomDiag.reads,
    (unsigned long long)sdRomDiag.bytesRead, (unsigned long)avgRead,
    (unsigned long)sdRomDiag.maxReadUs, (unsigned long)sdRomDiag.seeks,
    (unsigned long)avgSeek, (unsigned long)sdRomDiag.maxSeekUs,
    (unsigned long)sdRomDiag.slowReads, (unsigned long)sdRomDiag.shortReads,
    (unsigned long)sdRomDiag.retries, (unsigned long)sdRomDiag.failures,
    (unsigned long)sdRomDiag.cacheHits, (unsigned long)sdRomDiag.cacheMisses,
    (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
}
static bool sdMount() {
  if (sdMounted && !sdIoFault) return true;
  if (sdMounted && sdIoFault) { SD_MMC.end(); sdMounted = false; }
  // Pins must be (re)assigned while the card is unmounted. The target 1.54-inch board
  // uses the schematic mapping declared in the main .ino.
#if SD_1BIT
  if (!SD_MMC.setPins(PIN_SD_CLK, PIN_SD_CMD, PIN_SD_D0, -1, -1, -1)) {
    Serial.println("SD: setPins failed (1-bit bus)");
    strlcpy(loadErr1, "SD PIN CONFIG", sizeof(loadErr1));
    strlcpy(loadErr2, "FAILED", sizeof(loadErr2));
    return false;
  }
#else
  if (!SD_MMC.setPins(PIN_SD_CLK, PIN_SD_CMD, PIN_SD_D0, PIN_SD_D1, PIN_SD_D2, PIN_SD_D3)) {
    Serial.println("SD: setPins failed (4-bit bus)");
    strlcpy(loadErr1, "SD PIN CONFIG", sizeof(loadErr1));
    strlcpy(loadErr2, "FAILED", sizeof(loadErr2));
    return false;
  }
#endif
  Serial.printf("SD: mounting on CLK=%d CMD=%d D0=%d bus=%s freq=%u kHz\n",
                PIN_SD_CLK, PIN_SD_CMD, PIN_SD_D0, SD_1BIT ? "1-bit" : "4-bit",
                (unsigned)SD_FREQ_KHZ);
  if (!SD_MMC.begin("/sdcard", SD_1BIT != 0, false, SD_FREQ_KHZ)) {
    Serial.println("SD: mount failed; check FAT32 format, card seating, and board pinout");
    strlcpy(loadErr1, "NO SD CARD", sizeof(loadErr1));
    strlcpy(loadErr2, "CHECK FAT32", sizeof(loadErr2));
    return false;
  }
  sdMounted = true;
  uint8_t cardType = SD_MMC.cardType();
  Serial.printf("SD: mounted OK, type=%u size=%llu MB\n", (unsigned)cardType,
                (unsigned long long)(SD_MMC.cardSize() / (1024ULL * 1024ULL)));
  sdIoFault = false;
  return true;
}
static void sdUnmount() {
  if (!sdMounted) return;
  SD_MMC.end();
  sdMounted = false;
}

// Create a directory only when it is missing. Parent directories are created
// first by sdInitFolders(), since SD_MMC.mkdir() may not create parents itself.
static bool sdEnsureDir(const char *path) {
  if (SD_MMC.exists(path)) {
    File existing = SD_MMC.open(path);
    const bool isDir = existing && existing.isDirectory();
    if (existing) existing.close();
    if (isDir) return true;
    Serial.printf("SD: path exists but is not a directory: %s\n", path);
    return false;
  }
  // Retry once: the first filesystem operation immediately after mounting can
  // fail on some cards/controllers while the card finishes becoming ready.
  for (int attempt = 0; attempt < 2; ++attempt) {
    if (SD_MMC.mkdir(path)) {
      Serial.printf("SD: created %s\n", path);
      return true;
    }
    // Some SD implementations report mkdir failure if the directory now exists;
    // verify the final state before treating it as an error.
    File created = SD_MMC.open(path);
    const bool isDir = created && created.isDirectory();
    if (created) created.close();
    if (isDir) return true;
    delay(20);
  }
  Serial.printf("SD: could not create directory: %s\n", path);
  return false;
}

// Called during startup after SPI initialization. Creates all expected ROM and
// BIOS folders so a fresh card is ready for ROMs/BIOS files without manual setup.
// If no SD card is present, startup continues and normal SD error handling applies.
// Build a system-specific, ROM-specific battery-save path. The basename alone
// is not unique: two folders can contain different games named "game.gb".
// A stable FNV-1a suffix derived from the full ROM path keeps their saves apart
// while retaining a readable filename. ASCII case is folded because FAT paths
// are case-insensitive on the target SD card.
static bool sdBuildSavePath(const char *romPath, const char *systemDir,
                            char *out, size_t outSize) {
  if (!romPath || !romPath[0] || !systemDir || !systemDir[0] || !out || !outSize) return false;
  const char *base = strrchr(romPath, '/');
  base = base ? base + 1 : romPath;
  if (!base[0]) return false;
  size_t n = strlen(base);
  const char *dot = strrchr(base, '.');
  if (dot && dot != base) n = (size_t)(dot - base);
  uint32_t hash = 2166136261u;
  for (const unsigned char *p = (const unsigned char *)systemDir; *p; ++p) {
    unsigned char c = *p;
    if (c >= 'A' && c <= 'Z') c = (unsigned char)(c + ('a' - 'A'));
    hash = (hash ^ c) * 16777619u;
  }
  hash = (hash ^ '/') * 16777619u;
  for (const unsigned char *p = (const unsigned char *)romPath; *p; ++p) {
    unsigned char c = *p;
    if (c >= 'A' && c <= 'Z') c = (unsigned char)(c + ('a' - 'A'));
    hash = (hash ^ c) * 16777619u;
  }
  int written = snprintf(out, outSize, "/saves/%s/%.*s_%08lx.sav", systemDir,
                         (int)n, base, (unsigned long)hash);
  return written > 0 && (size_t)written < outSize;
}

// Build the previous central save path for migration, but only for ROMs placed
// directly in their system root. For nested paths, importing a basename-only
// save could silently attach another game's progress to this ROM.
static bool sdBuildLegacyManagedSavePath(const char *romPath, const char *systemDir,
                                         char *out, size_t outSize) {
  if (!romPath || !systemDir || !out || !outSize) return false;
  char expected[48];
  int rootLen = snprintf(expected, sizeof(expected), "/roms/%s/", systemDir);
  if (rootLen <= 0 || (size_t)rootLen >= sizeof(expected) || strncmp(romPath, expected, (size_t)rootLen) != 0) return false;
  const char *base = romPath + rootLen;
  if (!base[0] || strchr(base, '/')) return false;
  size_t n = strlen(base);
  const char *dot = strrchr(base, '.');
  if (dot && dot != base) n = (size_t)(dot - base);
  int written = snprintf(out, outSize, "/saves/%s/%.*s.sav", systemDir, (int)n, base);
  return written > 0 && (size_t)written < outSize;
}

// Construct the former beside-ROM path to allow existing saves to be imported
// automatically on first launch after this directory-layout change.
static bool sdBuildLegacySavePath(const char *romPath, char *out, size_t outSize) {
  if (!romPath || !romPath[0] || !out || !outSize) return false;

  // Build the destination directly and check the final length. Replacing a
  // short extension (for example ".gb") with ".sav" can require more bytes
  // than the original path, so copying first and using strcpy() at the dot
  // could write one or more bytes past the caller's buffer.
  const char *slash = strrchr(romPath, '/');
  const char *dot = strrchr(romPath, '.');
  if (dot && (!slash || dot > slash)) {
    size_t prefixLen = (size_t)(dot - romPath);
    if (prefixLen > outSize || sizeof(".sav") > outSize - prefixLen) return false;
    memcpy(out, romPath, prefixLen);
    memcpy(out + prefixLen, ".sav", sizeof(".sav"));
    return true;
  }

  size_t pathLen = strlen(romPath);
  if (pathLen > outSize || sizeof(".sav") > outSize - pathLen) return false;
  memcpy(out, romPath, pathLen);
  memcpy(out + pathLen, ".sav", sizeof(".sav"));
  return true;
}

// Save-state files are separate from battery saves. Their names use the same
// stable ROM-path hash as battery saves so same-named games cannot collide.
struct SdStateHeader { char magic[8]; uint16_t version, system; uint32_t coreSize, memSize, ramSize; uint32_t crc32; };
struct SdStateHeaderV1 { char magic[8]; uint16_t version, system; uint32_t coreSize, memSize, ramSize; };
static uint32_t sdStateCrc32Update(uint32_t crc, const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xEDB88320UL & (uint32_t)-(int32_t)(crc & 1));
  }
  return crc;
}
static uint32_t sdStatePayloadCrc(const void *core, uint32_t coreSize,
                                  const void *mem, uint32_t memSize,
                                  const void *ramData, uint32_t ramSize) {
  uint32_t crc = 0xFFFFFFFFUL;
  if (coreSize) crc = sdStateCrc32Update(crc, (const uint8_t *)core, coreSize);
  if (memSize) crc = sdStateCrc32Update(crc, (const uint8_t *)mem, memSize);
  if (ramSize) crc = sdStateCrc32Update(crc, (const uint8_t *)ramData, ramSize);
  return crc ^ 0xFFFFFFFFUL;
}

// Read the completed temporary state back from the SD card and verify both its
// metadata and payload checksum before replacing the previous slot. Checking
// only file size misses same-length write corruption on marginal cards.
static bool sdStateVerifyFile(const char *path, const SdStateHeader &expected, size_t expectedSize) {
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) return false;
  SdStateHeader actual{};
  bool ok = f.size() == expectedSize &&
            f.read((uint8_t *)&actual, sizeof(actual)) == sizeof(actual) &&
            memcmp(actual.magic, expected.magic, sizeof(actual.magic)) == 0 &&
            actual.version == expected.version && actual.system == expected.system &&
            actual.coreSize == expected.coreSize && actual.memSize == expected.memSize &&
            actual.ramSize == expected.ramSize && actual.crc32 == expected.crc32;
  uint8_t chunk[128];
  uint32_t remaining = expected.coreSize + expected.memSize + expected.ramSize;
  uint32_t crc = 0xFFFFFFFFUL;
  while (ok && remaining) {
    size_t want = remaining < sizeof(chunk) ? remaining : sizeof(chunk);
    size_t got = f.read(chunk, want);
    if (got != want) { ok = false; break; }
    crc = sdStateCrc32Update(crc, chunk, got);
    remaining -= (uint32_t)got;
  }
  f.close();
  crc ^= 0xFFFFFFFFUL;
  return ok && crc == expected.crc32;
}
static bool sdBuildStatePath(const char *romPath, const char *systemDir, char *out, size_t outSize, uint8_t slot = 0) {
  if (!out || !outSize) return false;
  out[0] = '\0';
  if (!romPath || !romPath[0] || !systemDir || !systemDir[0] || slot >= 3) return false;
  char savePath[ROM_PATH_LEN + 32];
  if (!sdBuildSavePath(romPath, systemDir, savePath, sizeof(savePath))) return false;
  int n = snprintf(out, outSize, "/states/%s/%s", systemDir, strrchr(savePath, '/') ? strrchr(savePath, '/') + 1 : savePath);
  if (n <= 0 || (size_t)n >= outSize) { out[0] = '\0'; return false; }
  char *slash = strrchr(out, '/');
  char *ext = strrchr(out, '.');
  // Only replace an extension in the filename, never a dot in a directory name.
  if (!ext || (slash && ext < slash) || ext[1] == '\0') { out[0] = '\0'; return false; }
  size_t remaining = outSize - (size_t)(ext - out);
  int tail = snprintf(ext, remaining, ".s%u.state", (unsigned)(slot + 1));
  if (tail <= 0 || (size_t)tail >= remaining) { out[0] = '\0'; return false; }
  return true;
}
static bool sdStateWrite(const char *path, uint16_t system, const void *core, uint32_t coreSize,
                         const void *mem, uint32_t memSize, const void *ramData, uint32_t ramSize) {
  sdSaveLastError[0] = '\0';
  if (!path || !core || !coreSize || (memSize && !mem) || (ramSize && !ramData)) { sdSaveSetError("INVALID STATE DATA"); return false; }
  if (!sdMount()) { sdSaveSetError("SD CARD UNAVAILABLE"); return false; }
  char tmp[208], bak[208];
  if (snprintf(tmp, sizeof(tmp), "%s.tmp", path) >= (int)sizeof(tmp) ||
      snprintf(bak, sizeof(bak), "%s.bak", path) >= (int)sizeof(bak)) { sdSaveSetError("STATE PATH TOO LONG"); return false; }
  SdStateHeader h = {{'E','M','U','S','T','A','T','E'}, 2, system, coreSize, memSize, ramSize,
                     sdStatePayloadCrc(core, coreSize, mem, memSize, ramData, ramSize)};
  SD_MMC.remove(tmp);
  File f = SD_MMC.open(tmp, FILE_WRITE);
  if (!f) { sdSaveSetError("STATE WRITE FAILED / CARD FULL"); sdMarkIoFault("state open for write"); return false; }
  bool ok = f.write((const uint8_t *)&h, sizeof(h)) == sizeof(h) &&
            f.write((const uint8_t *)core, coreSize) == coreSize &&
            (!memSize || f.write((const uint8_t *)mem, memSize) == memSize) &&
            (!ramSize || f.write((const uint8_t *)ramData, ramSize) == ramSize);
  f.flush(); f.close();
  const size_t expected = sizeof(h) + coreSize + memSize + ramSize;
  ok = ok && sdStateVerifyFile(tmp, h, expected);
  if (!ok) { sdSaveSetError("STATE WRITE/READBACK VERIFY FAILED"); sdMarkIoFault("state short write or verification"); SD_MMC.remove(tmp); return false; }
  if (SD_MMC.exists(path)) {
    SD_MMC.remove(bak);
    if (!SD_MMC.rename(path, bak)) { sdSaveSetError("STATE BACKUP FAILED"); sdMarkIoFault("state backup rename"); SD_MMC.remove(tmp); return false; }
  }
  if (!SD_MMC.rename(tmp, path)) {
    sdSaveSetError("STATE INSTALL FAILED"); sdMarkIoFault("state promote rename");
    if (SD_MMC.exists(bak)) SD_MMC.rename(bak, path);
    SD_MMC.remove(tmp); return false;
  }
  sdSaveLastError[0] = '\0';
  Serial.printf("STATE: verified %s (%u bytes, CRC32 %08lx)\n", path, (unsigned)expected, (unsigned long)h.crc32);
  return true;
}
// File.read() may return fewer bytes than requested. Keep reading until the
// requested state data is complete, or fail cleanly on EOF/I/O failure.
static bool sdStateReadExact(File &f, uint8_t *dst, size_t length) {
  if (length && !dst) return false;
  size_t total = 0;
  while (total < length) {
    size_t got = f.read(dst + total, length - total);
    if (!got) return false;
    total += got;
  }
  return true;
}

static bool sdStateRead(const char *path, uint16_t system, void *core, uint32_t coreSize,
                        void *mem, uint32_t memSize, void *ramData, uint32_t ramSize) {
  sdSaveLastError[0] = '\0';
  if (!path || !core || !coreSize || (memSize && !mem) || (ramSize && !ramData)) { sdSaveSetError("INVALID STATE DATA"); return false; }
  if (!sdMount()) { sdSaveSetError("SD CARD UNAVAILABLE"); return false; }
  char bak[ROM_PATH_LEN + 16]; if (snprintf(bak, sizeof(bak), "%s.bak", path) >= (int)sizeof(bak)) { sdSaveSetError("STATE PATH TOO LONG"); return false; }
  const char *candidate[2] = { path, bak };
  for (int i=0;i<2;i++) {
    File f = SD_MMC.open(candidate[i], FILE_READ); if (!f) continue;
    SdStateHeaderV1 old{};
    bool ok = sdStateReadExact(f, (uint8_t *)&old, sizeof(old)) &&
      memcmp(old.magic, "EMUSTATE", 8) == 0 && (old.version == 1 || old.version == 2) && old.system == system &&
      old.coreSize == coreSize && old.memSize == memSize && old.ramSize == ramSize;
    const uint64_t payloadWide = (uint64_t)coreSize + (uint64_t)memSize + (uint64_t)ramSize;
    const size_t headerSize = old.version == 2 ? sizeof(SdStateHeader) : sizeof(SdStateHeaderV1);
    const bool payloadFits = payloadWide <= (uint64_t)SIZE_MAX &&
                             payloadWide <= (uint64_t)(SIZE_MAX - headerSize);
    const size_t payloadSize = payloadFits ? (size_t)payloadWide : 0;
    ok = ok && payloadFits && f.size() == headerSize + payloadSize;
    uint32_t expectedCrc = 0;
    if (ok && old.version == 2) {
      ok = sdStateReadExact(f, (uint8_t *)&expectedCrc, sizeof(expectedCrc));
    }

    // Stage the complete payload before touching live emulator memory. A
    // second SD read directly into core/mem/ram could fail partway through and
    // leave the running game with a half-restored state.
    uint8_t *staged = ok ? (uint8_t *)emuAlloc(payloadSize) : nullptr;
    if (ok && !staged) {
      f.close();
      sdSaveSetError("NOT ENOUGH RAM FOR STATE");
      return false;
    }
    if (ok) ok = sdStateReadExact(f, staged, payloadSize);
    uint32_t crc = ok ? (sdStateCrc32Update(0xFFFFFFFFUL, staged, payloadSize) ^ 0xFFFFFFFFUL) : 0;
    if (ok && old.version == 2 && crc != expectedCrc) {
      ok = false;
      Serial.printf("STATE: CRC32 mismatch %s (expected %08lx, got %08lx)\n", candidate[i], (unsigned long)expectedCrc, (unsigned long)crc);
    }
    f.close();
    if (ok) {
      size_t dataOffset = 0;
      memcpy(core, staged + dataOffset, coreSize); dataOffset += coreSize;
      if (memSize) { memcpy(mem, staged + dataOffset, memSize); dataOffset += memSize; }
      if (ramSize) memcpy(ramData, staged + dataOffset, ramSize);
    }
    free(staged);
    if (ok) {
      sdSaveLastError[0] = '\0';
      if (i) Serial.printf("STATE: recovered from backup %s\n", candidate[i]);
      if (old.version == 1) Serial.printf("STATE: legacy v1 state loaded without checksum: %s\n", candidate[i]);
      return true;
    }
    Serial.printf("STATE: invalid or corrupted state file: %s\n", candidate[i]);
  }
  sdSaveSetError("NO VALID STATE / CORRUPT");
  return false;
}

static bool sdInitFolders() {
  if (!sdMount()) {
    Serial.println("SD: folder setup deferred; card mount failed");
    return false;
  }
  static const char *const dirs[] = {
    "/roms", "/system",
    "/roms/nes", "/roms/gb", "/roms/gbc", "/roms/a2600", "/roms/genesis",
    "/saves", "/saves/gb", "/saves/gbc",
    "/states", "/states/gb", "/states/gbc", "/states/nes", "/states/a2600", "/states/genesis",
    // Optional per-ROM PNG cover art, organized by system.
    "/imgs", "/imgs/nes", "/imgs/gb", "/imgs/gbc", "/imgs/a2600", "/imgs/genesis",
    // Only handheld systems with optional boot ROM support have BIOS folders.
    // NES and Atari 2600 do not use a separate BIOS in these cores.
    "/bios", "/bios/gb", "/bios/gbc"
  };
  bool ok = true;
  for (size_t i = 0; i < sizeof(dirs) / sizeof(dirs[0]); ++i) {
    if (!sdEnsureDir(dirs[i])) ok = false;
  }
  if (ok) Serial.println("SD: ROM, save, state, image, and supported BIOS folders ready");
  else Serial.println("SD: folder setup incomplete; check card/filesystem");
  return ok;
}

// Compare ROM suffixes using ASCII-only case folding. This avoids depending on
// locale/library behavior for strcasecmp and handles mixed-case extensions
// (for example, Game.NES, GAME.GbC, and Atari.BIN) consistently on the device.
static char romAsciiLower(char c) {
  return (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c;
}

// Deterministic, locale-independent filename ordering. Compare ASCII letters
// without case first; if names differ only by case, use their original bytes
// as a tie-breaker so the order does not depend on SD directory enumeration.
static int romNameCompare(const char *a, const char *b) {
  if (!a) return b ? -1 : 0;
  if (!b) return 1;
  const unsigned char *ua = (const unsigned char *)a;
  const unsigned char *ub = (const unsigned char *)b;
  while (*ua && *ub) {
    const char ca = romAsciiLower((char)*ua);
    const char cb = romAsciiLower((char)*ub);
    if (ca != cb) return (unsigned char)ca < (unsigned char)cb ? -1 : 1;
    ++ua;
    ++ub;
  }
  if (*ua != *ub) return *ua ? 1 : -1;
  return strcmp(a, b);
}

static bool hasExt(const char *n, const char *ext) {
  if (!n || !ext || !n[0] || !ext[0] || n[0] == '.') return false; // ignore hidden/._mac files
  const size_t nameLen = strlen(n);
  const size_t extLen = strlen(ext);
  if (nameLen <= extLen) return false; // require a basename before the suffix
  const char *suffix = n + nameLen - extLen;
  for (size_t i = 0; i < extLen; ++i) {
    if (romAsciiLower(suffix[i]) != romAsciiLower(ext[i])) return false;
  }
  return true;
}
static bool isRomName(const char *n) {
  if (curSys == SYS_NES) return hasExt(n, ".nes");
  if (curSys == SYS_A2600) return hasExt(n, ".a26") || hasExt(n, ".bin");
  if (curSys == SYS_GENESIS) return hasExt(n, ".md") || hasExt(n, ".gen") || hasExt(n, ".bin");
  // GBC-compatible cartridges are commonly distributed with either .gbc
  // or .gb extensions. The selected system determines the emulator core, so
  // allow both extensions in the GBC folder; keep .gbc out of the GB folder.
  // Extension matching is case-insensitive via hasExt().
  if (curSys == SYS_GBC) return hasExt(n, ".gbc") || hasExt(n, ".gb");
  return hasExt(n, ".gb");
}

static void freeRomList() {
  // Filename pointers live inside the arena blocks; free each block once.
  RomNameBlock *block = romNameBlocks;
  while (block) {
    RomNameBlock *next = block->next;
    free(block);
    block = next;
  }
  romNameBlocks = romNameTail = nullptr;
  free(romNames);
  romNames = nullptr;
  romCount = 0;
  romCapacity = 0;
  romListTruncated = false;
}

// Grow the ROM list on demand instead of silently stopping at a fixed count.
// realloc preserves the existing entries if allocation succeeds; on failure
// the browser keeps the entries already collected and displays a warning.
static bool reserveRomList(int needed) {
  if (needed <= romCapacity) return true;
  int nextCapacity = romCapacity > 0 ? romCapacity : ROM_LIST_INITIAL_CAPACITY;
  while (nextCapacity < needed) {
    if (nextCapacity > INT_MAX / 2) { nextCapacity = needed; break; }
    nextCapacity *= 2;
  }
  if ((size_t)nextCapacity > SIZE_MAX / sizeof(*romNames)) return false;
  void *grown = realloc(romNames, (size_t)nextCapacity * sizeof(*romNames));
  if (!grown) return false;
  romNames = (char **)grown;
  romCapacity = nextCapacity;
  return true;
}

// Fill romNames with the current contents of curSys' folder. Preserve the
// selected filename across rescans (rather than only preserving its numeric
// index), so adding/removing/sorting files cannot silently move the cursor to
// a different game. If the selected file disappeared, keep the old index and
// clamp it to the refreshed list below.
static bool scanRoms() {
  char previousSelection[ROM_NAME_LEN] = {0};
  const int previousIndex = menuSel[curSys];
  if (romNames && previousIndex >= 0 && previousIndex < romCount) {
    strlcpy(previousSelection, romNames[previousIndex], sizeof(previousSelection));
  }

  // Allocate the replacement pointer table before releasing the current list.
  // If RAM is exhausted, keep the previous usable list rather than blanking it.
  const int initialCapacity = ROM_LIST_INITIAL_CAPACITY;
  if ((size_t)initialCapacity > SIZE_MAX / sizeof(*romNames)) {
    strlcpy(loadErr1, "NOT ENOUGH RAM", sizeof(loadErr1));
    strlcpy(loadErr2, "ROM LIST", sizeof(loadErr2));
    return false;
  }
  char **freshNames = (char **)malloc((size_t)initialCapacity * sizeof(*freshNames));
  if (!freshNames) {
    strlcpy(loadErr1, "NOT ENOUGH RAM", sizeof(loadErr1));
    strlcpy(loadErr2, "OLD LIST KEPT", sizeof(loadErr2));
    return false;
  }
  freeRomList();
  romNames = freshNames;
  romCapacity = initialCapacity;

  const char *dirPath = sysDir[curSys];
  File dir = SD_MMC.open(dirPath);
  if (!dir || !dir.isDirectory()) {
    // A missing system folder is different from a card that vanished. Probe the
    // root once; failure marks the mount stale so the caller can recover.
    File rootProbe = SD_MMC.open("/");
    const bool rootOk = rootProbe && rootProbe.isDirectory();
    if (rootProbe) rootProbe.close();
    if (!rootOk) {
      sdMarkIoFault("ROM directory scan");
      strlcpy(loadErr1, "SD CARD LOST", sizeof(loadErr1));
      strlcpy(loadErr2, "REINSERT CARD", sizeof(loadErr2));
    } else {
      if (dir) dir.close();
      strlcpy(loadErr1, "NO ROM FOLDER", sizeof(loadErr1));
      strlcpy(loadErr2, dirPath, sizeof(loadErr2));
    }
    freeRomList();
    return false;
  }
  // A valid directory means the previous scan error is no longer current.
  // Clear it before scanning; this scan will set a fresh message if it finds
  // no ROMs or runs into an allocation problem.
  loadErr1[0] = '\0';
  loadErr2[0] = '\0';

  for (File e = dir.openNextFile(); e; e = dir.openNextFile()) {
    if (!e.isDirectory()) {
      const char *n = e.name();
      const char *slash = strrchr(n, '/');
      if (slash) n = slash + 1;
      if (isRomName(n)) {
        // Long supported ROM filenames cannot fit in the fixed name buffer.
        // Report the incomplete list instead of silently hiding those games.
        if (strlen(n) >= ROM_NAME_LEN) {
          romListTruncated = true;
        } else if (reserveRomList(romCount + 1)) {
          char *nameCopy = copyRomName(n);
          if (nameCopy) {
            romNames[romCount++] = nameCopy;
          } else {
            romListTruncated = true;
          }
        } else {
          romListTruncated = true;
        }
      }
    }
    e.close();
  }
  dir.close();

  // In-place heapsort: O(n log n) worst-case time and O(1) extra memory.
  // This avoids insertion sort's quadratic scan time for large ROM libraries.
  if (romCount > 1) {
    // Build a max heap using the same case-insensitive/natural comparator.
    for (int start = romCount / 2 - 1; start >= 0; --start) {
      int root = start;
      while (root < romCount / 2) {
        int child = root * 2 + 1;
        if (child + 1 < romCount && romNameCompare(romNames[child], romNames[child + 1]) < 0)
          ++child;
        if (romNameCompare(romNames[root], romNames[child]) >= 0) break;
        char *tmp = romNames[root];
        romNames[root] = romNames[child];
        romNames[child] = tmp;
        root = child;
      }
    }

    // Repeatedly move the largest entry to the end, restoring the heap.
    for (int end = romCount - 1; end > 0; --end) {
      char *tmp = romNames[0];
      romNames[0] = romNames[end];
      romNames[end] = tmp;
      int root = 0;
      while (root < end / 2) {
        int child = root * 2 + 1;
        if (child + 1 < end && romNameCompare(romNames[child], romNames[child + 1]) < 0)
          ++child;
        if (romNameCompare(romNames[root], romNames[child]) >= 0) break;
        tmp = romNames[root];
        romNames[root] = romNames[child];
        romNames[child] = tmp;
        root = child;
      }
    }
  }
  // A fresh scan happens whenever the browser is entered (including after
  // leaving a game or returning from a failed launch). Keep selection attached
  // to the same filename when possible; otherwise clamp the old index so it
  // remains valid after files are added or removed on the SD card.
  if (romCount > 0) {
    int restored = -1;
    if (previousSelection[0]) {
      for (int i = 0; i < romCount; ++i) {
        if (romNameCompare(romNames[i], previousSelection) == 0 ||
            strcasecmp(romNames[i], previousSelection) == 0) {
          restored = i;
          break;
        }
      }
    }
    if (restored >= 0) menuSel[curSys] = restored;
    else menuSel[curSys] = constrain(previousIndex, 0, romCount - 1);
  } else {
    menuSel[curSys] = 0;
  }

  Serial.printf("Found %d %s ROM(s) in %s%s\n", romCount, sysName[curSys], dirPath,
                romListTruncated ? " (LIST INCOMPLETE: LOW RAM)" : "");
  if (romListTruncated) {
    strlcpy(loadErr1, "ROM LIST INCOMPLETE", sizeof(loadErr1));
    strlcpy(loadErr2, "LOW RAM - SOME HIDDEN", sizeof(loadErr2));
  }
  if (romCount == 0) {
    // Keep the empty list allocated and let romMenu() display a persistent,
    // actionable empty state. Missing folders and allocation failures remain
    // errors handled by the caller.
    strlcpy(loadErr1, "NO ROMS FOUND", sizeof(loadErr1));
    strlcpy(loadErr2, dirPath, sizeof(loadErr2));
  }
  return true;
}

// file name without its extension
static void baseName(const char *src, char *dst, size_t cap) {
  // A zero-sized destination must be a no-op; cap - 1 would otherwise
  // underflow and allow an oversized memcpy into an invalid buffer.
  if (!src || !dst || cap == 0) return;
  size_t n = strlen(src);
  const char *dot = strrchr(src, '.');
  if (dot && dot > src) n = dot - src;
  if (n > cap - 1) n = cap - 1;
  memcpy(dst, src, n);
  dst[n] = 0;
}

