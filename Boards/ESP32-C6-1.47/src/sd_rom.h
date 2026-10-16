// SD-card mounting, ROM discovery, filename helpers and ROM-list state.
// Keep this module included after the system enum, directory defines and Arduino/SD headers.
#pragma once

static const char *const sysName[SYS_COUNT]  = { "NES", "GAME BOY", "GAME BOY COLOR", "ATARI 2600" };
static const char *const sysShort[SYS_COUNT] = { "NES", "GB", "GBC", "2600" };
static const char *const sysSub[SYS_COUNT]   = { "8-BIT CONSOLE", "HANDHELD", "COLOR HANDHELD", "VCS CONSOLE" };
static const char *const sysDir[SYS_COUNT]   = { NES_DIR, GB_DIR, GBC_DIR, A2600_DIR };

static char loadErr1[24], loadErr2[24];          // last load / scan error, shown on screen

static char (*romNames)[ROM_NAME_LEN] = nullptr;
static int   romCount = 0;
static int   menuSel[SYS_COUNT];                 // remembered per system between visits
static int   sysSel   = 0;                       // remembered main-menu choice

// ---- SD card: mounted on demand. The NES unmounts it once the ROM is in RAM;
// ---- the Game Boy keeps it mounted (battery saves, streaming big ROMs).
static bool sdMounted = false;
// Last save failure reason for serial diagnostics and user-facing messages.
static char sdSaveLastError[64] = "";
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
  char tmp[192], bak[192];
  if (snprintf(tmp, sizeof(tmp), "%s.tmp", path) >= (int)sizeof(tmp) ||
      snprintf(bak, sizeof(bak), "%s.bak", path) >= (int)sizeof(bak)) { sdSaveSetError("SAVE PATH TOO LONG"); return false; }
  SD.remove(tmp);
  File f = SD.open(tmp, FILE_WRITE);
  if (!f) { sdSaveSetError("SD WRITE FAILED / CARD FULL"); Serial.printf("SAVE: cannot open temp %s\n", tmp); return false; }
  size_t written = f.write(data, len);
  f.flush();
  f.close();
  if (written != len) {
    sdSaveSetError("SD CARD FULL OR WRITE FAILED");
    Serial.printf("SAVE: short write %u/%u bytes: %s\n", (unsigned)written, (unsigned)len, tmp);
    SD.remove(tmp);
    return false;
  }
  File check = SD.open(tmp, FILE_READ);
  bool verified = check && check.size() == len;
  if (check) check.close();
  if (!verified) {
    sdSaveSetError("SAVE VERIFY FAILED");
    Serial.printf("SAVE: verify failed: %s\n", tmp);
    SD.remove(tmp);
    return false;
  }

  // Preserve a prior backup until the new temp file is safely verified.
  bool hadMain = SD.exists(path);
  if (hadMain) SD.remove(bak);
  if (hadMain && !SD.rename(path, bak)) {
    sdSaveSetError("CANNOT BACK UP OLD SAVE");
    Serial.printf("SAVE: could not protect previous save: %s\n", path);
    SD.remove(tmp);
    return false;
  }
  if (!SD.rename(tmp, path)) {
    sdSaveSetError("CANNOT INSTALL SAVE");
    Serial.printf("SAVE: promote failed: %s\n", path);
    if (hadMain) SD.rename(bak, path);
    SD.remove(tmp);
    return false;
  }
  sdSaveLastError[0] = '\0';
  Serial.printf("SAVE: verified %s (%u bytes)\n", path, (unsigned)len);
  return true;
}

// Load a save only when its complete length matches the cartridge RAM size.
// If the primary save is missing/corrupt, recover from the last known-good .bak.
static bool sdAtomicSaveLoad(const char *path, uint8_t *data, size_t len) {
  if (!path || !path[0] || !data || !len) return false;
  char bak[192];
  if (snprintf(bak, sizeof(bak), "%s.bak", path) >= (int)sizeof(bak)) return false;
  const char *candidates[2] = { path, bak };
  for (uint8_t i = 0; i < 2; ++i) {
    File f = SD.open(candidates[i], FILE_READ);
    if (!f) continue;
    size_t size = f.size();
    if (size != len) { f.close(); Serial.printf("SAVE: invalid size %u (expected %u): %s\n", (unsigned)size, (unsigned)len, candidates[i]); continue; }
    size_t got = f.read(data, len);
    f.close();
    if (got == len) {
      Serial.printf("SAVE: loaded %s (%u bytes)%s\n", candidates[i], (unsigned)len, i ? " [backup recovery]" : "");
      return true;
    }
    Serial.printf("SAVE: short read %u/%u: %s\n", (unsigned)got, (unsigned)len, candidates[i]);
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
  if (!f || !dst || !len) { sdRomDiag.failures++; return 0; }
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
    return 0; // normalize negative reads; callers treat this as a short read
  }
  sdRomDiag.failures++;
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
  if (sdMounted) return true;
  if (!SD.begin(PIN_SD_CS, SPI, 20000000)) {
    strlcpy(loadErr1, "NO SD CARD", sizeof(loadErr1));
    strlcpy(loadErr2, "CHECK CARD", sizeof(loadErr2));
    return false;
  }
  sdMounted = true;
  return true;
}
static void sdUnmount() {
  if (!sdMounted) return;
  SD.end();
  sdMounted = false;
}

// Create a directory only when it is missing. Parent directories are created
// first by sdInitFolders(), since SD.mkdir() may not create parents itself.
static bool sdEnsureDir(const char *path) {
  if (SD.exists(path)) {
    File existing = SD.open(path);
    const bool isDir = existing && existing.isDirectory();
    if (existing) existing.close();
    if (isDir) return true;
    Serial.printf("SD: path exists but is not a directory: %s\n", path);
    return false;
  }
  // Retry once: the first filesystem operation immediately after mounting can
  // fail on some cards/controllers while the card finishes becoming ready.
  for (int attempt = 0; attempt < 2; ++attempt) {
    if (SD.mkdir(path)) {
      Serial.printf("SD: created %s\n", path);
      return true;
    }
    // Some SD implementations report mkdir failure if the directory now exists;
    // verify the final state before treating it as an error.
    File created = SD.open(path);
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
// Build a system-specific battery-save path from a ROM path. Saves are kept
// separately from ROMs: /saves/gb/<rom>.sav and /saves/gbc/<rom>.sav.
static bool sdBuildSavePath(const char *romPath, const char *systemDir,
                            char *out, size_t outSize) {
  if (!romPath || !systemDir || !out || !outSize) return false;
  const char *base = strrchr(romPath, '/');
  base = base ? base + 1 : romPath;
  if (!base[0]) return false;
  size_t n = strlen(base);
  const char *dot = strrchr(base, '.');
  if (dot && dot != base) n = (size_t)(dot - base);
  int written = snprintf(out, outSize, "/saves/%s/%.*s.sav", systemDir, (int)n, base);
  return written > 0 && (size_t)written < outSize;
}

// Construct the former beside-ROM path to allow existing saves to be imported
// automatically on first launch after this directory-layout change.
static bool sdBuildLegacySavePath(const char *romPath, char *out, size_t outSize) {
  if (!romPath || !out || !outSize) return false;
  int written = snprintf(out, outSize, "%s", romPath);
  if (written <= 0 || (size_t)written >= outSize) return false;
  char *dot = strrchr(out, '.');
  if (dot && (!strrchr(out, '/') || dot > strrchr(out, '/'))) strcpy(dot, ".sav");
  else strlcat(out, ".sav", outSize);
  return true;
}

static bool sdInitFolders() {
  if (!sdMount()) {
    Serial.println("SD: folder setup deferred; card mount failed");
    return false;
  }
  static const char *const dirs[] = {
    "/roms",
    "/roms/nes", "/roms/gb", "/roms/gbc", "/roms/a2600",
    "/saves", "/saves/gb", "/saves/gbc",
    // Only handheld systems with optional boot ROM support have BIOS folders.
    // NES and Atari 2600 do not use a separate BIOS in these cores.
    "/bios", "/bios/gb", "/bios/gbc"
  };
  bool ok = true;
  for (size_t i = 0; i < sizeof(dirs) / sizeof(dirs[0]); ++i) {
    if (!sdEnsureDir(dirs[i])) ok = false;
  }
  if (ok) Serial.println("SD: ROM, save, and supported BIOS folders ready");
  else Serial.println("SD: folder setup incomplete; check card/filesystem");
  return ok;
}

// Compare ROM suffixes using ASCII-only case folding. This avoids depending on
// locale/library behavior for strcasecmp and handles mixed-case extensions
// (for example, Game.NES, GAME.GbC, and Atari.BIN) consistently on the device.
static char romAsciiLower(char c) {
  return (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c;
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
  if (curSys == SYS_GBC) return hasExt(n, ".gbc") || hasExt(n, ".gb");
  return hasExt(n, ".gb") || hasExt(n, ".gbc");
}

static void freeRomList() { free(romNames); romNames = nullptr; romCount = 0; }

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

  freeRomList();
  romNames = (char (*)[ROM_NAME_LEN])malloc(MAX_ROMS * ROM_NAME_LEN);
  if (!romNames) {
    strlcpy(loadErr1, "NOT ENOUGH RAM", sizeof(loadErr1));
    strlcpy(loadErr2, "ROM LIST", sizeof(loadErr2));
    return false;
  }

  const char *dirPath = sysDir[curSys];
  File dir = SD.open(dirPath);
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    strlcpy(loadErr1, "NO ROM FOLDER", sizeof(loadErr1));
    strlcpy(loadErr2, dirPath, sizeof(loadErr2));
    freeRomList();
    return false;
  }
  for (File e = dir.openNextFile(); e && romCount < MAX_ROMS; e = dir.openNextFile()) {
    if (!e.isDirectory()) {
      const char *n = e.name();
      const char *slash = strrchr(n, '/');
      if (slash) n = slash + 1;
      if (isRomName(n) && strlen(n) < ROM_NAME_LEN) strcpy(romNames[romCount++], n);
    }
    e.close();
  }
  dir.close();

  for (int i = 1; i < romCount; i++) {           // case-insensitive insertion sort
    char t[ROM_NAME_LEN];
    strcpy(t, romNames[i]);
    int j = i - 1;
    while (j >= 0 && strcasecmp(romNames[j], t) > 0) { strcpy(romNames[j + 1], romNames[j]); j--; }
    strcpy(romNames[j + 1], t);
  }
  // A fresh scan happens whenever the browser is entered (including after
  // leaving a game or returning from a failed launch). Keep selection attached
  // to the same filename when possible; otherwise clamp the old index so it
  // remains valid after files are added or removed on the SD card.
  if (romCount > 0) {
    int restored = -1;
    if (previousSelection[0]) {
      for (int i = 0; i < romCount; ++i) {
        if (strcasecmp(romNames[i], previousSelection) == 0) {
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

  Serial.printf("Found %d %s ROM(s) in %s\n", romCount, sysName[curSys], dirPath);
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
  size_t n = strlen(src);
  const char *dot = strrchr(src, '.');
  if (dot && dot > src) n = dot - src;
  if (n > cap - 1) n = cap - 1;
  memcpy(dst, src, n);
  dst[n] = 0;
}

