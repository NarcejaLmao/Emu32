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
static bool sdInitFolders() {
  if (!sdMount()) {
    Serial.println("SD: folder setup deferred; card mount failed");
    return false;
  }
  static const char *const dirs[] = {
    "/roms",
    "/roms/nes", "/roms/gb", "/roms/gbc", "/roms/a2600",
    // Only handheld systems with optional boot ROM support have BIOS folders.
    // NES and Atari 2600 do not use a separate BIOS in these cores.
    "/bios", "/bios/gb", "/bios/gbc"
  };
  bool ok = true;
  for (size_t i = 0; i < sizeof(dirs) / sizeof(dirs[0]); ++i) {
    if (!sdEnsureDir(dirs[i])) ok = false;
  }
  if (ok) Serial.println("SD: ROM folders and supported BIOS folders ready");
  else Serial.println("SD: folder setup incomplete; check card/filesystem");
  return ok;
}

static bool hasExt(const char *n, const char *ext) {
  size_t l = strlen(n), e = strlen(ext);
  return l > e && n[0] != '.' && strcasecmp(n + l - e, ext) == 0;     // skips ._mac files
}
static bool isRomName(const char *n) {
  if (curSys == SYS_NES) return hasExt(n, ".nes");
  if (curSys == SYS_A2600) return hasExt(n, ".a26") || hasExt(n, ".bin");
  if (curSys == SYS_GBC) return hasExt(n, ".gbc") || hasExt(n, ".gb");
  return hasExt(n, ".gb") || hasExt(n, ".gbc");
}

static void freeRomList() { free(romNames); romNames = nullptr; romCount = 0; }

// Fill romNames with the ROMs of curSys. Returns false (and sets loadErr1/2)
// when the folder is missing or empty, so the caller can go back to the main menu.
static bool scanRoms() {
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
  Serial.printf("Found %d %s ROM(s) in %s\n", romCount, sysName[curSys], dirPath);
  if (romCount == 0) {
    strlcpy(loadErr1, "NO ROMS FOUND", sizeof(loadErr1));
    strlcpy(loadErr2, dirPath, sizeof(loadErr2));
    freeRomList();
    return false;
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

