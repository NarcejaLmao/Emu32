// Optional BIOS/boot-ROM loader for SD-card based emulator cores.
// BIOS files live under /bios/<system>/; files are supplied by the user.
#pragma once

// Return false for missing files, directories, wrong-sized files, and short reads.
// Keep diagnostics here so both handheld cores report BIOS problems consistently.
static bool biosReadFile(const char *path, uint8_t *dst, size_t expectedSize) {
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) return false;
  if (f.isDirectory()) { f.close(); return false; }

  const size_t actualSize = f.size();
  if (actualSize != expectedSize) {
    Serial.printf("BIOS: invalid size for %s (got %u, expected %u bytes)\n",
                  path, (unsigned)actualSize, (unsigned)expectedSize);
    f.close();
    return false;
  }

  const int bytesRead = f.read(dst, expectedSize);
  f.close();
  if (bytesRead != (int)expectedSize) {
    Serial.printf("BIOS: short read for %s (got %d of %u bytes)\n",
                  path, bytesRead, (unsigned)expectedSize);
    return false;
  }
  return true;
}

// Try known conventional filenames in order. These boot ROMs are optional in
// the current cores, so missing/invalid files are diagnosed but don't block launch.
static bool biosLoadFirst(const char *systemDir, const char *const *names,
                          size_t nameCount, uint8_t *dst, size_t expectedSize,
                          char *loadedPath, size_t loadedPathSize) {
  char path[96];
  bool foundCandidate = false;
  for (size_t i = 0; i < nameCount; ++i) {
    snprintf(path, sizeof(path), "/bios/%s/%s", systemDir, names[i]);
    if (SD_MMC.exists(path)) foundCandidate = true;
    if (biosReadFile(path, dst, expectedSize)) {
      if (loadedPath && loadedPathSize) strlcpy(loadedPath, path, loadedPathSize);
      Serial.printf("BIOS: loaded %s (%u bytes)\n", path, (unsigned)expectedSize);
      return true;
    }
  }
  if (loadedPath && loadedPathSize) loadedPath[0] = 0;
  if (foundCandidate) {
    Serial.printf("BIOS: no valid boot ROM found in /bios/%s; expected %u bytes. Check filename and file size.\n",
                  systemDir, (unsigned)expectedSize);
  } else {
    Serial.printf("BIOS: optional boot ROM not found for %s.\n", systemDir);
    Serial.printf("BIOS: put a %u-byte file in /bios/%s/; accepted filenames:",
                  (unsigned)expectedSize, systemDir);
    for (size_t i = 0; i < nameCount; ++i) Serial.printf(" %s", names[i]);
    Serial.println();
    Serial.println("BIOS: continuing with built-in post-boot state (no boot animation). ");
  }
  return false;
}


// Pre-launch BIOS check for systems with boot-ROM support. The boot ROM is
// optional in these cores: if absent, they use their built-in post-boot state.
// If a candidate file exists but has the wrong size, reject launch so the user
// can correct the SD card instead of silently ignoring a broken BIOS.
static bool biosPreLaunchCheck(uint8_t system) {
  const char *dir = nullptr;
  const char *const *names = nullptr;
  size_t nameCount = 0;
  size_t expectedSize = 0;
  static const char *const gbNames[] = { "dmg_boot.bin", "boot.bin", "gb.bin" };
  static const char *const gbcNames[] = { "cgb_boot.bin", "cgb_bios.bin", "gbc_bios.bin", "gbc_boot.bin", "boot.bin" };

  if (system == SYS_GB) {
    dir = "gb"; names = gbNames;
    nameCount = sizeof(gbNames) / sizeof(gbNames[0]);
    expectedSize = 256;
  } else if (system == SYS_GBC) {
    dir = "gbc"; names = gbcNames;
    nameCount = sizeof(gbcNames) / sizeof(gbcNames[0]);
    expectedSize = 0x900;
  } else {
    return true; // NES and Atari 2600 do not use an external BIOS.
  }

  char path[96];
  bool candidateFound = false;
  // Validate the actual bytes, not only the directory entry's reported size.
  // A card/read error can otherwise pass the size check and the core silently
  // falls back to its post-boot state instead of using the supplied BIOS.
  uint8_t probe[0x900];
  for (size_t i = 0; i < nameCount; ++i) {
    int pathLen = snprintf(path, sizeof(path), "/bios/%s/%s", dir, names[i]);
    if (pathLen < 0 || pathLen >= (int)sizeof(path)) continue;
    if (!SD_MMC.exists(path)) continue;
    candidateFound = true;

    if (biosReadFile(path, probe, expectedSize)) {
      Serial.printf("BIOS CHECK: valid and readable %s (%u bytes)\\n",
                    path, (unsigned)expectedSize);
      return true;
    }
    Serial.printf("BIOS CHECK: unusable BIOS candidate %s\\n", path);
  }

  if (candidateFound) {
    strlcpy(loadErr1, "INVALID BIOS FILE", sizeof(loadErr1));
    snprintf(loadErr2, sizeof(loadErr2), "CHECK /BIOS/%s", dir);
    return false;
  }

  // No BIOS is supplied. This is supported by both handheld cores, but inform
  // the player before launch that the authentic boot animation will be skipped.
  Serial.printf("BIOS CHECK: no optional BIOS in /bios/%s; using post-boot state\n", dir);
  return true;
}
