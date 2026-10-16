// Optional BIOS/boot-ROM loader for SD-card based emulator cores.
// BIOS files live under /bios/<system>/; files are supplied by the user.
#pragma once

// Return false for missing files, directories, wrong-sized files, and short reads.
// Keep diagnostics here so both handheld cores report BIOS problems consistently.
static bool biosReadFile(const char *path, uint8_t *dst, size_t expectedSize) {
  File f = SD.open(path, FILE_READ);
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
    if (SD.exists(path)) foundCandidate = true;
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
