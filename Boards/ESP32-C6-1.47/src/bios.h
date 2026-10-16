// Optional BIOS/boot-ROM loader for SD-card based emulator cores.
// BIOS files live under /bios/<system>/; files are supplied by the user.
#pragma once

static bool biosReadFile(const char *path, uint8_t *dst, size_t expectedSize) {
  File f = SD.open(path, FILE_READ);
  if (!f || f.isDirectory()) { if (f) f.close(); return false; }
  const bool sizeOk = f.size() == expectedSize;
  const bool readOk = sizeOk && f.read(dst, expectedSize) == expectedSize;
  f.close();
  return readOk;
}

// Try known conventional filenames in order. A missing or invalid BIOS is not
// fatal: systems with no required BIOS continue using their normal reset path.
static bool biosLoadFirst(const char *systemDir, const char *const *names,
                          size_t nameCount, uint8_t *dst, size_t expectedSize,
                          char *loadedPath, size_t loadedPathSize) {
  char path[96];
  for (size_t i = 0; i < nameCount; ++i) {
    snprintf(path, sizeof(path), "/bios/%s/%s", systemDir, names[i]);
    if (biosReadFile(path, dst, expectedSize)) {
      if (loadedPath && loadedPathSize) strlcpy(loadedPath, path, loadedPathSize);
      Serial.printf("BIOS: loaded %s (%u bytes)\n", path, (unsigned)expectedSize);
      return true;
    }
  }
  if (loadedPath && loadedPathSize) loadedPath[0] = 0;
  Serial.printf("BIOS: none found in /bios/%s (optional)\n", systemDir);
  return false;
}
