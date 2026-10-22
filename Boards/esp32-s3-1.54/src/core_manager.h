// ============================================================================
// Emu32 core manager: stable frontend adapter for all built-in emulators.
// Core implementations remain isolated in their own headers; this layer owns
// cross-core lifecycle, frame pacing, save-state dispatch, and fault reporting.
// ============================================================================
#ifndef EMU32_CORE_MANAGER_H
#define EMU32_CORE_MANAGER_H

struct CoreDescriptor {
  uint8_t system;
  const char *name;
  uint32_t defaultFramePeriodUs;
  bool streamsRomFromSd;
  bool needsSdWhileRunning;
};

static const CoreDescriptor coreDescriptors[SYS_COUNT] = {
  { SYS_NES,   "NES",             16639, true,  true  },
  { SYS_GB,    "GAME BOY",        16742, true,  true  },
  { SYS_GBC,   "GAME BOY COLOR",  16742, true,  true  },
  { SYS_A2600, "ATARI 2600",      16688, true,  true  },
  { SYS_GENESIS, "GENESIS",       16688, true,  true  }
};

static const CoreDescriptor *coreDescriptor(uint8_t system) {
  return system < SYS_COUNT ? &coreDescriptors[system] : nullptr;
}

static void coreInitializeBuiltins() {
  nesInit();
  a26Init();
}

static bool coreLoad(uint8_t system, const char *path) {
  switch (system) {
    case SYS_NES:   return nesLoad(path);
    case SYS_GB:    return gbLoad(path);
    case SYS_GBC:   return gbcLoad(path);
    case SYS_A2600: return a26Load(path);
    case SYS_GENESIS: return genLoad(path);
    default:        return false;
  }
}

static void coreShutdown(uint8_t system) {
  switch (system) {
    case SYS_GB:    gbShutdown(); break;
    case SYS_GBC:   gbcShutdown(); break;
    case SYS_A2600: a26Shutdown(); break;
    case SYS_GENESIS: genShutdown(); break;
    case SYS_NES:   default: nesShutdown(); break;
  }
}

static void coreReset(uint8_t system) {
  switch (system) {
    case SYS_NES:   nesReset(); break;
    case SYS_GB:    gbReset(); break;
    case SYS_GBC:   gbcReset(); break;
    case SYS_A2600: a26Reset(); break;
    case SYS_GENESIS: genReset(); break;
    default: break;
  }
}

static uint32_t coreFramePeriodUs(uint8_t system) {
  if (system == SYS_A2600) return a26FrameUs; // Atari core updates this for PAL/NTSC.
  const CoreDescriptor *descriptor = coreDescriptor(system);
  return descriptor ? descriptor->defaultFramePeriodUs : 16639;
}

// Height in rows of the picture each core outputs (used by screenshot.h).
static uint16_t shotPicH(uint8_t sys) {
  switch (sys) {
    case SYS_NES:   return GAME_H;
    case SYS_GB:    return GB_OUT_H;
    case SYS_GBC:   return GBC_OUT_H;
    case SYS_A2600: return A26_OUT_H;
    case SYS_GENESIS: return GAME_H;
    default:        return 0;
  }
}

static void coreFrame(uint8_t system, bool draw) {
  switch (system) {
    case SYS_NES:   nesFrame(draw); break;
    case SYS_GB:    gbFrame(draw); break;
    case SYS_GBC:   gbcFrame(draw); break;
    case SYS_A2600: a26Frame(draw); break;
    case SYS_GENESIS: genFrame(draw); break;
    default: break;
  }
}

static bool coreSaveState(uint8_t system, uint8_t slot) {
  switch (system) {
    case SYS_NES:   return nesStateSave(slot);
    case SYS_GB:    return gbStateSave(slot);
    case SYS_GBC:   return gbcStateSave(slot);
    case SYS_A2600: return a26::stateSave(slot);
    case SYS_GENESIS: return gen::stateSave(slot);
    default:        return false;
  }
}

static bool coreLoadState(uint8_t system, uint8_t slot) {
  switch (system) {
    case SYS_NES:   return nesStateLoad(slot);
    case SYS_GB:    return gbStateLoad(slot);
    case SYS_GBC:   return gbcStateLoad(slot);
    case SYS_A2600: return a26::stateLoad(slot);
    case SYS_GENESIS: return gen::stateLoad(slot);
    default:        return false;
  }
}

// Normalize private per-core ROM-streaming errors for the frontend.
static bool coreRomReadFailed(uint8_t system) {
  switch (system) {
    case SYS_NES: return nesRomReadError;
    case SYS_GB:  return gRomReadError;
    case SYS_GBC: return gcRomReadError;
    case SYS_A2600: return a26RomReadFailed();
    case SYS_GENESIS: return genRomReadFailed();
    default:      return false;
  }
}

#endif // EMU32_CORE_MANAGER_H
