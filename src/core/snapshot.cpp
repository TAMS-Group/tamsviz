// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "snapshot.h"

#include <atomic>

static std::atomic<size_t> g_snapshot_count;

SnapshotBase::SnapshotBase() { g_snapshot_count++; }

SnapshotBase::SnapshotBase(const SnapshotBase &other) { g_snapshot_count++; }

SnapshotBase::~SnapshotBase() { g_snapshot_count--; }

size_t SnapshotBase::instanceCount() { return g_snapshot_count; }
