#pragma once

// ★ 2026-10-03（related/devai_dll/hakodrone_unreal_drone_migration_20260930.md・A1）:
//   箱庭の共有メモリ（shakoc の hako_capi.h）を使う所は、このヘッダを通して読む。
//   * HAKO_SHM_ENABLE=1（Win64。HakoniwaPdu.Build.cs が立てる）: 従来どおり hako_capi.h を読む。何も変わらない。
//   * それ以外（Android など）: 共有メモリは無い（オンラインは WebSocket だけ・Q-U5）。同じ名前の関数が
//     「使えない」を返すだけの代用品を出し、共有メモリのクライアントは初期化で失敗して止まる（落ちない）。
#ifndef HAKO_SHM_ENABLE
#define HAKO_SHM_ENABLE 0
#endif

#if HAKO_SHM_ENABLE
#include "hako_capi.h"
#else
#include <cstddef>

typedef long long int hako_time_t;
typedef int HakoPduChannelIdType;

inline bool hako_asset_init() { return false; }
inline bool hako_asset_register_polling(const char*) { return false; }
inline bool hako_asset_unregister(const char*) { return false; }
inline int hako_asset_get_event(const char*) { return 0; }
inline hako_time_t hako_asset_get_worldtime() { return 0; }
inline void hako_asset_notify_simtime(const char*, hako_time_t) {}
inline bool hako_asset_is_pdu_created() { return false; }
inline bool hako_asset_is_pdu_dirty(const char*, const char*, HakoPduChannelIdType) { return false; }
inline bool hako_asset_is_pdu_sync_mode(const char*) { return false; }
inline bool hako_asset_is_simulation_mode() { return false; }
inline bool hako_asset_create_pdu_lchannel(const char*, HakoPduChannelIdType, size_t) { return false; }
inline bool hako_asset_read_pdu(const char*, const char*, HakoPduChannelIdType, char*, size_t) { return false; }
inline bool hako_asset_write_pdu(const char*, const char*, HakoPduChannelIdType, const char*, size_t) { return false; }
inline bool hako_asset_write_pdu_nolock(const char*, HakoPduChannelIdType, const char*, size_t) { return false; }
inline void hako_asset_notify_read_pdu_done(const char*) {}
inline void hako_asset_notify_write_pdu_done(const char*) {}
inline bool hako_asset_start_feedback(const char*, bool) { return false; }
inline bool hako_asset_stop_feedback(const char*, bool) { return false; }
inline bool hako_asset_reset_feedback(const char*, bool) { return false; }
inline int hako_simevent_get_state() { return 0; }
inline bool hako_simevent_start() { return false; }
inline bool hako_simevent_stop() { return false; }
inline bool hako_simevent_reset() { return false; }
#endif
