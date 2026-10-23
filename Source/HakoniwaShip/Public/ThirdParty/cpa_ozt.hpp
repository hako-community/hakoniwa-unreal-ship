#pragma once

#include <cmath>
#include <vector>
#include <algorithm>
#include <string>
#include <utility>

namespace hako::ship::autonomy {

static constexpr double PI_AUTONOMY = 3.14159265358979323846;
static constexpr double NM_TO_M = 1852.0;
static constexpr double M_TO_NM = 1.0 / 1852.0;
static constexpr double KT_TO_MPS = 0.514444;
static constexpr double MPS_TO_KT = 1.943844;
static constexpr double DEG_TO_RAD_A = 3.14159265358979323846 / 180.0;
static constexpr double RAD_TO_DEG_A = 180.0 / 3.14159265358979323846;

/**
 * @brief 2D ベクトル
 */
struct Vector2D {
    double x {0.0};
    double y {0.0};

    Vector2D operator+(const Vector2D& o) const { return {x + o.x, y + o.y}; }
    Vector2D operator-(const Vector2D& o) const { return {x - o.x, y - o.y}; }
    Vector2D operator*(double s) const { return {x * s, y * s}; }
    double dot(const Vector2D& o) const { return x * o.x + y * o.y; }
    double norm_sq() const { return x * x + y * y; }
    double norm() const { return std::sqrt(norm_sq()); }
};

/**
 * @brief 最接近諸元 (CPA / TCPA / BCR / BCT)
 */
struct CpaMetrics {
    double cpa_distance_m {0.0};             // 最接近距離 CPA [m]
    double cpa_distance_nm {0.0};            // 最接近距離 CPA [NM]
    double tcpa_seconds {0.0};               // 最接近時間 TCPA [s]
    double tcpa_minutes {0.0};               // 最接近時間 TCPA [min]
    double bcr_distance_m {0.0};             // 船首横切り距離 BCR (Bow Crossing Range) [m]
    double bct_seconds {0.0};                // 船首横切り時間 BCT (Bow Crossing Time) [s]
    bool bow_cross_ahead {false};            // 自船首前方を横切るか (true: 前方, false: 船尾後方)
    bool is_closing {false};                 // 接近中フラグ (TCPA > 0)
    bool collision_risk {false};             // 衝突危険警報フラグ
};

/**
 * @brief 危険進路の連続区間 (真方位、start から時計回りに end まで。start > end は真北を跨ぐ)
 */
struct OztArc {
    double start_deg {0.0};
    double end_deg {0.0};
};

/**
 * @brief OZT (Obstacle Zone by Target: 他船による進路妨害危険領域) 算出結果
 */
struct OztSector {
    static constexpr int COURSE_STEP_DEG = 2;  // 進路スイープ刻み [deg]

    int target_id {0};
    // 全危険進路を覆う最小の時計回り扇形 [deg] (真方位 0..358)。
    // min > max は真北を跨ぐ (例: 350 -> 4)。全周が危険なら 0 -> 358。
    double min_danger_course_deg {0.0};
    double max_danger_course_deg {0.0};
    bool has_danger {false};                 // 危険領域存在フラグ
    std::vector<int> danger_courses_deg;     // 危険進路 (COURSE_STEP_DEG 刻み、昇順)
    std::vector<OztArc> danger_arcs;         // 危険進路の連続区間 (真北跨ぎは 1 区間)
    std::vector<Vector2D> danger_polygon;    // 衝突予測座標列 (danger_courses_deg と同順、ワールド直交座標系 [m])

    /**
     * @brief 進路 course_deg (任意の角度) が危険進路か。最も近い COURSE_STEP_DEG 刻みの進路で判定する
     */
    bool contains_course(double course_deg) const {
        double c = std::fmod(course_deg, 360.0);
        if (c < 0.0) c += 360.0;
        const int bin = (static_cast<int>(std::lround(c / COURSE_STEP_DEG)) * COURSE_STEP_DEG) % 360;
        return std::binary_search(danger_courses_deg.begin(), danger_courses_deg.end(), bin);
    }

    /**
     * @brief danger_courses_deg (昇順) から danger_arcs と min/max (被覆扇形) を求める
     */
    void build_arcs() {
        danger_arcs.clear();
        has_danger = !danger_courses_deg.empty();
        if (!has_danger) {
            min_danger_course_deg = max_danger_course_deg = 0.0;
            return;
        }
        const int n = static_cast<int>(danger_courses_deg.size());
        if (n * COURSE_STEP_DEG >= 360) {
            danger_arcs.push_back({0.0, 360.0 - COURSE_STEP_DEG});
            min_danger_course_deg = 0.0;
            max_danger_course_deg = 360.0 - COURSE_STEP_DEG;
            return;
        }
        int start = danger_courses_deg[0];
        int prev = start;
        std::vector<std::pair<int, int>> runs;
        for (int i = 1; i < n; ++i) {
            const int c = danger_courses_deg[i];
            if (c - prev != COURSE_STEP_DEG) {
                runs.push_back({start, prev});
                start = c;
            }
            prev = c;
        }
        runs.push_back({start, prev});
        // 0° から始まる区間と 358° で終わる区間は真北を跨ぐ 1 区間
        if (runs.size() > 1 && runs.front().first == 0 && runs.back().second == 360 - COURSE_STEP_DEG) {
            runs.front().first = runs.back().first;
            runs.pop_back();
        }
        // 被覆扇形: 区間同士の隙間 (時計回り) が最大の箇所の反対側を開始とする
        size_t widest = 0;
        int widest_gap = -1;
        for (size_t i = 0; i < runs.size(); ++i) {
            const int end = runs[i].second;
            const int next_start = runs[(i + 1) % runs.size()].first;
            const int gap = ((next_start - end) % 360 + 360) % 360;  // 単一区間なら 360 - 幅
            if (gap > widest_gap) {
                widest_gap = gap;
                widest = i;
            }
        }
        for (const auto& r : runs) {
            danger_arcs.push_back({static_cast<double>(r.first), static_cast<double>(r.second)});
        }
        min_danger_course_deg = runs[(widest + 1) % runs.size()].first;
        max_danger_course_deg = runs[widest].second;
    }
};

/**
 * @brief CPA / TCPA / OZT 計算エンジン
 */
class CpaOztCalculator {
public:
    /**
     * @brief 自船と他船の動態から CPA / TCPA / BCR を厳密計算
     * @param own_pos 自船位置 (ENU直交座標 [m])
     * @param own_vel 自船対地速度 (ENU速度 [m/s])
     * @param own_heading_rad 自船真船首方位 [rad] (0: 北, PI/2: 東)
     * @param tgt_pos 他船位置 (ENU直交座標 [m])
     * @param tgt_vel 他船対地速度 (ENU速度 [m/s])
     * @param cpa_alarm_dist_m 衝突危険 CPA 閾値 [m] (例: 0.5 NM = 926m)
     * @param tcpa_alarm_time_s 衝突危険 TCPA 閾値 [s] (例: 15 min = 900s)
     */
    static CpaMetrics compute_cpa(
        const Vector2D& own_pos, const Vector2D& own_vel, double own_heading_rad,
        const Vector2D& tgt_pos, const Vector2D& tgt_vel,
        double cpa_alarm_dist_m = 926.0, double tcpa_alarm_time_s = 900.0)
    {
        CpaMetrics res;

        Vector2D p_rel = tgt_pos - own_pos; // 相対位置 (自船から見た他船)
        Vector2D v_rel = tgt_vel - own_vel; // 相対速度

        double v_rel_sq = v_rel.norm_sq();
        if (v_rel_sq < 1e-4) {
            // 相対速度がほぼゼロ (同速・同コース航行)
            res.cpa_distance_m = p_rel.norm();
            res.cpa_distance_nm = res.cpa_distance_m * M_TO_NM;
            res.tcpa_seconds = 0.0;
            res.tcpa_minutes = 0.0;
            res.is_closing = false;
            res.collision_risk = (res.cpa_distance_m <= cpa_alarm_dist_m);
            return res;
        }

        // 最接近時間 TCPA = -(P_rel . V_rel) / |V_rel|^2
        double p_dot_v = p_rel.dot(v_rel);
        double tcpa = -p_dot_v / v_rel_sq;

        res.tcpa_seconds = tcpa;
        res.tcpa_minutes = tcpa / 60.0;
        res.is_closing = (tcpa > 0.0);

        // 最接近位置 P_cpa = P_rel + V_rel * t_cpa
        Vector2D p_cpa = p_rel + v_rel * tcpa;
        res.cpa_distance_m = p_cpa.norm();
        res.cpa_distance_nm = res.cpa_distance_m * M_TO_NM;

        // 衝突警報判定 (接近中 かつ 設定時間以内 かつ CPA閾値未満)
        if (res.is_closing && res.tcpa_seconds <= tcpa_alarm_time_s && res.cpa_distance_m <= cpa_alarm_dist_m) {
            res.collision_risk = true;
        }

        // BCR (Bow Crossing Range) の計算
        // 自船首方向単位ベクトル (ENU: 北=y, 東=x)
        Vector2D bow_dir = {std::sin(own_heading_rad), std::cos(own_heading_rad)};
        // 他船の進路直線: P_tgt(t) = tgt_pos + tgt_vel * t
        // 自船首線: P_own(s) = own_pos + bow_dir * s (s > 0: 前方, s < 0: 後方)
        // 交点条件: tgt_pos + tgt_vel * t = own_pos + bow_dir * s
        // -> bow_dir * s - tgt_vel * t = tgt_pos - own_pos = p_rel
        double det = bow_dir.x * (-tgt_vel.y) - bow_dir.y * (-tgt_vel.x);
        if (std::abs(det) > 1e-4) {
            // 連立方程式求解 (Cramer's Rule)
            double s = (p_rel.x * (-tgt_vel.y) - p_rel.y * (-tgt_vel.x)) / det;
            double t = (bow_dir.x * p_rel.y - bow_dir.y * p_rel.x) / det;
            res.bcr_distance_m = s;
            res.bct_seconds = t;
            res.bow_cross_ahead = (s > 0.0);
        } else {
            res.bcr_distance_m = 1e6;
            res.bct_seconds = 1e6;
            res.bow_cross_ahead = false;
        }

        return res;
    }

    /**
     * @brief OZT (Obstacle Zone by Target: 危険進路帯および衝突危険ポリゴン) の算出
     * @param own_pos 自船位置 [m]
     * @param own_speed_mps 自船速力 [m/s]
     * @param tgt_pos 他船位置 [m]
     * @param tgt_vel 他船対地速度 [m/s]
     * @param safe_domain_m 安全離隔ドメイン半径 [m] (例: 0.5 NM = 926m)
     * @param time_horizon_s 予測時間範囲 [s] (例: 15 min = 900s)
     * @return OZT 危険セクターおよびポリゴン
     */
    static OztSector compute_ozt(
        const Vector2D& own_pos, double own_speed_mps,
        const Vector2D& tgt_pos, const Vector2D& tgt_vel,
        double safe_domain_m = 926.0, double time_horizon_s = 900.0)
    {
        OztSector sector;
        if (own_speed_mps <= 0.1) return sector;

        // 全進路角 0°〜360° を 2° 刻みでスイープ
        for (int deg = 0; deg < 360; deg += OztSector::COURSE_STEP_DEG) {
            double course_rad = deg * DEG_TO_RAD_A;
            // 仮定する自船速度ベクトル (ENU: 北=cos, 東=sin)
            Vector2D test_own_vel = {
                own_speed_mps * std::sin(course_rad),
                own_speed_mps * std::cos(course_rad)
            };

            auto metrics = compute_cpa(
                own_pos, test_own_vel, course_rad,
                tgt_pos, tgt_vel,
                safe_domain_m, time_horizon_s);

            if (metrics.collision_risk) {
                sector.danger_courses_deg.push_back(deg);
                // 衝突予測座標 P_col = own_pos + test_own_vel * tcpa
                sector.danger_polygon.push_back(own_pos + test_own_vel * metrics.tcpa_seconds);
            }
        }

        sector.build_arcs();
        return sector;
    }
};

} // namespace hako::ship::autonomy
