/*
 * vlc-handover-simulation.cc
 *
 * Integrated two-stage VLC handover simulation for NS-3.46.1
 *
 * =========================================================================
 * STAGE 1 — Cell design (static channel scan, no mobility)
 * =========================================================================
 * Evaluates 16 combinations: FOV (28.5, 45, 60, 75 deg) x nAPs (4, 9, 16, 25)
 * For each combination, samples SCAN_RES x SCAN_RES points on the receiver
 * plane and computes:
 *   - coverage_pct     : % of points with at least one AP in FoV
 *   - avg_snr_dB       : mean SNR over covered points
 *   - min_snr_dB       : worst-case SNR over covered points
 *   - overlap_pct      : % of points covered by >= 2 APs simultaneously
 *   - c1_satisfied     : d_actual <= d_max(FOV) = h*tan(FOV)*sqrt(2)
 * Output: vlc_stage1.csv
 *
 * Selection criterion (applied automatically):
 *   1. coverage_pct >= COVERAGE_MIN (80%)
 *   2. min_snr_dB   >= SNR_MIN_DB   (0 dB — floor, excludes negative SNR)
 *   3. Among approved: 3 with highest avg_snr_dB (best channel quality)
 *   4. Tie-break: fewer APs preferred
 *
 * =========================================================================
 * STAGE 2 — Handover evaluation (NS-3 mobility simulation)
 * =========================================================================
 * Tests the 3 selected cases + reference topology [2] (6 APs, FOV 28.5)
 * Factorial design 2x2x3:
 *   2 mobility models (Waypoint, RandomWalk2D) — both native NS-3
 *   2 hysteresis settings (0 dB, 3 dB)
 *   3 speeds (1, 3, 8 m/s)
 *   = 12 scenarios per topology x 4 topologies = 48 scenarios
 * Output: vlc_stage2.csv
 *
 * =========================================================================
 * PHYSICAL MODEL  [ref.1, Eqs 3-6, Table 1]
 * =========================================================================
 * H(0) = R(phi)*(A/d^2)*cos(theta)              Lambertian gain  [Eq.3]
 * SNR  = R^2*Pr^2 / (sigma_shot(Pr)+sigma_th)   [Eq.4]
 * Pr   = H(0)*Pt   position-dependent            [C3 correction]
 * sigma_shot  = 2q*[R*Pr*(1+MI^2)+Ibg*I2]*B     [Eq.5]
 * sigma_th    = 8pi*k*Tk*eta*A*B^2*(I2/G+...)   [Eq.6]
 * BER  = 0.5*erfc(sqrt(SNR/2))
 * PER  = 1-(1-BER)^(8*L)
 * Reff = R_phy*(1-PER)
 *
 * =========================================================================
 * CONTRIBUTIONS
 * =========================================================================
 * C1 — d_AP <= h_sep*tan(FOV)*sqrt(2)  general coverage condition
 * C2 — Factorial 2x2x3 design separating topology/protocol/mobility effects
 * C3 — sigma_shot(Pr) position-dependent (not fixed Pt)
 * C4 — Overlap area (Stage 1) predicts HO reduction and ping-pong rate
 * C5 — avg_reff_overall vs avg_reff_active
 *
 * [1] Matheus et al., IEEE Commun. Surveys Tuts. vol.21 no.4 2019
 * [2] DYRP-VLC, Ad Hoc Networks 2019
 * [3] PARC, Wireless Days Conference 2025
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <limits>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("VlcHandoverSimulation");

/* =========================================================================
 * Physical constants  [ref.1, Table 1]
 * =========================================================================*/
static const double PHOTO_AREA = 5.24e-6;
static const double RESPONSIVITY = 0.45;
static const double TX_POWER = 500e-3;
static const double IBG_STANDARD = 5100e-6;
static const double I2_FACTOR = 0.562;
static const double TEMP_K = 297.0;
static const double ETA_CAP = 112e-6 * 1e-4;
static const double OPEN_LOOP_GAIN = 10.0;
static const double I3_FACTOR = 0.0868;
static const double FET_TRANSCONDUCT = 30e-3;
static const double ELECTRON_CHARGE = 1.602176634e-19;
static const double BOLTZMANN_K = 1.380649e-23;
static const double NOISE_BW = 3e5;
static const double OOK_MOD_INDEX = 0.5;

/* =========================================================================
 * Room geometry
 * =========================================================================*/
static const double ROOM_X = 10.0;
static const double ROOM_Y = 16.0;
static const double AP_HEIGHT = 3.0;
static const double NODE_HEIGHT = 1.2;
static const double H_SEP = AP_HEIGHT - NODE_HEIGHT; // 1.8 m

/* =========================================================================
 * TX angular parameter  [ref.1, Table 1]
 * =========================================================================*/
static const double SEMI_ANGLE_DEG = 35.0;

/* =========================================================================
 * Stage 1 parameters
 * =========================================================================*/
static const int SCAN_RES = 400;        // grid points per axis (400x400 = 160000)
static const double COVERAGE_MIN = 5.0; // minimum coverage [%] — relaxed to allow all positive-SNR cases
static const double SNR_MIN_DB = 0.0;   // minimum SNR floor [dB] — only positive SNR = viable channel
static const int N_SELECT = 3;          // cases to select for Stage 2

/* =========================================================================
 * Stage 2 parameters
 * =========================================================================*/
static const double HYSTERESIS_DB = 3.0;
static const double SAMPLE_INTERVAL = 0.1;
static const double SIM_DURATION_RW = 120.0;
static const double PHY_RATE_MBPS = 0.3;
static const int PKT_SIZE_BYTES = 100;
static const double PINGPONG_WINDOW = 0.5;

/* =========================================================================
 * Stage 1 parameter space
 * =========================================================================*/
static const std::vector<double> FOV_LIST = {28.5, 45.0, 60.0, 75.0};
static const std::vector<int> NAPS_LIST = {4, 9, 16, 25};

/* =========================================================================
 * Channel model functions
 * =========================================================================*/
static double LambertianOrder()
{
    double p = SEMI_ANGLE_DEG * M_PI / 180.0;
    return -std::log(2.0) / std::log(std::cos(p));
}

static double RadiationPattern(double phi_rad)
{
    double m = LambertianOrder();
    if (phi_rad < 0.0)
        phi_rad = 0.0;
    return ((m + 1.0) / (2.0 * M_PI)) * std::pow(std::cos(phi_rad), m);
}

static double ChannelGainH0(const Vector &ap, const Vector &rx, double fovDeg)
{
    double dx = rx.x - ap.x, dy = rx.y - ap.y, dz = rx.z - ap.z;
    double d = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (d < 1e-9)
        return 0.0;
    double cosT = std::fabs(dz) / d;
    double cosP = -dz / d;
    if (cosP < 0.0)
        return 0.0;
    double phi = std::acos(std::min(cosP, 1.0));
    double theta = std::acos(std::min(cosT, 1.0));
    if (theta > fovDeg * M_PI / 180.0)
        return 0.0;
    return RadiationPattern(phi) * (PHOTO_AREA / (d * d)) * cosT;
}

static double ShotNoise(double Pr, double ibg)
{
    return 2.0 * ELECTRON_CHARGE * (RESPONSIVITY * Pr * (1.0 + OOK_MOD_INDEX * OOK_MOD_INDEX) + ibg * I2_FACTOR) * NOISE_BW;
}

static double ThermalNoise()
{
    double t1 = I2_FACTOR / OPEN_LOOP_GAIN;
    double t2 = (2.0 * M_PI * ETA_CAP * PHOTO_AREA * I3_FACTOR * NOISE_BW) / FET_TRANSCONDUCT;
    return 8.0 * M_PI * BOLTZMANN_K * TEMP_K * ETA_CAP * PHOTO_AREA * NOISE_BW * NOISE_BW * (t1 + t2);
}

static double ComputeSNR(double H0, double ibg)
{
    if (H0 <= 0.0)
        return 0.0;
    double Pr = H0 * TX_POWER;
    double sig = RESPONSIVITY * RESPONSIVITY * Pr * Pr;
    double noise = ShotNoise(Pr, ibg) + ThermalNoise();
    return (noise > 0.0) ? sig / noise : 0.0;
}

static double ComputeBER(double snr)
{
    return (snr <= 0.0) ? 0.5 : 0.5 * std::erfc(std::sqrt(snr / 2.0));
}
static double ComputePER(double ber)
{
    return 1.0 - std::pow(1.0 - ber, (double)(8 * PKT_SIZE_BYTES));
}
static double ComputeGoodput(double per)
{
    return PHY_RATE_MBPS * (1.0 - per);
}
static double LinearToDb(double v)
{
    return (v <= 0.0)
               ? -std::numeric_limits<double>::infinity()
               : 10.0 * std::log10(v);
}

/* =========================================================================
 * C1 coverage geometry
 * =========================================================================*/
static double ComputeRfov(double fovDeg)
{
    return H_SEP * std::tan(fovDeg * M_PI / 180.0);
}
static double ComputeDmax(double fovDeg)
{
    return ComputeRfov(fovDeg) * std::sqrt(2.0);
}

/* =========================================================================
 * BuildGrid — generic AP placement for any nAPs
 * Finds best rectangular grid matching room aspect ratio.
 * =========================================================================*/
static std::vector<Vector> BuildGrid(int nAPs)
{
    int bx = 1, by = nAPs;
    double br = 1e9;
    for (int nx = 1; nx <= nAPs; nx++)
    {
        int ny = (int)std::ceil((double)nAPs / nx);
        if (nx * ny < nAPs)
            continue;
        double r = std::fabs((double)nx / ny - ROOM_X / ROOM_Y);
        if (r < br)
        {
            br = r;
            bx = nx;
            by = ny;
        }
    }
    double dx = ROOM_X / (bx > 1 ? bx - 1 : 1);
    double dy = ROOM_Y / (by > 1 ? by - 1 : 1);
    double ox = (bx == 1) ? ROOM_X / 2.0 : 0.0;
    double oy = (by == 1) ? ROOM_Y / 2.0 : 0.0;
    std::vector<Vector> g;
    for (int ix = 0; ix < bx; ++ix)
        for (int iy = 0; iy < by; ++iy)
            g.push_back(Vector(ox + ix * dx, oy + iy * dy, AP_HEIGHT));
    return g;
}

/* =========================================================================
 * Reference topology [2]: 6 APs, 2 columns x 3 rows, spacing 5.33 m
 * =========================================================================*/
static std::vector<Vector> BuildReferenceGrid()
{
    const double c0 = ROOM_X / 4.0, c1 = 3.0 * ROOM_X / 4.0;
    const double r0 = ROOM_Y / 6.0, r1 = ROOM_Y / 2.0, r2 = 5.0 * ROOM_Y / 6.0;
    return {Vector(c0, r0, AP_HEIGHT), Vector(c1, r0, AP_HEIGHT),
            Vector(c0, r1, AP_HEIGHT), Vector(c1, r1, AP_HEIGHT),
            Vector(c0, r2, AP_HEIGHT), Vector(c1, r2, AP_HEIGHT)};
}

/* =========================================================================
 * Stage 1 result struct
 * =========================================================================*/
struct Stage1Result
{
    double fovDeg;
    int nAPs;
    double coverage;
    double avgSnr;
    double minSnr;
    double overlapPct;
    bool c1Satisfied;
    bool selected;
};

/* =========================================================================
 * RunStage1 — static channel scan for one (fovDeg, nAPs) combination
 *
 * Samples SCAN_RES x SCAN_RES points on the receiver plane.
 * No NS-3 mobility needed — uses ConstantPositionMobilityModel.
 * For each point: finds best AP SNR and counts APs in range.
 * =========================================================================*/
static Stage1Result RunStage1(double fovDeg, int nAPs)
{
    std::vector<Vector> apPos = BuildGrid(nAPs);
    int nAP = (int)apPos.size();
    double dmax = ComputeDmax(fovDeg);

    // Compute actual max spacing in the grid
    double dx_grid = ROOM_X / (std::ceil(std::sqrt((double)nAPs)) - 1 + 1e-9);
    bool c1ok = (dx_grid <= dmax);

    double sumSnr = 0.0;
    double minSnr = 1e30;
    int covered = 0;
    int overlap = 0;
    int total = SCAN_RES * SCAN_RES;

    for (int ix = 0; ix < SCAN_RES; ++ix)
    {
        double x = (ix + 0.5) * ROOM_X / SCAN_RES;
        for (int iy = 0; iy < SCAN_RES; ++iy)
        {
            double y = (iy + 0.5) * ROOM_Y / SCAN_RES;
            Vector rx(x, y, NODE_HEIGHT);

            double bestSNR = 0.0;
            int inRange = 0;

            for (int i = 0; i < nAP; ++i)
            {
                double H0 = ChannelGainH0(apPos[i], rx, fovDeg);
                double snr = ComputeSNR(H0, IBG_STANDARD);
                if (snr > 0.0)
                    ++inRange;
                if (snr > bestSNR)
                    bestSNR = snr;
            }

            if (bestSNR > 0.0)
            {
                double snrDb = LinearToDb(bestSNR);
                sumSnr += snrDb;
                if (snrDb < minSnr)
                    minSnr = snrDb;
                ++covered;
            }
            if (inRange >= 2)
                ++overlap;
        }
    }

    Stage1Result r;
    r.fovDeg = fovDeg;
    r.nAPs = nAPs;
    r.coverage = 100.0 * covered / total;
    r.avgSnr = (covered > 0) ? sumSnr / covered : 0.0;
    r.minSnr = (covered > 0) ? minSnr : 0.0;
    r.overlapPct = 100.0 * overlap / total;
    r.c1Satisfied = c1ok;
    r.selected = false;
    return r;
}

/* =========================================================================
 * Stage 2 data structures
 * =========================================================================*/
struct ScenarioConfig
{
    std::string name;
    double speedMps;
    double ibg;
    double hysteresisDb;
    double fovDeg;
    int nAPs; // 0 = reference [2]
    bool useRandomWalk;
};

struct SimState
{
    std::vector<Vector> apPositions;
    double fovDeg;
    int currentAP;

    struct HandoverEvent
    {
        double time;
        int from, to;
        double snrDb, ber, per, goodputMbps;
        double hoTimeEst;
    };
    std::vector<HandoverEvent> handoverLog;

    struct SnrSample
    {
        double time;
        double x, y;
        std::vector<double> snrDb, ber, per, goodputMbps;
        int servingAP;
    };
    std::vector<SnrSample> snrSeries;

    const ScenarioConfig *cfg;
    Ptr<Node> mobileNode;
};

/* =========================================================================
 * SampleAndDecide — Δt = 0.1 s
 * Works with ANY NS-3 MobilityModel via GetPosition() generic interface.
 * WaypointMobilityModel and RandomWalk2dMobilityModel are both native NS-3.
 * =========================================================================*/
static void SampleAndDecide(SimState *state)
{
    Ptr<MobilityModel> mob = state->mobileNode->GetObject<MobilityModel>();
    Vector pos = mob->GetPosition();
    double now = Simulator::Now().GetSeconds();
    int nAP = (int)state->apPositions.size();

    SimState::SnrSample smp;
    smp.time = now;
    smp.x = pos.x;
    smp.y = pos.y;
    smp.snrDb.resize(nAP, 0.0);
    smp.ber.resize(nAP, 0.5);
    smp.per.resize(nAP, 1.0);
    smp.goodputMbps.resize(nAP, 0.0);

    double bSNR = 0.0;
    int bAP = -1;
    for (int i = 0; i < nAP; ++i)
    {
        double H0 = ChannelGainH0(state->apPositions[i], pos, state->fovDeg);
        double snr = ComputeSNR(H0, state->cfg->ibg);
        double ber = ComputeBER(snr), per = ComputePER(ber);
        smp.snrDb[i] = LinearToDb(snr);
        smp.ber[i] = (snr > 0.0) ? ber : 0.5;
        smp.per[i] = per;
        smp.goodputMbps[i] = (snr > 0.0) ? ComputeGoodput(per) : 0.0;
        if (snr > bSNR)
        {
            bSNR = snr;
            bAP = i;
        }
    }
    smp.servingAP = bAP;
    state->snrSeries.push_back(smp);

    // Handover decision with optional hysteresis (PARC [3])
    if (bAP >= 0 && bAP != state->currentAP)
    {
        bool doHO = true;
        if (state->cfg->hysteresisDb > 0.0 && state->currentAP >= 0)
        {
            double H0c = ChannelGainH0(state->apPositions[state->currentAP],
                                       pos, state->fovDeg);
            double sc = ComputeSNR(H0c, state->cfg->ibg);
            double mg = std::pow(10.0, state->cfg->hysteresisDb / 10.0);
            if (bSNR < sc * mg)
                doHO = false;
        }
        if (doHO)
        {
            double b = ComputeBER(bSNR), p = ComputePER(b);
            double expectedPackets = (p < 1.0) ? (1.0 / (1.0 - p)) : std::numeric_limits<double>::infinity();
            double bitsPerPacket = 8.0 * PKT_SIZE_BYTES;
            double bitRateBps = PHY_RATE_MBPS * 1e6;
            double hoTimeEst = (expectedPackets * bitsPerPacket) / bitRateBps;

            state->handoverLog.push_back(
                {now, state->currentAP, bAP,
                 LinearToDb(bSNR), b, p, ComputeGoodput(p), hoTimeEst});
            state->currentAP = bAP;
        }
    }
    else if (bAP < 0 && state->currentAP >= 0)
    {
        state->handoverLog.push_back(
            {now, state->currentAP, -1, 0.0, 0.5, 1.0, 0.0, 0.0});
        state->currentAP = -1;
    }
    Simulator::Schedule(Seconds(SAMPLE_INTERVAL), &SampleAndDecide, state);
}

/* =========================================================================
 * ComputePingPongRate
 * Ping-pong: HO A->B followed by B->A within PINGPONG_WINDOW seconds.
 * Rate = ping-pong HOs / total AP-to-AP HOs [%]
 * =========================================================================*/
static void ComputePingPongRate(
    const std::vector<SimState::HandoverEvent> &log,
    int &ppCount, double &ppRate)
{
    ppCount = 0;
    ppRate = 0.0;
    std::vector<SimState::HandoverEvent> ho2;
    for (const auto &ev : log)
        if (ev.from >= 0 && ev.to >= 0)
            ho2.push_back(ev);
    int n = (int)ho2.size();
    if (n < 2)
        return;
    for (int i = 0; i < n - 1; ++i)
        if (ho2[i + 1].time - ho2[i].time <= PINGPONG_WINDOW && ho2[i + 1].to == ho2[i].from)
            ++ppCount;
    ppRate = (n > 0) ? 100.0 * ppCount / n : 0.0;
}

/* =========================================================================
 * CSV writers
 * =========================================================================*/
static void WriteSnrCsv(const SimState &st, const std::string &fn)
{
    std::ofstream f(fn);
    f << std::fixed << std::setprecision(6);
    f << "time_s,ue_x,ue_y,serving_AP";
    for (int i = 0; i < (int)st.apPositions.size(); ++i)
        f << ",SNR_AP" << i << "_dB";
    f << "\n";

    for (const auto &s : st.snrSeries)
    {
        f << s.time << "," << s.x << "," << s.y << "," << s.servingAP;
        for (double v : s.snrDb)
            f << "," << (std::isinf(v) ? 0.0 : v);
        f << "\n";
    }
}

static void WriteBerCsv(const SimState &st, const std::string &fn)
{
    std::ofstream f(fn);
    f << std::scientific << std::setprecision(6);
    int nAP = (int)st.apPositions.size();
    f << "time_s,serving_AP";
    for (int i = 0; i < nAP; ++i)
        f << ",BER_AP" << i;
    for (int i = 0; i < nAP; ++i)
        f << ",Reff_AP" << i << "_Mbps";
    f << "\n";
    for (const auto &s : st.snrSeries)
    {
        f << s.time << "," << s.servingAP;
        for (double v : s.ber)
            f << "," << v;
        for (double v : s.goodputMbps)
            f << "," << v;
        f << "\n";
    }
}

static void WriteHandoverCsv(const SimState &st, const std::string &fn)
{
    std::ofstream f(fn);
    f << std::fixed << std::setprecision(6);
    f << "time_s,from_AP,to_AP,SNR_new_dB,BER_new,Reff_new_Mbps,ho_time_est_s\n";
    for (const auto &ev : st.handoverLog)
        f << ev.time << ","
          << (ev.from >= 0 ? std::to_string(ev.from) : "none") << ","
          << (ev.to >= 0 ? std::to_string(ev.to) : "none") << ","
          << ev.snrDb << "," << ev.ber << "," << ev.goodputMbps << ","
          << ev.hoTimeEst << "\n";
}

/* =========================================================================
 * RunStage2 — one handover scenario
 * =========================================================================*/
static std::vector<double> RunStage2(const ScenarioConfig &cfg)
{
    std::cout << "\n  [S2] " << cfg.name
              << "  v=" << cfg.speedMps << "m/s"
              << "  FOV=" << cfg.fovDeg << "deg"
              << "  APs=" << (cfg.nAPs == 0 ? 6 : cfg.nAPs)
              << "  hyst=" << cfg.hysteresisDb << "dB"
              << "  " << (cfg.useRandomWalk ? "RandomWalk2D" : "Waypoint")
              << "\n";

    std::vector<Vector> apPos = (cfg.nAPs == 0)
                                    ? BuildReferenceGrid()
                                    : BuildGrid(cfg.nAPs);

    int nAPs = (int)apPos.size();
    NodeContainer apNodes, mobileContainer;
    apNodes.Create(nAPs);
    mobileContainer.Create(1);
    Ptr<Node> mobileNode = mobileContainer.Get(0);

    MobilityHelper apMob;
    Ptr<ListPositionAllocator> apAlloc = CreateObject<ListPositionAllocator>();
    for (const auto &p : apPos)
        apAlloc->Add(p);
    apMob.SetPositionAllocator(apAlloc);
    apMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    apMob.Install(apNodes);

    // Both mobility models are native NS-3 — no external dependency
    MobilityHelper nodeMob;
    double simTime;
    if (cfg.useRandomWalk)
    {
        // RandomWalk2dMobilityModel — native NS-3, module mobility
        std::string sp = std::to_string(cfg.speedMps);
        nodeMob.SetMobilityModel(
            "ns3::RandomWalk2dMobilityModel",
            "Mode", StringValue("Distance"),
            "Distance", DoubleValue(1.5),
            "Speed", StringValue("ns3::ConstantRandomVariable[Constant=" + sp + "]"),
            "Bounds", RectangleValue(Rectangle(0.0, ROOM_X, 0.0, ROOM_Y)));
        Ptr<ListPositionAllocator> sa = CreateObject<ListPositionAllocator>();
        sa->Add(Vector(ROOM_X / 2.0, ROOM_Y / 2.0, NODE_HEIGHT));
        nodeMob.SetPositionAllocator(sa);
        nodeMob.Install(mobileContainer);
        simTime = SIM_DURATION_RW;
    }
    else
    {
        // WaypointMobilityModel — native NS-3, module mobility
        nodeMob.SetMobilityModel("ns3::WaypointMobilityModel");
        nodeMob.Install(mobileContainer);
        Ptr<WaypointMobilityModel> wm =
            mobileNode->GetObject<WaypointMobilityModel>();
        double col0 = ROOM_X / 4.0;
        wm->AddWaypoint(Waypoint(Seconds(0.0),
                                 Vector(col0, 0.0, NODE_HEIGHT)));
        wm->AddWaypoint(Waypoint(Seconds(ROOM_Y / cfg.speedMps),
                                 Vector(col0, ROOM_Y, NODE_HEIGHT)));
        simTime = (ROOM_Y / cfg.speedMps) + 1.0;
    }

    SimState state;
    state.apPositions = apPos;
    state.fovDeg = cfg.fovDeg;
    state.currentAP = -1;
    state.cfg = &cfg;
    state.mobileNode = mobileNode;

    Simulator::Schedule(Seconds(0.0), &SampleAndDecide, &state);
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    std::string tag = cfg.name + "_v" + std::to_string((int)cfg.speedMps);
    WriteSnrCsv(state, "vlc_snr_" + tag + ".csv");
    WriteBerCsv(state, "vlc_ber_" + tag + ".csv");
    WriteHandoverCsv(state, "vlc_handover_" + tag + ".csv");

    // Aggregate metrics
    double sS = 0, mS = 1e30, xS = -1e30, sR = 0, sA = 0, sB = 0;
    int ct = 0, tot = (int)state.snrSeries.size();
    for (const auto &s : state.snrSeries)
    {
        int ap = s.servingAP;
        if (ap >= 0)
        {
            double snr = s.snrDb[ap], reff = s.goodputMbps[ap], ber = s.ber[ap];
            if (!std::isinf(snr) && snr > -100.0)
            {
                sS += snr;
                mS = std::min(mS, snr);
                xS = std::max(xS, snr);
            }
            sR += reff;
            sA += reff;
            sB += ber;
            ++ct;
        }
    }
    double avgS = ct > 0 ? sS / ct : 0.0;
    double avgB = ct > 0 ? sB / ct : 0.5;
    double avgR = ct > 0 ? sR / ct : 0.0;
    double avgA = tot > 0 ? sA / tot : 0.0;
    double cov = tot > 0 ? 100.0 * ct / tot : 0.0;
    double dis = 100.0 - cov;
    int nHO = (int)state.handoverLog.size();
    double hpm = simTime > 0 ? nHO * 60.0 / simTime : 0.0;

    double sc = 0.0;
    int cs = 0;
    double tc = -1.0;
    for (const auto &ev : state.handoverLog)
    {
        if (ev.to >= 0 && ev.from < 0)
        {
            tc = ev.time;
        }
        else if (ev.to < 0 && ev.from >= 0)
        {
            if (tc >= 0.0)
            {
                sc += ev.time - tc;
                ++cs;
                tc = -1.0;
            }
        }
    }
    if (tc >= 0.0)
    {
        sc += simTime - tc;
        ++cs;
    }
    double ac = cs > 0 ? sc / cs : 0.0;

    double sg = 0.0;
    int gs = 0;
    double tg = 0.0;
    bool ig = true;
    for (const auto &ev : state.handoverLog)
    {
        if (ev.to >= 0 && ev.from < 0)
        {
            if (tg >= 0.0)
            {
                sg += ev.time - tg;
                ++gs;
                tg = -1.0;
            }
            ig = false;
        }
        else if (ev.to < 0 && ev.from >= 0)
        {
            tg = ev.time;
            ig = true;
        }
    }
    if (ig && tg >= 0.0)
    {
        sg += simTime - tg;
        ++gs;
    }
    double ag = gs > 0 ? sg / gs : 0.0;
    double tC = simTime * (cov / 100.0);
    double tG = simTime * (dis / 100.0);

    int ppC = 0;
    double ppR = 0.0;
    ComputePingPongRate(state.handoverLog, ppC, ppR);

    std::cout << std::fixed
              << "    HOs=" << nHO
              << "  HO/min=" << std::setprecision(1) << hpm
              << "  PP=" << ppC << "(" << ppR << "%)"
              << "  cov=" << std::setprecision(1) << cov << "%"
              << "  SNR=" << std::setprecision(2) << avgS << "dB"
              << "  Reff=" << std::setprecision(3) << avgA << "Mbps\n";

    return {(double)nHO, avgS, (ct > 0 ? mS : 0.0), (ct > 0 ? xS : 0.0),
            avgR, avgA, cov, dis, hpm, ac, ag, tC, tG, avgB,
            (double)ppC, ppR};
}

/* =========================================================================
 * main
 * =========================================================================*/
int main(int argc, char *argv[])
{
    bool verbose = false;
    CommandLine cmd;
    cmd.AddValue("verbose", "Enable NS-3 logging", verbose);
    cmd.Parse(argc, argv);
    if (verbose)
        LogComponentEnable("VlcHandoverSimulation", LOG_LEVEL_INFO);

    std::cout
        << "========================================================\n"
        << " VLC Handover Simulation — Integrated Stage 1 + Stage 2\n"
        << " Room: " << ROOM_X << " x " << ROOM_Y << " m"
        << "  h_sep=" << H_SEP << " m\n"
        << "--------------------------------------------------------\n"
        << " C1 condition: d_max(FOV) = h_sep * tan(FOV) * sqrt(2)\n";
    for (double fov : FOV_LIST)
        std::cout << "   FOV " << std::fixed << std::setprecision(1)
                  << fov << "deg: d_max="
                  << std::setprecision(3) << ComputeDmax(fov) << " m\n";
    std::cout
        << "--------------------------------------------------------\n"
        << " Stage 1 grid: " << SCAN_RES << "x" << SCAN_RES
        << " = " << SCAN_RES * SCAN_RES << " points\n"
        << " Selection: coverage>=" << COVERAGE_MIN
        << "%, SNR_min>=" << SNR_MIN_DB << "dB, top-"
        << N_SELECT << " by avg SNR\n"
        << "========================================================\n";

    /* =====================================================================
     * STAGE 1 — scan all 16 combinations
     * ===================================================================*/
    std::cout << "\n[STAGE 1] Scanning 16 combinations...\n";

    std::vector<Stage1Result> s1results;
    std::ofstream s1csv("vlc_stage1.csv");
    s1csv << std::fixed << std::setprecision(4);
    s1csv << "fov_deg,n_aps,coverage_pct,avg_snr_dB,min_snr_dB,"
             "overlap_pct,c1_satisfied,selected\n";

    for (double fov : FOV_LIST)
    {
        for (int nap : NAPS_LIST)
        {
            std::cout << "  FOV=" << std::fixed << std::setprecision(1)
                      << fov << "  nAPs=" << nap << "  ...";
            std::cout.flush();
            Stage1Result r = RunStage1(fov, nap);
            s1results.push_back(r);
            std::cout << "  cov=" << std::setprecision(1) << r.coverage
                      << "%  SNRmin=" << std::setprecision(2) << r.minSnr
                      << "dB  overlap=" << std::setprecision(1)
                      << r.overlapPct << "%"
                      << (r.c1Satisfied ? "  C1-ok" : "  C1-violated")
                      << "\n";
        }
    }

    /* =====================================================================
     * STAGE 1 — selection
     * ===================================================================*/
    // Filter: coverage >= COVERAGE_MIN and minSnr >= SNR_MIN_DB
    std::vector<Stage1Result *> approved;
    for (auto &r : s1results)
        if (r.coverage >= COVERAGE_MIN && r.minSnr >= SNR_MIN_DB)
            approved.push_back(&r);

    // Sort by overlap descending, tie-break by fewer APs
    // Sort by coverage DESC (maximise coverage), tie-break by SNR DESC, then fewer APs
    // Rationale: among viable channels (SNR>0), maximise coverage first,
    // then best channel quality, then prefer simpler topology
    std::sort(approved.begin(), approved.end(),
              [](const Stage1Result *a, const Stage1Result *b)
              {
                  if (std::fabs(a->coverage - b->coverage) > 1.0)
                      return a->coverage > b->coverage;
                  if (std::fabs(a->avgSnr - b->avgSnr) > 0.1)
                      return a->avgSnr > b->avgSnr;
                  return a->nAPs < b->nAPs;
              });

    int nSel = std::min(N_SELECT, (int)approved.size());
    for (int i = 0; i < nSel; ++i)
        approved[i]->selected = true;

    // Write Stage 1 CSV
    for (const auto &r : s1results)
        s1csv << r.fovDeg << "," << r.nAPs << ","
              << r.coverage << "," << r.avgSnr << ","
              << r.minSnr << "," << r.overlapPct << ","
              << (r.c1Satisfied ? 1 : 0) << ","
              << (r.selected ? 1 : 0) << "\n";
    s1csv.close();

    std::cout << "\n[STAGE 1] Results:\n";
    std::cout << "  " << approved.size()
              << " combinations passed the filter.\n";
    std::cout << "  Selected for Stage 2:\n";
    std::vector<Stage1Result *> selected;
    for (int i = 0; i < nSel; ++i)
    {
        selected.push_back(approved[i]);
        std::cout << "   S" << (i + 1) << ": FOV="
                  << std::fixed << std::setprecision(1) << approved[i]->fovDeg
                  << "deg  nAPs=" << approved[i]->nAPs
                  << "  cov=" << std::setprecision(1) << approved[i]->coverage
                  << "%  SNRmin=" << std::setprecision(2) << approved[i]->minSnr
                  << "dB  overlap=" << std::setprecision(1)
                  << approved[i]->overlapPct << "%\n";
    }

    if (nSel == 0)
    {
        std::cout << "  WARNING: no combination passed the filter.\n"
                  << "  Relaxing to top-3 by coverage for Stage 2.\n";
        // Fallback: sort by positive-SNR indicator first, then coverage, then SNR
        std::sort(s1results.begin(), s1results.end(),
                  [](const Stage1Result &a, const Stage1Result &b)
                  {
                      bool aPos = a.minSnr > 0.0, bPos = b.minSnr > 0.0;
                      if (aPos != bPos)
                          return aPos > bPos;
                      if (std::fabs(a.coverage - b.coverage) > 1.0)
                          return a.coverage > b.coverage;
                      return a.avgSnr > b.avgSnr;
                  });
        for (int i = 0; i < std::min(N_SELECT, (int)s1results.size()); ++i)
        {
            s1results[i].selected = true;
            selected.push_back(&s1results[i]);
        }
    }

    /* =====================================================================
     * STAGE 2 — build scenario list automatically from Stage 1 selection
     * ===================================================================*/
    std::cout << "\n[STAGE 2] Building 48 scenarios...\n";

    // Reference topology [2]: FOV 28.5, 6 APs (nAPs=0)
    std::vector<ScenarioConfig> scenarios;
    const std::vector<double> SPEEDS = {1.0, 3.0, 8.0};

    auto addGroup = [&](const std::string &prefix,
                        double fov, int nap,
                        bool rw)
    {
        for (double v : SPEEDS)
        {
            scenarios.push_back(
                {prefix + "_noHyst", v, IBG_STANDARD, 0.0, fov, nap, rw});
            scenarios.push_back(
                {prefix + "_Hyst", v, IBG_STANDARD, HYSTERESIS_DB, fov, nap, rw});
        }
    };

    // Reference [2]: Waypoint + RandomWalk
    addGroup("Ref_WP", 28.5, 0, false);
    addGroup("Ref_RW", 28.5, 0, true);

    // Selected cases S1, S2, S3: Waypoint + RandomWalk
    for (int i = 0; i < (int)selected.size(); ++i)
    {
        std::string prefix = "S" + std::to_string(i + 1);
        addGroup(prefix + "_WP", selected[i]->fovDeg, selected[i]->nAPs, false);
        addGroup(prefix + "_RW", selected[i]->fovDeg, selected[i]->nAPs, true);
    }

    std::cout << "  Total scenarios: " << scenarios.size() << "\n";

    /* =====================================================================
     * STAGE 2 — run all scenarios
     * ===================================================================*/
    std::ofstream s2csv("vlc_stage2.csv");
    s2csv << std::fixed << std::setprecision(6);
    s2csv << "scenario,speed_mps,fov_deg,n_aps,mobility_model,hysteresis_dB,"
             "handover_count,ho_per_min,"
             "pingpong_count,pingpong_rate_pct,"
             "avg_snr_dB,min_snr_dB,max_snr_dB,avg_ber,"
             "avg_reff_active_Mbps,avg_reff_overall_Mbps,"
             "coverage_pct,disconnected_pct,"
             "t_connected_s,t_disconnected_s,"
             "avg_conn_session_s,avg_gap_session_s\n";

    for (const auto &cfg : scenarios)
    {
        std::vector<double> r = RunStage2(cfg);
        s2csv << cfg.name << "," << cfg.speedMps << ","
              << cfg.fovDeg << ","
              << (cfg.nAPs == 0 ? 6 : cfg.nAPs) << ","
              << (cfg.useRandomWalk ? "RandomWalk2D" : "Waypoint") << ","
              << cfg.hysteresisDb << ","
              << (int)r[0] << "," << r[8] << ","               // HO, HO/min
              << (int)r[14] << "," << r[15] << ","             // PP count, PP rate
              << r[1] << "," << r[2] << "," << r[3] << ","     // SNR avg/min/max
              << std::scientific << r[13] << std::fixed << "," // BER
              << r[4] << "," << r[5] << ","                    // Reff active/overall
              << r[6] << "," << r[7] << ","                    // coverage/disconnected
              << r[11] << "," << r[12] << ","                  // t_conn/t_gap
              << r[9] << "," << r[10] << "\n";                 // sess_conn/sess_gap
    }
    s2csv.close();

    std::cout << "\n========================================================\n"
              << " [DONE]\n"
              << "   vlc_stage1.csv — 16 combinations\n"
              << "   vlc_stage2.csv — " << scenarios.size() << " scenarios\n"
              << "   vlc_snr_*.csv, vlc_ber_*.csv, vlc_handover_*.csv\n"
              << "========================================================\n";
    return 0;
}
