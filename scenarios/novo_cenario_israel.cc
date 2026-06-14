/*
 * vlc-handover-simulation.cc
 *
 * Simulation of handover dynamics in mobile Visible Light Communication (VLC)
 * networks using the NS-3 simulator with the VLC module.
 *
 * Network topology: 6 APs arranged in a 2x3 grid over a 10m x 16m room.
 * A mobile node traverses a linear trajectory beneath three aligned APs.
 *
 * Channel model based on:
 *   [1] Matheus et al., "Visible light communication: concepts, applications
 *       and challenges," IEEE Commun. Surveys Tuts., vol. 21, no. 4, 2019.
 *   [2] DYRP-VLC: A dynamic routing protocol for Wireless Ad-Hoc VLC Networks,
 *       Ad Hoc Networks, 2019.
 *   [3] PARC: Proactive Protocol for Wireless Optical Ad Hoc Networks,
 *       Wireless Days Conference, 2025.
 *
 * =========================================================================
 * PHYSICAL CHANNEL MODEL
 * =========================================================================
 * LOS DC channel gain  H(0)  [ref. 1, Eq. 3]:
 *   H(0) = R(phi) * (A / d^2) * cos(theta)
 *
 *   where R(phi) = (m+1)/(2*pi) * cos^m(phi)   (Lambertian radiation pattern)
 *         m = -ln(2) / ln(cos(phi_1/2))         (Lambertian order)
 *         phi   = angle of irradiance  [rad]
 *         theta = angle of incidence at RX [rad]
 *         A     = photodetector active area [m^2]
 *         d     = TX-RX distance [m]
 *   H(0) = 0 when theta > FOV/2  (receiver outside field of view)
 *
 * SNR  [ref. 1, Eq. 4]:
 *   SNR = R^2 * H^2(0) * Pt^2 / (sigma_shot + sigma_thermal)
 *
 * Shot noise variance  [ref. 1, Eq. 5]:
 *   sigma_shot = 2q * [R*Pr*(1+MI^2) + Ibg*I2] * B
 *
 * Thermal noise variance  [ref. 1, Eq. 6]:
 *   sigma_thermal = 8*pi*k*Tk*eta*A*B^2 * (I2/G + (2*pi*eta*A*I3*B)/gm)
 *
 * =========================================================================
 * TRANSMISSION RATE MODEL (AWGN channel with OOK modulation)
 * =========================================================================
 * The additive white Gaussian noise (AWGN) model for the VLC receiver
 * combines shot noise and thermal noise as the dominant Gaussian noise
 * sources, consistent with refs [1][2].  Under OOK modulation (MI=0.5),
 * the bit error rate is [ref. 1]:
 *
 *   BER = Q( sqrt(SNR) / (M-1) )    with M=2 for OOK
 *       = Q( sqrt(SNR) )
 *       = 0.5 * erfc( sqrt(SNR/2) )
 *
 * Packet error rate for a packet of L bits:
 *   PER = 1 - (1 - BER)^L
 *
 * Effective (goodput) rate:
 *   R_eff = R_phy * (1 - PER)     [Mbps]
 *
 * =========================================================================
 * HANDOVER DECISION
 * =========================================================================
 * The mobile node associates with the AP delivering the highest SNR among
 * those within FoV, provided SNR > 0.  A hysteresis margin is applied in
 * the Hysteresis scenario to reduce unnecessary handovers (PARC, ref. [3]).
 *
 * =========================================================================
 * SIMULATION SCENARIOS
 * =========================================================================
 *   Baseline     - v=1/3/8 m/s, standard noise, no hysteresis
 *   HighMobility - v=1/3/8 m/s, standard noise, no hysteresis (speed emphasis)
 *   HighNoise    - v=1/3/8 m/s, elevated Ibg (x10), no hysteresis
 *   Hysteresis   - v=1/3/8 m/s, standard noise, hysteresis margin = 3 dB
 *
 * =========================================================================
 * OUTPUT FILES
 * =========================================================================
 *   vlc_snr_<scenario>_v<speed>.csv      - SNR [dB] per AP vs time
 *   vlc_ber_<scenario>_v<speed>.csv      - BER and effective rate vs time
 *   vlc_handover_<scenario>_v<speed>.csv - handover event log
 *   vlc_summary.csv                      - aggregated metrics per run
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
// VLC physical layer is implemented directly in this file.
// No external VLC module is required; only standard NS-3 modules are used.
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <limits>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("VlcHandoverSimulation");

/* =========================================================================
 * Physical layer constants  (Tabela 1 from reference [1])
 * =========================================================================*/
static const double PHOTO_AREA        = 5.24e-6;    // A  [m^2]
static const double RESPONSIVITY      = 0.45;        // R  [A/W]
static const double TX_POWER          = 500e-3;      // Pt [W]  LED luminaire (ref. [2])
static const double IBG_STANDARD      = 5100e-6;     // Ibg [A]  standard
static const double I2_FACTOR         = 0.562;       // I2
static const double TEMP_K            = 297.0;       // Tk [K]
static const double ETA_CAP           = 112e-6*1e-4; // eta [F/m^2]  (112e-6 pF/cm^2)
static const double OPEN_LOOP_GAIN    = 10.0;        // G
static const double I3_FACTOR         = 0.0868;      // I3
static const double FET_TRANSCONDUCT  = 30e-3;       // gm [S]
static const double ELECTRON_CHARGE   = 1.602176634e-19; // q [C]
static const double BOLTZMANN_K       = 1.380649e-23;    // k [J/K]
static const double NOISE_BW          = 3e5;         // B [Hz]
static const double OOK_MOD_INDEX     = 0.5;         // MI  (OOK: MI=0.5)

// AP geometry
static const double ROOM_X            = 10.0;   // [m]
static const double ROOM_Y            = 16.0;   // [m]
static const double AP_HEIGHT         = 3.0;    // [m]
static const double NODE_HEIGHT       = 1.2;    // [m]

// Angular parameters
static const double SEMI_ANGLE_DEG    = 35.0;   // TX semi-angle at half power [deg]
static const double FOV_DEG           = 28.5;   // RX FoV half-angle [deg]

// Handover hysteresis margin
static const double HYSTERESIS_DB     = 3.0;    // [dB]

// Sampling and transmission
static const double SAMPLE_INTERVAL   = 0.1;    // [s]
static const double PHY_RATE_MBPS     = 0.3;    // R_phy [Mbps]
static const int    PKT_SIZE_BYTES    = 100;    // packet size for PER calculation


/* =========================================================================
 * Lambertian order  m = -ln(2) / ln(cos(phi_1/2))
 * =========================================================================*/
static double LambertianOrder()
{
    double phi_half = SEMI_ANGLE_DEG * M_PI / 180.0;
    return -std::log(2.0) / std::log(std::cos(phi_half));
}

/* =========================================================================
 * Lambertian radiation pattern  R(phi)
 *   R(phi) = (m+1)/(2*pi) * cos^m(phi)
 * =========================================================================*/
static double RadiationPattern(double phi_rad)
{
    double m = LambertianOrder();
    if (phi_rad < 0.0) phi_rad = 0.0;
    return ((m + 1.0) / (2.0 * M_PI)) * std::pow(std::cos(phi_rad), m);
}

/* =========================================================================
 * LOS DC channel gain  H(0)  [ref. 1, Eq. 3]
 * Returns 0 if receiver is outside FoV.
 * =========================================================================*/
static double ChannelGainH0(const Vector &apPos, const Vector &rxPos, double fovDeg)
{
    double dx = rxPos.x - apPos.x;
    double dy = rxPos.y - apPos.y;
    double dz = rxPos.z - apPos.z;

    double d = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (d < 1e-9) return 0.0;

    double cosTheta = std::fabs(dz) / d;
    double cosPhi   = -dz / d;
    if (cosPhi < 0.0) return 0.0;

    double phi_rad   = std::acos(std::min(cosPhi,   1.0));
    double theta_rad = std::acos(std::min(cosTheta, 1.0));
    double fov_rad   = fovDeg * M_PI / 180.0;

    if (theta_rad > fov_rad) return 0.0;

    double R_phi = RadiationPattern(phi_rad);
    return R_phi * (PHOTO_AREA / (d * d)) * cosTheta;
}

/* =========================================================================
 * Shot noise variance  [ref. 1, Eq. 5]
 *
 *   sigma_shot = 2q * [R * Pr * (1 + MI^2) + Ibg * I2] * B
 *
 * where Pr = H(0) * Pt  is the average received optical power.
 * The first term accounts for the signal-dependent shot noise (OOK, MI=0.5);
 * the second term accounts for background-radiation shot noise.
 * =========================================================================*/
static double ShotNoise(double Pr, double ibg)
{
    return 2.0 * ELECTRON_CHARGE
           * (RESPONSIVITY * Pr * (1.0 + OOK_MOD_INDEX * OOK_MOD_INDEX)
              + ibg * I2_FACTOR)
           * NOISE_BW;
}

/* =========================================================================
 * Thermal noise variance  [ref. 1, Eq. 6]
 *
 *   sigma_thermal = 8*pi*k*Tk*eta*A*B^2 * ( I2/G  +  2*pi*eta*A*I3*B/gm )
 *
 * Parameters (Tabela 1, ref. [1]):
 *   k   = Boltzmann constant  [J/K]
 *   Tk  = absolute temperature = 297 K
 *   eta = fixed capacitance of PD = 112e-6 pF/cm^2  (converted to F/m^2)
 *   A   = photodetector area = 5.24e-6 m^2
 *   B   = noise bandwidth = 3e5 Hz
 *   I2  = 0.562,  I3 = 0.0868  (noise bandwidth factors)
 *   G   = open-loop voltage gain = 10
 *   gm  = FET transconductance = 30e-3 S
 * =========================================================================*/
static double ThermalNoise()
{
    double term1 = I2_FACTOR / OPEN_LOOP_GAIN;
    double term2 = (2.0 * M_PI * ETA_CAP * PHOTO_AREA * I3_FACTOR * NOISE_BW)
                   / FET_TRANSCONDUCT;
    return 8.0 * M_PI * BOLTZMANN_K * TEMP_K
           * ETA_CAP * PHOTO_AREA
           * NOISE_BW * NOISE_BW
           * (term1 + term2);
}

/* =========================================================================
 * SNR  [ref. 1, Eq. 4]
 *
 *   SNR = R^2 * Pr^2 / (sigma_shot(Pr) + sigma_thermal)
 *
 * where Pr = H(0) * Pt.
 * sigma_shot depends on Pr (position-dependent).
 * sigma_thermal depends only on receiver hardware (position-independent).
 * =========================================================================*/
static double ComputeSNR(double H0, double ibg)
{
    if (H0 <= 0.0) return 0.0;
    double Pr     = H0 * TX_POWER;
    double signal = RESPONSIVITY * RESPONSIVITY * Pr * Pr;
    double noise  = ShotNoise(Pr, ibg) + ThermalNoise();
    if (noise <= 0.0) return 0.0;
    return signal / noise;
}

/* =========================================================================
 * BER for OOK under AWGN  [ref. 1]
 *
 *   BER_OOK = Q( sqrt(SNR) )
 *           = 0.5 * erfc( sqrt(SNR/2) )
 *
 * For OOK (M=2):  Q( sqrt(SNR) / (M-1) ) = Q( sqrt(SNR) ).
 * =========================================================================*/
static double ComputeBER(double snr_linear)
{
    if (snr_linear <= 0.0) return 0.5;
    return 0.5 * std::erfc(std::sqrt(snr_linear / 2.0));
}

/* =========================================================================
 * Packet error rate
 *   PER = 1 - (1 - BER)^(8 * L)
 * =========================================================================*/
static double ComputePER(double ber)
{
    int bits = 8 * PKT_SIZE_BYTES;
    return 1.0 - std::pow(1.0 - ber, (double)bits);
}

/* =========================================================================
 * Effective goodput
 *   R_eff = R_phy * (1 - PER)   [Mbps]
 * =========================================================================*/
static double ComputeGoodput(double per)
{
    return PHY_RATE_MBPS * (1.0 - per);
}

static double LinearToDb(double v)
{
    if (v <= 0.0) return -std::numeric_limits<double>::infinity();
    return 10.0 * std::log10(v);
}

/* =========================================================================
 * Scenario descriptor
 * =========================================================================*/
struct ScenarioConfig
{
    std::string name;
    double      speedMps;
    double      ibg;
    double      hysteresisDb;
};

/* =========================================================================
 * Simulation state
 * =========================================================================*/
struct SimState
{
    std::vector<Vector> apPositions;
    int currentAP;

    struct HandoverEvent {
        double time;
        int    from;
        int    to;
        double snrDb;
        double ber;
        double goodputMbps;
    };
    std::vector<HandoverEvent> handoverLog;

    struct SnrSample {
        double time;
        std::vector<double> snrDb;
        std::vector<double> ber;
        std::vector<double> goodputMbps;
        int    servingAP;
    };
    std::vector<SnrSample> snrSeries;

    const ScenarioConfig *cfg;
    Ptr<Node> mobileNode;
};

/* =========================================================================
 * Periodic sampling callback: SNR, BER, goodput, handover decision
 * =========================================================================*/
static void SampleAndDecide(SimState *state)
{
    Ptr<MobilityModel> mob = state->mobileNode->GetObject<MobilityModel>();
    Vector pos = mob->GetPosition();
    double now = Simulator::Now().GetSeconds();
    int    nAP = (int)state->apPositions.size();

    SimState::SnrSample sample;
    sample.time = now;
    sample.snrDb.resize(nAP, 0.0);
    sample.ber.resize(nAP, 0.5);
    sample.goodputMbps.resize(nAP, 0.0);

    double bestSNR_lin = 0.0;
    int    bestAP      = -1;

    for (int i = 0; i < nAP; ++i)
    {
        double H0   = ChannelGainH0(state->apPositions[i], pos, FOV_DEG);
        double snr  = ComputeSNR(H0, state->cfg->ibg);
        double ber  = ComputeBER(snr);
        double per  = ComputePER(ber);
        double reff = ComputeGoodput(per);

        sample.snrDb[i]       = LinearToDb(snr);
        sample.ber[i]         = (snr > 0.0) ? ber  : 0.5;
        sample.goodputMbps[i] = (snr > 0.0) ? reff : 0.0;

        if (snr > bestSNR_lin) { bestSNR_lin = snr; bestAP = i; }
    }

    sample.servingAP = bestAP;
    state->snrSeries.push_back(sample);

    // Handover decision with optional hysteresis (PARC-inspired, ref. [3])
    if (bestAP >= 0 && bestAP != state->currentAP)
    {
        bool doHandover = true;

        if (state->cfg->hysteresisDb > 0.0 && state->currentAP >= 0)
        {
            double H0_curr  = ChannelGainH0(state->apPositions[state->currentAP],
                                            pos, FOV_DEG);
            double snr_curr = ComputeSNR(H0_curr, state->cfg->ibg);
            double margin   = std::pow(10.0, state->cfg->hysteresisDb / 10.0);
            if (bestSNR_lin < snr_curr * margin) doHandover = false;
        }

        if (doHandover)
        {
            double ber_new = ComputeBER(bestSNR_lin);
            double per_new = ComputePER(ber_new);
            SimState::HandoverEvent ev;
            ev.time        = now;
            ev.from        = state->currentAP;
            ev.to          = bestAP;
            ev.snrDb       = LinearToDb(bestSNR_lin);
            ev.ber         = ber_new;
            ev.goodputMbps = ComputeGoodput(per_new);
            state->handoverLog.push_back(ev);
            state->currentAP = bestAP;
        }
    }
    else if (bestAP < 0 && state->currentAP >= 0)
    {
        SimState::HandoverEvent ev;
        ev.time        = now;
        ev.from        = state->currentAP;
        ev.to          = -1;
        ev.snrDb       = 0.0;
        ev.ber         = 0.5;
        ev.goodputMbps = 0.0;
        state->handoverLog.push_back(ev);
        state->currentAP = -1;
    }

    Simulator::Schedule(Seconds(SAMPLE_INTERVAL), &SampleAndDecide, state);
}

/* =========================================================================
 * Write SNR CSV
 * =========================================================================*/
static void WriteSnrCsv(const SimState &state, const std::string &filename)
{
    std::ofstream f(filename);
    f << std::fixed << std::setprecision(6);
    f << "time_s,serving_AP";
    for (int i = 0; i < (int)state.apPositions.size(); ++i)
        f << ",SNR_AP" << i << "_dB";
    f << "\n";
    for (const auto &s : state.snrSeries)
    {
        f << s.time << "," << s.servingAP;
        for (double v : s.snrDb)
            f << "," << (std::isinf(v) ? 0.0 : v);
        f << "\n";
    }
}

/* =========================================================================
 * Write BER / goodput CSV
 * =========================================================================*/
static void WriteBerCsv(const SimState &state, const std::string &filename)
{
    std::ofstream f(filename);
    f << std::scientific << std::setprecision(6);
    int nAP = (int)state.apPositions.size();
    f << "time_s,serving_AP";
    for (int i = 0; i < nAP; ++i) f << ",BER_AP"  << i;
    for (int i = 0; i < nAP; ++i) f << ",Reff_AP" << i << "_Mbps";
    f << "\n";
    for (const auto &s : state.snrSeries)
    {
        f << s.time << "," << s.servingAP;
        for (double v : s.ber)         f << "," << v;
        for (double v : s.goodputMbps) f << "," << v;
        f << "\n";
    }
}

/* =========================================================================
 * Write handover log CSV
 * =========================================================================*/
static void WriteHandoverCsv(const SimState &state, const std::string &filename)
{
    std::ofstream f(filename);
    f << std::fixed << std::setprecision(6);
    f << "time_s,from_AP,to_AP,SNR_new_dB,BER_new,Reff_new_Mbps\n";
    for (const auto &ev : state.handoverLog)
    {
        f << ev.time << ","
          << (ev.from >= 0 ? std::to_string(ev.from) : "none") << ","
          << (ev.to   >= 0 ? std::to_string(ev.to)   : "none") << ","
          << ev.snrDb << "," << ev.ber << "," << ev.goodputMbps << "\n";
    }
}

/* =========================================================================
 * Run one scenario
 * =========================================================================*/
static std::vector<double> RunScenario(const ScenarioConfig &cfg)
{
    std::cout << "\n[SCENARIO] " << cfg.name
              << "  v=" << cfg.speedMps << " m/s"
              << "  Ibg=" << cfg.ibg * 1e6 << " uA"
              << "  hyst=" << cfg.hysteresisDb << " dB"
              << std::endl;

    const double col0_x = ROOM_X / 4.0;
    const double col1_x = 3.0 * ROOM_X / 4.0;
    const double row0_y = ROOM_Y / 6.0;
    const double row1_y = ROOM_Y / 2.0;
    const double row2_y = 5.0 * ROOM_Y / 6.0;

    std::vector<Vector> apPos = {
        Vector(col0_x, row0_y, AP_HEIGHT),   // AP 0
        Vector(col1_x, row0_y, AP_HEIGHT),   // AP 1
        Vector(col0_x, row1_y, AP_HEIGHT),   // AP 2
        Vector(col1_x, row1_y, AP_HEIGHT),   // AP 3
        Vector(col0_x, row2_y, AP_HEIGHT),   // AP 4
        Vector(col1_x, row2_y, AP_HEIGHT),   // AP 5
    };

    double simTime = (ROOM_Y / cfg.speedMps) + 1.0;

    NodeContainer apNodes;
    apNodes.Create(6);
    NodeContainer mobileContainer;
    mobileContainer.Create(1);
    Ptr<Node> mobileNode = mobileContainer.Get(0);

    // APs: ConstantPositionMobilityModel (NS-3 native)
    MobilityHelper apMobility;
    Ptr<ListPositionAllocator> apAlloc = CreateObject<ListPositionAllocator>();
    for (const auto &p : apPos) apAlloc->Add(p);
    apMobility.SetPositionAllocator(apAlloc);
    apMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    apMobility.Install(apNodes);

    // Mobile node: WaypointMobilityModel (NS-3 native)
    MobilityHelper nodeMobility;
    nodeMobility.SetMobilityModel("ns3::WaypointMobilityModel");
    nodeMobility.Install(mobileContainer);

    Ptr<WaypointMobilityModel> waypointMob =
        mobileNode->GetObject<WaypointMobilityModel>();
    waypointMob->AddWaypoint(
        Waypoint(Seconds(0.0), Vector(col0_x, 0.0, NODE_HEIGHT)));
    waypointMob->AddWaypoint(
        Waypoint(Seconds(ROOM_Y / cfg.speedMps),
                 Vector(col0_x, ROOM_Y, NODE_HEIGHT)));

    SimState state;
    state.apPositions = apPos;
    state.currentAP   = -1;
    state.cfg         = &cfg;
    state.mobileNode  = mobileNode;

    Simulator::Schedule(Seconds(0.0), &SampleAndDecide, &state);
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    std::string tag     = cfg.name + "_v" + std::to_string((int)cfg.speedMps);
    std::string snrFile = "vlc_snr_"      + tag + ".csv";
    std::string berFile = "vlc_ber_"      + tag + ".csv";
    std::string hoFile  = "vlc_handover_" + tag + ".csv";

    WriteSnrCsv(state, snrFile);
    WriteBerCsv(state, berFile);
    WriteHandoverCsv(state, hoFile);

    std::cout << "  [OUT] " << snrFile << "  " << berFile << "  " << hoFile << std::endl;

    double sumSnr=0, minSnr=1e30, maxSnr=-1e30;
    double sumReff=0, minReff=1e30, maxReff=-1e30;
    int count=0;

    for (const auto &s : state.snrSeries)
    {
        int ap = s.servingAP;
        if (ap < 0) continue;
        double snr  = s.snrDb[ap];
        double reff = s.goodputMbps[ap];
        if (std::isinf(snr) || snr < -100.0) continue;
        sumSnr  += snr;  minSnr  = std::min(minSnr,  snr);  maxSnr  = std::max(maxSnr,  snr);
        sumReff += reff; minReff = std::min(minReff, reff); maxReff = std::max(maxReff, reff);
        ++count;
    }

    double avgSnr  = count > 0 ? sumSnr  / count : 0.0;
    double avgReff = count > 0 ? sumReff / count : 0.0;

    std::cout << "  [RESULT] HOs=" << state.handoverLog.size()
              << "  avgSNR=" << avgSnr << " dB"
              << "  avgReff=" << avgReff << " Mbps" << std::endl;

    return { (double)state.handoverLog.size(),
             avgSnr,  (count>0?minSnr:0.0),  (count>0?maxSnr:0.0),
             avgReff, (count>0?minReff:0.0), (count>0?maxReff:0.0) };
}

/* =========================================================================
 * Main
 * =========================================================================*/
int main(int argc, char *argv[])
{
    bool verbose = false;
    CommandLine cmd;
    cmd.AddValue("verbose", "Enable NS-3 logging", verbose);
    cmd.Parse(argc, argv);

    if (verbose)
        LogComponentEnable("VlcHandoverSimulation", LOG_LEVEL_INFO);

    std::cout << "================================================" << std::endl;
    std::cout << " VLC Handover Simulation" << std::endl;
    std::cout << "================================================" << std::endl;
    std::cout << " Room     : " << ROOM_X << " x " << ROOM_Y << " m" << std::endl;
    std::cout << " AP z     : " << AP_HEIGHT  << " m  (ceiling)" << std::endl;
    std::cout << " Node z   : " << NODE_HEIGHT << " m" << std::endl;
    std::cout << " m_order  : " << std::fixed << std::setprecision(3)
              << LambertianOrder() << std::endl;
    std::cout << " FoV      : " << FOV_DEG << " deg" << std::endl;
    std::cout << " R_phy    : " << PHY_RATE_MBPS << " Mbps" << std::endl;
    std::cout << " Pkt size : " << PKT_SIZE_BYTES << " bytes" << std::endl;
    {
        double d_ref  = AP_HEIGHT - NODE_HEIGHT;
        double m_ord  = LambertianOrder();
        double H0_ref = ((m_ord+1.0)/(2.0*M_PI)) * (PHOTO_AREA / (d_ref*d_ref));
        double Pr_ref = H0_ref * TX_POWER;
        std::cout << std::scientific;
        std::cout << " H0_nadir        : " << H0_ref << std::endl;
        std::cout << " Pr_nadir        : " << Pr_ref*1e6 << " uW" << std::endl;
        std::cout << " sigma_shot(std) : " << ShotNoise(Pr_ref, IBG_STANDARD) << " A^2" << std::endl;
        std::cout << " sigma_thermal   : " << ThermalNoise() << " A^2" << std::endl;
        std::cout << std::fixed;
    }
    std::cout << "================================================" << std::endl;

    std::vector<ScenarioConfig> scenarios = {
        { "Baseline",     1.0, IBG_STANDARD,        0.0           },
        { "Baseline",     3.0, IBG_STANDARD,        0.0           },
        { "Baseline",     8.0, IBG_STANDARD,        0.0           },
        { "HighMobility", 1.0, IBG_STANDARD,        0.0           },
        { "HighMobility", 3.0, IBG_STANDARD,        0.0           },
        { "HighMobility", 8.0, IBG_STANDARD,        0.0           },
        { "HighNoise",    1.0, IBG_STANDARD * 10.0, 0.0           },
        { "HighNoise",    3.0, IBG_STANDARD * 10.0, 0.0           },
        { "HighNoise",    8.0, IBG_STANDARD * 10.0, 0.0           },
        { "Hysteresis",   1.0, IBG_STANDARD,        HYSTERESIS_DB },
        { "Hysteresis",   3.0, IBG_STANDARD,        HYSTERESIS_DB },
        { "Hysteresis",   8.0, IBG_STANDARD,        HYSTERESIS_DB },
    };

    std::ofstream summary("vlc_summary.csv");
    summary << std::fixed << std::setprecision(6);
    summary << "scenario,speed_mps,handover_count,"
               "avg_snr_dB,min_snr_dB,max_snr_dB,"
               "avg_reff_Mbps,min_reff_Mbps,max_reff_Mbps\n";

    for (const auto &cfg : scenarios)
    {
        std::vector<double> res = RunScenario(cfg);
        summary << cfg.name     << ","
                << cfg.speedMps << ","
                << (int)res[0]  << ","
                << res[1] << "," << res[2] << "," << res[3] << ","
                << res[4] << "," << res[5] << "," << res[6] << "\n";
    }

    summary.close();
    std::cout << "\n[DONE]  vlc_summary.csv written." << std::endl;
    return 0;
}
