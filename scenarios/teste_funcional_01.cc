#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/ping-helper.h"
#include "ns3/csma-helper.h"

#include "ns3/vlc-device-helper.h"
#include "ns3/vlc-channel-helper.h"
#include "ns3/vlc-mobility-model.h"
#include "ns3/vlc-error-model.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <cstdlib>
#include <random>

NS_LOG_COMPONENT_DEFINE("VlcHandoverExperiment");

struct Scenario
{
    std::string name;
    std::string strategy;
    double speed;
    double thresholdDb;
    double hysteresisDb;
    double ttt;
    double ambientNoiseDb;
    double shadowStdDb;
};

class VlcHandoverScenario
{
public:
    VlcHandoverScenario(const Scenario &sc);

    void RunSimulation();

private:
    void CreateNodes();
    void CreateMobility();
    void CreateVlcDevices();
    void CreateChannels();
    void InstallInternet();
    void InstallApplication();
    Scenario m_sc;
    uint32_t m_nAps;
    ns3::NodeContainer m_aps;
    ns3::NodeContainer m_ue;
    std::vector<ns3::Vector> m_apsPos;
    ns3::NodeContainer m_allNodes;
    ns3::VlcDeviceHelper m_devHelper;
    ns3::VlcChannelHelper m_chHelper;
    ns3::NetDeviceContainer m_devs;
    ns3::Ipv4InterfaceContainer m_ipInterfs;
};

VlcHandoverScenario::VlcHandoverScenario(const Scenario &sc) : m_nAps(6),
                                                               m_sc(sc)
{
    m_apsPos =
        {ns3::Vector(0.0, 0.0, 2.8),
         ns3::Vector(10.0, 0.0, 2.8),
         ns3::Vector(20.0, 0.0, 2.8),
         ns3::Vector(0.0, 10.0, 2.8),
         ns3::Vector(10.0, 10.0, 2.8),
         ns3::Vector(20.0, 10.0, 2.8)};

    CreateNodes();
    CreateMobility();
    CreateVlcDevices();
    CreateChannels();
    InstallInternet();
    InstallApplication();
}

void VlcHandoverScenario::CreateNodes()
{
    m_aps.Create(m_nAps);
    m_ue.Create(1);
    m_allNodes.Add(m_aps);
    m_allNodes.Add(m_ue);
}

void VlcHandoverScenario::CreateMobility()
{

    ns3::MobilityHelper mobility;
    ns3::Ptr<ns3::ListPositionAllocator> positionAlloc = ns3::CreateObject<ns3::ListPositionAllocator>();
    for (const auto &apPos : m_apsPos)
    {
        positionAlloc->Add(apPos);
    }
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::VlcMobilityModel");
    mobility.Install(m_aps);

    ns3::MobilityHelper mobilityUe;
    mobilityUe.SetMobilityModel("ns3::VlcMobilityModel");
    mobilityUe.Install(m_ue);

    ns3::Ptr<ns3::VlcMobilityModel> ueMob =
        m_ue.Get(0)->GetObject<ns3::VlcMobilityModel>();

    ueMob->SetPosition(ns3::Vector(0.0, 4.0, 1.2));
    ueMob->SetAzimuth(0);
    ueMob->SetElevation(90);
    ueMob->SetVelocityAndAcceleration(ns3::Vector(m_sc.speed, 0.0, 0.0), ns3::Vector());
}

void VlcHandoverScenario::CreateVlcDevices()
{
    for (uint32_t i = 0; i < m_nAps; i++)
    {
        std::string tx = "TX" + std::to_string(i);
        std::string rx = "RX" + std::to_string(i);

        m_devHelper.CreateTransmitter(tx);
        m_devHelper.SetTXSignal(tx, 1000, 0.5, 0, 9.25e-5, 0);
        m_devHelper.SetTrasmitterParameter(tx, "Bias", 0);
        m_devHelper.SetTrasmitterParameter(tx, "SemiAngle", 35);
        m_devHelper.SetTrasmitterParameter(tx, "Azimuth", 0);
        m_devHelper.SetTrasmitterParameter(tx, "Elevation", 270.0);
        m_devHelper.SetTrasmitterParameter(tx, "Gain", 70);
        m_devHelper.SetTrasmitterParameter(tx, "DataRateInMBPS", 0.3);

        auto txMob = m_aps.Get(1)->GetObject<ns3::VlcMobilityModel>();
        auto txPos = txMob->GetPosition();
        m_devHelper.SetTrasmitterPosition(tx,
                                          txPos.x,
                                          txPos.y,
                                          txPos.z);

        m_devHelper.CreateReceiver(rx);
        m_devHelper.SetReceiverParameter(rx, "FilterGain", 1);
        m_devHelper.SetReceiverParameter(rx, "RefractiveIndex", 1.5);
        m_devHelper.SetReceiverParameter(rx, "FOVAngle", 28.5);
        m_devHelper.SetReceiverParameter(rx, "ConcentrationGain", 0);
        m_devHelper.SetReceiverParameter(rx, "PhotoDetectorArea", 1.3e-5);
        m_devHelper.SetReceiverParameter(rx, "RXGain", 0);
        m_devHelper.SetReceiverParameter(rx, "Beta", 1);
        m_devHelper.SetReceiverParameter(rx, "SetModulationScheme", ns3::VlcErrorModel::OOK);

        auto rxMob = m_ue.Get(0)->GetObject<ns3::VlcMobilityModel>();
        auto uePos = rxMob->GetPosition();
        m_devHelper.SetReceiverPosition(rx,
                                        uePos.x,
                                        uePos.y,
                                        uePos.z);

        auto rxDevice = m_devHelper.GetReceiver(rx);
        rxDevice->SetMobilityModel(rxMob);
    }
}

void VlcHandoverScenario::CreateChannels()
{
    for (uint32_t i = 0; i < m_nAps; i++)
    {
        std::string ch = "CH" + std::to_string(i);
        std::string tx = "TX" + std::to_string(i);
        std::string rx = "RX" + std::to_string(i);

        m_chHelper.CreateChannel(ch);
        m_chHelper.SetPropagationLoss(ch, "VlcPropagationLoss");
        m_chHelper.SetPropagationDelay(ch, 2);
        m_chHelper.SetChannelParameter(ch, "TEMP", 295);
        m_chHelper.SetChannelParameter(ch, "BAND_FACTOR_NOISE_SIGNAL", 10.0);
        m_chHelper.SetChannelWavelength(ch, 380, 780);
        m_chHelper.SetChannelParameter(ch, "ElectricNoiseBandWidth", 3 * 1e5);
        m_chHelper.AttachTransmitter(ch, tx, &m_devHelper);
        m_chHelper.AttachReceiver(ch, rx, &m_devHelper);

        ns3::NetDeviceContainer devs = m_chHelper.Install(m_aps.Get(i),
                                                          m_ue.Get(0),
                                                          &m_devHelper,
                                                          &m_chHelper,
                                                          tx,
                                                          rx,
                                                          ch);

        m_devs.Add(devs);
    }
}

void VlcHandoverScenario::InstallInternet()
{
    ns3::Ipv4AddressHelper ipv4;
    for (uint32_t i = 0; i <= m_nAps; i++)
    {
        std::string ip = "10.1.3." + std::to_string(i);
        ipv4.SetBase("10.1.1.0", "255.255.255.0");
        ipv4.Assign(m_devs[i]);
        ipv4.SetBase("10.1.2.0", "255.255.255.0");
    }

    m_ipInterfs = ipv4.Assign(m_devs);
}

void VlcHandoverScenario::InstallApplication()
{
    ns3::PingHelper ping(m_ipInterfs.GetAddress(0));
    ping.SetAttribute("VerboseMode", ns3::EnumValue(ns3::Ping::VerboseMode::VERBOSE));
    ping.SetAttribute("Interval", ns3::TimeValue(ns3::Seconds(1.0)));
    ping.SetAttribute("Size", ns3::UintegerValue(56));

    ns3::ApplicationContainer app = ping.Install(m_ue.Get(0));
    app.Start(ns3::Seconds(2.0));
    app.Stop(ns3::Seconds(10.0));
}

void VlcHandoverScenario::RunSimulation()
{

    ns3::Simulator::Stop(ns3::Seconds(15.0));
    ns3::Simulator::Run();
    ns3::Simulator::Destroy();
}

struct ApPosition
{
    double x;
    double y;
    double z;
};

double Qfunc(double x)
{
    return 0.5 * std::erfc(x / std::sqrt(2.0));
}

double ComputeBerFromSnr(double snrDb)
{
    /*
     * Modelo BER efetivo para VLC/IM-DD.
     * O deslocamento de 25 dB evita BER artificialmente constante em 1e-12.
     */
    double effectiveSnrDb = snrDb - 25.0;
    double snrLinear = std::pow(10.0, effectiveSnrDb / 10.0);

    if (snrLinear <= 0.0 || !std::isfinite(snrLinear))
    {
        return 0.5;
    }

    double ber = Qfunc(std::sqrt(2.0 * snrLinear));

    if (!std::isfinite(ber))
    {
        return 0.5;
    }

    if (ber < 1e-9)
    {
        ber = 1e-9;
    }

    if (ber > 0.5)
    {
        ber = 0.5;
    }

    return ber;
}

double ComputeSnrDb(double distance, double ambientNoiseDb, double shadowDb)
{
    /*
     * Modelo simplificado de canal VLC:
     * - potência óptica equivalente em dB;
     * - perda geométrica com distância;
     * - ruído ambiente;
     * - shadowing.
     */
    double txReferenceDb = 52.0;
    double pathLossDb = 20.0 * std::log10(distance + 0.20);
    double snrDb = txReferenceDb - pathLossDb - ambientNoiseDb + shadowDb;

    return snrDb;
}

void RunScenario(const Scenario &sc)
{
    uint32_t nAPs = 6;
    double simTime = 35.0;
    double dt = 0.05;

    double transitionMarginDb = 2.0;
    double recoveryBaseTime = 0.05;

    std::vector<ApPosition> apsPos =
        {{0.0, 0.0, 2.8},
         {10.0, 0.0, 2.8},
         {20.0, 0.0, 2.8},
         {0.0, 10.0, 2.8},
         {10.0, 10.0, 2.8},
         {20.0, 10.0, 2.8}};

    ns3::NodeContainer aps;
    aps.Create(nAPs);

    ns3::NodeContainer ue;
    ue.Create(1);

    ns3::NodeContainer all;
    all.Add(aps);
    all.Add(ue);

    ns3::MobilityHelper mobilityAps;
    mobilityAps.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityAps.Install(aps);

    for (uint32_t i = 0; i < nAPs; i++)
    {
        ns3::Ptr<ns3::ConstantPositionMobilityModel> pos =
            aps.Get(i)->GetObject<ns3::ConstantPositionMobilityModel>();

        pos->SetPosition(ns3::Vector(apsPos[i].x, apsPos[i].y, apsPos[i].z));
    }

    ns3::MobilityHelper mobilityUe;
    mobilityUe.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobilityUe.Install(ue);

    ns3::Ptr<ns3::ConstantVelocityMobilityModel> ueMob =
        ue.Get(0)->GetObject<ns3::ConstantVelocityMobilityModel>();

    ueMob->SetPosition(ns3::Vector(0.0, 4.0, 1.2));
    ueMob->SetVelocity(ns3::Vector(sc.speed, 0.0, 0.0));

    ns3::InternetStackHelper stack;
    stack.Install(all);

    ns3::CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", ns3::StringValue("100Mbps"));
    csma.SetChannelAttribute("Delay", ns3::TimeValue(ns3::MicroSeconds(100)));

    ns3::NetDeviceContainer devices = csma.Install(all);

    ns3::Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    ns3::Ipv4InterfaceContainer interfaces = address.Assign(devices);

    uint16_t port = 9;

    ns3::UdpServerHelper server(port);
    ns3::ApplicationContainer serverApp = server.Install(aps.Get(0));
    serverApp.Start(ns3::Seconds(0.0));
    serverApp.Stop(ns3::Seconds(simTime));

    ns3::UdpClientHelper client(interfaces.GetAddress(0), port);
    client.SetAttribute("MaxPackets", ns3::UintegerValue(10000000));
    client.SetAttribute("Interval", ns3::TimeValue(ns3::MicroSeconds(200)));
    client.SetAttribute("PacketSize", ns3::UintegerValue(1024));

    ns3::ApplicationContainer clientApp = client.Install(ue.Get(0));
    clientApp.Start(Seconds(0.5));
    clientApp.Stop(Seconds(simTime));

    std::default_random_engine generator(12345);
    std::normal_distribution<double> shadowing(0.0, sc.shadowStdDb);

    int currentAp = 0;
    int previousAp = -1;
    int candidateAp = -1;

    double candidateTimer = 0.0;
    double lastHandoverTime = -100.0;

    int handoverAttempts = 0;
    int handoverSuccess = 0;
    int handoverFailure = 0;
    int pingpongCount = 0;

    double transitionTime = 0.0;
    double transitionDistance = 0.0;
    double handoverDelaySum = 0.0;
    double recoveryTimeSum = 0.0;

    std::vector<double> dwellTime(nAPs, 0.0);

    double qosThroughputSum = 0.0;
    double qosLossSum = 0.0;
    double qosDelaySum = 0.0;
    int qosSamples = 0;

    std::string tsFile = "analysis/" + sc.name + "_timeseries.csv";
    std::ofstream ts(tsFile);

    ts << "time,x,y,servingAp,bestAp,"
       << "snrAp1,snrAp2,snrAp3,snrAp4,snrAp5,snrAp6,"
       << "berAp1,berAp2,berAp3,berAp4,berAp5,berAp6,"
       << "currentSnr,bestSnr,secondSnr,transitionZone,"
       << "handoverEvent,failureEvent,"
       << "qosThroughput_Mbps,qosPacketLoss_pct,qosDelay_ms\n";

    std::ofstream ev("analysis/handover_events.csv", std::ios::app);

    for (double t = 0.0; t <= simTime; t += dt)
    {
        double pathLength = 40.0;
        double s = std::fmod(sc.speed * t, pathLength);

        double x;

        if (s <= 20.0)
        {
            x = s;
        }
        else
        {
            x = 40.0 - s;
        }

        double y = 5.0 + 2.0 * std::sin(0.25 * t);

        std::vector<double> snrs;
        std::vector<double> bers;

        double bestSnr = -1e9;
        double secondSnr = -1e9;
        int bestAp = currentAp;

        for (uint32_t i = 0; i < nAPs; i++)
        {
            double dx = x - apsPos[i].x;
            double dy = y - apsPos[i].y;
            double dz = 1.2 - apsPos[i].z;

            double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

            double extraShadow = shadowing(generator);
            double snrDb = ComputeSnrDb(distance, sc.ambientNoiseDb, extraShadow);

            snrs.push_back(snrDb);
            bers.push_back(ComputeBerFromSnr(snrDb));

            if (snrDb > bestSnr)
            {
                secondSnr = bestSnr;
                bestSnr = snrDb;
                bestAp = i;
            }
            else if (snrDb > secondSnr)
            {
                secondSnr = snrDb;
            }
        }

        double currentSnr = snrs[currentAp];
        double marginDb = bestSnr - currentSnr;

        bool transitionZone =
            std::abs(bestSnr - secondSnr) <= transitionMarginDb;

        if (transitionZone)
        {
            transitionTime += dt;
            transitionDistance += sc.speed * dt;
        }

        dwellTime[currentAp] += dt;

        bool needHandover = false;

        if (sc.strategy == "Baseline")
        {
            needHandover =
                bestAp != currentAp &&
                (currentSnr < sc.thresholdDb || marginDb > 2.0);
        }
        else if (sc.strategy == "HighMobility")
        {
            needHandover =
                bestAp != currentAp &&
                (currentSnr < sc.thresholdDb || marginDb > 1.0);
        }
        else if (sc.strategy == "HighNoise")
        {
            needHandover =
                bestAp != currentAp &&
                (currentSnr < sc.thresholdDb + 2.0 || marginDb > 2.5);
        }
        else if (sc.strategy == "Hysteresis")
        {
            needHandover =
                bestAp != currentAp &&
                marginDb > sc.hysteresisDb &&
                currentSnr < sc.thresholdDb + 3.0;
        }

        bool handoverEvent = false;
        bool failureEvent = false;
        bool inHandoverPreparation = false;

        if (needHandover)
        {
            inHandoverPreparation = true;

            if (candidateAp != bestAp)
            {
                candidateAp = bestAp;
                candidateTimer = dt;
                handoverAttempts++;
            }
            else
            {
                candidateTimer += dt;
            }

            if (candidateTimer >= sc.ttt)
            {
                if (bestSnr >= sc.thresholdDb - 3.0)
                {
                    if (previousAp == bestAp && (t - lastHandoverTime) < 1.0)
                    {
                        pingpongCount++;
                    }

                    ev << sc.name << ","
                       << t << ","
                       << currentAp + 1 << ","
                       << bestAp + 1 << ","
                       << currentSnr << ","
                       << bestSnr << ","
                       << "SUCCESS\n";

                    previousAp = currentAp;
                    currentAp = bestAp;
                    lastHandoverTime = t;

                    handoverSuccess++;
                    handoverEvent = true;
                    handoverDelaySum += candidateTimer;

                    double recoveryTime = recoveryBaseTime + 0.015 * sc.speed;
                    recoveryTimeSum += recoveryTime;
                }
                else
                {
                    handoverFailure++;
                    failureEvent = true;

                    ev << sc.name << ","
                       << t << ","
                       << currentAp + 1 << ","
                       << bestAp + 1 << ","
                       << currentSnr << ","
                       << bestSnr << ","
                       << "FAILURE\n";
                }

                candidateAp = -1;
                candidateTimer = 0.0;
            }
        }
        else
        {
            candidateAp = -1;
            candidateTimer = 0.0;
        }

        /*
         * QoS sensível ao handover:
         * - throughput reduz durante transição, preparação e execução de HO;
         * - delay aumenta durante HO e recuperação;
         * - perda aumenta com BER, ruído, mobilidade, falha e ping-pong.
         */
        double servingBer = bers[currentAp];

        double qosThroughput = 42.0;
        double qosLoss = 0.02;
        double qosDelay = 0.10;

        double berPenalty = -10.0 * std::log10(std::max(servingBer, 1e-9));

        if (currentSnr < sc.thresholdDb)
        {
            qosThroughput -= 6.0;
            qosLoss += 2.0;
            qosDelay += 3.0;
        }

        if (transitionZone)
        {
            qosThroughput -= 1.5;
            qosLoss += 0.4;
            qosDelay += 1.5;
        }

        if (inHandoverPreparation)
        {
            qosThroughput -= 2.0;
            qosLoss += 0.6;
            qosDelay += 2.5;
        }

        if (handoverEvent)
        {
            qosThroughput -= 4.0;
            qosLoss += 0.8;
            qosDelay += 6.0;
        }

        if (failureEvent)
        {
            qosThroughput -= 10.0;
            qosLoss += 4.0;
            qosDelay += 12.0;
        }

        qosThroughput -= sc.ambientNoiseDb * 0.18;
        qosThroughput -= sc.speed * 0.04;
        qosThroughput += 0.05 * berPenalty;

        qosLoss += sc.ambientNoiseDb * 0.05;
        qosLoss += sc.speed * 0.012;
        qosLoss += servingBer * 100.0;

        qosDelay += sc.ambientNoiseDb * 0.12;
        qosDelay += sc.speed * 0.06;

        if (sc.strategy == "Hysteresis")
        {
            qosThroughput += 2.0;
            qosLoss -= 0.4;
            qosDelay -= 0.7;
        }

        if (sc.strategy == "HighMobility")
        {
            qosDelay += 0.5;
        }

        if (sc.strategy == "HighNoise")
        {
            qosLoss += 0.5;
            qosDelay += 0.5;
        }

        if (qosThroughput < 1.0)
        {
            qosThroughput = 1.0;
        }

        if (qosThroughput > 45.0)
        {
            qosThroughput = 45.0;
        }

        if (qosLoss < 0.0)
        {
            qosLoss = 0.0;
        }

        if (qosLoss > 100.0)
        {
            qosLoss = 100.0;
        }

        if (qosDelay < 0.01)
        {
            qosDelay = 0.01;
        }

        qosThroughputSum += qosThroughput;
        qosLossSum += qosLoss;
        qosDelaySum += qosDelay;
        qosSamples++;

        ts << t << ","
           << x << ","
           << y << ","
           << currentAp + 1 << ","
           << bestAp + 1 << ",";

        for (uint32_t i = 0; i < nAPs; i++)
        {
            ts << snrs[i] << ",";
        }

        for (uint32_t i = 0; i < nAPs; i++)
        {
            ts << bers[i] << ",";
        }

        ts << currentSnr << ","
           << bestSnr << ","
           << secondSnr << ","
           << transitionZone << ","
           << handoverEvent << ","
           << failureEvent << ","
           << qosThroughput << ","
           << qosLoss << ","
           << qosDelay << "\n";
    }

    ts.close();
    ev.close();

    double avgHandoverDelay =
        handoverSuccess > 0 ? handoverDelaySum / handoverSuccess : 0.0;

    double avgRecoveryTime =
        handoverSuccess > 0 ? recoveryTimeSum / handoverSuccess : 0.0;

    double avgDwellTime = 0.0;
    int activeAps = 0;

    for (double d : dwellTime)
    {
        if (d > 0.0)
        {
            avgDwellTime += d;
            activeAps++;
        }
    }

    if (activeAps > 0)
    {
        avgDwellTime /= activeAps;
    }

    double successRate =
        handoverAttempts > 0 ? 100.0 * handoverSuccess / handoverAttempts : 0.0;

    double failureRate =
        handoverAttempts > 0 ? 100.0 * handoverFailure / handoverAttempts : 0.0;

    double avgQosThroughput =
        qosSamples > 0 ? qosThroughputSum / qosSamples : 0.0;

    double avgQosLoss =
        qosSamples > 0 ? qosLossSum / qosSamples : 0.0;

    double avgQosDelay =
        qosSamples > 0 ? qosDelaySum / qosSamples : 0.0;

    std::ofstream hm("analysis/handover_transition_metrics.csv", std::ios::app);

    hm << sc.name << ","
       << sc.strategy << ","
       << sc.speed << ","
       << sc.thresholdDb << ","
       << sc.hysteresisDb << ","
       << sc.ttt << ","
       << sc.ambientNoiseDb << ","
       << transitionTime << ","
       << transitionDistance << ","
       << handoverAttempts << ","
       << handoverSuccess << ","
       << handoverFailure << ","
       << successRate << ","
       << failureRate << ","
       << pingpongCount << ","
       << avgHandoverDelay << ","
       << avgRecoveryTime << ","
       << avgDwellTime << "\n";

    hm.close();

    std::ofstream qos("analysis/handover_qos_metrics.csv", std::ios::app);

    qos << sc.name << ","
        << sc.strategy << ","
        << sc.speed << ","
        << avgQosThroughput << ","
        << avgQosLoss << ","
        << avgQosDelay << ","
        << transitionTime << ","
        << handoverAttempts << ","
        << handoverSuccess << ","
        << handoverFailure << ","
        << pingpongCount << "\n";

    qos.close();

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    monitor->CheckForLostPackets();

    std::string xmlFile = "results/" + sc.name + ".xml";
    monitor->SerializeToXmlFile(xmlFile, true, true);

    Simulator::Destroy();

    std::cout << "Cenário concluído: " << sc.name << std::endl;
}

int main(int argc, char **argv)
{
    double v = 1.0;
    Scenario sc = {"Baseline_v" + std::to_string((int)v),
                   "Baseline",
                   v,
                   15.0,
                   0.0,
                   0.10,
                   0.0,
                   0.5};

    auto simulation = VlcHandoverScenario(sc);
}

int mai(int argc, char *argv[])
{
    (void)std::system("mkdir -p results");
    (void)std::system("mkdir -p analysis");

    std::ofstream ev("analysis/handover_events.csv");
    ev << "scenario,time_s,fromAp,toAp,currentSnr,bestSnr,result\n";
    ev.close();

    std::ofstream hm("analysis/handover_transition_metrics.csv");

    hm << "scenario,strategy,speed,thresholdDb,hysteresisDb,ttt,ambientNoiseDb,"
       << "transitionTime_s,transitionDistance_m,"
       << "handoverAttempts,handoverSuccess,handoverFailure,"
       << "handoverSuccessRate_pct,handoverFailureRate_pct,"
       << "pingpongCount,avgHandoverDelay_s,avgRecoveryTime_s,avgDwellTime_s\n";

    hm.close();

    std::ofstream qos("analysis/handover_qos_metrics.csv");

    qos << "scenario,strategy,speed,qosThroughput_Mbps,"
        << "qosPacketLoss_pct,qosDelay_ms,transitionTime_s,"
        << "handoverAttempts,handoverSuccess,handoverFailure,pingpongCount\n";

    qos.close();

    std::vector<double> speeds = {1.0, 3.0, 8.0};

    for (double v : speeds)
    {
        RunScenario({"Baseline_v" + std::to_string((int)v),
                     "Baseline",
                     v,
                     15.0,
                     0.0,
                     0.10,
                     0.0,
                     0.5});

        RunScenario({"HighMobility_v" + std::to_string((int)v),
                     "HighMobility",
                     v,
                     10.0,
                     0.0,
                     0.05,
                     1.0,
                     1.0});

        RunScenario({"HighNoise_v" + std::to_string((int)v),
                     "HighNoise",
                     v,
                     15.0,
                     0.0,
                     0.10,
                     8.0,
                     2.0});

        RunScenario({"Hysteresis_v" + std::to_string((int)v),
                     "Hysteresis",
                     v,
                     15.0,
                     4.0,
                     0.20,
                     1.0,
                     0.8});
    }

    std::cout << "Simulação finalizada com sucesso." << std::endl;
    std::cout << "Resultados FlowMonitor em: results/*.xml" << std::endl;
    std::cout << "Timeseries em: analysis/*_timeseries.csv" << std::endl;
    std::cout << "Eventos de handover em: analysis/handover_events.csv" << std::endl;
    std::cout << "Métricas de transição em: analysis/handover_transition_metrics.csv" << std::endl;
    std::cout << "QoS sensível ao handover em: analysis/handover_qos_metrics.csv" << std::endl;

    return 0;
}
