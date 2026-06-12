#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/ping-helper.h"

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
#include <functional>

NS_LOG_COMPONENT_DEFINE("VlcHandoverExperiment");

struct SnrRecord
{
    double time;
    std::vector<double> snrs;
};

using MobilityFunc = std::function<ns3::Vector(double time, ns3::Vector currentPos)>;
using HandoverFunc = std::function<int32_t(double time,
                                           const std::vector<SnrRecord> &snrhist,
                                           int32_t currentAp)>;

struct Scenario
{
    std::string name;
    double simDurationSeconds;
    double scheduleIntervalSeconds;
    double metricsIntervalSeconds;
    std::string strategy;
    double thresholdDb;
    double hysteresisDb;
    double ttt;
    double ambientNoiseDb;
    double shadowStdDb;

    std::vector<ns3::Vector> apPositions;
    ns3::Vector ueFirstPosition;
    MobilityFunc ueMobilityFunc;
    HandoverFunc handoverFunc;
};

class VlcHandoverScenario
{
public:
    VlcHandoverScenario(const Scenario &sc);

    void RunSimulation();

private:
    void ConfigureRx(std::string rx);
    void ConfigureTx(std::string tx, uint32_t i);
    void ConfigureCh(std::string ch,
                     std::string tx,
                     std::string rx,
                     uint32_t node);

    void CreateNodes();
    void CreateMobility();
    void CreateVlcDevices();
    void CreateChannels();
    void InstallInternet();

    void PrintApInfo();
    void SetSchedule();

    void UpdateUeMobility();
    void CollectMetrics();
    void EvaluateHandover();

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
    std::vector<ns3::Ipv4Address> m_apIps;

    int32_t m_currentApIndex;
    std::vector<SnrRecord> m_snrHistory;
};

VlcHandoverScenario::VlcHandoverScenario(const Scenario &sc) : m_sc(sc),
                                                               m_nAps(sc.apPositions.size()),
                                                               m_apsPos(sc.apPositions),
                                                               m_currentApIndex(-1)

{
    std::cout << "Iniciando preparo da simulação...\n";

    CreateNodes();
    CreateMobility();
    CreateVlcDevices();
    CreateChannels();
    InstallInternet();
    SetSchedule();

    std::cout << "Preparo inicial da simulação concluído.\n";
}

void VlcHandoverScenario::ConfigureRx(std::string rx)
{
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

void VlcHandoverScenario::ConfigureTx(std::string tx, uint32_t node)
{
    m_devHelper.CreateTransmitter(tx);
    m_devHelper.SetTXSignal(tx, 1000, 0.5, 0, 9.25e-5, 0);
    m_devHelper.SetTrasmitterParameter(tx, "Bias", 0);
    m_devHelper.SetTrasmitterParameter(tx, "SemiAngle", 35);
    m_devHelper.SetTrasmitterParameter(tx, "Azimuth", 0);
    m_devHelper.SetTrasmitterParameter(tx, "Elevation", 270.0);
    m_devHelper.SetTrasmitterParameter(tx, "Gain", 70);
    m_devHelper.SetTrasmitterParameter(tx, "DataRateInMBPS", 0.3);

    auto txMob = m_aps.Get(node)->GetObject<ns3::VlcMobilityModel>();
    auto txPos = txMob->GetPosition();
    m_devHelper.SetTrasmitterPosition(tx,
                                      txPos.x,
                                      txPos.y,
                                      txPos.z);
}

void VlcHandoverScenario::ConfigureCh(std::string ch,
                                      std::string tx,
                                      std::string rx,
                                      uint32_t node)
{
    std::cout << "Criando canal " << tx << "<->" << rx << "...\n";

    m_chHelper.CreateChannel(ch);
    m_chHelper.SetPropagationLoss(ch, "VlcPropagationLoss");
    m_chHelper.SetPropagationDelay(ch, 2);
    m_chHelper.SetChannelParameter(ch, "TEMP", 295);
    m_chHelper.SetChannelParameter(ch, "BAND_FACTOR_NOISE_SIGNAL", 10.0);
    m_chHelper.SetChannelWavelength(ch, 380, 780);
    m_chHelper.SetChannelParameter(ch, "ElectricNoiseBandWidth", 3 * 1e5);
    m_chHelper.AttachTransmitter(ch, tx, &m_devHelper);
    m_chHelper.AttachReceiver(ch, rx, &m_devHelper);

    double ambientNoisePower = std::pow(10, (m_sc.ambientNoiseDb - 30) / 10);
    m_chHelper.SetChannelAmbientNoisePower(ch, ambientNoisePower);

    ns3::NetDeviceContainer devs = m_chHelper.Install(m_aps.Get(node),
                                                      m_ue.Get(0),
                                                      &m_devHelper,
                                                      &m_chHelper,
                                                      tx,
                                                      rx,
                                                      ch);

    m_devs.Add(devs);

    std::cout << "Canal " << tx << " <-> " << rx << " criado.\n";
}

void VlcHandoverScenario::CreateNodes()
{
    std::cout << "Iniciando criação dos nós...\n";

    m_aps.Create(m_nAps);
    m_ue.Create(1);
    m_allNodes.Add(m_aps);
    m_allNodes.Add(m_ue);

    std::cout << "Criação dos nós feita com sucesso.\n";
}

void VlcHandoverScenario::CreateMobility()
{
    std::cout << "Iniciando configuração de mobilidade...\n";

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

    ueMob->SetPosition(m_sc.ueFirstPosition);
    ueMob->SetAzimuth(0);
    ueMob->SetElevation(90);

    std::cout << "Configuração de mobilidade feita com sucesso.\n";
}

void VlcHandoverScenario::CreateVlcDevices()
{
    std::cout << "Iniciando a criação dos dispositivos VLC...\n";

    for (uint32_t i = 0; i < m_nAps; i++)
    {
        std::string tx = "TX_" + std::to_string(i);
        ConfigureTx(tx, i);
    }

    for (uint32_t i = 0; i < m_nAps; i++)
    {
        std::string rx = "RX_" + std::to_string(i);
        ConfigureRx(rx);
    }

    std::cout << "Criação dos dispositivos VLC feita com sucesso.\n";
}

void VlcHandoverScenario::CreateChannels()
{
    std::cout << "Iniciando a criação dos canais ópticos...\n";

    for (uint32_t i = 0; i < m_nAps; i++)
    {
        std::string ch = "CH_" + std::to_string(i);
        std::string tx = "TX_" + std::to_string(i);
        std::string rx = "RX_" + std::to_string(i);

        ConfigureCh(ch, tx, rx, i);
    }

    std::cout << "Criação dos canais ópticos feita com sucesso.\n";
}

void VlcHandoverScenario::InstallInternet()
{
    ns3::InternetStackHelper internet;
    internet.Install(m_allNodes);

    ns3::Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    m_ipInterfs = ipv4.Assign(m_devs);

    std::cout << "Configuração:" << std::endl;
    std::cout << "# devices: " << m_devs.GetN() << "\n";
    std::cout << "# interfaces: " << m_ipInterfs.GetN() << "\n";

    auto ueId = m_ue.Get(0)->GetId();
    for (uint32_t i = 0; i < m_devs.GetN(); i++)
    {
        auto dev = m_devs.Get(i);
        auto id = dev->GetNode()->GetId();
        if (id != ueId)
        {
            m_apIps.push_back(m_ipInterfs.GetAddress(i));
        }
    }
}

void VlcHandoverScenario::PrintApInfo()
{
    for (uint32_t i = 0; i < m_nAps; i++)
    {
        auto ap = m_aps.Get(i);
        auto mob = ap->GetObject<ns3::MobilityModel>();
        auto pos = mob->GetPosition();
        std::cout
            << "IP: " << m_apIps[i]
            << " (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
    }
}

void VlcHandoverScenario::SetSchedule()
{
    PrintApInfo();

    ns3::Simulator::Schedule(ns3::Seconds(0.0),
                             &VlcHandoverScenario::UpdateUeMobility,
                             this);

    ns3::Simulator ::Schedule(ns3::Seconds(0.0),
                              &VlcHandoverScenario::CollectMetrics,
                              this);

    ns3::Simulator::Schedule(ns3::Seconds(0.0),
                             &VlcHandoverScenario::EvaluateHandover,
                             this);
}

void VlcHandoverScenario::RunSimulation()
{

    ns3::Simulator::Stop(ns3::Seconds(m_sc.simDurationSeconds));
    ns3::Simulator::Run();
    ns3::Simulator::Destroy();
}

void VlcHandoverScenario::UpdateUeMobility()
{
    double t = ns3::Simulator::Now().GetSeconds();

    auto ueMob = m_ue.Get(0)->GetObject<ns3::VlcMobilityModel>();
    auto curPos = ueMob->GetPosition();
    auto newPos = m_sc.ueMobilityFunc(t, curPos);

    ueMob->SetPosition(newPos);

    for (uint32_t i = 0; i < m_nAps; i++)
    {
        std::string rx = "RX_" + std::to_string(i);
        auto ueRxMob = m_devHelper.GetReceiver(rx);
        ueRxMob->SetPosition(newPos);
    }

    ns3::Simulator::Schedule(ns3::Seconds(m_sc.scheduleIntervalSeconds),
                             &VlcHandoverScenario::UpdateUeMobility,
                             this);
}

void VlcHandoverScenario::CollectMetrics()
{
    double t = ns3::Simulator::Now().GetSeconds();
    auto ueMob = m_ue.Get(0)->GetObject<ns3::VlcMobilityModel>();
    auto pos = ueMob->GetPosition();

    std::cout
        << "t=" << t
        << " x=" << pos.x
        << " y=" << pos.y
        << " z=" << pos.z
        << "\n";

    SnrRecord currentRecord;
    currentRecord.time = t;

    for (uint32_t i = 0; i < m_nAps; i++)
    {
        std::string ch = "CH_" + std::to_string(i);
        double snrVal = m_chHelper.GetChannelSNR(ch);
        currentRecord.snrs.push_back(snrVal);
        std::cout << "SNR[" << i << "]=" << snrVal << "\n";
    }

    m_snrHistory.push_back(currentRecord);

    ns3::Simulator::Schedule(ns3::Seconds(m_sc.metricsIntervalSeconds),
                             &VlcHandoverScenario::CollectMetrics,
                             this);
}

void VlcHandoverScenario::EvaluateHandover()
{
    double t = ns3::Simulator::Now().GetSeconds();

    if (m_sc.handoverFunc)
    {
        int32_t target = m_sc.handoverFunc(t, m_snrHistory, m_currentApIndex);
        if (target != m_currentApIndex)
        {
            std::cout << "Handover: de AP " << m_currentApIndex
                      << " para AP " << target << "\n";
            m_currentApIndex = target;
        }
    }

    ns3::Simulator::Schedule(ns3::Seconds(m_sc.scheduleIntervalSeconds),
                             &VlcHandoverScenario::EvaluateHandover,
                             this);
}

int main(int argc, char **argv)
{
    double vel = 1.0;
    std::string name = "Baseline_v" + std::to_string((int)vel);
    double simDurationSeconds = 20.0;
    double scheduleIntervalSeconds = 0.1;
    double metricsIntervalSeconds = 1.0;
    std::string strategy = "Baseline";
    double thresholdDb = 15.0;
    double hysteresisDb = 0.0;
    double ttt = 0.1;
    double ambientNoiseDb = 0.0;
    double shadowStdDb = 0.5;

    std::vector apPositions = {ns3::Vector(0.0, 0.0, 2.8),
                               ns3::Vector(10.0, 0.0, 2.8),
                               ns3::Vector(20.0, 0.0, 2.8),
                               ns3::Vector(0.0, 10.0, 2.8),
                               ns3::Vector(10.0, 10.0, 2.8),
                               ns3::Vector(20.0, 10.0, 2.8)};

    ns3::Vector ueFirstPosition = ns3::Vector(0.0, 0.0, 1.2);

    MobilityFunc ueMobilityFunc = [vel](double t, ns3::Vector curPos)
    { return ns3::Vector(vel * t, curPos.y, curPos.z); };

    HandoverFunc handoverFunc = [](double t,
                                   const std::vector<SnrRecord> &snrHist,
                                   int32_t curAp)
    { return -1; };

    Scenario sc = {name,
                   simDurationSeconds,
                   scheduleIntervalSeconds,
                   metricsIntervalSeconds,
                   strategy,
                   thresholdDb,
                   hysteresisDb,
                   ttt,
                   ambientNoiseDb,
                   shadowStdDb,
                   apPositions,
                   ueFirstPosition,
                   ueMobilityFunc,
                   handoverFunc};

    auto simulation = VlcHandoverScenario(sc);
    simulation.RunSimulation();
}