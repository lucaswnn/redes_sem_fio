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

VlcHandoverScenario::VlcHandoverScenario(const Scenario &sc) : m_sc(sc),
                                                               m_nAps(6)

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
    ns3::InternetStackHelper internet;
    internet.Install(m_allNodes);

    ns3::Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
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
    simulation.RunSimulation();
}