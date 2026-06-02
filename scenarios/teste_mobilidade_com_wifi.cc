/*
Teste de mobilidade usando protocolo Wi-Fi
AP2 liga após 20 s
*/

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiMobilityHandoverControlled");

// BSSIDs globais
Mac48Address ap1Bssid;
Mac48Address ap2Bssid;

// ===== Callback de associação =====
void AssocCallback(Mac48Address bssid)
{
    std::cout << "t=" << Simulator::Now().GetSeconds()
              << "s -> Associou com BSSID: " << bssid << std::endl;

    if (bssid == ap1Bssid)
    {
        std::cout << ">>> AP1 connected\n";
    }
    else if (bssid == ap2Bssid)
    {
        std::cout << ">>> AP2 connected\n";
    }
    else
    {
        std::cout << ">>> AP desconhecido\n";
    }
}

// ===== Função para ligar AP2 =====
void EnableAp2(Ptr<YansWifiPhy> phy)
{
    phy->SetTxPowerStart(16.0);
    phy->SetTxPowerEnd(16.0);

    std::cout << "t=" << Simulator::Now().GetSeconds()
              << "s -> AP2 LIGADO\n";
}

int main(int argc, char *argv[])
{
    NodeContainer apNodes;
    apNodes.Create(2);

    NodeContainer staNode;
    staNode.Create(1);

    // ===== MOBILIDADE =====

    MobilityHelper mobilityAp;

    Ptr<ListPositionAllocator> apPos = CreateObject<ListPositionAllocator>();
    apPos->Add(Vector(0.0, 0.0, 0.0));     // AP1
    apPos->Add(Vector(200.0, 0.0, 0.0));   // AP2

    mobilityAp.SetPositionAllocator(apPos);
    mobilityAp.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityAp.Install(apNodes);

    MobilityHelper mobilitySta;
    mobilitySta.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobilitySta.Install(staNode);

    Ptr<ConstantVelocityMobilityModel> mob =
        staNode.Get(0)->GetObject<ConstantVelocityMobilityModel>();

    mob->SetPosition(Vector(-1500.0, 0.0, 0.0));
    mob->SetVelocity(Vector(1.0, 0.0, 0.0));

    // ===== WIFI =====

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g);

    YansWifiPhyHelper phy;
    YansWifiChannelHelper channel;

    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                               "Exponent", DoubleValue(4.0));

    phy.SetChannel(channel.Create());

    WifiMacHelper mac;

    Ssid ssid1 = Ssid("AP1-network");
    Ssid ssid2 = Ssid("AP2-network");

    // AP1
    mac.SetType("ns3::ApWifiMac",
                "Ssid", SsidValue(ssid1));
    NetDeviceContainer apDev1 = wifi.Install(phy, mac, apNodes.Get(0));

    // AP2
    mac.SetType("ns3::ApWifiMac",
                "Ssid", SsidValue(ssid2));
    NetDeviceContainer apDev2 = wifi.Install(phy, mac, apNodes.Get(1));

    // STA
    mac.SetType("ns3::StaWifiMac",
                "ActiveProbing", BooleanValue(false));
    NetDeviceContainer staDev = wifi.Install(phy, mac, staNode);

    // ===== PEGAR BSSIDs =====

    Ptr<WifiNetDevice> apDevPtr1 = DynamicCast<WifiNetDevice>(apDev1.Get(0));
    Ptr<WifiNetDevice> apDevPtr2 = DynamicCast<WifiNetDevice>(apDev2.Get(0));

    ap1Bssid = apDevPtr1->GetMac()->GetAddress();
    ap2Bssid = apDevPtr2->GetMac()->GetAddress();

    std::cout << "AP1 BSSID: " << ap1Bssid << std::endl;
    std::cout << "AP2 BSSID: " << ap2Bssid << std::endl;

    // ===== DESLIGAR AP2 INICIALMENTE =====

    Ptr<WifiNetDevice> ap2Dev = DynamicCast<WifiNetDevice>(apDev2.Get(0));
    Ptr<YansWifiPhy> ap2Phy = DynamicCast<YansWifiPhy>(ap2Dev->GetPhy());

    ap2Phy->SetTxPowerStart(0.0);
    ap2Phy->SetTxPowerEnd(0.0);

    std::cout << "t=0s -> AP2 DESLIGADO\n";

    // ===== INTERNET =====
    InternetStackHelper stack;
    stack.Install(apNodes);
    stack.Install(staNode);

    // ===== TRACE =====
    Ptr<WifiNetDevice> staDevPtr = DynamicCast<WifiNetDevice>(staDev.Get(0));

    staDevPtr->GetMac()->TraceConnectWithoutContext(
        "Assoc", MakeCallback(&AssocCallback));

    // ===== AGENDAR LIGAMENTO DO AP2 =====
    Simulator::Schedule(Seconds(20.0), &EnableAp2, ap2Phy);

    Simulator::Stop(Seconds(10000.0));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
