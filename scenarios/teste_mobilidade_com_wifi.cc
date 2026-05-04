/*
Teste de mobilidade usando protocolo Wi-Fi
*/

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiMobilityHandover");

// callback de associação
void AssocCallback(std::string context, Mac48Address bssid)
{
    std::cout << "t=" << Simulator::Now().GetSeconds()
              << "s -> Associou com BSSID: " << bssid << std::endl;

    // você pode mapear manualmente os APs
    // (vamos assumir ordem de criação)
    // AP1 = primeiro, AP2 = segundo
    static bool first = true;

    if (first)
    {
        std::cout << ">>> AP1 connected\n";
        first = false;
    }
    else
    {
        std::cout << ">>> AP2 connected\n";
    }
}

int main(int argc, char *argv[])
{
    NodeContainer apNodes;
    apNodes.Create(2);

    NodeContainer staNode;
    staNode.Create(1);

    // ===== MOBILIDADE =====

    // APs fixos
    MobilityHelper mobilityAp;

    Ptr<ListPositionAllocator> apPos = CreateObject<ListPositionAllocator>();
    apPos->Add(Vector(0.0, 0.0, 0.0));     // AP1
    apPos->Add(Vector(100.0, 0.0, 0.0));   // AP2

    mobilityAp.SetPositionAllocator(apPos);
    mobilityAp.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityAp.Install(apNodes);

    // STA móvel
    MobilityHelper mobilitySta;
    mobilitySta.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobilitySta.Install(staNode);

    Ptr<ConstantVelocityMobilityModel> mob =
        staNode.Get(0)->GetObject<ConstantVelocityMobilityModel>();

    mob->SetPosition(Vector(-50.0, 0.0, 0.0));
    mob->SetVelocity(Vector(10.0, 0.0, 0.0));

    // ===== WIFI =====

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g);

    YansWifiPhyHelper phy;
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    phy.SetChannel(channel.Create());

    WifiMacHelper mac;

    // SSIDs diferentes
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

    // STA (pode conectar em qualquer um)
    mac.SetType("ns3::StaWifiMac",
                "ActiveProbing", BooleanValue(true));

    NetDeviceContainer staDev = wifi.Install(phy, mac, staNode);

    // ===== INTERNET STACK (necessário para wifi funcionar direito) =====
    InternetStackHelper stack;
    stack.Install(apNodes);
    stack.Install(staNode);

    // ===== TRACE DE ASSOCIAÇÃO =====
    Config::Connect(
        "/NodeList/2/DeviceList/0/$ns3::WifiNetDevice/Mac/$ns3::StaWifiMac/Assoc",
        MakeCallback(&AssocCallback));

    Simulator::Stop(Seconds(20.0));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
