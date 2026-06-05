#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/vlc-channel-helper.h"
#include "ns3/vlc-device-helper.h"
#include "ns3/ping-helper.h"
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("TestVlcConnectivity");

int main(int argc, char *argv[]) {
    std::cout << "🧪 TESTE DE CONECTIVIDADE VLC - 2 NÓS" << std::endl;
    
    // Criar 2 nós
    NodeContainer nodes;
    nodes.Create(2);
    
    // Configurar mobilidade
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));
    positionAlloc->Add(Vector(2.0, 0.0, 0.0));  // 2m de distância
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);
    
    // Configurar dispositivos VLC (mesma configuração das simulações unificadas)
    VlcDeviceHelper devHelper;
    VlcChannelHelper chHelper;
    NetDeviceContainer allDevices;
    
    // Criar transmissores e receptores
    for (uint32_t i = 0; i < 2; ++i) {
        Ptr<ConstantPositionMobilityModel> nodeMobility = 
            nodes.Get(i)->GetObject<ConstantPositionMobilityModel>();
        Vector pos = nodeMobility->GetPosition();
        
        std::string txName = "TX_" + std::to_string(i);
        std::string rxName = "RX_" + std::to_string(i);
        
        devHelper.CreateTransmitter(txName);
        devHelper.SetTXSignal(txName, 1000, 0.5, 0, 9.25e-5, 0);
        devHelper.SetTrasmitterParameter(txName, "SemiAngle", 35);
        devHelper.SetTrasmitterParameter(txName, "Azimuth", 0);
        devHelper.SetTrasmitterParameter(txName, "Elevation", 180);
        devHelper.SetTrasmitterPosition(txName, pos.x, pos.y, pos.z);
        
        devHelper.CreateReceiver(rxName);
        devHelper.SetReceiverParameter(rxName, "FOVAngle", 28.5);
        devHelper.SetReceiverParameter(rxName, "PhotoDetectorArea", 1.3e-5);
    }
    
    // Criar canal VLC entre os 2 nós
    std::string chName = "CH_0_1";
    std::string txName = "TX_0";
    std::string rxName = "RX_1";
    
    chHelper.CreateChannel(chName);
    chHelper.SetPropagationLoss(chName, "VlcPropagationLoss");
    chHelper.AttachTransmitter(chName, txName, &devHelper);
    chHelper.AttachReceiver(chName, rxName, &devHelper);
    chHelper.SetChannelWavelength(chName, 380, 780);
    chHelper.SetChannelParameter(chName, "ElectricNoiseBandwidth", 3e5);
    
    NetDeviceContainer devices = chHelper.Install(
        nodes.Get(0), nodes.Get(1), &devHelper, &chHelper, 
        txName.c_str(), rxName.c_str(), chName.c_str());
    allDevices.Add(devices);
    
    std::cout << "[CHANNEL] Criado canal VLC entre Nó 0 e Nó 1" << std::endl;
    
    // Instalar pilha IP (sem protocolo de roteamento específico)
    InternetStackHelper internet;
    internet.Install(nodes);
    
    // Atribuir endereços IP
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(allDevices);
    
    std::cout << "📊 Configuração:" << std::endl;
    std::cout << "   - Nó 0: " << interfaces.GetAddress(0) << std::endl;
    std::cout << "   - Nó 1: " << interfaces.GetAddress(1) << std::endl;
    
    // Teste de ping do Nó 0 para o Nó 1
    PingHelper ping(interfaces.GetAddress(1));
    ping.SetAttribute("VerboseMode", EnumValue(Ping::VerboseMode::VERBOSE));
    ping.SetAttribute("Interval", TimeValue(Seconds(1.0)));
    ping.SetAttribute("Size", UintegerValue(56));
    
    ApplicationContainer app = ping.Install(nodes.Get(0));
    app.Start(Seconds(2.0));
    app.Stop(Seconds(10.0));
    
    std::cout << "[PING] Iniciando ping do Nó 0 para o Nó 1..." << std::endl;
    
    Simulator::Stop(Seconds(15.0));
    Simulator::Run();
    Simulator::Destroy();
    
    std::cout << "✅ Teste de conectividade VLC concluído!" << std::endl;
    return 0;
} 
