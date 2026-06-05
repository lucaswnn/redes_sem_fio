#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/vlc-channel-helper.h"
#include "ns3/vlc-device-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/olsr-helper.h"
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("VlcMobilityExperiment");

// Funções de trace para uso com TraceConnectWithoutContext
static void PhyTxTrace(Ptr<OutputStreamWrapper> stream, Ptr<const Packet> packet) {
    *stream->GetStream() << Simulator::Now().GetSeconds() << ",TX," << packet->GetSize() << std::endl;
}

static void PhyRxTrace(Ptr<OutputStreamWrapper> stream, Ptr<const Packet> packet) {
    *stream->GetStream() << Simulator::Now().GetSeconds() << ",RX," << packet->GetSize() << std::endl;
}

int main(int argc, char *argv[]) {
    double simTime = 10.0;
    std::string packetTracerFile = "vlc_mobility_trace.tr";

    CommandLine cmd;
    cmd.AddValue("simTime", "Simulation time (seconds)", simTime);
    cmd.AddValue("packetTracerFile", "File to save packets resume", packetTracerFile);
    cmd.Parse(argc, argv);

    NodeContainer nodes;
    nodes.Create(2);

    // Mobilidade baseada no exemplo do vlc-example.cc
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(250.0, 500.0, 0.0)); // Nó 0
    positionAlloc->Add(Vector(500.0, 500.0, 0.0)); // Nó 1
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::VlcMobilityModel");
    mobility.Install(nodes);

    VlcDeviceHelper devHelper;
    VlcChannelHelper chHelper;

    // Transmissor
    devHelper.CreateTransmitter("TX1");
    devHelper.SetTXSignal("TX1", 1000, 0.5, 0, 9.25e-5, 0);
    devHelper.SetTrasmitterParameter("TX1", "Bias", 0);
    devHelper.SetTrasmitterParameter("TX1", "SemiAngle", 35);
    devHelper.SetTrasmitterParameter("TX1", "Azimuth", 0);
    devHelper.SetTrasmitterParameter("TX1", "Elevation", 180.0);
    devHelper.SetTrasmitterPosition("TX1", 0.0, 0.0, 52.0);
    devHelper.SetTrasmitterParameter("TX1", "Gain", 70);
    devHelper.SetTrasmitterParameter("TX1", "DataRateInMBPS", 0.3);

    // Receptor
    devHelper.CreateReceiver("RX1");
    devHelper.SetReceiverParameter("RX1", "FilterGain", 1);
    devHelper.SetReceiverParameter("RX1", "RefractiveIndex", 1.5);
    devHelper.SetReceiverParameter("RX1", "FOVAngle", 28.5);
    devHelper.SetReceiverParameter("RX1", "ConcentrationGain", 0);
    devHelper.SetReceiverParameter("RX1", "PhotoDetectorArea", 1.3e-5);
    devHelper.SetReceiverParameter("RX1", "RXGain", 0);
    devHelper.SetReceiverParameter("RX1", "Beta", 1);
    devHelper.SetReceiverParameter("RX1", "SetModulationScheme", VlcErrorModel::OOK);

    // Canal
    chHelper.CreateChannel("CH1");
    chHelper.SetPropagationLoss("CH1", "VlcPropagationLoss");
    chHelper.SetPropagationDelay("CH1", 2);
    chHelper.AttachTransmitter("CH1", "TX1", &devHelper);
    chHelper.AttachReceiver("CH1", "RX1", &devHelper);
    chHelper.SetChannelParameter("CH1", "TEMP", 295);
    chHelper.SetChannelParameter("CH1", "BAND_FACTOR_NOISE_SIGNAL", 10.0);
    chHelper.SetChannelWavelength("CH1", 380, 780);
    chHelper.SetChannelParameter("CH1", "ElectricNoiseBandWidth", 3 * 1e5);

    NetDeviceContainer devices = chHelper.Install(nodes.Get(0), nodes.Get(1), &devHelper, &chHelper, "TX1", "RX1", "CH1");

    // Pilha de protocolos com OLSR
    OlsrHelper olsr;
    Ipv4StaticRoutingHelper staticRouting;
    Ipv4ListRoutingHelper list;
    list.Add(staticRouting, 0);
    list.Add(olsr, 10);
    InternetStackHelper internet;
    internet.SetRoutingHelper(list);
    internet.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    // Aplicação UDP simples
    uint16_t port = 4000;
    UdpEchoServerHelper echoServer(port);
    ApplicationContainer serverApps = echoServer.Install(nodes.Get(1));
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(simTime));

    UdpEchoClientHelper echoClient(interfaces.GetAddress(1), port);
    echoClient.SetAttribute("MaxPackets", UintegerValue(10));
    echoClient.SetAttribute("Interval", TimeValue(Seconds(1.0)));
    echoClient.SetAttribute("PacketSize", UintegerValue(1024));
    ApplicationContainer clientApps = echoClient.Install(nodes.Get(0));
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(simTime));

    // Trace
    AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream(packetTracerFile);
    devices.Get(0)->TraceConnectWithoutContext("PhyTxEnd", MakeBoundCallback(&PhyTxTrace, stream));
    devices.Get(1)->TraceConnectWithoutContext("PhyRxEnd", MakeBoundCallback(&PhyRxTrace, stream));

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
