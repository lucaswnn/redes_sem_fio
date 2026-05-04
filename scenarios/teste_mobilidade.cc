/*
Cenário para testar a funcionalidade de mobilidade
Apenas para teste
*/

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("teste_mobilidade");

// raio de cobertura dos APs (metros)
double coverageRadius = 60.0;

// estados anteriores (para detectar entrada/saída)
bool inRangeAp1 = false;
bool inRangeAp2 = false;

void CheckPosition(Ptr<Node> sta, Ptr<Node> ap1, Ptr<Node> ap2)
{
    Ptr<MobilityModel> mobSta = sta->GetObject<MobilityModel>();
    Ptr<MobilityModel> mobAp1 = ap1->GetObject<MobilityModel>();
    Ptr<MobilityModel> mobAp2 = ap2->GetObject<MobilityModel>();

    Vector posSta = mobSta->GetPosition();
    Vector posAp1 = mobAp1->GetPosition();
    Vector posAp2 = mobAp2->GetPosition();

    double d1 = mobSta->GetDistanceFrom(mobAp1);
    double d2 = mobSta->GetDistanceFrom(mobAp2);

    bool nowInAp1 = d1 <= coverageRadius;
    bool nowInAp2 = d2 <= coverageRadius;

    std::cout << "t=" << Simulator::Now().GetSeconds()
              << "s | pos=(" << posSta.x << ", " << posSta.y << ")"
              << " | d(AP1)=" << d1
              << " | d(AP2)=" << d2
              << std::endl;

    // eventos AP1
    if (nowInAp1 && !inRangeAp1)
        std::cout << "  -> Entrou no alcance do AP1\n";

    if (!nowInAp1 && inRangeAp1)
        std::cout << "  -> Saiu do alcance do AP1\n";

    // eventos AP2
    if (nowInAp2 && !inRangeAp2)
        std::cout << "  -> Entrou no alcance do AP2\n";

    if (!nowInAp2 && inRangeAp2)
        std::cout << "  -> Saiu do alcance do AP2\n";

    // overlap
    if (nowInAp1 && nowInAp2)
        std::cout << "  -> Está na REGIÃO DE SOBREPOSIÇÃO\n";

    inRangeAp1 = nowInAp1;
    inRangeAp2 = nowInAp2;

    // agenda próxima checagem
    Simulator::Schedule(Seconds(1.0), &CheckPosition, sta, ap1, ap2);
}

int main(int argc, char *argv[])
{
    NodeContainer apNodes;
    apNodes.Create(2);

    NodeContainer staNode;
    staNode.Create(1);

    // ===== APs fixos =====
    MobilityHelper mobilityAp;

    Ptr<ListPositionAllocator> apPos = CreateObject<ListPositionAllocator>();
    apPos->Add(Vector(0.0, 0.0, 0.0));     // AP1
    apPos->Add(Vector(100.0, 0.0, 0.0));   // AP2

    mobilityAp.SetPositionAllocator(apPos);
    mobilityAp.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityAp.Install(apNodes);

    // ===== STA móvel =====
    MobilityHelper mobilitySta;
    mobilitySta.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobilitySta.Install(staNode);

    Ptr<ConstantVelocityMobilityModel> mob =
        staNode.Get(0)->GetObject<ConstantVelocityMobilityModel>();

    mob->SetPosition(Vector(-50.0, 0.0, 0.0)); // começa fora
    mob->SetVelocity(Vector(10.0, 0.0, 0.0));  // 10 m/s →

    // ===== iniciar monitoramento =====
    Simulator::Schedule(Seconds(0.0), &CheckPosition,
                        staNode.Get(0),
                        apNodes.Get(0),
                        apNodes.Get(1));

    Simulator::Stop(Seconds(20.0));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
