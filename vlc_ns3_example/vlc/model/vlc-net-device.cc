#include "ns3/vlc-net-device.h"

namespace ns3
{

	NS_LOG_COMPONENT_DEFINE("vlcNetDevice");

	NS_OBJECT_ENSURE_REGISTERED(VlcNetDevice);

	ns3::TypeId VlcNetDevice::GetTypeId(void) // returns meta-information about VLC_ErrorModel class
	{										  // including parent class, group name, constructor, and attributes
		static ns3::TypeId tid = ns3::TypeId("VlcNetDevice").SetParent<ns3::PointToPointNetDevice>().AddConstructor<VlcNetDevice>();
		return tid;
	}

	VlcNetDevice::VlcNetDevice()
	{ //: m_txMachineState(READY), m_channel(0), m_linkUp(false), m_currentPkt(0){
		m_mobilityModel = ns3::CreateObject<VlcMobilityModel>();
		m_mobilityModel->SetAzimuth(0);
		m_mobilityModel->SetElevation(0);
		m_mobilityModel->SetPosition(ns3::Vector(0.0, 0.0, 0.0));
	}

	double VlcNetDevice::GetAzmuth()
	{
		return this->m_mobilityModel->GetAzimuth();
	}

	void VlcNetDevice::SetAzmuth(double az)
	{
		this->m_mobilityModel->SetAzimuth(az);
	}

	ns3::Vector VlcNetDevice::GetPosition()
	{
		return this->m_mobilityModel->GetPosition();
	}

	void VlcNetDevice::SetPosition(ns3::Vector position)
	{
		m_mobilityModel->SetPosition(position);
	}

	double VlcNetDevice::GetElevation()
	{
		return m_mobilityModel->GetElevation();
	}

	void VlcNetDevice::SetElevation(double elevation)
	{

		m_mobilityModel->SetElevation(elevation);
	}

	ns3::Ptr<VlcMobilityModel> VlcNetDevice::GetMobilityModel()
	{
		return m_mobilityModel;
	}
	void VlcNetDevice::SetMobilityModel(ns3::Ptr<VlcMobilityModel> model)
	{
		m_mobilityModel = model;
	}

	VlcNetDevice::~VlcNetDevice()
	{
	}

	void VlcNetDevice::Receive(Ptr<Packet> p)
	{
		return;
	}

	bool VlcNetDevice::Send(Ptr<Packet> packet, const Address &dest,
							uint16_t protocolNumber)
	{
		return PointToPointNetDevice::Send(packet, dest, protocolNumber);
	}