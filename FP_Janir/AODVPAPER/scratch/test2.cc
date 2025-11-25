/*
 * AODV-D Performance Analysis Script
 * Output: aodv-d-results.csv (Time, Speed, PDR, Throughput, Delay)
 */

#include "ns3/aodv-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("AodvDMetrics");

int main (int argc, char *argv[])
{
  // --- PARAMETER SIMULASI (Bisa diubah lewat terminal) ---
  double nodeSpeed = 20.0; // Kecepatan Node (m/s) - Default 20 m/s (72 km/h)
  uint32_t nNodes = 20;    // Jumlah Kendaraan
  uint32_t simTime = 50;   // Durasi Simulasi (detik)
  std::string csvFileName = "aodv-d-results.csv";

  CommandLine cmd (__FILE__);
  cmd.AddValue ("speed", "Speed of nodes (m/s)", nodeSpeed);
  cmd.AddValue ("nodes", "Number of nodes", nNodes);
  cmd.AddValue ("time", "Simulation time", simTime);
  cmd.AddValue ("csv", "Output CSV file name", csvFileName);
  cmd.Parse (argc, argv);

  // --- KONFIGURASI WIFI (ADHOC) ---
  NodeContainer nodes;
  nodes.Create (nNodes);

  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211b);

  YansWifiPhyHelper wifiPhy;
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());

  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");

  NetDeviceContainer devices;
  devices = wifi.Install (wifiPhy, wifiMac, nodes);

  // --- MOBILITAS (PENTING UNTUK AODV-D) ---
  MobilityHelper mobility;
  // Area Simulasi 1000x1000 meter
  mobility.SetPositionAllocator ("ns3::RandomRectanglePositionAllocator",
                                 "X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"),
                                 "Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"));
  
  // Model Gerak: RandomWaypoint dengan Kecepatan KONSTAN (sesuai parameter input)
  // Agar kita bisa menguji dampak kecepatan terhadap protokol
  std::stringstream speedString;
  speedString << "ns3::ConstantRandomVariable[Constant=" << nodeSpeed << "]";
  mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                             "Speed", StringValue (speedString.str ()),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"),
                             "PositionAllocator", StringValue ("ns3::RandomRectanglePositionAllocator"));
  
  mobility.Install (nodes);

  // --- PROTOKOL ROUTING (AODV-D) ---
  AodvHelper aodv; 
  // Kita gunakan bobot default (Ws=1.0, Wd=1.0) yang sudah diset di kode .cc
  
  Ipv4ListRoutingHelper list;
  list.Add (aodv, 100);

  InternetStackHelper internet;
  internet.SetRoutingHelper (list);
  internet.Install (nodes);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = ipv4.Assign (devices);

  // --- TRAFFIC APLIKASI (UDP) ---
  // Skenario: 5 Pasang Komunikasi Acak
  uint16_t port = 9;
  OnOffHelper onoff ("ns3::UdpSocketFactory", Address ());
  onoff.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  onoff.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
  onoff.SetAttribute ("DataRate", StringValue ("2kbps")); // Beban trafik ringan-sedang
  onoff.SetAttribute ("PacketSize", UintegerValue (1024));

  ApplicationContainer apps;

  // Buat 5 aliran data (Flow) dari node genap ke node ganjil
  for (uint32_t i = 0; i < 10; i += 2) // Node 0->1, 2->3, dst...
  {
      AddressValue remoteAddress (InetSocketAddress (interfaces.GetAddress (i + 1), port));
      onoff.SetAttribute ("Remote", remoteAddress);
      apps.Add (onoff.Install (nodes.Get (i))); // Sender

      PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), port));
      apps.Add (sink.Install (nodes.Get (i + 1))); // Receiver
  }

  apps.Start (Seconds (1.0));
  apps.Stop (Seconds (simTime - 1.0));

  // --- FLOW MONITOR (PENGUMPUL DATA) ---
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

  NS_LOG_UNCOND ("--- SIMULASI BERJALAN ---");
  NS_LOG_UNCOND ("Nodes: " << nNodes << ", Speed: " << nodeSpeed << " m/s");

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  // --- ANALISIS & OUTPUT CSV ---
  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();

  double totalTx = 0;
  double totalRx = 0;
  double totalThroughput = 0;
  double totalDelay = 0;
  uint32_t flows = 0;

  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
  {
      // Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
      if (i->second.rxPackets > 0)
      {
          flows++;
          totalTx += i->second.txPackets;
          totalRx += i->second.rxPackets;
          
          // Throughput dalam Kbps
          double throughput = i->second.rxBytes * 8.0 / (simTime * 1000); 
          totalThroughput += throughput;

          // Delay dalam ms
          totalDelay += i->second.delaySum.GetSeconds();
      }
  }

  double avgPDR = 0;
  if (totalTx > 0) avgPDR = (totalRx / totalTx) * 100.0;
  
  double avgDelay = 0;
  if (totalRx > 0) avgDelay = (totalDelay / totalRx) * 1000.0; // ms

  // Output ke Layar
  std::cout << "  > Total Tx Packets: " << totalTx << std::endl;
  std::cout << "  > Total Rx Packets: " << totalRx << std::endl;
  std::cout << "  > Packet Delivery Ratio (PDR): " << avgPDR << " %" << std::endl;
  std::cout << "  > Total Throughput: " << totalThroughput << " Kbps" << std::endl;
  std::cout << "  > Average Delay: " << avgDelay << " ms" << std::endl;

  // Output ke CSV (Append Mode)
  std::ofstream out (csvFileName.c_str (), std::ios::app);
  
  // Cek apakah file kosong (jika ya, tulis Header dulu)
  std::ifstream testEmpty(csvFileName);
  if (testEmpty.peek() == std::ifstream::traits_type::eof()) {
      out << "Speed(m/s),Nodes,PDR(%),Throughput(Kbps),Delay(ms)" << std::endl;
  }
  
  out << nodeSpeed << "," << nNodes << "," << avgPDR << "," << totalThroughput << "," << avgDelay << std::endl;
  out.close ();

  NS_LOG_UNCOND ("Data disimpan ke " << csvFileName);

  Simulator::Destroy ();
  return 0;
}
