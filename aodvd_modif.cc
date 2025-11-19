/* scratch/aodv-d-final.cpp
 *
 * AODV-D final: AODV + LET + Dijkstra + Node Selection (speed & direction)
 * Compatible dengan NS-3.43
 *
 * Fitur:
 * - Node selection: hanya tetangga dengan |Δspeed| <= SpeedThreshold dan
 *   |Δheading| <= AngleThreshold (derajat) yang dipertimbangkan.
 * - Compute LET antara pasangan node.
 * - Dijkstra dengan bobot = 1 / LET (memilih rute dengan LET agregat terbesar).
 * - Install host static routes (snapshot) sepanjang path.
 * - Output throughput per detik ke CSV.
 *
 * Author: Adapted by ChatGPT (for user)
 */

#include "ns3/aodv-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/yans-wifi-helper.h"

#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <tuple>
#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("AODV-D-FINAL");

// ---------------------- UTIL: LET & Dijkstra with Node Selection ----------------------

double ComputeLET(Ptr<MobilityModel> m1, Ptr<MobilityModel> m2, double R)
{
    Vector p1 = m1->GetPosition();
    Vector p2 = m2->GetPosition();
    Vector v1 = m1->GetVelocity();
    Vector v2 = m2->GetVelocity();

    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dvx = v1.x - v2.x;
    double dvy = v1.y - v2.y;

    double a = dvx*dvx + dvy*dvy;
    double b = 2.0*(dx*dvx + dy*dvy);
    double c = dx*dx + dy*dy - R*R;

    if (std::abs(a) < 1e-9)
    {
        if (c <= 0.0) return std::numeric_limits<double>::infinity();
        return 0.0;
    }

    double disc = b*b - 4.0*a*c;
    if (disc < 0.0)
    {
        if (c <= 0.0) return std::numeric_limits<double>::infinity();
        return 0.0;
    }

    double sqrtD = std::sqrt(disc);
    double t1 = (-b + sqrtD) / (2.0*a);
    double t2 = (-b - sqrtD) / (2.0*a);

    double tExit = std::numeric_limits<double>::infinity();
    if (t1 > 0.0) tExit = std::min(tExit, t1);
    if (t2 > 0.0) tExit = std::min(tExit, t2);

    if (tExit == std::numeric_limits<double>::infinity())
    {
        if (c <= 0.0) return std::numeric_limits<double>::infinity();
        return 0.0;
    }
    return tExit;
}

/*
 BuildLetGraph with Node Selection:
 - hanya tetangga dengan |Δspeed| <= speedTh dan |Δheading| <= angleTh (deg) yang dipertimbangkan.
 - R adalah radio range (m).
*/
std::vector<std::vector<std::pair<int,double>>> BuildLetGraph(
    const NodeContainer &nodes, double R, double speedTh, double angleThDeg)
{
    uint32_t N = nodes.GetN();
    std::vector<std::vector<std::pair<int,double>>> adj(N);

    auto radToDeg = [](double r) { return r * 180.0 / M_PI; };
    for (uint32_t i = 0; i < N; ++i)
    {
        Ptr<MobilityModel> mi = nodes.Get(i)->GetObject<MobilityModel>();
        Vector vi = mi->GetVelocity();
        double speed_i = std::sqrt(vi.x*vi.x + vi.y*vi.y);
        double theta_i = std::atan2(vi.y, vi.x); // radians

        for (uint32_t j = 0; j < N; ++j)
        {
            if (i == j) continue;
            Ptr<MobilityModel> mj = nodes.Get(j)->GetObject<MobilityModel>();
            double dist = mi->GetDistanceFrom(mj);
            if (dist > R * 1.05) continue; // di luar jangkauan

            Vector vj = mj->GetVelocity();
            double speed_j = std::sqrt(vj.x*vj.x + vj.y*vj.y);
            double theta_j = std::atan2(vj.y, vj.x);

            double dV = std::fabs(speed_i - speed_j);
            double dTheta = std::fabs(radToDeg(theta_i - theta_j));
            if (dTheta > 180.0) dTheta = 360.0 - dTheta;

            // Node selection sesuai paper: hanya consider jika Δspeed <= speedTh AND Δheading <= angleThDeg
            if (dV > speedTh || dTheta > angleThDeg) continue;

            double let = ComputeLET(mi, mj, R);
            adj[i].push_back({static_cast<int>(j), let});
        }
    }
    return adj;
}

// Dijkstra: bobot = 1.0 / LET
std::vector<int> DijkstraPath(int src, int dst,
                              const std::vector<std::vector<std::pair<int,double>>> &adj)
{
    int N = (int)adj.size();
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> dist(N, INF);
    std::vector<int> prev(N, -1);
    using PDI = std::pair<double,int>;
    std::priority_queue<PDI, std::vector<PDI>, std::greater<PDI>> pq;

    if (src < 0 || src >= N || dst < 0 || dst >= N) return {};

    dist[src] = 0.0;
    pq.push({0.0, src});

    while (!pq.empty())
    {
        auto [cost,u] = pq.top(); pq.pop();
        if (cost > dist[u]) continue;
        if (u == dst) break;

        for (auto &e : adj[u])
        {
            int v = e.first;
            double let = e.second;
            if (let <= 0.0) continue;
            double w = 1.0 / let;
            if (dist[v] > dist[u] + w)
            {
                dist[v] = dist[u] + w;
                prev[v] = u;
                pq.push({dist[v], v});
            }
        }
    }

    if (dist[dst] == INF) return {};
    std::vector<int> path;
    for (int cur = dst; cur != -1; cur = prev[cur]) path.push_back(cur);
    std::reverse(path.begin(), path.end());
    return path;
}

// Safe static route installer (NS-3.43)
void InstallStaticPathRoutes(const std::vector<int> &path, const Ipv4InterfaceContainer &ifaces)
{
    if (path.size() < 2) return;
    Ipv4StaticRoutingHelper helper;
    Ipv4Address dst = ifaces.GetAddress(path.back());
    uint32_t totalNodes = NodeList::GetNNodes();

    for (size_t k = 0; k + 1 < path.size(); ++k)
    {
        int nodeId = path[k];
        int nextHopId = path[k+1];
        if (nodeId < 0 || nextHopId < 0 ||
            (uint32_t)nodeId >= totalNodes || (uint32_t)nextHopId >= totalNodes)
        {
            NS_LOG_UNCOND("⚠️ Invalid node index in path: " << nodeId << " -> " << nextHopId);
            continue;
        }
        Ptr<Node> node = NodeList::GetNode(nodeId);
        if (!node) { NS_LOG_UNCOND("⚠️ Null Node pointer: " << nodeId); continue; }
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
        Ptr<Ipv4StaticRouting> srt = helper.GetStaticRouting(ipv4);
        if (!srt) { NS_LOG_UNCOND("⚠️ Static routing null for node " << nodeId); continue; }
        Ipv4Address nextHopAddr = ifaces.GetAddress(nextHopId);
        srt->AddHostRouteTo(dst, nextHopAddr, 1);
    }
}

// ---------------------- Simulation class ----------------------

class AodvDExperiment
{
public:
    AodvDExperiment() = default;
    void CommandSetup(int argc, char **argv);
    void Run();

private:
    Ptr<Socket> SetupReceive(Ipv4Address addr, Ptr<Node> node);
    void Receive(Ptr<Socket> socket);
    void CheckThroughput();

    // params
    uint32_t port{9};
    uint32_t bytesTotal{0};
    uint32_t packetsReceived{0};

    std::string csv{"aodv-d-final.csv"};
    int nSinks{10};
    double txp{7.5};
    double radioRange{100.0};
    double minLET{0.5};
    double snapshotTime{49.5};
    double totalTime{200.0};
    double speedThreshold{5.0};   // m/s
    double angleThreshold{30.0};  // degrees
};

Ptr<Socket> AodvDExperiment::SetupReceive(Ipv4Address addr, Ptr<Node> node)
{
    Ptr<Socket> sink = Socket::CreateSocket(node, TypeId::LookupByName("ns3::UdpSocketFactory"));
    sink->Bind(InetSocketAddress(addr, port));
    sink->SetRecvCallback(MakeCallback(&AodvDExperiment::Receive, this));
    return sink;
}

void AodvDExperiment::Receive(Ptr<Socket> socket)
{
    Ptr<Packet> p;
    Address from;
    while ((p = socket->RecvFrom(from)))
    {
        bytesTotal += p->GetSize();
        packetsReceived++;
        NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s Node[" << socket->GetNode()->GetId()
                                                    << "] received from "
                                                    << InetSocketAddress::ConvertFrom(from).GetIpv4());
    }
}

void AodvDExperiment::CheckThroughput()
{
    double kbs = (bytesTotal * 8.0) / 1000.0;
    std::ofstream out(csv, std::ios::app);
    out << Simulator::Now().GetSeconds() << "," << kbs << "," << packetsReceived
        << "," << nSinks << ",AODV-D," << txp << std::endl;
    out.close();
    bytesTotal = 0;
    packetsReceived = 0;
    Simulator::Schedule(Seconds(1.0), &AodvDExperiment::CheckThroughput, this);
}

void AodvDExperiment::CommandSetup(int argc, char **argv)
{
    CommandLine cmd(__FILE__);
    cmd.AddValue("CSVfileName", "Output CSV filename", csv);
    cmd.AddValue("TxPower", "Tx power (dBm)", txp);
    cmd.AddValue("NumSinks", "Number of sink nodes", nSinks);
    cmd.AddValue("RadioRange", "Radio range (m)", radioRange);
    cmd.AddValue("MinLET", "Minimum LET threshold (s)", minLET);
    cmd.AddValue("SnapshotTime", "Snapshot time (s)", snapshotTime);
    cmd.AddValue("TotalTime", "Total simulation time (s)", totalTime);
    cmd.AddValue("SpeedThreshold", "Max speed difference (m/s) to accept neighbor", speedThreshold);
    cmd.AddValue("AngleThreshold", "Max heading difference (deg) to accept neighbor", angleThreshold);
    cmd.Parse(argc, argv);
}

void AodvDExperiment::Run()
{
    Packet::EnablePrinting();

    std::ofstream out(csv);
    out << "Time(s),Throughput(Kbps),PacketsReceived,NumSinks,Protocol,TxPower(dBm)" << std::endl;
    out.close();

    int nWifis = 50;
    std::string rate("2048bps");
    std::string phyMode("DsssRate11Mbps");

    Config::SetDefault("ns3::OnOffApplication::PacketSize", StringValue("64"));
    Config::SetDefault("ns3::OnOffApplication::DataRate", StringValue(rate));
    Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue(phyMode));

    NodeContainer nodes;
    nodes.Create(nWifis);

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211b);

    YansWifiPhyHelper phy;
    YansWifiChannelHelper chan;
    chan.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    chan.AddPropagationLoss("ns3::FriisPropagationLossModel");
    phy.SetChannel(chan.Create());
    phy.Set("TxPowerStart", DoubleValue(txp));
    phy.Set("TxPowerEnd", DoubleValue(txp));

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devs = wifi.Install(phy, mac, nodes);

    MobilityHelper mob;
    ObjectFactory pos;
    pos.SetTypeId("ns3::RandomRectanglePositionAllocator");
    pos.Set("X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));
    pos.Set("Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=1500.0]"));
    Ptr<PositionAllocator> alloc = pos.Create()->GetObject<PositionAllocator>();

    mob.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                         "Speed", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=20.0]"),
                         "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"),
                         "PositionAllocator", PointerValue(alloc));
    mob.SetPositionAllocator(alloc);
    mob.Install(nodes);

    // Install AODV
    AodvHelper aodv;
    Ipv4ListRoutingHelper list;
    list.Add(aodv, 100);
    InternetStackHelper internet;
    internet.SetRoutingHelper(list);
    internet.Install(nodes);

    Ipv4AddressHelper addr;
    addr.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer ifaces = addr.Assign(devs);

    // Setup OnOff UDP pairs: sink i at node i, source at node i + nSinks
    OnOffHelper onoff("ns3::UdpSocketFactory", Address());
    onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
    onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));

    for (int i = 0; i < nSinks; ++i)
    {
        Ptr<Socket> sink = SetupReceive(ifaces.GetAddress(i), nodes.Get(i));
        AddressValue remote(InetSocketAddress(ifaces.GetAddress(i), port));
        onoff.SetAttribute("Remote", remote);
        ApplicationContainer app = onoff.Install(nodes.Get(i + nSinks));
        // start in [50,51) as before
        Ptr<UniformRandomVariable> vr = CreateObject<UniformRandomVariable>();
        app.Start(Seconds(vr->GetValue(50.0, 51.0)));
        app.Stop(Seconds(totalTime));
    }

    // Snapshot: compute LET graph using node selection, run Dijkstra, install static routes
    Simulator::Schedule(Seconds(snapshotTime), [&]() {
        NS_LOG_UNCOND("AODV-D snapshot at t=" << Simulator::Now().GetSeconds() << "s (speedTh="
                                             << speedThreshold << " m/s, angleTh=" << angleThreshold << " deg)");
        auto adj = BuildLetGraph(nodes, radioRange, speedThreshold, angleThreshold);

        // filter edges with LET < minLET
        for (auto &nbrs : adj)
        {
            std::vector<std::pair<int,double>> f;
            for (auto &p : nbrs) if (p.second >= minLET) f.push_back(p);
            nbrs.swap(f);
        }
        // Run Dijkstra for each pair source=i+nSinks -> dst=i
        for (int i = 0; i < nSinks; ++i)
        {
            int src = i + nSinks;
            int dst = i;
            auto path = DijkstraPath(src, dst, adj);
            if (!path.empty())
            {
                std::ostringstream oss;
                oss << "AODV-D path " << src << "->" << dst << ": ";
                for (auto nd : path) oss << nd << " ";
                NS_LOG_UNCOND(oss.str());
                InstallStaticPathRoutes(path, ifaces);
            }
            else
            {
                NS_LOG_UNCOND("⚠️ No LET-based path found for " << src << "->" << dst);
            }
        }
    });

    // Start throughput logging shortly after traffic start
    Simulator::Schedule(Seconds(snapshotTime + 1.0), &AodvDExperiment::CheckThroughput, this);

    Simulator::Stop(Seconds(totalTime));
    Simulator::Run();
    Simulator::Destroy();
}

// ---------------------- main ----------------------

int main(int argc, char *argv[])
{
    AodvDExperiment exp;
    exp.CommandSetup(argc, argv);
    exp.Run();
    return 0;
}
hafizh@LAPTOP-MJ2LK0H9:~/ns-allinone-3.43/ns-3.43/scratch$ cat aodvdqe.cc.cc
cat: aodvdqe.cc.cc: No such file or directory
hafizh@LAPTOP-MJ2LK0H9:~/ns-allinone-3.43/ns-3.43/scratch$ cat aodvdqe.cc
/* aodv-d-43-qos-energy.cc
 *
 * AODV-D + LET + Dijkstra + QoS (FlowMonitor) + Energy (BasicEnergySource)
 * Target: NS-3.43
 *
 * Outputs:
 *  - aodv-d-43-qos-energy.csv : throughput per second
 *  - console logs: selected Dijkstra path snapshot and FlowMonitor summary
 *
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/aodv-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/energy-module.h"

#include <fstream>
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("AODV-D-43-QoS-Energy");

/////////////////////
// Utility: LET
/////////////////////
double ComputeLET(Ptr<MobilityModel> m1, Ptr<MobilityModel> m2, double R)
{
    Vector p1 = m1->GetPosition();
    Vector p2 = m2->GetPosition();
    Vector v1 = m1->GetVelocity();
    Vector v2 = m2->GetVelocity();

    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dvx = v1.x - v2.x;
    double dvy = v1.y - v2.y;

    double a = dvx*dvx + dvy*dvy;
    double b = 2.0*(dx*dvx + dy*dvy);
    double c = dx*dx + dy*dy - R*R;

    if (fabs(a) < 1e-9)
    {
        if (c <= 0.0) return std::numeric_limits<double>::infinity();
        return 0.0;
    }
    double disc = b*b - 4.0*a*c;
    if (disc < 0.0)
    {
        if (c <= 0.0) return std::numeric_limits<double>::infinity();
        return 0.0;
    }
    double sqrtD = std::sqrt(disc);
    double t1 = (-b + sqrtD) / (2.0*a);
    double t2 = (-b - sqrtD) / (2.0*a);

    double tExit = std::numeric_limits<double>::infinity();
    if (t1 > 0.0) tExit = std::min(tExit, t1);
    if (t2 > 0.0) tExit = std::min(tExit, t2);

    if (tExit == std::numeric_limits<double>::infinity())
    {
        if (c <= 0.0) return std::numeric_limits<double>::infinity();
        return 0.0;
    }
    return tExit;
}

/////////////////////
// Graph builder (LET + node selection by speed & heading)
/////////////////////
static inline double RadToDeg(double r) { return r * 180.0 / M_PI; }

std::vector<std::vector<std::pair<int,double>>> BuildLetGraph(
    const NodeContainer &nodes, double R, double speedThreshold, double angleThresholdDeg)
{
    uint32_t N = nodes.GetN();
    std::vector<std::vector<std::pair<int,double>>> adj(N);

    for (uint32_t i = 0; i < N; ++i)
    {
        Ptr<MobilityModel> mi = nodes.Get(i)->GetObject<MobilityModel>();
        Vector vi = mi->GetVelocity();
        double speed_i = std::sqrt(vi.x*vi.x + vi.y*vi.y);
        double theta_i = std::atan2(vi.y, vi.x);

        for (uint32_t j = 0; j < N; ++j)
        {
            if (i == j) continue;
            Ptr<MobilityModel> mj = nodes.Get(j)->GetObject<MobilityModel>();
            double dist = mi->GetDistanceFrom(mj);
            if (dist > R) continue;

            Vector vj = mj->GetVelocity();
            double speed_j = std::sqrt(vj.x*vj.x + vj.y*vj.y);
            double theta_j = std::atan2(vj.y, vj.x);

            double dV = fabs(speed_i - speed_j);
            double dTheta = fabs(RadToDeg(theta_i - theta_j));
            if (dTheta > 180.0) dTheta = 360.0 - dTheta;

            if (dV > speedThreshold || dTheta > angleThresholdDeg) continue;

            double let = ComputeLET(mi, mj, R);
            adj[i].push_back({(int)j, let});
        }
    }
    return adj;
}

/////////////////////
// Dijkstra with QoS weights (LET + Energy + delay + (1-PDR))
/////////////////////
struct LinkQoS {
    double pdr = 1.0;        // 0..1
    double avgDelay = 0.0;   // seconds
};

std::vector<int> DijkstraQoS(
    int src, int dst,
    const std::vector<std::vector<std::pair<int,double>>> &adj,
    const NodeContainer &nodes,
    const energy::EnergySourceContainer &energySources,
    const std::unordered_map<uint64_t, LinkQoS> &linkQos,
    double wLET, double wEnergy, double wDelay, double wPDR)
{
    int N = (int)adj.size();
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> dist(N, INF);
    std::vector<int> prev(N, -1);
    using PDI = std::pair<double,int>;
    std::priority_queue<PDI,std::vector<PDI>,std::greater<PDI>> pq;

    if (src < 0 || src >= N || dst < 0 || dst >= N) return {};

    dist[src] = 0.0;
    pq.push({0.0, src});

    while (!pq.empty())
    {
        auto [cost, u] = pq.top(); pq.pop();
        if (cost > dist[u]) continue;
        if (u == dst) break;

        for (auto &edge : adj[u])
        {
            int v = edge.first;
            double let = edge.second;
            if (let <= 0.0) continue;

            // Remaining energy: try to get energy source for node v
            double Erem = 0.0;
            if ((uint32_t)v < energySources.GetN())
            {
                Ptr<energy::BasicEnergySource> bs = DynamicCast<energy::BasicEnergySource>(energySources.Get(v));
                if (bs) Erem = bs->GetRemainingEnergy();
            }
            if (Erem <= 0.0) Erem = 0.0001;

            // Link QoS lookup (u->v)
            uint64_t key = (static_cast<uint64_t>(u) << 32) | static_cast<uint64_t>(v);
            LinkQoS lq;
            auto it = linkQos.find(key);
            if (it != linkQos.end()) lq = it->second;

            double pdr = lq.pdr; if (pdr <= 0.0) pdr = 0.0001;
            double delay = lq.avgDelay;

            // Compose weight (lower better)
            double w = 0.0;
            w += wLET * (1.0 / let);
            w += wEnergy * (1.0 / Erem);
            w += wDelay * delay;
            w += wPDR * (1.0 - pdr);

            if (dist[v] > dist[u] + w)
            {
                dist[v] = dist[u] + w;
                prev[v] = u;
                pq.push({dist[v], v});
            }
        }
    }

    if (dist[dst] == INF) return {};
    std::vector<int> path;
    for (int cur = dst; cur != -1; cur = prev[cur]) path.push_back(cur);
    std::reverse(path.begin(), path.end());
    return path;
}

/////////////////////
// Install static host routes along path
/////////////////////
void InstallStaticPathRoutes(const std::vector<int> &path, const Ipv4InterfaceContainer &ifaces)
{
    if (path.size() < 2) return;
    Ipv4StaticRoutingHelper helper;
    Ipv4Address dst = ifaces.GetAddress(path.back());
    uint32_t total = NodeList::GetNNodes();

    for (size_t i=0;i+1<path.size();++i)
    {
        int n = path[i];
        int next = path[i+1];
        if (n < 0 || next < 0 || (uint32_t)n >= total || (uint32_t)next >= total) continue;
        Ptr<Node> node = NodeList::GetNode(n);
        if (!node) continue;
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
        Ptr<Ipv4StaticRouting> s = helper.GetStaticRouting(ipv4);
        if (!s) continue;
        s->AddHostRouteTo(dst, ifaces.GetAddress(next), 1);
    }
}

/////////////////////
// Simulation class
/////////////////////
class AodvD43
{
public:
    void Setup(int argc, char **argv);
    void Run();

private:
    // parameters (defaults)
    std::string csv{"aodv-d-43-qos-energy.csv"};
    double txp{7.5};
    int nSinks{10};
    double radioRange{100.0};
    double minLET{0.5};
    double snapshot{50.0};
    double totalTime{200.0};
    double speedThreshold{5.0};
    double angleThreshold{30.0};
    // Dijkstra weights
    double wLET{1.0}, wEnergy{1.0}, wDelay{1.0}, wPDR{1.0};

    // internal
    NodeContainer nodes;
    NetDeviceContainer devs;
    Ipv4InterfaceContainer ifaces;
    energy::EnergySourceContainer energySources;

    // FlowMonitor
    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> monitor;
    Ptr<Ipv4FlowClassifier> classifier;

    // methods
    Ptr<Socket> SetupReceive(Ipv4Address addr, Ptr<Node> node);
    void Receive(Ptr<Socket> socket);
    void CheckThroughput();
    uint32_t port{9};
    uint32_t bytesTotal{0};
    uint32_t packetsReceived{0};
};

void AodvD43::Setup(int argc, char **argv)
{
    CommandLine cmd(__FILE__);
    cmd.AddValue("CSV", "Output CSV filename", csv);
    cmd.AddValue("TxPower", "Tx power (dBm)", txp);
    cmd.AddValue("NumSinks", "Number of sinks", nSinks);
    cmd.AddValue("RadioRange", "Radio range (m)", radioRange);
    cmd.AddValue("MinLET", "Minimum LET (s)", minLET);
    cmd.AddValue("Snapshot", "Snapshot time (s)", snapshot);
    cmd.AddValue("TotalTime", "Total sim time (s)", totalTime);
    cmd.AddValue("SpeedThreshold", "Speed threshold (m/s)", speedThreshold);
    cmd.AddValue("AngleThreshold", "Angle threshold (deg)", angleThreshold);
    cmd.AddValue("wLET", "Weight LET", wLET);
    cmd.AddValue("wEnergy", "Weight Energy", wEnergy);
    cmd.AddValue("wDelay", "Weight Delay", wDelay);
    cmd.AddValue("wPDR", "Weight PDR", wPDR);
    cmd.Parse(argc, argv);
}

Ptr<Socket> AodvD43::SetupReceive(Ipv4Address addr, Ptr<Node> node)
{
    Ptr<Socket> sink = Socket::CreateSocket(node, TypeId::LookupByName("ns3::UdpSocketFactory"));
    sink->Bind(InetSocketAddress(addr, port));
    sink->SetRecvCallback(MakeCallback(&AodvD43::Receive, this));
    return sink;
}

void AodvD43::Receive(Ptr<Socket> socket)
{
    Ptr<Packet> p;
    Address from;
    while ((p = socket->RecvFrom(from)))
    {
        bytesTotal += p->GetSize();
        packetsReceived++;
        NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s Node[" << socket->GetNode()->GetId()
                                                    << "] received from "
                                                    << InetSocketAddress::ConvertFrom(from).GetIpv4());
    }
}

void AodvD43::CheckThroughput()
{
    double kbs = (bytesTotal * 8.0) / 1000.0;
    std::ofstream out(csv, std::ios::app);
    out << Simulator::Now().GetSeconds() << "," << kbs << "," << packetsReceived << "," << nSinks << ",AODV-D-43," << txp << std::endl;
    out.close();
    bytesTotal = 0;
    packetsReceived = 0;
    Simulator::Schedule(Seconds(1.0), &AodvD43::CheckThroughput, this);
}

void AodvD43::Run()
{
    Packet::EnablePrinting();

    // prepare CSV
    std::ofstream out(csv);
    out << "Time(s),Throughput(Kbps),PacketsReceived,NumSinks,Protocol,TxPower(dBm)" << std::endl;
    out.close();

    int nNodes = 50;
    nodes.Create(nNodes);

    // Wifi
    WifiHelper wifi; wifi.SetStandard(WIFI_STANDARD_80211b);
    YansWifiPhyHelper phy; YansWifiChannelHelper chan;
    chan.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    chan.AddPropagationLoss("ns3::FriisPropagationLossModel");
    phy.SetChannel(chan.Create());
    phy.Set("TxPowerStart", DoubleValue(txp));
    phy.Set("TxPowerEnd", DoubleValue(txp));
    WifiMacHelper mac; mac.SetType("ns3::AdhocWifiMac");

    devs = wifi.Install(phy, mac, nodes);

    // Mobility: PositionAllocator required by RandomWaypoint
    MobilityHelper mob;
    ObjectFactory pos;
    pos.SetTypeId("ns3::RandomRectanglePositionAllocator");
    pos.Set("X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));
    pos.Set("Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=1500.0]"));
    Ptr<PositionAllocator> pa = pos.Create()->GetObject<PositionAllocator>();

    mob.SetPositionAllocator(pa);
    mob.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                         "Speed", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=20.0]"),
                         "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"),
                         "PositionAllocator", PointerValue(pa));
    mob.Install(nodes);

    // Internet + AODV
    AodvHelper aodv; Ipv4ListRoutingHelper list; list.Add(aodv, 100);
    InternetStackHelper internet; internet.SetRoutingHelper(list); internet.Install(nodes);

    Ipv4AddressHelper addr; addr.SetBase("10.1.1.0","255.255.255.0");
    ifaces = addr.Assign(devs);

    // Applications OnOff (pairs)
    OnOffHelper onoff("ns3::UdpSocketFactory", Address());
    onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
    onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
    onoff.SetAttribute("DataRate", StringValue("2048bps"));
    onoff.SetAttribute("PacketSize", UintegerValue(64));

    for (int i=0;i<nSinks;i++)
    {
        Ptr<Socket> sink = SetupReceive(ifaces.GetAddress(i), nodes.Get(i));
        AddressValue remote(InetSocketAddress(ifaces.GetAddress(i), port));
        onoff.SetAttribute("Remote", remote);
        ApplicationContainer app = onoff.Install(nodes.Get(i + nSinks));
        Ptr<UniformRandomVariable> vr = CreateObject<UniformRandomVariable>();
        app.Start(Seconds(vr->GetValue(50.0, 51.0)));
        app.Stop(Seconds(totalTime));
    }

    // Energy: BasicEnergySource + Wifi radio energy model
    BasicEnergySourceHelper basicSourceHelper;
    basicSourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(100.0));
    energySources = basicSourceHelper.Install(nodes);

    WifiRadioEnergyModelHelper radioHelper;
    radioHelper.Set("IdleCurrentA", DoubleValue(0.017));
    radioHelper.Set("TxCurrentA", DoubleValue(0.18));
    radioHelper.Set("RxCurrentA", DoubleValue(0.05));
    // note: Install returns DeviceEnergyModelContainer but helper handles internally
    radioHelper.Install(devs, energySources);

    // FlowMonitor
    monitor = fmHelper.InstallAll();
    classifier = DynamicCast<Ipv4FlowClassifier>(fmHelper.GetClassifier());

    // Snapshot: compute LET graph, fetch FlowMonitor stats, run DijkstraQoS, install static routes
    Simulator::Schedule(Seconds(snapshot), [&]() {
        NS_LOG_UNCOND("Snapshot at t=" << Simulator::Now().GetSeconds());
        auto adj = BuildLetGraph(nodes, radioRange, speedThreshold, angleThreshold);

        // Filter edges by minLET
        for (auto &nbrs : adj)
        {
            std::vector<std::pair<int,double>> tmp;
            for (auto &p : nbrs)
                if (p.second >= minLET) tmp.push_back(p);
            nbrs.swap(tmp);
        }

        // Build link QoS map from FlowMonitor: key = (u<<32)|v
        std::unordered_map<uint64_t, LinkQoS> linkQos;

        std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();
        for (auto &kv : stats)
        {
            FlowId fid = kv.first;
            FlowMonitor::FlowStats fs = kv.second;
            Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(fid);

            // find src node index and dst node index using ifaces
            int srcNode = -1, dstNode = -1;
            for (uint32_t k=0;k<ifaces.GetN();++k)
            {
                if (ifaces.GetAddress(k) == t.sourceAddress) srcNode = k;
                if (ifaces.GetAddress(k) == t.destinationAddress) dstNode = k;
            }
            if (srcNode == -1 || dstNode == -1) continue;

            // use (srcNode -> dstNode) as proxy for link QoS
            uint64_t key = (static_cast<uint64_t>(srcNode) << 32) | static_cast<uint64_t>(dstNode);
            LinkQoS q;
            q.pdr = (fs.txPackets > 0) ? (double)fs.rxPackets / fs.txPackets : 1.0;
            q.avgDelay = (fs.rxPackets > 0) ? fs.delaySum.GetSeconds() / fs.rxPackets : 0.0;
            linkQos[key] = q;
        }

        // Run Dijkstra for each application pair (src = i+nSinks -> dst = i)
        for (int i=0;i<nSinks;i++)
        {
            int src = i + nSinks;
            int dst = i;
            auto path = DijkstraQoS(src, dst, adj, nodes, energySources, linkQos, wLET, wEnergy, wDelay, wPDR);
            if (!path.empty())
            {
                std::ostringstream oss;
                oss << "AODV-D path " << src << "->" << dst << ": ";
                for (auto nd : path) oss << nd << " ";
                NS_LOG_UNCOND(oss.str());
                InstallStaticPathRoutes(path, ifaces);
            }
            else
            {
                NS_LOG_UNCOND("No LET path for " << src << "->" << dst);
            }
        }
    });

    // Start throughput logging 1s after snapshot
    Simulator::Schedule(Seconds(snapshot + 1.0), &AodvD43::CheckThroughput, this);

    Simulator::Stop(Seconds(totalTime));
    Simulator::Run();

    // At end, print FlowMonitor summary
    monitor->CheckForLostPackets();
    std::map<FlowId, FlowMonitor::FlowStats> finalStats = monitor->GetFlowStats();
    uint64_t tx=0, rx=0; double delay=0.0;
    for (auto &kv : finalStats)
    {
        tx += kv.second.txPackets;
        rx += kv.second.rxPackets;
        delay += kv.second.delaySum.GetSeconds();
    }
    NS_LOG_UNCOND("Final: tx=" << tx << " rx=" << rx << " avgDelay(s)=" << (rx? delay/rx : 0.0));

    Simulator::Destroy();
}

int main(int argc, char *argv[])
{
    AodvD43 sim;
    sim.Setup(argc, argv);
    sim.Run();
    return 0;
}
