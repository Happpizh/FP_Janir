/*
 * Script Sederhana untuk Menguji Header AODV-D (Step 1)
 * Tujuan: Memastikan Serialize/Deserialize Posisi & Kecepatan berfungsi normal.
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/aodv-packet.h" // Mengakses header yang baru kita ubah
#include "ns3/vector.h"

using namespace ns3;
using namespace ns3::aodv;

int main (int argc, char *argv[])
{
  CommandLine cmd (__FILE__);
  cmd.Parse (argc, argv);

  NS_LOG_UNCOND ("--- MULAI PENGUJIAN STEP 1: AODV-D PACKET HEADER ---");

  // ==========================================
  // TEST 1: RREQ Header
  // ==========================================
  NS_LOG_UNCOND ("\n[1] Testing RREQ Header...");

  // 1. Buat Header RREQ
  RreqHeader rreqSrc;
  rreqSrc.SetId (100);
  rreqSrc.SetDst (Ipv4Address ("10.0.0.1"));
  rreqSrc.SetOrigin (Ipv4Address ("10.0.0.2"));
  
  // 2. Set Posisi dan Kecepatan (Data Dummy)
  Vector posSrc(150.5, 75.2, 0.0);
  Vector velSrc(20.0, 5.5, 0.0);
  
  rreqSrc.SetPosition(posSrc);
  rreqSrc.SetVelocity(velSrc);

  // 3. Masukkan ke dalam Packet (Proses Serialize terjadi di sini)
  Ptr<Packet> p1 = Create<Packet> ();
  p1->AddHeader (rreqSrc);

  // 4. Baca kembali dari Packet (Proses Deserialize terjadi di sini)
  RreqHeader rreqDst;
  p1->RemoveHeader (rreqDst);

  // 5. Verifikasi Hasil
  Vector posResult = rreqDst.GetPosition();
  Vector velResult = rreqDst.GetVelocity();

  NS_LOG_UNCOND ("   Input Posisi: " << posSrc);
  NS_LOG_UNCOND ("   Hasil Posisi: " << posResult);
  
  if (posSrc.x == posResult.x && posSrc.y == posResult.y) {
      NS_LOG_UNCOND ("   >> RREQ Position: SUKSES (Data Cocok)");
  } else {
      NS_LOG_UNCOND ("   >> RREQ Position: GAGAL (Data Berubah!)");
  }

  NS_LOG_UNCOND ("   Input Velocity: " << velSrc);
  NS_LOG_UNCOND ("   Hasil Velocity: " << velResult);
  
  if (velSrc.x == velResult.x && velSrc.y == velResult.y) {
      NS_LOG_UNCOND ("   >> RREQ Velocity: SUKSES (Data Cocok)");
  } else {
      NS_LOG_UNCOND ("   >> RREQ Velocity: GAGAL (Data Berubah!)");
  }


  // ==========================================
  // TEST 2: RREP Header
  // ==========================================
  NS_LOG_UNCOND ("\n[2] Testing RREP Header...");

  // 1. Buat Header RREP
  RrepHeader rrepSrc;
  rrepSrc.SetDst (Ipv4Address ("10.0.0.1"));
  rrepSrc.SetOrigin (Ipv4Address ("10.0.0.2"));

  // 2. Set Posisi dan Kecepatan (Data Dummy berbeda)
  Vector posRrep(500.0, 500.0, 0.0);
  Vector velRrep(-10.0, -10.0, 0.0);
  
  rrepSrc.SetPosition(posRrep);
  rrepSrc.SetVelocity(velRrep);

  // 3. Serialize
  Ptr<Packet> p2 = Create<Packet> ();
  p2->AddHeader (rrepSrc);

  // 4. Deserialize
  RrepHeader rrepDst;
  p2->RemoveHeader (rrepDst);

  // 5. Verifikasi
  Vector posRrepRes = rrepDst.GetPosition();
  
  NS_LOG_UNCOND ("   Input Posisi: " << posRrep);
  NS_LOG_UNCOND ("   Hasil Posisi: " << posRrepRes);

  if (posRrep.x == posRrepRes.x && posRrep.y == posRrepRes.y) {
      NS_LOG_UNCOND ("   >> RREP Position: SUKSES");
  } else {
      NS_LOG_UNCOND ("   >> RREP Position: GAGAL");
  }

  NS_LOG_UNCOND ("\n--- PENGUJIAN SELESAI ---");

  return 0;
}
