## model
Direktori "model" itu file yang dari src/aodv/model tak otak-atik, kamu salain terus tempel ke direktori src/aodv/model di aodv, keterangan
   1. Yang ku otak-atik file aodv-packet.h, aodv-packet.cc, aodv-routing-protocol.h, dan aodv-routing-protocol.cc
   2. File backup itu bawaan file asli aodv bawaan dari sana, buat jaga-jaga aja kemarin

## scratch
File scratch iku isine kayak config simulasi lah, kamu salin masukin ke directory scratch ada test1 sama test2
1. test1 buat cek posisi tadi intinya ngecek header e lah
2. test2 ngecek aodv-d nya

## cara jalanin
1. Copas semua file file yang udah tak suruh di atas iku
2. Tarus jalanin kode
   ```
   ns3 build
   ```
3. Terus jalanin test1 (kalao mau tau aja si)
   ```
   ./ns3 run "scratch/tes1"
   ```
4. Terus jalanin test2
   ```
   ./ns3 run "scratch/test2 --speed=10"
   ```
5. Kalau mau tau log threshold Rreq drop dll
   ```
   NS_LOG="AodvRoutingProtocol" ./ns3 run "scratch/test2 --nodes=10 --speed=20" 2>&1 | grep "AODV-D CHECK"
   ```
