## FP_Janir
Direktori "Janir" ada direkori lagi isinya AODVPAPER dan AODVDQE itu isinya file-file yang dari src/aodv/model tak otak-atik, bisa di copas file-file salah satu direktori (AODVPAPER atau AODVDQE) ke direktori src/aodv/model di ns3. keterangan :
1. Direktori AODVPAPER itu implementasi dari paper
2. Direktori AODVDQE itu implemnetasi dari proposal 
3. Yang ku otak-atik dari AODVPAPER adalah file aodv-packet.h, aodv-packet.cc, aodv-routing-protocol.h, dan aodv-routing-protocol.cc
4. Sedangkan untuk AODVDQE yang ku otak atik file aodv-packet.h, aodv-packet.cc, aodv-routing-protocol.h, aodv-routing-protocol.cc, aodv-rtable.h dan aodv-rtable.cc
5. Direktori scratch isinya file file confog buat simulasi. 

## scratch
File scratch iku isine kayak config simulasi lah, kamu salin masukin ke directory scratch ada test1 sama test2
1. test1 buat cek posisi tadi intinya ngecek header e lah
2. test2 ngecek aodv-d nya

## catatan tambahan
File aodvd-biasa odvd_modif sama direktori model gak kepake, tapi direktori model ada file backup buat jaga-jaga

## cara jalanin
1. Copas semua file file yang udah tak suruh di atas itu, bisa pake AODVPAPER atau AODVDQE tergantung mau testing apa
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
   atau
   ```
   ./ns3 run "scratch/test2 --nodes=50 --speed=20"
   ```
