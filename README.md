Riešenie bludiska: technická dokumentácia
ROS 2 balík pre autonómnu jazdu robotom v mriežkovom bludisku 8×8.
Prehľad
Robot prejde mriežkové bludisko 8×8 so 40 cm bunkami. Pohyb je vždy v násobkoch 90°. Riešenie využíva
LiDAR na detekciu stien, IMU na presný uhol natočenia a kameru na čítanie ArUco markerov, ktoré určujú
smer odbočenia. Všetko beží v ROS 2 (rclcpp) ako jeden proces s viacerými uzlami a multi-thread
executorom.
Konvencie
• Súradnice robota: pozícia (x, y, θ), lineárna rýchlosť v [m/s], uhlová rýchlosť ω [rad/s).
• Yaw: 0° = počiatočný smer po kalibrácii, kladný smer = proti smeru hodinových ručičiek (CCW).
• LiDAR meria v metroch, inf znamená žiadne platné meranie v sektore.
Architektúra
Aplikácia má dve vrstvy. Spodná vrstva sú senzorové a akčné uzly (nodes). Horná vrstva je riadiaca slučka
(loop), ktorá kombinuje dáta a posiela príkazy motorom.
Vstupný bod
Súbor src/main.cpp vytvorí všetky uzly, pripojí ich do MultiThreadedExecutor a spustí spin. Tým bežia
callbacky senzorov paralelne s riadiacou slučkou.
// main.cpp - skrátené
auto executor = std::make_shared&lt;rclcpp::executors::MultiThreadedExecutor&gt;();
auto motor_node = std::make_shared&lt;nodes::MotorNode&gt;();
auto lidar_node = std::make_shared&lt;nodes::LidarNode&gt;();
auto imu_node = std::make_shared&lt;nodes::ImuNode&gt;();
auto camera_node = std::make_shared&lt;CameraNode&gt;();
auto loop = std::make_shared&lt;loops::CorridorLoop&gt;(
lidar_node, motor_node, imu_node, camera_node);
executor-&gt;add_node(motor_node); /* ...ostatné... */ executor-&gt;spin();
Komponenty
• LidarNode – odoberá /bpc_prp_robot/lidar. Z 360° skenu vypočíta medián v 8 sektoroch (front, back,
left, right + 4 diagonály) so šírkou ±5°. Medián je odolnejší voči šumu než priemer.
• ImuNode – odoberá /bpc_prp_robot/imu. Najprv zbiera 200 vzoriek gyro_z na výpočet biasu (kalibrácia),
potom integruje yaw cez triedu PlanarImuIntegrator (Eulerova metóda, dt obmedzené na 0–0.5 s).
• MotorNode – publikuje UInt8MultiArray na /bpc_prp_robot/set_motor_speeds. Hodnota 127 = stop, 0 =
plná vzad, 255 = plná vpred.
• CameraNode – odoberá komprimované JPEG snímky, dekóduje ich cez OpenCV a hľadá ArUco markery zo
slovníka DICT_4X4_50. Detekciu robí trieda ArucoDetector.
• CorridorLoop (slučka v loops/maze_loop.cpp) – konečný automat, ktorý všetko riadi. Beží v timeri 50 ms
(20 Hz).
ArUco markery a rozhodovanie
V bludisku sú pred každou križovatkou dva markery nad sebou: escape (ID 0–9) a treasure (ID 10–19).
Slovník: 0/1/2 = straight/left/right pre escape, 10/11/12 = to isté pre treasure.
CameraNode si pre oba typy pamätá poslednú detekciu vrátane času. CorridorLoop volá
get_paired_detection a používa marker mladší ako 0.3 s. Smer odbočky sa odloží ako pending_turn a použije
pri najbližšej platnej križovatke (platnosť 30 s).
Stavový automat slučky
CorridorLoop má 5 stavov definovaných v enum NavState:
• WAITING – prvé 2 s motory stoja, čaká sa na nábeh uzlov.
• CALIBRATION – robot stojí, IMU zbiera 200 vzoriek a vypočíta gyro offset.
• CORRIDOR_FOLLOWING – jazda v koridore pomocou yaw-PID. Tu sa deteguje bočná odbočka aj stena
vpredu.
• TURNING – otáčanie na mieste o 90° s proporcionálnym spomalením.
• POST_TURN_DRIVE – krátky priamy pohyb (0.5 s) po otočke, aby sa robot odlepil od rohu.
Riadenie pohybu v koridore

Robot drží smer pomocou IMU. Vzdialenosť od stien je len doplnková korekcia. Cieľový uhol target_yaw_ je
vždy násobok 90° (snap_target_yaw_to_grid). Tým sa nehromadí chyba z otáčok.
Yaw PID (hlavná zložka)
Vstup: chyba uhla. Výstup: rozdielová korekcia rýchlosti kolies.
// drive_yaw_based() - jadro výpočtu
float yaw_err = target_yaw_ - current_yaw; // normalizované na -180..180
float d_err = (yaw_err - yaw_prev_err_) / dt;
float corr = yaw_kp_ * yaw_err + yaw_kd_ * d_err;
corr = std::clamp(corr, -yaw_max_correction_, yaw_max_correction_);
int left = base_speed_ - corr; // base_speed_ = 155
int right = base_speed_ + corr;
Parametre: kp = 0.1, kd = 0.05, max korekcia = 3.5. Rýchlosti kolies sú clampnuté do rozsahu 120–180
(bezpečnostné limity).
Lateral korekcia (slabá doplnková)
Ak je yaw chyba malá (&lt; 5°) a obe steny sú bližšie ako 0.40 m, pridá sa centrovacia korekcia (lateral_kp_ =
30, max 5). Pri jednej stene je korekcia ešte 2× slabšia. Mŕtve pásmo 0.04 m bráni reakcii na šum.
Detekcia odbočiek a otáčanie
Bočná odbočka
Aktívna iba ak existuje pending_turn s LEFT/RIGHT. Otvor sa potvrdí dvojitou kontrolou: diagonálny sektor
(FL/FR) aj kolmý sektor (L/R) musia byť &gt; 0.40 m. Potom robot ešte 0.5 s ide rovno, aby sa dostal do stredu
bunky, a až potom začne otáčku.
Stena vpredu
Ak je front &lt; 0.25 m, ide o roh alebo T-križovatku. Smer vyberie marker, alebo (ak marker chýba či ukazuje na
zatvorenú stranu) lidar – otvorenejšia strana vyhráva.
Otáčanie o 90°
Otáčka je proporcionálna: ďaleko od cieľa max rýchlosť 30, blízko cieľa lineárne klesá na 8. Tolerancia 2°,
timeout 4 s, settle pauza 0.3 s. Po otočke sa target_yaw aktualizuje na aktuálny yaw a snapne na násobok
90°.
Pomocné algoritmy
• PlanarImuIntegrator – izolovaná trieda bez závislosti na ROS. Update krok: θ += (gyro_z − offset) ·
dt.
• ArucoDetector – wrapper okolo cv::aruco::detectMarkers so slovníkom 4×4_50.
• Kinematics (differential drive) – konverzie medzi rýchlosťami kolies a robotu pomocou polomeru R a
rozchodu L: v = R·(ωL+ωR)/2, ω = R·(ωR−ωL)/L.
ROS 2 rozhrania
• Odber: /bpc_prp_robot/lidar (LaserScan), /bpc_prp_robot/imu (Imu), /bpc_prp_robot/camera/compressed
(CompressedImage).
• Publikovanie: /bpc_prp_robot/set_motor_speeds (UInt8MultiArray, [left, right]), debug
camera_node/image.
Vlákna a synchronizácia
MultiThreadedExecutor spúšťa callbacky uzlov paralelne. Každý uzol, ktorý drží stav čítaný iným uzlom,
používa std::mutex. Konkrétne: LidarNode chráni sektory, ImuNode chráni mód a integrátor, CameraNode
chráni paired detekcie.
Praktické tipy
• Pri štarte robota nehýbať s ním ~3 s, kým prebehne IMU kalibrácia.
• Ak LiDAR vracia veľa inf hodnôt, skontrolujte hodnotu sektora ±5° – príliš úzky sektor v rohoch nezachytí
stenu.
• Ak yaw driftuje za pohybu, zvýšte počet vzoriek kalibrácie alebo predĺžte trvanie WAITING.
