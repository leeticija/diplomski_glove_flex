# Diplomski rad - Glove Flex Control

Ovaj projekt implementira sistem za mjerenje i kontrolu fleksije prstiju pomoću senzora pomaka (displacement sensors) integriranih u rukavicu (glove). Podaci sa senzora se obrađuju kroz kombinovani median i eksponencijalni pomični prosjek (EMA) za filtriranje šuma i otklanjanje izuzetaka (outliera).

U radu će biti implementirano pomicanje grippera upravljano senzorima na rukavici, pri čemu gripper također ima svoj vlastiti senzor za povratnu informaciju.

Podaci se zatim šalju putem ROS (Robot Operating System) čvora u obliku poruka za dalju analizu i upotrebu u robotskoj kontroli ili drugim aplikacijama.

## Ključne karakteristike:
- Integracija SparkFun displacement senzora sa mikrokontrolerom
- Filtracija signala korištenjem kombinovanog median i EMA filtera
- Upravljanje pomicanjem grippera na temelju senzora rukavice i povratnih podataka sa senzora grippera
- ROS publisher za slanje obrađenih podataka u realnom vremenu
- Efikasno upravljanje memorijom za rad na ograničenim uređajima poput Arduino platforme

---

Projekt je dio diplomskog rada i služi kao osnova za razvoj sistema za preciznu detekciju pokreta prstiju i kontrolu grippera u robotskoj ili medicinskoj primjeni.
