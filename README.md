# Diplomski rad - Kontrola fleksije rukavice i grippera

Ovaj projekt implementira sustav za mjerenje i kontrolu pokreta prstiju pomoću senzora pomaka ugrađenih u rukavicu (glove). Podaci sa senzora obrađuju se kombinacijom medijanskog i eksponencijalnog pokretnog prosjeka (EMA) radi uklanjanja šuma i iznimaka (outliera).

U radu će biti implementirano upravljanje pomicanjem grippera koristeći podatke sa senzora na rukavici, dok gripper također ima vlastiti senzor za povratnu informaciju.

Podaci se prenose putem ROS (Robot Operating System) čvora kao poruke za daljnju obradu i kontrolu u robotskim sustavima.

## Ključne značajke:
- Integracija SparkFun senzora pomaka sa mikrokontrolerom
- Filtracija signala kombiniranim medijanskim i EMA filtrom
- Upravljanje gripperom na temelju senzora rukavice i senzora grippera
- ROS publisher za slanje obrađenih podataka u stvarnom vremenu
- Efikasno upravljanje memorijom za rad na ograničenim uređajima poput Arduina

---

Projekt je dio diplomskog rada i predstavlja osnovu za razvoj sustava precizne detekcije pokreta prstiju i kontrole grippera u robotskim ili medicinskim primjenama.
