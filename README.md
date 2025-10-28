# or_lab
Repozitorij za laboratorijske vježbe kolegija "Osnove Robotike" (https://www.fer.unizg.hr/predmet/osnrob)

## Upute
Laboratorijske vježbe akademske godine 2025./2026. se provode na tri načina:

**1. Analitički**: Raspisivanje DH metode, određivanje DH parametara, izračun vektora konfiguracije, itd.
Za sve korake dozvoljeno je korištenje MATLAB ili Python okruženja za simboličke izračune.

**2. Python skripta**: Analitički određeni dijelovi vježbe (direktna, inverzna kinematika, itd.) se
implementiraju u Python skriptu (*lv_priprema.py* u klasi *LV2526_priprema*. Skriptu je moguće
izvoditi neovisno o ROS2 okruženju, kroz UBUNTU, Conda ili bilo koji drugi način.

**3. ROS2 okruženje**: Sadrži kompletno ROS2 okruženje te sve popratne skripte za upravljanje robotom.
Okruženje je koncipirano na način da koristi klasu implementiranu kroz pripremu,
što omogućava studentima jednostavno testiranje implementiranih algoritama neovisno o poznavanju ROS2 okruženja.

Na vježbama postoje računala koja imaju spremna okruženja za izvođenje vježbe.
Student treba donijeti svoju skriptu iz prethodnog koraka te ju kopirati na predviđeno mjesto.
Ukoliko student želi koristiti vlastito računalo za izvođenje vježbe, repozitorij s potrebnim
programskim kodovima i uputama se nalazi na https://github.com/larics/or_lab.
U repozitoriju je dostupan i Dockerfile s instaliranim ros2 jazzy okruženjem i kloniranim repozitorijem,
preporučamo korištenje istog ako student želi koristiti vlastito računalo.

## NAPOMENA
Svaki student za sve tri laboratorijske vježbe koristi VLASTITU skriptu *lv_priprema.py* u kojoj implementira tražene dijelove.
Implementirani dijelovi iz prethodnih vježbi će se koristiti u narednim.
Uz svaki izvještaj laboratorijske vježbe, potrebno je predati i navedenu skriptu uz naziv *JMBAG_lv_priprema.py*.
Sve predane skripte će se kontrolirati te uspoređivati.

Napominjemo da je priprema svake laboratorijske vježbe samostalni uradak i prepisivanje će se strogo kažnjavati!
