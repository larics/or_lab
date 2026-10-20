# or_lab
Repozitorij za laboratorijske vježbe kolegija "Osnove Robotike". (https://www.fer.unizg.hr/predmet/osnrob)

## Upute
Laboratorijske vježbe akademske godine 2026./2027. se provode na tri načina:

**1. Analitički**: Raspisivanje DH metode, određivanje DH parametara, izračun vektora konfiguracije, itd.
Za sve korake dozvoljeno je korištenje MATLAB ili Python okruženja za simboličke izračune.

**2. Python skripta**: Analitički određeni dijelovi vježbe (direktna, inverzna kinematika, itd.) se
implementiraju u Python skriptu `open_manipulator_lab/scripts/lv_priprema.py` u klasi `LV2526_priprema`.
Skriptu je moguće izvoditi neovisno o ROS2 okruženju, kroz UBUNTU, Conda ili bilo koji drugi način.

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

## Upute za docker
Kako biste pokrenuli docker container, trebate pokrenuti sljedeće naredbe u glavnom direktoriju ovog repozitorija:
```
docker build -t or_lab -f or_lab.Dockerfile  .
docker run -it \
    --privileged \
    -v /dev:/dev \
    or_lab
```
Ako želite koristiti ovaj container za daljnje razvijanje drugih projekata, ovaj način instalacije ima manu:
container nema pristup grafičkom sučelju te nećete unutar njega moći otvoriti nove prozore.

Detaljnije upute za docker dostupne su na: https://github.com/larics/docker_files/wiki.

## Upute za pokretanje robota
Kako biste pokrenuli robota, u terminalu unutar containera ili vašeg lokalnog radnog okruženja
pokrenite skriptu za upravljanje robotom:
```
ros2 launch open manipulator bringup open manipulator x.launch.py
```
Nakon pokretanja kontrole robota, možete pokrenuti skriptu **orlab lv.py** preko drugog terminala
ili otvaranjem skripte u editoru i pritiskom ”Play” gumba u gornjem desnom kutu (to će otvoriti
novi terminal, možete promijeniti prikazani terminal u donjem desnom kutu).
Nakon pokretanja skripte, pritisnite Enter za izvedbu na robotu.
Istu skriptu možete modificirati kako biste robotu zadali druge poze.
