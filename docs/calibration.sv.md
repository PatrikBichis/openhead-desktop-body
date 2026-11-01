# Kalibrering: servon och huvudet på riggen

Två flöden:

- **A. Servona:** hitta, identifiera (modell, utväxling), ge ID enligt Open Ducks numrering, märka upp
  och köra till nolläget.
- **B. Huvudet på bänkriggen** (`cad/benchhead`): hitta sambandet mellan servots gradtal och huvudets
  verkliga vinkel per led (neck_pitch, head_pitch, head_yaw, head_roll), och spara det i `joints.json`.

Verktyg: `firmware/odm_servo_setup` (textmeny, ett servo i taget), `host/odm_bench.py` (bänktestet) och
`host/odm_calib.py` (kontroller, nollkörning, kalibrering). Allt i `host/` går att prova utan hårdvara:
`--port fake` ger en simulerad Uno med servon på halsens ID 30–33, `--phone fake` en simulerad telefon.

```bash
cd host
.venv/Scripts/python odm_calib.py --port fake audit
.venv/Scripts/python odm_calib.py head-plan
```

## Vad "noll" betyder

Tre olika lägen som lätt blandas ihop:

| Läge | Definition | Används till |
|---|---|---|
| **Rå 2048** | Servots mittläge (halvt varv av 4096), med servots egen offset = 0 | Referens när hornet sätts på. `noll` i setup-firmware, `odm_calib.py zero --raw` |
| **0 rad** | Ledens nolläge i simulatorn (MJCF). Benen raka, halsen i CAD-läget | `odm_calib.py zero`, kalibrering, ledgränser |
| **Standardpose** | `home` i MJCF: benen böjda (knä ca 1,37 rad), halsen 0 | Policyns utgångsläge, `odm_bench.py home` |

Sambandet mellan rå och rad står i `host/joints.json` per led:

```
q [rad] = tecken · (rå − 2048) / 651,9 − offset_rad          (651,9 = 4096 / 2π steg per rad)
```

- **Offset bor bara i `joints.json`.** Servots eget offsetregister (31–32) ska vara 0. Då betyder rå
  2048 alltid samma sak, ett utbytt servo behöver bara ny `offset_rad`, och offseten versionshanteras.
  LeRobot skriver egna offset och vinkelgränser i SO-101-servona; de måste bort (`konfig` i
  setup-firmware, `odm_calib.py fix`). Registret är tecken–belopp: bit 11 = negativt, bit 0–10 belopp.
  Det tidigare `regs` i `odm_bench.py` visade det som ett osignerat tal; `odm_calib.py audit` och `info`
  i setup-firmware visar det nu med tecken.
- **Utväxlingen påverkar inte omräkningen.** Positionsgivaren (magnet) sitter på utgående axeln, så det
  är 4096 steg per varv för 1/345, 1/191 och 1/147. Utväxlingen ändrar moment, fart och glapp, och
  därmed ställdonsmodellen i simulatorn (benen ska vara 1/345).
- **Hornets delning:** STS3215-axeln har 25 kuggar **(verifiera)**, alltså 14,4° per kugg. Ett horn som
  sätts på vid rå 2048 hamnar upp till ±7,2° från CAD-läget. Det tas upp av `offset_rad`, inte genom
  att tvinga hornet. Sätt hornet på den kugg som ligger närmast och mät sedan offseten.
- **Tecken:** positiv riktning i simulatorn följer högerhandsregeln kring ledaxeln i MJCF. Om servot
  räknar åt andra hållet (beror på hur det är monterat) är tecknet −1.

Positiv riktning för halsen (ur CAD-axlarna i `cad/benchhead/reference/frames.json`, `odm_calib.py head-plan`):

| Led | Axel (riggen) | +10° betyder |
|---|---|---|
| neck_pitch | −y | nacken lutar bakåt, skärmen vrids uppåt |
| head_pitch | −y | huvudet tippar bakåt, skärmen vrids uppåt |
| head_yaw | +z | huvudet vrids åt robotens vänster |
| head_roll | −x | huvudet lutar åt robotens vänster (telefonens överkant går nedåt) |

## A. Servona: identifiera, ID, märkning och nolläge

### Luckor i det befintliga flödet och vad som gjorts

| Lucka | Åtgärd |
|---|---|
| Utväxlingen kontrollerades inte. Modellnumret (777) är detsamma för C001, C044 och C046, och positionen visar den inte | Hastighetstest: `vaxel <id>` i setup-firmware (erbjuds i `guide`) och `odm_calib.py gear <led>` via `odm_uno`. Tomgångsfarten normeras till 7,4 V och jämförs med 1/345 ≈ 3070 steg/s (0,222 s/60°), 1/191 ≈ 5550 och 1/147 ≈ 7200 **(nominella värden, verifiera)**. Utväxlingen sparas i `joints.json` (`gear`, `gear_test`) |
| `odm_bench.py configure` nollställde inte vinkelgränser och offset (setup-firmwarens `konfig` gjorde det) | `odm_calib.py fix <led>` skriver hela standardinställningen. `odm_calib.py audit` kontrollerar alla leder: modell, läge, P/I/D, gränser, egen offset, maxmoment, acceleration, spänning, och att benen har 1/345 |
| Servots offset (register 31) visades utan tecken | `audit` och setup-firmwarens `info` visar tecken (bit 11) |
| "Noll" var bara rå 2048; inget kommando körde till 0 rad i simulatorn | `odm_calib.py zero` kör till 0 rad (efter tecken och offset), `zero --raw` till rå 2048 |
| Säker rörelse: `move`/`home` kör med fullt moment och utan lastvakt | `zero` och `head-capture` sätter momentgräns (register 48, RAM; standard 30 % resp. 50 %), begränsar toppfarten (20 °/s resp. 30 °/s, min-jerk), avbryter och släpper momentet om lasten går över gränsen eller Uno rapporterar servofel, och återställer momentgränsen till 1000 efteråt. Unos stegbegränsning (`max_step`) och watchdog gäller som vanligt |
| Utväxling och kalibreringsdata hade ingen plats i `joints.json` | `joints.py` bevarar extra fält per led (`gear`, `gear_test`, `kalibrering`) |

### Steg för steg

**Ett servo i taget, löst på bänken** (setup-firmware, seriell terminal, se `firmware/README.md`):

1. Läs dekalen: utväxling och kod (C001 = 1/345, C044 = 1/191, C046 = 1/147) och spänning. 12 V-modeller
   (ST3215-HS, ST3020) hör inte hemma på 7,4 V-bussen i benen.
2. `guide`, koppla in servot. Guiden visar ID, modell, position, spänning och temperatur.
3. Välj plats 1–14. Servot får Open Ducks ID (höger ben 10–14, vänster 20–24, hals 30–33) och
   standardinställningen (positionsläge, P 32, gränser 0..4095, offset 0).
4. Svara **j** på hastighetstestet om servot sitter löst. Stämmer inte testet med dekalen: läs dekalen
   igen, mät spänningen, och notera avvikelsen. Benen ska ha 1/345.
5. Servot kör långsamt till rå 2048 och håller. Etiketten skrivs ut. Märk servot med ID, plats och
   utväxling.
6. När alla är klara: `tabell` (alla 14 hittade, inga okända ID).

**Kontroll med robotens firmware** (`odm_uno`, alla servon på bussen, momentet av):

7. `odm_calib.py audit` ska ge "alla ok". Avvikelser: `odm_calib.py fix <led>`.
8. Spara utväxlingen i `joints.json`: `odm_calib.py gear <led> --decal 1/345` (testet körs igen; bara
   med löst servo) eller lägg in `"gear": "1/345"` för hand.

**Montering:**

9. `odm_calib.py zero --raw <led>` kör leden långsamt till rå 2048 och håller i `--hold` sekunder
   (Ctrl+C släpper tidigare). Sätt hornet på närmaste kugg så att delen ligger så nära CAD-läget (0 rad)
   som möjligt. Hornet sitter med skruv, inte bara friktion.
10. Tecken: `odm_bench.py move <led> 10` och jämför med tabellen ovan (halsen) eller simulatorns visare
    (benen). Fel håll: `odm_bench.py config --sign <led>=-1`.
11. Offset: ställ leden i 0 rad med mall eller mätning (för halsen: del B) och kör `odm_bench.py offsets <led>`.
12. Kontroll: `odm_calib.py zero <led>` ska ge delen i CAD-läget. `odm_bench.py home` först när roboten
    hänger upp.

## B. Huvudet på bänkriggen

### Metoder

| Metod | Mäter | Noggrannhet | För | Emot |
|---|---|---|---|---|
| **Mall/jigg** (utskriven) | en led i taget, 0 rad | ca 0,5–1° | enkel, oberoende av elektronik, upprepbar | en mall per led; glapp gör att leden kan ligga var som helst inom glappet när momentet är av |
| **Telefonens IMU** (Core Motion, gravitationen) med anpassning | tecken, offset, skala och glapp för flera leder samtidigt | ca 0,1–0,3° per pose, bättre efter anpassning | telefonen sitter redan i huvudet; mäter med momentet på och lasten som i drift; ger residualer som avslöjar fel | gravitationen ser inte rotation kring lodlinjen; vissa parametrar går inte att skilja åt (se nedan); kräver data från telefonen |
| **Vattenpass/nivåapp** (Mätverktyg-appen) | en vinkel mot lodlinjen i taget, för hand | ca 0,5–1° | inget att bygga | manuellt, en led i taget, ser inte yaw |
| **Ändlägen** | ett mekaniskt stopp | 1–3° | ingen extra utrustning | delarna böjer sig, servot pressas mot stoppet (risk), stoppen är inte CAD-referenser |

**Rekommendation: mall för neck_pitch, telefonens IMU för resten, ändlägen bara som kontroll.**

Skälet är vad gravitationen kan och inte kan se i den här kinematiken:

- **neck_pitch och head_pitch har parallella axlar** (båda −y). Telefonens orientering beror bara på
  summan, så gravitationen kan inte fördela offseten mellan dem. neck_pitch sätts därför med mall, och
  head_pitch anpassas mot den.
- **Stativets lutning kring y** är också parallell med pitch-axlarna och mäts separat: telefonen
  liggande på basplattan (`head-base`). Lutningen kring x anpassas.
- **head_yaw syns bara när yaw-axeln lutar**, alltså när neck_pitch + head_pitch ≠ 0. Mätposerna lutar
  därför huvudet ±25° när yaw varieras.
- **Telefonens rotation i hållaren kring roll-axeln** går inte att skilja från head_roll-offset. Den
  hamnar i head_roll (en telefon som sitter 1° snett ger 1° fel i head_roll). Hållaren har snäppfästen;
  kontrollera att telefonen ligger an.
- Mätningen ger **speglade lösningar** (omvända pitch-tecken och yaw-offset 180° passar lika bra).
  Anpassningen väljer bort lösningar med offset > 25° eller skala långt från 1.

En mall för neck_pitch är enkel eftersom båda pitch-axlarna ligger på samma x (1 mm) i nolläget: linjen
mellan neck_pitch- och head_pitch-axlarna är **lodrät** när neck_pitch = 0 (66 mm mellan axlarna). En
mall med två urtag som läggs på axlarnas horn eller skruvar och har en rak kant räcker, med vattenpass
eller nivåappen mot kanten (stativet mätt först).

**Mallen:** `cad/benchhead/out/print/nollmall_neck_pitch.stl` (skript `cad/benchhead/nollmall.py`, 34 × 99 × 8 mm,
ca 18 g PLA, skrivs ut med konerna uppåt). Den läggs mot halsplåtens och head_pitch_to_yaw:s yttre sida, som
ligger i samma plan, med de två ihåliga konerna i de försänkta Ø7-hålen på axlarna (där hornens mittskruvar
sitter). Konerna centrerar utan spel och borrningen ger plats åt skruvhuvudet. Mallen är symmetrisk och passar
på båda sidor. Fram- och bakkanten är parallella med linjen mellan axlarna.

- head_pitch påverkar inte mätningen men ska stå inom ±0,3 rad (±17°), annars tar head_yaw_to_roll i mallens
  överände. Ställ head_yaw nära 0.
- Nivåappen (Mätverktyg → Nivå): håll telefonen **liggande med kortsidan** mot mallens fram- eller bakkant.
  Stående når telefonen upp i head_yaw_to_roll. Ett litet vattenpass mot kanten går också.
- Lutar stativet kring y (`head-base`), ska kanten luta lika mycket åt samma håll som basplattan (lutar
  basplattan framåt, ska mallens överände luta framåt). Annars hamnar stativets lutning i neck_pitch-offset.
- Passningen mot halsdelarna kontrolleras av skriptet (`out/nollmall.json`): fritt i nolläget, minst 0,2 mm till
  halsplåten utanför hålen och 3,9 mm till tornet, konerna ligger an mot hålkanterna.

### Glapp

- Givaren sitter på utgående axeln, så servot reglerar bort kugghjulsglappet i sin egen position.
  Det som återstår är dödbandet (några steg), glapp mellan horn och del, och böjning i utskrivna delar.
- Mät med **momentet på** och lasten som i drift, och använd servots **uppmätta** position (Unos
  tillstånd), inte det kommenderade målet. Det gör `head-capture`.
- Varje pose nås via en förpunkt 4° före (`--approach`), alltid från samma håll, så att glappet ligger
  åt samma håll i alla mätningar.
- `head-capture --both` mäter varje pose från båda hållen och `head-fit --backlash` skattar glappet per
  led (halva skillnaden). Det sparas som `glapp_grader` i `joints.json` och kan matas in i simulatorns
  backlash-modell.

### Procedur

Förutsättningar: servona har ID 30–33 och standardinställning (del A), hornen sitter på närmaste kugg
vid rå 2048, `odm_uno` är flashad och telefonen skickar data till datorn (nedan).

1. **Stativet:** ställ riggen där den ska stå. Lägg telefonen på basplattan, skärmen upp, överkanten
   framåt (+x). `odm_calib.py head-base` sparar lutningen i `host/kalibrering/stativ.json`.
   Över 2° lutning: rikta upp stativet och mät igen.
2. **Tecken, grovt:** `odm_calib.py head-plan` visar positiv riktning. `odm_bench.py move <led> 10` per
   led; fel håll → `odm_bench.py config --sign <led>=-1`. (Anpassningen hittar också tecknen, men en
   felvänd led kan köra åt fel håll under mätningen.)
3. **neck_pitch med mall:** momentet av, sätt mallen (`nollmall_neck_pitch.stl`, ovan) och håll den an, vrid
   halsen för hand tills kanten står lodrät (eller lutar som stativet), kör `odm_bench.py offsets neck_pitch`
   och ta bort mallen.
4. **Telefonen i hållaren**, skärmen framåt, överkanten åt robotens vänster (läge A i `placering.json`).
   Kontrollera att den ligger an. Lås skärmen inte (appen måste köra).
5. **Mätning:** `odm_calib.py head-capture` (lägg till `--both` för glapp). 24 poser, ca 2–3 minuter.
   Varje rad visar gravitationen, antal prover och rotationen; "RÖRDE SIG?" betyder att huvudet inte
   stod still (öka `--settle`). Ctrl+C avbryter och släpper momentet; det som mätts sparas.
6. **Anpassning:** `odm_calib.py head-fit kalibrering/huvud_<datum>.json` (torrkörning). Kontrollera:
   - residual RMS under 0,5° (över 1° varnar verktyget: glapp, flex, lös telefon eller fel montering),
   - "näst bästa teckenkombination" långt över bästa (annars är tecknen inte entydiga),
   - offseten inom ±7,2° plus monteringsfel; större tyder på fel kugg,
   - `--scale` ska ge skala inom 2 % från 1 (annars slirar något).
   Spara med `--save`: tecken och `offset_rad` för head_pitch, head_yaw och head_roll, och
   `kalibrering` (metod, datum, residual, glapp, skala) i `joints.json`.
7. **Kontroll:** `odm_calib.py zero neck_pitch head_pitch head_yaw head_roll` ska ge skärmen lodrät och
   rakt fram (nivåappen på telefonens kant, eller ny `head-capture --only 5`: gravitationen ska då vara
   (+1, 0, 0) i telefonens axlar inom några tusendelar). Kör om 5–6 efter att hornen eller telefonen
   flyttats.

Ändlägen som kontroll (valfritt): `odm_bench.py move <led> <grader>` i små steg mot gränsen med
momentgräns satt, och jämför var delen tar i med kollisionskontrollen (`cad/benchhead/check_fit.py`).

### Data från telefonen

Appen (planerad i `app/`) behöver för kalibreringen bara strömma Core Motion till datorn:

- `CMMotionManager.startDeviceMotionUpdates` med 100 Hz (`deviceMotionUpdateInterval = 0.01`),
  referensram spelar ingen roll (bara `gravity` och `rotationRate` används).
- Ett UDP-paket per uppdatering till datorns IP, port 5005, en rad text:
  `{"t": <timestamp>, "g": [gx, gy, gz], "w": [wx, wy, wz]}` eller `t,gx,gy,gz,wx,wy,wz`.
  `g` = `motion.gravity` (enhet g, telefonens axlar: x höger, y upp, z ut ur skärmen; skärmen upp ger
  (0, 0, −1)), `w` = `motion.rotationRate` (rad/s).
- `NSLocalNetworkUsageDescription` i Info.plist (annars tysta fel på lokalt nät) och
  `isIdleTimerDisabled = true`.

Mottagaren är `host/odm/phone.py` (`PhoneUDP`); den räknar medelvärde, spridning och rotation per pose.
Tills appen finns kan en färdig sensorapp som strömmar Core Motion över UDP användas, om formatet går att
ställa in (installation kräver användarens OK). Utan strömning återstår nivåappen för hand.

Telefonens montering läses ur `cad/benchhead/edits/placering.json` (`iphone12.R`, telefonens axlar i
riggen). Flyttas telefonen i CAD (Shapr3D, `import_edits.py`) följer kalibreringen med.

## Var resultatet sparas

`host/joints.json`, per led:

```json
"head_yaw": {
  "id": 32, "sign": 1, "offset_rad": -0.0903,
  "gear": "1/191",
  "gear_test": {"steg_per_s": 5480, "volt": 7.4, "gissning": "1/191"},
  "kalibrering": {"metod": "telefonens gravitation", "datum": "2026-10-28", "rms_grader": 0.21,
                  "glapp_grader": 0.4, "skala": 1.0}
}
```

`id`, `sign` och `offset_rad` används av `odm_bench.py`, `odm_calib.py` och (senare) appen. Övriga fält är
dokumentation. Mätfilerna hamnar i `host/kalibrering/` (stativ och poser); committa dem gärna tillsammans
med `joints.json` så att kalibreringen går att räkna om.

## Filer

| Fil | Innehåll |
|---|---|
| `odm_calib.py` | `audit`, `fix`, `gear`, `zero`, `head-base`, `head-plan`, `head-capture`, `head-fit` |
| `odm/servo.py` | STS3215-register utanför protokollet: offset med tecken, standardinställning, utväxlingstabell och klassning |
| `odm/headcal.py` | Halskinematik ur CAD-axlarna, gravitationsmodell, Levenberg–Marquardt, mätposer, syntetiska data. Ren Python |
| `odm/phone.py` | UDP-mottagare för telefonens data och simulerad telefon |
| `tests/test_calib.py`, `tests/test_headcal.py` | Mot simulerad Uno och telefon |
| `../firmware/odm_servo_setup` | `vaxel <id>` (hastighetstest), `info` med offsetens tecken |

### Ändringar i filer som andra arbeten också ändrar

Protokollet (`odm/protocol.py`, `protocol.h`) och `odm_uno` är **inte** ändrade. Övrigt:

- `odm/fake.py`: tre rader i `FakeServo.__init__` (fabriksvärden P 32, maxmoment 1000, momentgräns 1000 i
  registren 21, 16 och 48), så att `audit` ser ut som på riktiga servon.
- `odm/joints.py`: `Joints.extra` bevarar extra fält per led vid `load`/`save`.
- `odm_calib.py` importerar `active_names`, `minjerk`, `release` och `run` från `odm_bench.py`; byt inte
  namn på dem utan att ändra här.
- `odm_calib.py gear` sätter `PARAM_MAX_STEP` till 0 under testet och återställer det efteråt.

## Att verifiera på hårdvara

- Nominell tomgångsfart per utväxling (gränserna i `odm/servo.py` och setup-firmware), och att Goal_Speed 0
  verkligen betyder full fart.
- Antal kuggar på STS3215-axeln (25?).
- Register 48 (momentgräns, RAM) begränsar momentet utan att Uno skriver över det, och återställs till
  register 16 vid strömavbrott.
- Lastens skala (promille) och tecken i Unos tillstånd, så att `--max-load` stämmer.
- Core Motions `gravity`-tecken (skärmen upp ska ge z = −1) med telefonen i flera lägen.
- Att neck_pitch-axlarnas lodlinje i nolläget stämmer på den utskrivna riggen (mallen).
