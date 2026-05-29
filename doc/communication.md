## Messages between devices

### Movement:
- Forward =         FW
- Backward =        BW
- Turn Left =       TL
- Turn Right =      TR
- Rotate Left =     RL
- Rotate Right =    RR
- Stop =            SS

## Topic Layout:

### 1. Control Plane (Handshake & Verbinding)
- `Robots/Control/Connecting`        # Robot meldt zich aan met [MAC]
- `Robots/Control/{MAC}/Status`      # Server stuurt [checking] of [connected]

### 2. Data Plane (Posities & Besturing)
- `Robots/Data/Positions`             # Camera stuurt [ArUco, x, y, orientation, status]
- `Robots/Data/{MAC}/Commands`       # Server stuurt bewegingscode [FW, BW, SS, etc.]

---

# Hoe onze Robots verbinding maken (MQTT Protocol)

Dit document legt uit hoe de camera, de server en de robots met elkaar praten via MQTT. We hebben de architectuur opgesplitst in **Control** (voor verbindingen) en **Data** (voor acties). Hierdoor blijft de data zuiver en overlappen functies elkaar niet.

## 1. De Topics in het kort


| Topic | Wie stuurt naar wie? | Wat sturen ze mee? | Doel |
| :--- | :--- | :--- | :--- |
| `Robots/Control/Connecting` | Robot → Server | `[MAC]` | Een robot start op en vraagt om een verbinding. |
| `Robots/Control/{MAC}/Status` | Server → Robot | `[checking]` \| `[connected]` | De server stuurt statusupdates van de handshake naar de robot. |
| `Robots/Data/Positions` | Camera → Server | `[ArUco, x, y, orientation, status]` | De camera blijft de server vertellen waar elke robot rijdt en wat de LED-status is. |
| `Robots/Data/{MAC}/Commands` | Server → Robot | `[bewegingscode]` | Hier krijgt de robot zijn rij-instructies op binnen (alleen actief na verbinding). |

---

## 2. Het Stappenplan: Van opstarten tot rijden

### Stap 1: De robot klopt aan (Control)
De robot start op en stuurt zijn MAC-adres naar het centrale aanmeldtopic.
* **Topic:** `Robots/Control/Connecting`
* **Payload:** `[MAC]`
* **Actie robot:** De robot zet zijn fysieke LED-lampje direct op **Geel** (Connecting).

### Stap 2: De server checkt de robot (Control & Data)
De server hoort het verzoek en laat de robot weten dat hij in de wachtrij staat.
* **Topic:** `Robots/Control/{MAC}/Status` 
* **Payload:** `[checking]`

Nu vindt de automatische koppeling plaats via de camera op het data-kanaal:
1. De camera ziet een robot met een **Geel** lampje rijden.
2. De camera streamt deze informatie via `Robots/Data/Positions`.
3. **De Match:** De server leest dit data-topic uit, ziet dat `ArUco_1` op "connecting" staat, en weet dat dit hoort bij de robot die net `[MAC_1]` stuurde. De server koppelt ze intern: `ArUco_1 = MAC_1`.

### Stap 3: Groen licht! (Control $\rightarrow$ Data)
Nu de server de match heeft gemaakt, stuurt hij de definitieve bevestiging via het control-kanaal.
* **Topic:** `Robots/Control/{MAC}/Status` 
* **Payload:** `[connected]`

De robot vangt dit op:
* **Actie robot:** De robot zet zijn LED-lampje op **Groen** (Connected). Vanaf dit moment stopt de robot met luisteren naar het `Control`-topic en luistert hij **alleen nog maar naar het data-topic** `Robots/Data/{MAC}/Commands`.

### Stap 4: Rijden en data-efficiëntie (Data)
De server stuurt nu puur rijcommando's via `Robots/Data/{MAC}/Commands`. 

* **Uitval via Data:** Als de robot uitvalt, ziet de camera dit en stuurt via `Robots/Data/Positions` de status `"off"`. De server grijpt in en stopt de commando's.
* **Herverbinding via Control:** Verliest de robot de wifi? Dan springt zijn LED naar **Geel** en begint hij weer bij Stap 1 op `Robots/Control/Connecting`.

---

# MQTT Data & Topic Cheat Sheet (Alle Payloads)

## 1. Control Topics (Verbindingen)

### Verbindingsverzoek
* **Topic:** `Robots/Control/Connecting`
* **Payload:** `[00:B0:D0:63:C2:26]` *(Uniek MAC-adres van de robot)*

### Handshake Status
* **Topic:** `Robots/Control/{MAC}/Status`
* **Payloads:** 
  * `[checking]` *(Controlefase gestart)*
  * `[connected]` *(Handshake succesvol afgerond)*

## 2. Data Topics (Operatie)

### Positie & Camera Updates
* **Topic:** `Robots/Data/Positions`
* **Formaat:** `[ArUco_ID, x_positie, y_positie, oriëntatie_in_graden, led_status]`
* **Payloads:**
  * `[1, 544, 234, 90, "connecting"]` *(LED is geel)*
  * `[1, 550, 240, 92, "connected"]` *(LED is groen)*
  * `[1, 550, 240, 92, "off"]` *(LED is uit / robot kwijt)*

### Besturing & Commando's
* **Topic:** `Robots/Data/{MAC}/Commands`
* **Payloads:**
  * `[FW]` *(Forward)*
  * `[BW]` *(Backward)*
  * `[TL]` *(Turn Left)*
  * `[TR]` *(Turn Right)*
  * `[RL]` *(Rotate Left)*
  * `[RR]` *(Rotate Right)*
  * `[SS]` *(Stop)*
